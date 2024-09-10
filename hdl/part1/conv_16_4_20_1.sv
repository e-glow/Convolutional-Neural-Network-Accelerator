module conv_16_4_20_1_f_rom(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [19:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 20'd158;
        1: z <= -20'd514;
        2: z <= -20'd415;
        3: z <= -20'd64;
      endcase
   end
endmodule

// Jason Cheung and Evan Low

module conv_16_4_20_1(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);
   
   // x has length N and f has length M
   // take clog2 for address width
   parameter N = 16, M = 4, T = 20;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 

   // input and output
   input clk, reset, x_valid, y_ready;
   input signed [T-1:0] x_data;
   output 	      x_ready, y_valid;
   output signed [T-1:0] y_data;
   
   // signals between datapath and control
   logic 		wr_en_x;
   logic [LOGSIZE_N-1:0] addr_x;
   logic [LOGSIZE_M-1:0] addr_f;
   logic 		 clear_acc, en_acc;

   datapath #(N, M, T) d(.clk(clk),.reset(reset),.x_data(x_data),.y_data(y_data),.addr_x(addr_x),.addr_f(addr_f),.wr_en_x(wr_en_x),.en_acc(en_acc),.clear_acc(clear_acc));
   ctrlpath #(N, M) c(.clk(clk),.reset(reset),.x_valid(x_valid),.y_valid(y_valid),.x_ready(x_ready),.y_ready(y_ready),.addr_x(addr_x),.addr_f(addr_f),.wr_en_x(wr_en_x),.clear_acc(clear_acc),.en_acc(en_acc));

endmodule

// datapath from Figure 6 includes memories, multiplier, adder, and accumulator
module datapath(clk, reset, x_data, y_data, addr_x, addr_f, wr_en_x, en_acc, clear_acc);

   // x has length N and f has length M
   // take clog2 for address width
   parameter N = 112, M = 49, T = 10;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M);   
   
   localparam signed [T-1:0] max = 2**(T-1)-1;
   localparam signed [T-1:0] min = -1*(2**(T-1));

   // top level connections
   input clk, reset;
   input signed [T-1:0] x_data;
   output logic signed [T-1:0] y_data;
   
   // signals from control
   input [LOGSIZE_N-1:0]      addr_x;
   input [LOGSIZE_M-1:0]      addr_f;
   input 		      wr_en_x, en_acc, clear_acc;
   
   // output of memories
   logic signed [T-1:0] 	      x_out;
   logic signed [T-1:0] 	      f_out;
   
   // check overflow
   logic signed [2*T-1:0] product;
   logic signed [T-1:0] product_reg;
   logic signed [T:0] sum;
   logic signed [T-1:0] accum;
   
   // x memory has N 10 bit words 
   // f memory has M 10 bit words
   // addresses and enables come from control
   memory #(T, N) myMemInst_x(clk, x_data, x_out, addr_x, wr_en_x);
   conv_16_4_20_1_f_rom myMemInst_f(clk, addr_f, f_out);
   

   // product register pipeline with saturation
   assign product = x_out * f_out;
   
   always_ff @(posedge clk) begin
	  if (product > max) 
	     product_reg <= max;
	  else if (product < min)
	     product_reg <= min;
          else 
             product_reg <= product[T-1:0];
   end
   
// accumulator process with saturation
      assign sum = accum + product_reg;
   
   // clear and enable come from control
   always_ff @(posedge clk) begin
      if (reset == 1 || clear_acc == 1) 
   accum <= 0; 

      else if(en_acc == 1)
          if (sum > max) 
	     accum <= max;
	  else if (sum < min)
	     accum <= min;
          else 
             accum <= sum[T-1:0];
   end
   
   // relu
   always_comb begin
       if (accum < 0)
            y_data = 0;
       else 
            y_data = accum;
   end

endmodule

module ctrlpath(clk, reset, x_valid, y_valid, x_ready, y_ready, addr_x, addr_f, wr_en_x, clear_acc, en_acc);

   // x has length N, f has length M, and y has length L
   // take clog2 for address width
   parameter N = 112, M = 49;
   localparam L = N-M+1;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 
   localparam LOGSIZE_L=$clog2(L);    

   // top level connections
   input clk, reset, x_valid, y_ready;
   
   // to datapath
   output logic [LOGSIZE_N-1:0] addr_x;
   output logic [LOGSIZE_M-1:0] addr_f;
   output logic 		wr_en_x, clear_acc, en_acc, x_ready, y_valid;

   // counters for loading x, inner for loop, and outer for loop
   // we let load counters roll over 1, so length is LOGSIZE_* and not LOGSIZE_* - 1
   // this is because the counters increment when valid and ready are asserted, so rolling over means we are truly done
   // we also let inner counter roll over 1 because there is a cycle of delay for synchronous reads, so we are done when the counter is M, not M-1
   logic [LOGSIZE_N:0] 		load_x_counter;
   logic [LOGSIZE_M:0] 		inner_counter; 
   logic [LOGSIZE_L - 1:0] 	outer_counter;
   
   // signals for done, increment, and clear counters
   logic 			done_x, inc_inner, clear_counters, clear_inner, inc_outer, done_inner, done_outer;
   
   // states:
   // INIT: start here after reset 
   // LOAD: load x and f at the same time
   // CLEAR_ACC: clear accumulator, also delays 1 cycle for alignment of synchronous reads
   // INC_INNER: inner for loop to compute a single y[n]
   // WAIT_READY: output y[n], another 1 cycle overhead for loading the data into the accumulator
   // IDLE: for unknown state (make sure next_state is not a latch)
   enum 			{INIT, LOAD, CLEAR_ACC, FIRST_PIPE, INC_INNER, WAIT_READY, IDLE} state, next_state;
   
   //next_state logic
   always_comb begin
      if (state == INIT)
	next_state = LOAD;
      else if (state == LOAD) begin
	 // done loading if both counters are done
	 if (done_x == 1)
	   next_state = CLEAR_ACC;    
         else
	   next_state = LOAD;    
      end
      
      else if (state == CLEAR_ACC) 
        next_state = FIRST_PIPE;
      else if (state == FIRST_PIPE)
        next_state = INC_INNER;
      else if (state == INC_INNER) begin
	 
	 // output is valid when inner loop is done
	 if (done_inner == 1)
	   next_state = WAIT_READY;
         else
           next_state = INC_INNER;
      end
      else if (state == WAIT_READY) begin
	 
	 // wait for ready 
	 if (y_ready == 1)
	    
	   // load the next set (next convolution) if outer loop is done
           if (done_outer == 1)
             next_state = LOAD;
           else 
	     next_state = CLEAR_ACC;
	 else
	   next_state = WAIT_READY;
      end
      // no latch
      else 
        next_state = IDLE;
   end

   // state register
   always_ff @(posedge clk) begin
      if (reset == 1)
        state <= INIT;
      else
        state <= next_state;      
   end
   
   // the address comes from the counters and depends on the state
   assign addr_f = inner_counter;
   always_comb begin
      if (state == LOAD) begin
         addr_x = load_x_counter;
      end
      else if ( state == CLEAR_ACC || state == INC_INNER || state == FIRST_PIPE) begin
         addr_x = inner_counter + outer_counter; // x[n+k]
      end
      
      // no latch
      else begin
         addr_x = 0;
      end 
   end 

   // load counters 
   // increment if state is data is valid and system is ready  
   always_ff @(posedge clk) begin
      if (clear_counters == 1)
        load_x_counter <= 0;
      
      else if (x_valid == 1 && x_ready == 1)
        load_x_counter <= load_x_counter+1;
   end

   
   //inner counter
   always_ff @(posedge clk) begin
      if (clear_counters == 1 || clear_inner == 1) 
        inner_counter <= 0;

      else if (inc_inner == 1) 
	inner_counter <= inner_counter + 1;
   end	
   
   //outer counter
   always_ff @(posedge clk) begin
      if (clear_counters == 1)
        outer_counter <= 0;
      else if (inc_outer == 1)
        outer_counter <= outer_counter + 1;
   end	
   
   // clear when you finish the convolution
   assign clear_counters = (reset == 1 || (next_state == LOAD && state == WAIT_READY));
   
   // done if the counter rolled over or counter is at last address and valid is asserted
   assign done_x = (load_x_counter == N) || (load_x_counter == N-1 && x_valid == 1);

   // last iteration of inner counter is M because of cycle of delay for synchronous read
   // last iteration of outer counter is L - 1
   assign done_inner = (inner_counter == M + 1);
   assign done_outer = (outer_counter == L - 1);

   // increment inner in the INC_INNER state and you aren't done
   // also increment in clear_acc because the old value will be read by the memory
   assign inc_inner = (state == INC_INNER || state == CLEAR_ACC || state == FIRST_PIPE) && done_inner == 0;
   
   // y is valid in WAIT_READY state 
   // inc outer when y valid and y ready and counter is not done
   assign y_valid = (state == WAIT_READY);
   assign inc_outer = (state == WAIT_READY && y_ready == 1) && done_outer == 0;
   
   // x and f are ready if the counter has not rolled over and the state is load
   assign x_ready = (state == LOAD && load_x_counter < N);
   assign wr_en_x = (state == LOAD && load_x_counter < N);
   
   // clear the accumulator in clear acc
   assign clear_acc = (state == CLEAR_ACC);
   
   // inner counter is used for the memory address 
   // clearing the inner counter before CLEAR_ACC will make the address 0 during CLEAR_ACC
   // and value at y[0] will be output at the following cycle, and everything is aligned
   assign clear_inner = (next_state == CLEAR_ACC);
   
   // accumulate during the inner for loop
   assign en_acc = (state == INC_INNER);

   
endmodule

module memory(clk, data_in, data_out, addr, wr_en);
   
   parameter                   WIDTH=16, SIZE=64;
   localparam                  LOGSIZE=$clog2(SIZE);
   input [WIDTH-1:0]           data_in;
   output logic [WIDTH-1:0]    data_out;
   input [LOGSIZE-1:0] 	       addr;
   input                       clk, wr_en;
   
   logic [SIZE-1:0][WIDTH-1:0] mem;
   
   always_ff @(posedge clk) begin
      data_out <= mem[addr];
      if (wr_en)
        mem[addr] <= data_in;
   end
endmodule
