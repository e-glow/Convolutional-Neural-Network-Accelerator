module layer1_64_33_16_16_f_rom(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd5;
        1: z <= -16'd6;
        2: z <= 16'd6;
        3: z <= -16'd5;
        4: z <= 16'd3;
        5: z <= -16'd1;
        6: z <= -16'd2;
        7: z <= 16'd1;
        8: z <= -16'd8;
        9: z <= 16'd5;
        10: z <= -16'd6;
        11: z <= -16'd6;
        12: z <= 16'd6;
        13: z <= 16'd7;
        14: z <= 16'd0;
        15: z <= 16'd7;
        16: z <= -16'd4;
        17: z <= -16'd2;
        18: z <= -16'd7;
        19: z <= 16'd3;
        20: z <= -16'd1;
        21: z <= 16'd2;
        22: z <= -16'd4;
        23: z <= -16'd4;
        24: z <= -16'd5;
        25: z <= -16'd7;
        26: z <= -16'd3;
        27: z <= -16'd6;
        28: z <= -16'd5;
        29: z <= 16'd2;
        30: z <= 16'd3;
        31: z <= -16'd8;
        32: z <= 16'd4;
      endcase
   end
endmodule

// Jason Cheung and Evan Low

module layer1_64_33_16_16(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);
   
   // x has length N and f has length M
   // take clog2 for address width
   parameter N = 64, M = 33, T = 16, P = 16;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 

   // input and output
   input clk, reset, x_valid, y_ready;
   input signed [T-1:0] x_data;
   output x_ready, y_valid;
   output signed [T-1:0] y_data;

   topctrl_layer1_64_33_16_16 #(N, M, T, P) topc(.clk(clk),.reset(reset), .x_data(x_data), .x_valid(x_valid), .y_valid(y_valid), .x_ready(x_ready),.y_ready(y_ready), .y_data(y_data));

endmodule

module topctrl_layer1_64_33_16_16 (clk, reset, x_data, x_valid, y_valid, x_ready, y_ready, y_data);
   // x has length N, f has length M, and y has length L
   // take clog2 for address width
   parameter N = 112, M = 49, T = 10, P = 1;
   localparam L = N-M+1;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 
   localparam LOGSIZE_L=$clog2(L);    
   localparam LOGSIZE_P=$clog2(P);
   // top level connections
   input clk, reset, x_valid, y_ready;
   output logic 		x_ready, y_valid;
   input signed [T-1:0] x_data;
   output signed [T-1:0]y_data;

   // counters for loading x, inner for loop, and outer for loop
   // we let load counters roll over 1, so length is LOGSIZE_* and not LOGSIZE_* - 1
   // this is because the counters increment when valid and ready are asserted, so rolling over means we are truly done
   // we also let inner counter roll over 1 because there is a cycle of delay for synchronous reads, so we are done when the counter is M, not M-1
   logic [LOGSIZE_N:0] 		load_x_counter;
   logic [LOGSIZE_P-1:0]        p_counter;

   logic done_x, done_load, done_conv, clear_counters, clear_p, inc_p;
   // signals between datapath and control
   logic 		wr_en_x;
   logic [LOGSIZE_N-1:0] addr_x [P-1:0];
   logic [LOGSIZE_M-1:0] addr_f [P-1:0];
   logic [P-1:0]        clear_acc, en_acc;

   logic signed [T-1:0] y_data_p [P-1:0];
   logic [P-1:0] y_ready_p, y_valid_p, done_p;

   // select which parallel datapath to send ready signal and read from based on p_counter
   always_comb begin
      integer j;
      for (j = 0; j < P; j= j+1) begin
          y_ready_p[j] = 0;
      end 
      y_ready_p[p_counter] = y_ready;
   end 
   assign y_data = y_data_p[p_counter];
   assign y_valid = y_valid_p[p_counter];
   
   // generate P datapath and P control
   genvar i;
   generate 
       for (i = 0; i < P; i = i + 1) begin 
           datapath_i_layer1_64_33_16_16 #(N, M, T) d(.clk(clk),.reset(reset),.x_data(x_data),.y_data(y_data_p[i]),.addr_x(addr_x[i]),.addr_f(addr_f[i]),.wr_en_x(wr_en_x),.en_acc(en_acc[i]),.clear_acc(clear_acc[i]));
           ctrlpath_i #(i, N, M, T, P) c(.clk(clk), .reset(reset), .load_x_counter(load_x_counter), .y_valid(y_valid_p[i]), .y_ready(y_ready_p[i]), .addr_x(addr_x[i]), .addr_f(addr_f[i]), .clear_acc(clear_acc[i]), .en_acc(en_acc[i]), .done_p(done_p[i]), .done_load(done_load));
       end 
   endgenerate 

   // load counters 
   // increment if state is data is valid and system is ready  
   always_ff @(posedge clk) begin
      if (clear_counters == 1)
        load_x_counter <= 0;
      
      else if (x_valid == 1 && x_ready == 1)
        load_x_counter <= load_x_counter+1;
   end


  // p counter logic
   always_ff @(posedge clk) begin
      if (clear_counters == 1 || clear_p)
        p_counter <= 0;
      
      else if (inc_p)
        p_counter <= p_counter + 1;
   end

   // top level state machine is just load or process
   // we factor load logic out from individual control modules to save some counters
   // signals to change state come from the individual control modules
   enum {INIT, LOAD, PROCESS, IDLE} state, next_state;

   // clear when you finish the convolution
   assign clear_counters = reset || done_conv;
   
   // increment p when we output a value
   assign inc_p = y_valid_p[p_counter] == 1 && y_ready_p[p_counter] == 1;
   
   // clear p after we read from the last P-1 datapath
   assign clear_p = p_counter == P-1 && inc_p;
   
   // done loading if the counter rolled over or counter is at last address and valid is asserted
   assign done_x = (load_x_counter == N) || (load_x_counter == N-1 && x_valid == 1);
   
   // done signal to subcontrol modules
   assign done_load = done_x && state == LOAD;
   
   // done convolution if we read from the last P-1 datapath and receive done signal from P-1 control
   assign done_conv = (p_counter == P-1 && done_p[p_counter]);

   //next_state logic
   always_comb begin
      if (state == INIT)
	next_state = LOAD;
      else if (state == LOAD) begin
	 // done loading if both counters are done
	 if (done_x == 1)
	   next_state = PROCESS;    
         else
	   next_state = LOAD;    
      end
      
      else if (state == PROCESS) begin
	 
	 // wait for ready 
	 if (done_conv == 1)
	 	next_state = LOAD;
	 else
	   next_state = PROCESS;
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

   // x and f are ready if the counter has not rolled over and the state is load
   assign x_ready = (state == LOAD && load_x_counter < N);
   assign wr_en_x = (state == LOAD && load_x_counter < N);

endmodule

// datapath from Figure 6 includes memories, multiplier, adder, and accumulator
module datapath_i_layer1_64_33_16_16(clk, reset, x_data, y_data, addr_x, addr_f, wr_en_x, en_acc, clear_acc);

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
   layer1_64_33_16_16_f_rom myMemInst_f(clk, addr_f, f_out);


   // product pipeline register with saturation
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
   // clear and enable come from control
      assign sum = accum + product_reg;
      
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

module layer2_32_9_16_4_f_rom(clk, addr, z);
   input clk;
   input [3:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd1;
        1: z <= -16'd5;
        2: z <= -16'd1;
        3: z <= -16'd7;
        4: z <= 16'd2;
        5: z <= -16'd8;
        6: z <= -16'd7;
        7: z <= -16'd1;
        8: z <= -16'd5;
      endcase
   end
endmodule

// Jason Cheung and Evan Low

module layer2_32_9_16_4(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);
   
   // x has length N and f has length M
   // take clog2 for address width
   parameter N = 32, M = 9, T = 16, P = 4;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 

   // input and output
   input clk, reset, x_valid, y_ready;
   input signed [T-1:0] x_data;
   output x_ready, y_valid;
   output signed [T-1:0] y_data;

   topctrl_layer2_32_9_16_4 #(N, M, T, P) topc(.clk(clk),.reset(reset), .x_data(x_data), .x_valid(x_valid), .y_valid(y_valid), .x_ready(x_ready),.y_ready(y_ready), .y_data(y_data));

endmodule

module topctrl_layer2_32_9_16_4 (clk, reset, x_data, x_valid, y_valid, x_ready, y_ready, y_data);
   // x has length N, f has length M, and y has length L
   // take clog2 for address width
   parameter N = 112, M = 49, T = 10, P = 1;
   localparam L = N-M+1;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 
   localparam LOGSIZE_L=$clog2(L);    
   localparam LOGSIZE_P=$clog2(P);
   // top level connections
   input clk, reset, x_valid, y_ready;
   output logic 		x_ready, y_valid;
   input signed [T-1:0] x_data;
   output signed [T-1:0]y_data;

   // counters for loading x, inner for loop, and outer for loop
   // we let load counters roll over 1, so length is LOGSIZE_* and not LOGSIZE_* - 1
   // this is because the counters increment when valid and ready are asserted, so rolling over means we are truly done
   // we also let inner counter roll over 1 because there is a cycle of delay for synchronous reads, so we are done when the counter is M, not M-1
   logic [LOGSIZE_N:0] 		load_x_counter;
   logic [LOGSIZE_P-1:0]        p_counter;

   logic done_x, done_load, done_conv, clear_counters, clear_p, inc_p;
   // signals between datapath and control
   logic 		wr_en_x;
   logic [LOGSIZE_N-1:0] addr_x [P-1:0];
   logic [LOGSIZE_M-1:0] addr_f [P-1:0];
   logic [P-1:0]        clear_acc, en_acc;

   logic signed [T-1:0] y_data_p [P-1:0];
   logic [P-1:0] y_ready_p, y_valid_p, done_p;

   // select which parallel datapath to send ready signal and read from based on p_counter
   always_comb begin
      integer j;
      for (j = 0; j < P; j= j+1) begin
          y_ready_p[j] = 0;
      end 
      y_ready_p[p_counter] = y_ready;
   end 
   assign y_data = y_data_p[p_counter];
   assign y_valid = y_valid_p[p_counter];
   
   // generate P datapath and P control
   genvar i;
   generate 
       for (i = 0; i < P; i = i + 1) begin 
           datapath_i_layer2_32_9_16_4 #(N, M, T) d(.clk(clk),.reset(reset),.x_data(x_data),.y_data(y_data_p[i]),.addr_x(addr_x[i]),.addr_f(addr_f[i]),.wr_en_x(wr_en_x),.en_acc(en_acc[i]),.clear_acc(clear_acc[i]));
           ctrlpath_i #(i, N, M, T, P) c(.clk(clk), .reset(reset), .load_x_counter(load_x_counter), .y_valid(y_valid_p[i]), .y_ready(y_ready_p[i]), .addr_x(addr_x[i]), .addr_f(addr_f[i]), .clear_acc(clear_acc[i]), .en_acc(en_acc[i]), .done_p(done_p[i]), .done_load(done_load));
       end 
   endgenerate 

   // load counters 
   // increment if state is data is valid and system is ready  
   always_ff @(posedge clk) begin
      if (clear_counters == 1)
        load_x_counter <= 0;
      
      else if (x_valid == 1 && x_ready == 1)
        load_x_counter <= load_x_counter+1;
   end


  // p counter logic
   always_ff @(posedge clk) begin
      if (clear_counters == 1 || clear_p)
        p_counter <= 0;
      
      else if (inc_p)
        p_counter <= p_counter + 1;
   end

   // top level state machine is just load or process
   // we factor load logic out from individual control modules to save some counters
   // signals to change state come from the individual control modules
   enum {INIT, LOAD, PROCESS, IDLE} state, next_state;

   // clear when you finish the convolution
   assign clear_counters = reset || done_conv;
   
   // increment p when we output a value
   assign inc_p = y_valid_p[p_counter] == 1 && y_ready_p[p_counter] == 1;
   
   // clear p after we read from the last P-1 datapath
   assign clear_p = p_counter == P-1 && inc_p;
   
   // done loading if the counter rolled over or counter is at last address and valid is asserted
   assign done_x = (load_x_counter == N) || (load_x_counter == N-1 && x_valid == 1);
   
   // done signal to subcontrol modules
   assign done_load = done_x && state == LOAD;
   
   // done convolution if we read from the last P-1 datapath and receive done signal from P-1 control
   assign done_conv = (p_counter == P-1 && done_p[p_counter]);

   //next_state logic
   always_comb begin
      if (state == INIT)
	next_state = LOAD;
      else if (state == LOAD) begin
	 // done loading if both counters are done
	 if (done_x == 1)
	   next_state = PROCESS;    
         else
	   next_state = LOAD;    
      end
      
      else if (state == PROCESS) begin
	 
	 // wait for ready 
	 if (done_conv == 1)
	 	next_state = LOAD;
	 else
	   next_state = PROCESS;
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

   // x and f are ready if the counter has not rolled over and the state is load
   assign x_ready = (state == LOAD && load_x_counter < N);
   assign wr_en_x = (state == LOAD && load_x_counter < N);

endmodule

// datapath from Figure 6 includes memories, multiplier, adder, and accumulator
module datapath_i_layer2_32_9_16_4(clk, reset, x_data, y_data, addr_x, addr_f, wr_en_x, en_acc, clear_acc);

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
   layer2_32_9_16_4_f_rom myMemInst_f(clk, addr_f, f_out);


   // product pipeline register with saturation
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
   // clear and enable come from control
      assign sum = accum + product_reg;
      
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

module layer3_24_10_16_3_f_rom(clk, addr, z);
   input clk;
   input [3:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd5;
        1: z <= -16'd3;
        2: z <= -16'd6;
        3: z <= 16'd4;
        4: z <= -16'd4;
        5: z <= -16'd2;
        6: z <= -16'd6;
        7: z <= -16'd3;
        8: z <= -16'd7;
        9: z <= 16'd1;
      endcase
   end
endmodule

// Jason Cheung and Evan Low

module layer3_24_10_16_3(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);
   
   // x has length N and f has length M
   // take clog2 for address width
   parameter N = 24, M = 10, T = 16, P = 3;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 

   // input and output
   input clk, reset, x_valid, y_ready;
   input signed [T-1:0] x_data;
   output x_ready, y_valid;
   output signed [T-1:0] y_data;

   topctrl_layer3_24_10_16_3 #(N, M, T, P) topc(.clk(clk),.reset(reset), .x_data(x_data), .x_valid(x_valid), .y_valid(y_valid), .x_ready(x_ready),.y_ready(y_ready), .y_data(y_data));

endmodule

module topctrl_layer3_24_10_16_3 (clk, reset, x_data, x_valid, y_valid, x_ready, y_ready, y_data);
   // x has length N, f has length M, and y has length L
   // take clog2 for address width
   parameter N = 112, M = 49, T = 10, P = 1;
   localparam L = N-M+1;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 
   localparam LOGSIZE_L=$clog2(L);    
   localparam LOGSIZE_P=$clog2(P);
   // top level connections
   input clk, reset, x_valid, y_ready;
   output logic 		x_ready, y_valid;
   input signed [T-1:0] x_data;
   output signed [T-1:0]y_data;

   // counters for loading x, inner for loop, and outer for loop
   // we let load counters roll over 1, so length is LOGSIZE_* and not LOGSIZE_* - 1
   // this is because the counters increment when valid and ready are asserted, so rolling over means we are truly done
   // we also let inner counter roll over 1 because there is a cycle of delay for synchronous reads, so we are done when the counter is M, not M-1
   logic [LOGSIZE_N:0] 		load_x_counter;
   logic [LOGSIZE_P-1:0]        p_counter;

   logic done_x, done_load, done_conv, clear_counters, clear_p, inc_p;
   // signals between datapath and control
   logic 		wr_en_x;
   logic [LOGSIZE_N-1:0] addr_x [P-1:0];
   logic [LOGSIZE_M-1:0] addr_f [P-1:0];
   logic [P-1:0]        clear_acc, en_acc;

   logic signed [T-1:0] y_data_p [P-1:0];
   logic [P-1:0] y_ready_p, y_valid_p, done_p;

   // select which parallel datapath to send ready signal and read from based on p_counter
   always_comb begin
      integer j;
      for (j = 0; j < P; j= j+1) begin
          y_ready_p[j] = 0;
      end 
      y_ready_p[p_counter] = y_ready;
   end 
   assign y_data = y_data_p[p_counter];
   assign y_valid = y_valid_p[p_counter];
   
   // generate P datapath and P control
   genvar i;
   generate 
       for (i = 0; i < P; i = i + 1) begin 
           datapath_i_layer3_24_10_16_3 #(N, M, T) d(.clk(clk),.reset(reset),.x_data(x_data),.y_data(y_data_p[i]),.addr_x(addr_x[i]),.addr_f(addr_f[i]),.wr_en_x(wr_en_x),.en_acc(en_acc[i]),.clear_acc(clear_acc[i]));
           ctrlpath_i #(i, N, M, T, P) c(.clk(clk), .reset(reset), .load_x_counter(load_x_counter), .y_valid(y_valid_p[i]), .y_ready(y_ready_p[i]), .addr_x(addr_x[i]), .addr_f(addr_f[i]), .clear_acc(clear_acc[i]), .en_acc(en_acc[i]), .done_p(done_p[i]), .done_load(done_load));
       end 
   endgenerate 

   // load counters 
   // increment if state is data is valid and system is ready  
   always_ff @(posedge clk) begin
      if (clear_counters == 1)
        load_x_counter <= 0;
      
      else if (x_valid == 1 && x_ready == 1)
        load_x_counter <= load_x_counter+1;
   end


  // p counter logic
   always_ff @(posedge clk) begin
      if (clear_counters == 1 || clear_p)
        p_counter <= 0;
      
      else if (inc_p)
        p_counter <= p_counter + 1;
   end

   // top level state machine is just load or process
   // we factor load logic out from individual control modules to save some counters
   // signals to change state come from the individual control modules
   enum {INIT, LOAD, PROCESS, IDLE} state, next_state;

   // clear when you finish the convolution
   assign clear_counters = reset || done_conv;
   
   // increment p when we output a value
   assign inc_p = y_valid_p[p_counter] == 1 && y_ready_p[p_counter] == 1;
   
   // clear p after we read from the last P-1 datapath
   assign clear_p = p_counter == P-1 && inc_p;
   
   // done loading if the counter rolled over or counter is at last address and valid is asserted
   assign done_x = (load_x_counter == N) || (load_x_counter == N-1 && x_valid == 1);
   
   // done signal to subcontrol modules
   assign done_load = done_x && state == LOAD;
   
   // done convolution if we read from the last P-1 datapath and receive done signal from P-1 control
   assign done_conv = (p_counter == P-1 && done_p[p_counter]);

   //next_state logic
   always_comb begin
      if (state == INIT)
	next_state = LOAD;
      else if (state == LOAD) begin
	 // done loading if both counters are done
	 if (done_x == 1)
	   next_state = PROCESS;    
         else
	   next_state = LOAD;    
      end
      
      else if (state == PROCESS) begin
	 
	 // wait for ready 
	 if (done_conv == 1)
	 	next_state = LOAD;
	 else
	   next_state = PROCESS;
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

   // x and f are ready if the counter has not rolled over and the state is load
   assign x_ready = (state == LOAD && load_x_counter < N);
   assign wr_en_x = (state == LOAD && load_x_counter < N);

endmodule

// datapath from Figure 6 includes memories, multiplier, adder, and accumulator
module datapath_i_layer3_24_10_16_3(clk, reset, x_data, y_data, addr_x, addr_f, wr_en_x, en_acc, clear_acc);

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
   layer3_24_10_16_3_f_rom myMemInst_f(clk, addr_f, f_out);


   // product pipeline register with saturation
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
   // clear and enable come from control
      assign sum = accum + product_reg;
      
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

// Jason Cheung and Evan Low

module net_64_33_9_10_16_30(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);
   parameter N = 64, M1 = 33, M2 = 9, M3 = 10, T = 16, P1 = 16, P2 = 4, P3 = 3;
   localparam L1 = N-M1+1, L2 = L1-M2+1;

   // input and output
   input clk, reset, x_valid, y_ready;
   input signed [T-1:0] x_data;
   output x_ready, y_valid;
   output signed [T-1:0] y_data;

   // signals to connect layers
   logic signed [T-1:0] y_data1;
   logic y_valid1;

   logic x_ready2;
   logic signed [T-1:0] y_data2;
   logic y_valid2;

   logic x_ready3;

   // connect 3 layers together 
   // make sure the parameters are correct, input o layer is output of previous
   layer1_64_33_16_16 #(N, M1, T, P1) layer1(.clk(clk),.reset(reset), .x_data(x_data), .x_valid(x_valid), .x_ready(x_ready), .y_data(y_data1), .y_valid(y_valid1), .y_ready(x_ready2));
   layer2_32_9_16_4 #(L1, M2, T, P2) layer2(.clk(clk),.reset(reset), .x_data(y_data1), .x_valid(y_valid1), .x_ready(x_ready2), .y_data(y_data2), .y_valid(y_valid2), .y_ready(x_ready3));
   layer3_24_10_16_3 #(L2, M3, T, P3) layer3(.clk(clk),.reset(reset), .x_data(y_data2), .x_valid(y_valid2), .x_ready(x_ready3), .y_data(y_data), .y_valid(y_valid), .y_ready(y_ready));


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

module ctrlpath_i(clk, reset, load_x_counter, y_valid, y_ready, addr_x, addr_f, clear_acc, en_acc, done_p, done_load);

   // x has length N, f has length M, and y has length L
   // take clog2 for address width
   parameter index = 0, N = 112, M = 49, T = 10, P = 1;
   localparam L = N-M+1;
   localparam LOGSIZE_N=$clog2(N);
   localparam LOGSIZE_M=$clog2(M); 
   localparam LOGSIZE_L=$clog2(L);    

   // top level connections
   input clk, reset, y_ready, done_load;
   input [LOGSIZE_N:0] 		load_x_counter;
   output y_valid, done_p;
   
   // to datapath
   output logic [LOGSIZE_N-1:0] addr_x;
   output logic [LOGSIZE_M-1:0] addr_f;
   output logic 		clear_acc, en_acc;
   
   // signals for done, increment, and clear counters
   logic 			inc_inner, clear_counters, clear_inner, inc_outer, done_inner, done_outer;
   
   logic [LOGSIZE_M:0] 		inner_counter; 
   logic [LOGSIZE_L - 1:0] 	outer_counter;
   // states:
   // INIT: start here after reset 
   // LOAD: load x 
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
	 if (done_load == 1)
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
   always_comb begin
      if ( state == CLEAR_ACC || state == INC_INNER || state == FIRST_PIPE) begin
         addr_x = inner_counter + outer_counter; // x[n+k]
         addr_f = inner_counter;
      end
      else if (state == LOAD) begin
         addr_x = load_x_counter;
         addr_f = 0;
      end 
      // no latch
      else begin
         addr_x = 0;
         addr_f = 0;
      end 
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
        outer_counter <= index;
      else if (inc_outer == 1)
        outer_counter <= outer_counter + P;
   end	
   
   
   // clear when you finish the convolution
   assign clear_counters = (reset == 1 || (next_state == LOAD && state == WAIT_READY));
   assign done_p = clear_counters;

   // last iteration of inner counter is M because of cycle of delay for synchronous read
   // last iteration of outer counter is L - 1
   assign done_inner = (inner_counter == M + 1);
   assign done_outer = (outer_counter >= L - P);

   // increment inner in the INC_INNER state and you aren't done
   // also increment in clear_acc because the old value will be read by the memory
   assign inc_inner = (state == INC_INNER || state == CLEAR_ACC || state == FIRST_PIPE) && done_inner == 0;
   
   // y is valid in WAIT_READY state 
   // inc outer when y valid and y ready and counter is not done
   assign y_valid = (state == WAIT_READY);
   assign inc_outer = (state == WAIT_READY && y_ready == 1) && done_outer == 0;
   
   // clear the accumulator in clear acc
   assign clear_acc = (state == CLEAR_ACC);
   
   // inner counter is used for the memory address 
   // clearing the inner counter before CLEAR_ACC will make the address 0 during CLEAR_ACC
   // and value at y[0] will be output at the following cycle, and everything is aligned
   assign clear_inner = (next_state == CLEAR_ACC);
   
   // accumulate during the inner for loop
   assign en_acc = (state == INC_INNER);

   
endmodule
