module tb_conv_16_4_20_1();

    parameter T = 20;
    parameter NUMINPUTVALS = 10000;
    parameter NUMOUTPUTVALS = 8125;
    parameter INFILENAME = "tb_conv_16_4_20_1.in";
    parameter EXPFILENAME = "tb_conv_16_4_20_1.exp";

    logic clk, x_valid, x_ready, y_valid, y_ready, reset;
    logic  [T-1:0] x_data;
    logic signed [T-1:0] y_data;

    logic signed [T-1:0] inValues [NUMINPUTVALS-1:0];
    logic signed [T-1:0] expValues [NUMOUTPUTVALS-1:0];
    logic s;

    initial clk=0;
    always #5 clk = ~clk;
    
    conv_16_4_20_1 dut(clk, reset, x_data, x_valid, x_ready, y_data, y_valid, y_ready);

    logic rb, rb2;
    always begin
        @(posedge clk);
        #1;
        s=std::randomize(rb, rb2);
    end

    logic [31:0] j;

    always @* begin
        if (x_valid == 1)
            x_data = inValues[j];
        else
            x_data = 'x;
    end

    always @* begin
        if ((j>=0) && (j<NUMINPUTVALS) && (rb==1'b1))
            x_valid=1;
        else
            x_valid=0;
    end

    always @(posedge clk) begin
        if (x_valid && x_ready)
            j <= #1 j+1;
    end
  
    logic [31:0] i;
    always @* begin
        if ((i>=0) && (i<NUMOUTPUTVALS) && (rb2==1'b1))
            y_ready = 1;
        else
            y_ready = 0;
    end

    integer errors = 0;

    always @(posedge clk) begin
        if (y_ready && y_valid) begin
            if (y_data !== expValues[i]) begin
                $display($time,,"ERROR: y[%d] = %x; expected value = %x", i, y_data, expValues[i]);
                errors = errors+1;
            end
            i=i+1; 
        end 
    end

    ////////////////////////////////////////////////////////////////////////////////

    initial begin
      $readmemb(INFILENAME, inValues);
      $readmemb(EXPFILENAME, expValues);
      
        j=0; i=0;

        // Before first clock edge, initialize
        y_ready = 0; 
        reset = 0;
    
        // reset
        @(posedge clk); #1; reset = 1; 
        @(posedge clk); #1; reset = 0; 

        wait(i==NUMOUTPUTVALS);
        $display("Simulated %d outputs. Found %d errors.", NUMOUTPUTVALS, errors);
        $finish;
    end


endmodule
