module  FFT(
    input           clk      , 
    input           rst      , 
    input  [15:0]   fir_d    , 
    input           fir_valid,
    output          fft_valid, 
    output          done     ,
    output [15:0]   fft_d1   , 
    output [15:0]   fft_d2   ,
    output [15:0]   fft_d3   , 
    output [15:0]   fft_d4   , 
    output [15:0]   fft_d5   , 
    output [15:0]   fft_d6   , 
    output [15:0]   fft_d7   , 
    output [15:0]   fft_d8   ,
    output [15:0]   fft_d9   , 
    output [15:0]   fft_d10  , 
    output [15:0]   fft_d11  , 
    output [15:0]   fft_d12  , 
    output [15:0]   fft_d13  , 
    output [15:0]   fft_d14  , 
    output [15:0]   fft_d15  , 
    output [15:0]   fft_d0
);

/////////////////////////////////
// Please write your code here //
/////////////////////////////////

reg [31:0] x[0:15];
                        
S2P S2P1(.clk(clk)                  ,
         .rst(rst)                  ,
         .fir_d(fir_d)              ,
         .fir_valid(fir_valid)      ,
         .buffer_ready(buffer_ready),
         .buffer(x));
                        
                        
                        
                        
                        
                        



endmodule


module S2P(
    input             clk          , 
    input             rst          , 
    input      [15:0] fir_d        , 
    input             fir_valid    ,
    output reg        buffer_ready ,
    output reg [31:0] buffer [0:15]
);

integer   i         ;
reg [3:0] ctr       ;
reg       delay_flag;

always @(posedge clk or posedge rst) begin
	if (rst) begin
        ctr          <= 4'd0;
        buffer_ready <= 0   ;
        for (i = 0; i < 16; i = i + 1) buffer[i] <= 32'd0;

    end	else if (fir_valid) begin
		delay_flag   <= (ctr == 4'd15)                 ;
        buffer_ready <= delay_flag                     ;
        ctr          <= (ctr == 4'd15) ? 4'd0 : ctr + 1; 
        buffer[ctr]  <= {
                         {8{fir_d[15]}}, // sign extension of fir_d
                         fir_d,          // middle 16 bits
                         8'd0            // zero padding
        };
    end else begin
        delay_flag   <= 0;
        buffer_ready <= 0;
    end
end
endmodule

module butterfly(
    input  signed [31:0] a         ,
    input  signed [31:0] b         ,
    input  signed [31:0] c         ,
    input  signed [31:0] d         ,
    input  signed [31:0] W_real    ,
    input  signed [31:0] W_imag    ,
    output signed [31:0] FFT_A_real,
    output signed [31:0] FFT_A_imag,
    output signed [31:0] FFT_B_real,
    output signed [31:0] FFT_B_imag
);
// X = a + bj
// -------------------。--。---------> fft_a = (a+c) + (b+d)j
//                     \ /
//                      X
//                     / \
// -------------------。--。--(W^n)--> fft_b = (a+c) + (b+d)j * (W^n_real + j*W^n_imag)
// Y = c + dj
wire signed [31:0] ac_diff = a - c;
wire signed [31:0] bd_diff = b - d; 

wire signed [63:0] fft_mul1 =  ac_diff * W_real;
wire signed [63:0] fft_mul2 =  bd_diff * W_imag;    // (d-b)W_imag
wire signed [63:0] fft_mul3 =  ac_diff * W_imag;
wire signed [63:0] fft_mul4 = -bd_diff * W_real;    // (b-d)W_read

assign FFT_A_real = a               + c              ;
assign FFT_A_imag = b               + d              ;
assign FFT_B_real = fft_mul1[47:16] + fft_mul2[47:16];
assign FFT_B_imag = fft_mul3[47:16] + fft_mul4[47:16];  
endmodule

module FFT(
    input         clk           ;
    input         rst           ;
    input         buffer_ready  ;
    input  [31:0] in_real [0:15];
    input  [31:0] in_imag [0:15];
    output [31:0] out_real[0:15];
    output [31:0] out_imag[0:15];
    output        done          ;
);

parameter signed [31:0] w1_real = 32'h00010000, w1_imag = 32'h00000000,
                        w2_real = 32'h0000EC83, w2_imag = 32'hFFFF9E09,
                        w3_real = 32'h0000B504, w3_imag = 32'hFFFF4AFC,
                        w4_real = 32'h000061F7, w4_imag = 32'hFFFF137D,
                        w5_real = 32'h00000000, w5_imag = 32'hFFFF0000,
                        w6_real = 32'hFFFF9E09, w6_imag = 32'hFFFF137D,
                        w7_real = 32'hFFFF4AFC, w7_imag = 32'hFFFF4AFC,
                        w8_real = 32'hFFFF137D, w8_imag = 32'hFFFF9E09;

parameter [2:0] IDLE   = 3'd0,
                STAGE1 = 3'd1,
                STAGE2 = 3'd2,
                STAGE3 = 3'd3,
                STAGE4 = 3'd4,
                DONE   = 3'd7;

reg [2:0] curr_state, next_state;
reg [31:0] temp1_real[0:15], temp1_imag[0:15];
reg [31:0] temp2_real[0:15], temp2_imag[0:15];

// Wire based on curr_state
assign done = (curr_state == DONE) ? 1 : 0;

begin
    butterfly u_bf1 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );
    butterfly u_bf2 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf3 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf4 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf5 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf6 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf7 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );butterfly u_bf8 (
        .a(in_real[i  ]),
        .b(in_imag[i  ]),
        .c(in_real[i+8]),
        .d(in_imag[i+8]),
        .W_real(w1_real),
        .W_imag(w1_imag),
        .FFT_A_real(temp_real[i  ]),
        .FFT_A_imag(temp_real[i  ]),
        .FFT_B_real(temp_imag[i+8]),
        .FFT_B_imag(temp_imag[i+8])
    );
end



integer i;
always@(posedge clk or posedge rst)begin
	if(rst)
		curr_state <= IDLE;
        for (int i = 0; i < 16; i = i + 1) begin
            temp1_real <= 16'd0;
            temp1_imag <= 16'd0;
            temp2_real <= 16'd0;
            temp2_imag <= 16'd0;
        end
	else
		case (curr_state)
            IDLE   : 
            STAGE1 : begin
                    
                end
            end 
            STAGE2 : begin
                for (int i = 0; i < 4; i = i + 1) begin
                    butterfly u_bf (
                        .a(in_real[]),
                        .b(in_imag[]),
                        .c(in_real[]),
                        .d(in_imag[]),
                        .W_real(),
                        .W_imag(),
                        .FFT_A_real(),
                        .FFT_A_imag(),
                        .FFT_B_real(),
                        .FFT_B_imag()
                    );
                end
                for (int i = 0; i < 4; i = i + 1) begin
                        butterfly u_bf (
                        .a(in_real[]),
                        .b(in_imag[]),
                        .c(in_real[]),
                        .d(in_imag[]),
                        .W_real(),
                        .W_imag(),
                        .FFT_A_real(),
                        .FFT_A_imag(),
                        .FFT_B_real(),
                        .FFT_B_imag()
                    );
                end
            end 
        endcase

end









// FSM Next stage logic
always @(*) begin
	case(curr_state)
		IDLE   : 
			next_state = (buffer_ready) ? STAGE1 : IDLE;
        STAGE1 : 
			next_state = STAGE2;
		STAGE2 : 
			next_state = STAGE3;
        STAGE3 : 
			next_state = STAGE4;
		STAGE4 : 
            next_state = DONE  ;
		DONE : 
			next_state = DONE  ;
		default : 
			next_state = IDLE  ;
	endcase
end

always@(posedge clk or posedge rst)begin
	if(rst)
		curr_state <= IDLE;
	else
		curr_state <= next_state;
end

endmodule