// FSM comb : wire logic 
//            next_state logic
//     seq  : really basic state transition

// rst are used only in FSM seq state transition
//                    + other computational seq circuits


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

module FFT_CORE(
    input         clk           ;
    input         rst           ;
    input         buffer_ready  ;
    input  [31:0] in_real [0:15];
    input  [31:0] in_imag [0:15];
    output [31:0] out_real[0:15];
    output [31:0] out_imag[0:15];
    output        done          ;
);

parameter signed [31:0] w_real [0:7] = { 32'h00010000,
                                          32'h0000EC83, 
                                          32'h0000B504, 
                                          32'h000061F7, 
                                          32'h00000000, 
                                          32'hFFFF9E09, 
                                          32'hFFFF4AFC, 
                                          32'hFFFF137D
                                        };

parameter signed [31:0] w_imag [0:7] = {
                                         32'h00000000,
                                         32'hFFFF9E09,
                                         32'hFFFF4AFC,
                                         32'hFFFF137D,
                                         32'hFFFF0000,
                                         32'hFFFF137D,
                                         32'hFFFF4AFC,
                                         32'hFFFF9E09
                                        };                    
       
parameter [2:0] IDLE   = 3'd0,
                STAGE1 = 3'd1,
                STAGE2 = 3'd2,
                STAGE3 = 3'd3,
                STAGE4 = 3'd4,
                DONE   = 3'd7;

reg [2:0] curr_state, next_state;
reg [31:0] buf1_real[0:15], buf1_imag[0:15];
reg [31:0] buf2_real[0:15], buf2_imag[0:15];

// Wire based on curr_state
assign done = (curr_state == DONE) ? 1 : 0;

// === Index Pair & Twiddle Index Lookup ===
// fft layer order index
//twiddle_idx 15 => 1 (no coefficient)
reg [3:0] odr_a_idx[0:7], odr_b_idx[0:7];
reg [3:0] twiddle_idx[0:7];
reg ping_pong_switcher;  //using lyr_a => 1

// fft layer order assignment
always@(*)begin
	case (curr_state)
        IDLE   : begin
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd0 ;
            odr_a_idx[2] = 4'd0 ;
            odr_a_idx[3] = 4'd0 ;
            odr_a_idx[4] = 4'd0 ;
            odr_a_idx[5] = 4'd0 ;
            odr_a_idx[6] = 4'd0 ;
            odr_a_idx[7] = 4'd0 ;
            
            odr_b_idx[0] = 4'd0 ;
            odr_b_idx[1] = 4'd0 ;
            odr_b_idx[2] = 4'd0 ;
            odr_b_idx[3] = 4'd0 ;
            odr_b_idx[4] = 4'd0 ;
            odr_b_idx[5] = 4'd0 ;
            odr_b_idx[6] = 4'd0 ;
            odr_b_idx[7] = 4'd0 ;
            
            twiddle_idx[0] = 4'd15; 
            twiddle_idx[1] = 4'd15; 
            twiddle_idx[2] = 4'd15; 
            twiddle_idx[3] = 4'd15; 
            twiddle_idx[4] = 4'd15; 
            twiddle_idx[5] = 4'd15; 
            twiddle_idx[6] = 4'd15; 
            twiddle_idx[7] = 4'd15; 
        end
        STAGE1 : begin
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd1 ;
            odr_a_idx[2] = 4'd2 ;
            odr_a_idx[3] = 4'd3 ;
            odr_a_idx[4] = 4'd4 ;
            odr_a_idx[5] = 4'd5 ;
            odr_a_idx[6] = 4'd6 ;
            odr_a_idx[7] = 4'd7 ;
            
            odr_b_idx[0] = 4'd8 ;
            odr_b_idx[1] = 4'd9 ;
            odr_b_idx[2] = 4'd10;
            odr_b_idx[3] = 4'd11;
            odr_b_idx[4] = 4'd12;
            odr_b_idx[5] = 4'd13;
            odr_b_idx[6] = 4'd14;
            odr_b_idx[7] = 4'd15;
            
            twiddle_idx[0] = 4'd0; 
            twiddle_idx[1] = 4'd1; 
            twiddle_idx[2] = 4'd2; 
            twiddle_idx[3] = 4'd3; 
            twiddle_idx[4] = 4'd4; 
            twiddle_idx[5] = 4'd5; 
            twiddle_idx[6] = 4'd6; 
            twiddle_idx[7] = 4'd7; 
        end 
        STAGE2 : begin
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd1 ;
            odr_a_idx[2] = 4'd2 ;
            odr_a_idx[3] = 4'd3 ;
            odr_a_idx[4] = 4'd8 ;
            odr_a_idx[5] = 4'd9 ;
            odr_a_idx[6] = 4'd10;
            odr_a_idx[7] = 4'd11;
            
            odr_b_idx[0] = 4'd4 ;
            odr_b_idx[1] = 4'd5 ;
            odr_b_idx[2] = 4'd6 ;
            odr_b_idx[3] = 4'd7 ;
            odr_b_idx[4] = 4'd12;
            odr_b_idx[5] = 4'd13;
            odr_b_idx[6] = 4'd14;
            odr_b_idx[7] = 4'd15;
            
            twiddle_idx[0] = 4'd0; 
            twiddle_idx[1] = 4'd2; 
            twiddle_idx[2] = 4'd4; 
            twiddle_idx[3] = 4'd6; 
            twiddle_idx[4] = 4'd0; 
            twiddle_idx[5] = 4'd2; 
            twiddle_idx[6] = 4'd4; 
            twiddle_idx[7] = 4'd6; 
        end 
        STAGE3 : begin
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd1 ;
            odr_a_idx[2] = 4'd4 ;
            odr_a_idx[3] = 4'd5 ;
            odr_a_idx[4] = 4'd8 ;
            odr_a_idx[5] = 4'd9 ;
            odr_a_idx[6] = 4'd12;
            odr_a_idx[7] = 4'd13;
            
            odr_b_idx[0] = 4'd2 ;
            odr_b_idx[1] = 4'd3 ;
            odr_b_idx[2] = 4'd6 ;
            odr_b_idx[3] = 4'd7 ;
            odr_b_idx[4] = 4'd10;
            odr_b_idx[5] = 4'd11;
            odr_b_idx[6] = 4'd14;
            odr_b_idx[7] = 4'd15;
            
            twiddle_idx[0] = 4'd0; 
            twiddle_idx[1] = 4'd4; 
            twiddle_idx[2] = 4'd0; 
            twiddle_idx[3] = 4'd4; 
            twiddle_idx[4] = 4'd0; 
            twiddle_idx[5] = 4'd4; 
            twiddle_idx[6] = 4'd0; 
            twiddle_idx[7] = 4'd4; 
        end 
        STAGE4 : begin
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd2 ;
            odr_a_idx[2] = 4'd4 ;
            odr_a_idx[3] = 4'd6 ;
            odr_a_idx[4] = 4'd8 ;
            odr_a_idx[5] = 4'd10;
            odr_a_idx[6] = 4'd12;
            odr_a_idx[7] = 4'd14;
            
            odr_b_idx[0] = 4'd1 ;
            odr_b_idx[1] = 4'd3 ;
            odr_b_idx[2] = 4'd5 ;
            odr_b_idx[3] = 4'd7 ;
            odr_b_idx[4] = 4'd9 ;
            odr_b_idx[5] = 4'd11;
            odr_b_idx[6] = 4'd13;
            odr_b_idx[7] = 4'd15;
            
            twiddle_idx[0] = 4'd15; 
            twiddle_idx[1] = 4'd15; 
            twiddle_idx[2] = 4'd15; 
            twiddle_idx[3] = 4'd15; 
            twiddle_idx[4] = 4'd15; 
            twiddle_idx[5] = 4'd15; 
            twiddle_idx[6] = 4'd15; 
            twiddle_idx[7] = 4'd15; 
        end 
        DONE : begin   
            odr_a_idx[0] = 4'd0 ;
            odr_a_idx[1] = 4'd0 ;
            odr_a_idx[2] = 4'd0 ;
            odr_a_idx[3] = 4'd0 ;
            odr_a_idx[4] = 4'd0 ;
            odr_a_idx[5] = 4'd0 ;
            odr_a_idx[6] = 4'd0 ;
            odr_a_idx[7] = 4'd0 ;
            
            odr_b_idx[0] = 4'd0 ;
            odr_b_idx[1] = 4'd0 ;
            odr_b_idx[2] = 4'd0 ;
            odr_b_idx[3] = 4'd0 ;
            odr_b_idx[4] = 4'd0 ;
            odr_b_idx[5] = 4'd0 ;
            odr_b_idx[6] = 4'd0 ;
            odr_b_idx[7] = 4'd0 ;
            
            twiddle_idx[0] = 4'd15; 
            twiddle_idx[1] = 4'd15; 
            twiddle_idx[2] = 4'd15; 
            twiddle_idx[3] = 4'd15; 
            twiddle_idx[4] = 4'd15; 
            twiddle_idx[5] = 4'd15; 
            twiddle_idx[6] = 4'd15; 
            twiddle_idx[7] = 4'd15; 
        end 
    endcase
end

// === Butterfly Wiring Out for Wiring Outside ===
wire signed [31:0] lyr_in_a_real [0:7],
                   lyr_in_a_imag [0:7];
wire signed [31:0] lyr_in_b_real [0:7],
                   lyr_in_b_imag [0:7];

wire signed [31:0] lyr_out_a_real[0:7],
                   lyr_out_a_imag[0:7];
wire signed [31:0] lyr_out_b_real[0:7],
                   lyr_out_b_imag[0:7];

genvar i;
generate
    for (i = 0; i < 8; i = i + 1) begin : bf_layer
        // four inputs abcd
        assign lyr_in_a_real[i] = ping_pong_switcher ? buf1_real[i] : buf2_real[i];
        assign lyr_in_a_imag[i] = ping_pong_switcher ? buf1_imag[i] : buf2_imag[i];
        assign lyr_in_b_real[i] = ping_pong_switcher ? buf1_real[i] : buf2_real[i];
        assign lyr_in_b_imag[i] = ping_pong_switcher ? buf1_imag[i] : buf2_imag[i];

        butterfly u_bf (
            .a(lyr_in_a_real[i]),
            .b(lyr_in_a_imag[i]),
            .c(lyr_in_b_real[i]),
            .d(lyr_in_b_imag[i]),
            
            .W_real((twiddle_idx[i] == 4'd15) ? 4'd1 : w_real[twiddle_idx[i]]),
            .W_imag((twiddle_idx[i] == 4'd15) ? 4'd1 : w_imag[twiddle_idx[i]]),
            
            .FFT_A_real(lyr_out_a_real[i]),
            .FFT_A_imag(lyr_out_a_imag[i]),
            .FFT_B_real(lyr_out_b_real[i]),
            .FFT_B_imag(lyr_out_b_imag[i])
        );
    end
endgenerate

//wire ording logic....
always @(*) begin
// .....solving......


end 

// === layer wire Writeback into Buffer ===
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ping_pong_switcher <= 1;
    end else begin
        case (curr_state)
            STAGE1, STAGE3 : begin
                for (int i = 0; i < 8; i = i + 1) begin
                    buf2_real[i] <= lyr_out_a_real[i];
                    buf2_imag[i] <= lyr_out_a_imag[i];
                    buf2_real[i] <= lyr_out_b_real[i];
                    buf2_imag[i] <= lyr_out_b_imag[i];
                    ping_pong_switcher <= ~ping_pong_switcher;
                end
            end 
            STAGE2, STAGE4 : begin
                for (int i = 0; i < 8; i = i + 1) begin
                    buf1_real[i] <= lyr_out_a_real[i];
                    buf1_imag[i] <= lyr_out_a_imag[i];
                    buf1_real[i] <= lyr_out_b_real[i];
                    buf1_imag[i] <= lyr_out_b_imag[i];
                    ping_pong_switcher <= ~ping_pong_switcher;
                end
            end  
            IDLE : begin
                ping_pong_switcher <= 1;
            end 
            DONE : begin
                ping_pong_switcher <= 0;
            end          
        endcase
    end
end

begin
    assign out_real[0 ] = ping_pong_switcher ? buf_real_b[0 ] : buf_real_a[0 ];
    assign out_imag[0 ] = ping_pong_switcher ? buf_imag_b[0 ] : buf_imag_a[0 ];
    assign out_real[1 ] = ping_pong_switcher ? buf_real_b[8 ] : buf_real_a[8 ];
    assign out_imag[1 ] = ping_pong_switcher ? buf_imag_b[8 ] : buf_imag_a[8 ];
    assign out_real[2 ] = ping_pong_switcher ? buf_real_b[4 ] : buf_real_a[4 ];
    assign out_imag[2 ] = ping_pong_switcher ? buf_imag_b[4 ] : buf_imag_a[4 ];
    assign out_real[3 ] = ping_pong_switcher ? buf_real_b[12] : buf_real_a[12];
    assign out_imag[3 ] = ping_pong_switcher ? buf_imag_b[12] : buf_imag_a[12];
    assign out_real[4 ] = ping_pong_switcher ? buf_real_b[2 ] : buf_real_a[2 ];
    assign out_imag[4 ] = ping_pong_switcher ? buf_imag_b[2 ] : buf_imag_a[2 ];
    assign out_real[5 ] = ping_pong_switcher ? buf_real_b[10] : buf_real_a[10];
    assign out_imag[5 ] = ping_pong_switcher ? buf_imag_b[10] : buf_imag_a[10];
    assign out_real[6 ] = ping_pong_switcher ? buf_real_b[6 ] : buf_real_a[6 ];
    assign out_imag[6 ] = ping_pong_switcher ? buf_imag_b[6 ] : buf_imag_a[6 ];
    assign out_real[7 ] = ping_pong_switcher ? buf_real_b[14] : buf_real_a[14];
    assign out_imag[7 ] = ping_pong_switcher ? buf_imag_b[14] : buf_imag_a[14];
    assign out_real[8 ] = ping_pong_switcher ? buf_real_b[1 ] : buf_real_a[1 ];
    assign out_imag[8 ] = ping_pong_switcher ? buf_imag_b[1 ] : buf_imag_a[1 ];
    assign out_real[9 ] = ping_pong_switcher ? buf_real_b[9 ] : buf_real_a[9 ];
    assign out_imag[9 ] = ping_pong_switcher ? buf_imag_b[9 ] : buf_imag_a[9 ];
    assign out_real[10] = ping_pong_switcher ? buf_real_b[5 ] : buf_real_a[5 ];
    assign out_imag[10] = ping_pong_switcher ? buf_imag_b[5 ] : buf_imag_a[5 ];
    assign out_real[11] = ping_pong_switcher ? buf_real_b[13] : buf_real_a[13];
    assign out_imag[11] = ping_pong_switcher ? buf_imag_b[13] : buf_imag_a[13];
    assign out_real[12] = ping_pong_switcher ? buf_real_b[3 ] : buf_real_a[3 ];
    assign out_imag[12] = ping_pong_switcher ? buf_imag_b[3 ] : buf_imag_a[3 ];
    assign out_real[13] = ping_pong_switcher ? buf_real_b[11] : buf_real_a[11];
    assign out_imag[13] = ping_pong_switcher ? buf_imag_b[11] : buf_imag_a[11];
    assign out_real[14] = ping_pong_switcher ? buf_real_b[7 ] : buf_real_a[7 ];
    assign out_imag[14] = ping_pong_switcher ? buf_imag_b[7 ] : buf_imag_a[7 ];
    assign out_real[15] = ping_pong_switcher ? buf_real_b[5 ] : buf_real_a[5 ];
    assign out_imag[15] = ping_pong_switcher ? buf_imag_b[5 ] : buf_imag_a[5 ];
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