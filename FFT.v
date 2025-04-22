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

wire [15:0] p_out         [0:15];   //parallel output of S2P
wire        buffer_ready;           //parallel ready signal
wire        fft_out_ready;
wire [15:0] fft_out_real [0:15];    //fft output
wire [15:0] fft_out_imag [0:15];    //fft output
reg         fft_out_flag;           // 0 = real, 1 = imag
reg  [15:0] fft_d        [0:15]; 
reg         fft_valid_reg;          // 0 = real, 1 = imag
reg         fft_out_ready_d1;
reg         fft_out_ready_d2;

// s2p's buffer output are stored in reg
// , it's safe to connect directly into fft_core by wires
S2P S2P1(.clk(clk)                  ,
         .rst(rst)                  ,
         .fir_d(fir_d)              ,
         .fir_valid(fir_valid)      ,
         .buffer_ready(buffer_ready),
         .buffer0 (p_out[0 ]),
         .buffer1 (p_out[1 ]),
         .buffer2 (p_out[2 ]),
         .buffer3 (p_out[3 ]),
         .buffer4 (p_out[4 ]),
         .buffer5 (p_out[5 ]),
         .buffer6 (p_out[6 ]),
         .buffer7 (p_out[7 ]),
         .buffer8 (p_out[8 ]),
         .buffer9 (p_out[9 ]),
         .buffer10(p_out[10]),
         .buffer11(p_out[11]),
         .buffer12(p_out[12]),
         .buffer13(p_out[13]),
         .buffer14(p_out[14]),
         .buffer15(p_out[15])
        );

FFT_CORE FFT_CORE1(.clk(clk),   
                   .rst(rst),       
                   .buffer_ready(buffer_ready),
                   .in_real0 (p_out[0 ]),
                   .in_real1 (p_out[1 ]),
                   .in_real2 (p_out[2 ]),
                   .in_real3 (p_out[3 ]),
                   .in_real4 (p_out[4 ]),
                   .in_real5 (p_out[5 ]),
                   .in_real6 (p_out[6 ]),
                   .in_real7 (p_out[7 ]),
                   .in_real8 (p_out[8 ]),
                   .in_real9 (p_out[9 ]),
                   .in_real10(p_out[10]),
                   .in_real11(p_out[11]),
                   .in_real12(p_out[12]),
                   .in_real13(p_out[13]),
                   .in_real14(p_out[14]),
                   .in_real15(p_out[15]), 

                   .out_real0 (fft_out_real[0 ]),
                   .out_real1 (fft_out_real[1 ]),
                   .out_real2 (fft_out_real[2 ]),
                   .out_real3 (fft_out_real[3 ]),
                   .out_real4 (fft_out_real[4 ]),
                   .out_real5 (fft_out_real[5 ]),
                   .out_real6 (fft_out_real[6 ]),
                   .out_real7 (fft_out_real[7 ]),
                   .out_real8 (fft_out_real[8 ]),
                   .out_real9 (fft_out_real[9 ]),
                   .out_real10(fft_out_real[10]),
                   .out_real11(fft_out_real[11]),
                   .out_real12(fft_out_real[12]),
                   .out_real13(fft_out_real[13]),
                   .out_real14(fft_out_real[14]),
                   .out_real15(fft_out_real[15]),

                   .out_imag0 (fft_out_imag[0 ]),
                   .out_imag1 (fft_out_imag[1 ]),
                   .out_imag2 (fft_out_imag[2 ]),
                   .out_imag3 (fft_out_imag[3 ]),
                   .out_imag4 (fft_out_imag[4 ]),
                   .out_imag5 (fft_out_imag[5 ]),
                   .out_imag6 (fft_out_imag[6 ]),
                   .out_imag7 (fft_out_imag[7 ]),
                   .out_imag8 (fft_out_imag[8 ]),
                   .out_imag9 (fft_out_imag[9 ]),
                   .out_imag10(fft_out_imag[10]),
                   .out_imag11(fft_out_imag[11]),
                   .out_imag12(fft_out_imag[12]),
                   .out_imag13(fft_out_imag[13]),
                   .out_imag14(fft_out_imag[14]),
                   .out_imag15(fft_out_imag[15]),
                   .done(fft_out_ready)                   
                  );

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fft_out_ready_d1 <= 0;
        fft_out_ready_d2 <= 0;
    end else begin
        fft_out_ready_d1 <= fft_out_ready;
        fft_out_ready_d2 <= fft_out_ready_d1;
    end
end

// === Output Logic ===
always @(posedge clk or posedge rst) begin
    if (rst) begin
        fft_out_flag <= 0;
        fft_valid_reg <= 0;
    end else if (fft_out_ready | fft_out_ready_d1) begin
        fft_valid_reg <= 1;
        if (!fft_out_flag) begin
            // output real
            fft_d[0]  <= fft_out_real[0];
            fft_d[1]  <= fft_out_real[1];
            fft_d[2]  <= fft_out_real[2];
            fft_d[3]  <= fft_out_real[3];
            fft_d[4]  <= fft_out_real[4];
            fft_d[5]  <= fft_out_real[5];
            fft_d[6]  <= fft_out_real[6];
            fft_d[7]  <= fft_out_real[7];
            fft_d[8]  <= fft_out_real[8];
            fft_d[9]  <= fft_out_real[9];
            fft_d[10] <= fft_out_real[10];
            fft_d[11] <= fft_out_real[11];
            fft_d[12] <= fft_out_real[12];
            fft_d[13] <= fft_out_real[13];
            fft_d[14] <= fft_out_real[14];
            fft_d[15] <= fft_out_real[15];
        end else begin
            // output imag
            fft_d[0]  <= fft_out_imag[0];
            fft_d[1]  <= fft_out_imag[1];
            fft_d[2]  <= fft_out_imag[2];
            fft_d[3]  <= fft_out_imag[3];
            fft_d[4]  <= fft_out_imag[4];
            fft_d[5]  <= fft_out_imag[5];
            fft_d[6]  <= fft_out_imag[6];
            fft_d[7]  <= fft_out_imag[7];
            fft_d[8]  <= fft_out_imag[8];
            fft_d[9]  <= fft_out_imag[9];
            fft_d[10] <= fft_out_imag[10];
            fft_d[11] <= fft_out_imag[11];
            fft_d[12] <= fft_out_imag[12];
            fft_d[13] <= fft_out_imag[13];
            fft_d[14] <= fft_out_imag[14];
            fft_d[15] <= fft_out_imag[15];
        end
        fft_out_flag <= ~fft_out_flag;  // flip to output imag next
    end else begin
        fft_valid_reg <= 0;
    end
end

assign fft_valid = fft_out_ready_d2 | fft_out_ready_d1;
assign fft_d0  = fft_d[0];
assign fft_d1  = fft_d[1];
assign fft_d2  = fft_d[2];
assign fft_d3  = fft_d[3];
assign fft_d4  = fft_d[4];
assign fft_d5  = fft_d[5];
assign fft_d6  = fft_d[6];
assign fft_d7  = fft_d[7];
assign fft_d8  = fft_d[8];
assign fft_d9  = fft_d[9];
assign fft_d10 = fft_d[10];
assign fft_d11 = fft_d[11];
assign fft_d12 = fft_d[12];
assign fft_d13 = fft_d[13];
assign fft_d14 = fft_d[14];
assign fft_d15 = fft_d[15];


//  done logic
//  scanning negedge of fir_valid and fft_valid
reg fir_shadow;
always @(posedge clk) begin
    fir_shadow <= fir_valid;
end
wire in_end = fir_shadow & ~fir_valid;   

reg fft_shadow;
always @(posedge clk) begin
     fft_shadow <= fft_valid;
end
wire out_end =  fft_shadow & ~fft_valid;

reg wait_drain;   // fir are all sent , waiting for fft to finish
reg done_r;       // fft sent the last data out

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wait_drain <= 1'b0;
        done_r     <= 1'b0;
    end
    else begin
        if (in_end) begin          
            wait_drain <= 1'b1;
        end
        
        if (wait_drain && out_end) begin  // fir all sent + fft all sent
            done_r     <= 1'b1;   
            wait_drain <= 1'b0;   
        end
        else
            done_r <= 1'b0;
    end
end

assign done = done_r;

endmodule

module S2P(
    input             clk          , 
    input             rst          , 
    input      [15:0] fir_d        , 
    input             fir_valid    ,
    output reg        buffer_ready ,
    output     [15:0] buffer0      ,
    output     [15:0] buffer1      ,
    output     [15:0] buffer2      ,
    output     [15:0] buffer3      ,
    output     [15:0] buffer4      ,
    output     [15:0] buffer5      ,
    output     [15:0] buffer6      ,
    output     [15:0] buffer7      ,
    output     [15:0] buffer8      ,
    output     [15:0] buffer9      ,
    output     [15:0] buffer10     ,
    output     [15:0] buffer11     ,
    output     [15:0] buffer12     ,
    output     [15:0] buffer13     ,
    output     [15:0] buffer14     ,
    output     [15:0] buffer15
);

integer   i         ;
reg [3:0] ctr       ;
reg [31:0] buffer[0:15];

always @(posedge clk or posedge rst) begin
	if (rst) begin
        ctr          <= 4'd0;
        buffer_ready <= 0   ;
        for (i = 0; i < 16; i = i + 1) buffer[i] <= 32'd0;

    end	else if (fir_valid) begin
		buffer_ready <= (ctr == 4'd15)                 ;
        ctr          <= (ctr == 4'd15) ? 4'd0 : ctr + 1; 
        buffer[ctr]  <= fir_d;
        // buffer[ctr]  <= {
        //                  {8{fir_d[15]}}, // sign extension of fir_d
        //                  fir_d,          // middle 16 bits
        //                  8'd0            // zero padding
        // };
    end else begin
        buffer_ready <= 0;
    end
end
assign buffer0  = buffer[0 ];
assign buffer1  = buffer[1 ];
assign buffer2  = buffer[2 ];
assign buffer3  = buffer[3 ];
assign buffer4  = buffer[4 ];
assign buffer5  = buffer[5 ];
assign buffer6  = buffer[6 ];
assign buffer7  = buffer[7 ];
assign buffer8  = buffer[8 ];
assign buffer9  = buffer[9 ];
assign buffer10 = buffer[10];
assign buffer11 = buffer[11];
assign buffer12 = buffer[12];
assign buffer13 = buffer[13];
assign buffer14 = buffer[14];
assign buffer15 = buffer[15];
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
// -------------------。--。--(W^n)--> fft_b = (a+c) - (b+d)j * (W^n_real + j*W^n_imag)
//                                           = (a-c) * W^n_real + (d - b) * W^n_imag 
//                                       + j * (a-c) * W^n_imag + (b - d) * W^n_real
// Y = c + dj
wire signed [31:0] ac_diff = a - c;
wire signed [31:0] bd_diff = b - d; 

wire signed [63:0] fft_mul1 =   ac_diff * W_real;
wire signed [63:0] fft_mul2 =  -bd_diff * W_imag;    // (d-b)W_imag
wire signed [63:0] fft_mul3 =   ac_diff * W_imag;
wire signed [63:0] fft_mul4 =   bd_diff * W_real;    // (b-d)W_read

assign FFT_A_real = a               + c              ;
assign FFT_A_imag = b               + d              ;
assign FFT_B_real = fft_mul1[47:16] + fft_mul2[47:16];
assign FFT_B_imag = fft_mul3[47:16] + fft_mul4[47:16];  
endmodule

module FFT_CORE(
    input         clk           ,
    input         rst           ,
    input         buffer_ready  ,
    // input  [31:0] in_real [0:15],
    input  [15:0] in_real0 ,
    input  [15:0] in_real1 ,
    input  [15:0] in_real2 ,
    input  [15:0] in_real3 ,
    input  [15:0] in_real4 ,
    input  [15:0] in_real5 ,
    input  [15:0] in_real6 ,
    input  [15:0] in_real7 ,
    input  [15:0] in_real8 ,
    input  [15:0] in_real9 ,
    input  [15:0] in_real10,
    input  [15:0] in_real11,
    input  [15:0] in_real12,
    input  [15:0] in_real13,
    input  [15:0] in_real14,
    input  [15:0] in_real15,
    // output [15:0] out_real[0:15],
    output [15:0] out_real0,
    output [15:0] out_real1,
    output [15:0] out_real2,
    output [15:0] out_real3,
    output [15:0] out_real4,
    output [15:0] out_real5,
    output [15:0] out_real6,
    output [15:0] out_real7,
    output [15:0] out_real8,
    output [15:0] out_real9,
    output [15:0] out_real10,
    output [15:0] out_real11,
    output [15:0] out_real12,
    output [15:0] out_real13,
    output [15:0] out_real14,
    output [15:0] out_real15,
    // output [15:0] out_imag[0:15],
    output [15:0] out_imag0,
    output [15:0] out_imag1,
    output [15:0] out_imag2,
    output [15:0] out_imag3,
    output [15:0] out_imag4,
    output [15:0] out_imag5,
    output [15:0] out_imag6,
    output [15:0] out_imag7,
    output [15:0] out_imag8,
    output [15:0] out_imag9,
    output [15:0] out_imag10,
    output [15:0] out_imag11,
    output [15:0] out_imag12,
    output [15:0] out_imag13,
    output [15:0] out_imag14,
    output [15:0] out_imag15,
    output        done          
);



// === Twiddle Factor ===
reg signed [31:0] w_real [0:7];
initial begin
    w_real[0] = 32'sh00010000;
    w_real[1] = 32'sh0000EC83;
    w_real[2] = 32'sh0000B504;
    w_real[3] = 32'sh000061F7;
    w_real[4] = 32'sh00000000;
    w_real[5] = 32'shFFFF9E09;
    w_real[6] = 32'shFFFF4AFC;
    w_real[7] = 32'shFFFF137D;
end

reg signed [31:0] w_imag [0:7];
initial begin
    w_imag[0] = 32'sh00000000;
    w_imag[1] = 32'shFFFF9E09;
    w_imag[2] = 32'shFFFF4AFC;
    w_imag[3] = 32'shFFFF137D;
    w_imag[4] = 32'shFFFF0000;
    w_imag[5] = 32'shFFFF137D;
    w_imag[6] = 32'shFFFF4AFC;
    w_imag[7] = 32'shFFFF9E09;
end
                       
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
reg [3:0] odr_a_idx[0:7], odr_b_idx[0:7];
reg [2:0] twiddle_idx[0:7];
reg ping_pong_switcher;  //using lyr_a => 1

// fft layer order assignment
always@(*)begin
	case (curr_state)
        IDLE, DONE : begin
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
            
            twiddle_idx[0] = 3'd0; 
            twiddle_idx[1] = 3'd0; 
            twiddle_idx[2] = 3'd0; 
            twiddle_idx[3] = 3'd0; 
            twiddle_idx[4] = 3'd0; 
            twiddle_idx[5] = 3'd0; 
            twiddle_idx[6] = 3'd0; 
            twiddle_idx[7] = 3'd0; 
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
            
            twiddle_idx[0] = 3'd0; 
            twiddle_idx[1] = 3'd1; 
            twiddle_idx[2] = 3'd2; 
            twiddle_idx[3] = 3'd3; 
            twiddle_idx[4] = 3'd4; 
            twiddle_idx[5] = 3'd5; 
            twiddle_idx[6] = 3'd6; 
            twiddle_idx[7] = 3'd7; 
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
            
            twiddle_idx[0] = 3'd0; 
            twiddle_idx[1] = 3'd2; 
            twiddle_idx[2] = 3'd4; 
            twiddle_idx[3] = 3'd6; 
            twiddle_idx[4] = 3'd0; 
            twiddle_idx[5] = 3'd2; 
            twiddle_idx[6] = 3'd4; 
            twiddle_idx[7] = 3'd6; 
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
            
            twiddle_idx[0] = 3'd0; 
            twiddle_idx[1] = 3'd4; 
            twiddle_idx[2] = 3'd0; 
            twiddle_idx[3] = 3'd4; 
            twiddle_idx[4] = 3'd0; 
            twiddle_idx[5] = 3'd4; 
            twiddle_idx[6] = 3'd0; 
            twiddle_idx[7] = 3'd4; 
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
            
            twiddle_idx[0] = 4'd0; 
            twiddle_idx[1] = 4'd0; 
            twiddle_idx[2] = 4'd0; 
            twiddle_idx[3] = 4'd0; 
            twiddle_idx[4] = 4'd0; 
            twiddle_idx[5] = 4'd0; 
            twiddle_idx[6] = 4'd0; 
            twiddle_idx[7] = 4'd0; 
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
        butterfly u_bf (
            .a(lyr_in_a_real[i]),
            .b(lyr_in_a_imag[i]),
            .c(lyr_in_b_real[i]),
            .d(lyr_in_b_imag[i]),
            
            .W_real(w_real[twiddle_idx[i]]),
            .W_imag(w_imag[twiddle_idx[i]]),
            
            .FFT_A_real(lyr_out_a_real[i]),
            .FFT_A_imag(lyr_out_a_imag[i]),
            .FFT_B_real(lyr_out_b_real[i]),
            .FFT_B_imag(lyr_out_b_imag[i])
        );
    end
endgenerate


//  wire ording logic
genvar k;
generate
    //  for 8 butterfly units
    for (k = 0; k < 8; k = k + 1) begin : input_mux
        assign lyr_in_a_real[k] = ping_pong_switcher ? buf1_real[odr_a_idx[k]] : buf2_real[odr_a_idx[k]];
        assign lyr_in_a_imag[k] = ping_pong_switcher ? buf1_imag[odr_a_idx[k]] : buf2_imag[odr_a_idx[k]];
        assign lyr_in_b_real[k] = ping_pong_switcher ? buf1_real[odr_b_idx[k]] : buf2_real[odr_b_idx[k]];
        assign lyr_in_b_imag[k] = ping_pong_switcher ? buf1_imag[odr_b_idx[k]] : buf2_imag[odr_b_idx[k]];
    end
endgenerate


integer o;
integer l;
integer m;
// === layer wire Writeback into Buffer ===
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ping_pong_switcher <= 1;
    end else begin
        case (curr_state)
            IDLE : begin
                ping_pong_switcher <= 1;
                buf1_real [0 ] <= {{8{in_real0 [15]}}, in_real0 , 8'd0};
                buf1_real [1 ] <= {{8{in_real1 [15]}}, in_real1 , 8'd0};
                buf1_real [2 ] <= {{8{in_real2 [15]}}, in_real2 , 8'd0};
                buf1_real [3 ] <= {{8{in_real3 [15]}}, in_real3 , 8'd0};
                buf1_real [4 ] <= {{8{in_real4 [15]}}, in_real4 , 8'd0};
                buf1_real [5 ] <= {{8{in_real5 [15]}}, in_real5 , 8'd0};
                buf1_real [6 ] <= {{8{in_real6 [15]}}, in_real6 , 8'd0};
                buf1_real [7 ] <= {{8{in_real7 [15]}}, in_real7 , 8'd0};
                buf1_real [8 ] <= {{8{in_real8 [15]}}, in_real8 , 8'd0};
                buf1_real [9 ] <= {{8{in_real9 [15]}}, in_real9 , 8'd0};
                buf1_real [10] <= {{8{in_real10[15]}}, in_real10, 8'd0};
                buf1_real [11] <= {{8{in_real11[15]}}, in_real11, 8'd0};
                buf1_real [12] <= {{8{in_real12[15]}}, in_real12, 8'd0};
                buf1_real [13] <= {{8{in_real13[15]}}, in_real13, 8'd0};
                buf1_real [14] <= {{8{in_real14[15]}}, in_real14, 8'd0};
                buf1_real [15] <= {{8{in_real15[15]}}, in_real15, 8'd0};
                buf1_imag [0 ] <= 32'd0;
                buf1_imag [1 ] <= 32'd0;
                buf1_imag [2 ] <= 32'd0;
                buf1_imag [3 ] <= 32'd0;
                buf1_imag [4 ] <= 32'd0;
                buf1_imag [5 ] <= 32'd0;
                buf1_imag [6 ] <= 32'd0;
                buf1_imag [7 ] <= 32'd0;
                buf1_imag [8 ] <= 32'd0;
                buf1_imag [9 ] <= 32'd0;
                buf1_imag [10] <= 32'd0;
                buf1_imag [11] <= 32'd0;
                buf1_imag [12] <= 32'd0;
                buf1_imag [13] <= 32'd0;
                buf1_imag [14] <= 32'd0;
                buf1_imag [15] <= 32'd0;
                for (o = 0; o < 16; o = o + 1) begin
                    buf2_real[o] <= 32'd0;
                    buf2_imag[o] <= 32'd0;
                end
            end 
            //  checking corrctness
            STAGE1, STAGE3 : begin
                for (l = 0; l < 8; l = l + 1) begin
                    ping_pong_switcher <= ~ping_pong_switcher;
                    buf2_real[odr_a_idx[l]] <= lyr_out_a_real[l];
                    buf2_imag[odr_a_idx[l]] <= lyr_out_a_imag[l];
                    buf2_real[odr_b_idx[l]] <= lyr_out_b_real[l];
                    buf2_imag[odr_b_idx[l]] <= lyr_out_b_imag[l];
                end
            end 
            STAGE2, STAGE4 : begin
                    for (m = 0; m < 8; m = m + 1) begin
                    ping_pong_switcher <= ~ping_pong_switcher;
                    buf1_real[odr_a_idx[m]] <= lyr_out_a_real[m];
                    buf1_imag[odr_a_idx[m]] <= lyr_out_a_imag[m];
                    buf1_real[odr_b_idx[m]] <= lyr_out_b_real[m];
                    buf1_imag[odr_b_idx[m]] <= lyr_out_b_imag[m];
                end
            end  
            DONE : begin
                ping_pong_switcher <= 1;
            end          
        endcase
    end
end

assign out_real0  = buf1_real[0 ][23:8];
assign out_real8  = buf1_real[1 ][23:8];
assign out_real4  = buf1_real[2 ][23:8];
assign out_real12 = buf1_real[3 ][23:8];
assign out_real2  = buf1_real[4 ][23:8];
assign out_real10 = buf1_real[5 ][23:8];
assign out_real6  = buf1_real[6 ][23:8];
assign out_real14 = buf1_real[7 ][23:8];
assign out_real1  = buf1_real[8 ][23:8];
assign out_real9  = buf1_real[9 ][23:8];
assign out_real5  = buf1_real[10][23:8];
assign out_real13 = buf1_real[11][23:8];
assign out_real3  = buf1_real[12][23:8];
assign out_real11 = buf1_real[13][23:8];
assign out_real7  = buf1_real[14][23:8];
assign out_real15 = buf1_real[15][23:8];
assign out_imag0  = buf1_imag[0 ][23:8];
assign out_imag8  = buf1_imag[1 ][23:8];
assign out_imag4  = buf1_imag[2 ][23:8];
assign out_imag12 = buf1_imag[3 ][23:8];
assign out_imag2  = buf1_imag[4 ][23:8];
assign out_imag10 = buf1_imag[5 ][23:8];
assign out_imag6  = buf1_imag[6 ][23:8];
assign out_imag14 = buf1_imag[7 ][23:8];
assign out_imag1  = buf1_imag[8 ][23:8];
assign out_imag9  = buf1_imag[9 ][23:8];
assign out_imag5  = buf1_imag[10][23:8];
assign out_imag13 = buf1_imag[11][23:8];
assign out_imag3  = buf1_imag[12][23:8];
assign out_imag11 = buf1_imag[13][23:8];
assign out_imag7  = buf1_imag[14][23:8];
assign out_imag15 = buf1_imag[15][23:8];

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
			next_state = IDLE  ;
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