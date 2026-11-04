`default_nettype none

// Local centre Jacobian, fixed inverse-map composition R then P then Y.
// All trig inputs/coefficients are signed Q16; this is not full 3D perspective.
// start is accepted only idle; valid P/Y cosine >=0.5 (angles +/-60 deg).
module pitch_affine_coeff #(
    parameter signed [31:0] FY_Q16=32'sd54791085,
    parameter signed [31:0] FX_Q16=32'sd55478575
)(
    input wire clk,rst_n,start,
    input wire signed [31:0] roll_cos,roll_sin,pitch_cos,pitch_sin,
    input wire signed [31:0] yaw_cos,yaw_sin,
    output reg busy,done,error,
    output reg signed [31:0] a00,a01,a02,a10,a11,a12
);
    localparam [3:0] IDLE=0,DIVIDE=1,GAIN_PRODUCT=2,GAIN_SCALE=3,
        B_PRODUCT=4,B_SCALE=5,Y_PRODUCT=6,COMMIT=7,REJECT=8;
    reg [3:0] state;
    reg [5:0] bit_count;
    reg divide_y;
    reg signed [31:0] cr,sr,sp,cy,sy,sec_p,sec_y,tan_p,tan_y,sec2_p,sec2_y;
    reg [31:0] divisor;
    reg [32:0] dividend,remainder,quotient;
    wire [32:0] shifted_remainder={remainder[31:0],dividend[32]};
    wire take_bit=(shifted_remainder>={1'b0,divisor});
    wire [32:0] quotient_next={quotient[31:0],take_bit};
    reg signed [63:0] tan_p_product,tan_y_product,square_p_product,square_y_product;
    reg signed [63:0] p00,p01,p10,p11,p12;
    reg signed [31:0] b00,b01,b10,b11,b12;
    reg signed [63:0] y00,y01,y02,y10,y11,y12;

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;busy<=1'b0;done<=1'b0;error<=1'b0;
            a00<=32'sd65536;a01<=0;a02<=0;
            a10<=0;a11<=32'sd65536;a12<=0;
        end else begin
            done<=1'b0;
            case(state)
                IDLE: if(start) begin
                    busy<=1'b1;
                    if(pitch_cos<32'sd32768 || yaw_cos<32'sd32768) state<=REJECT;
                    else begin
                        cr<=roll_cos;sr<=roll_sin;sp<=pitch_sin;cy<=yaw_cos;sy<=yaw_sin;
                        divisor<=pitch_cos;dividend<=33'h1_0000_0000;
                        remainder<=0;quotient<=0;bit_count<=0;divide_y<=1'b0;
                        state<=DIVIDE;
                    end
                end
                DIVIDE: begin
                    remainder<=take_bit ? shifted_remainder-{1'b0,divisor} : shifted_remainder;
                    quotient<=quotient_next;dividend<={dividend[31:0],1'b0};
                    if(bit_count==6'd32) begin
                        if(!divide_y) begin
                            sec_p<=$signed(quotient_next[31:0]);divide_y<=1'b1;
                            divisor<=cy;dividend<=33'h1_0000_0000;
                            remainder<=0;quotient<=0;bit_count<=0;
                        end else begin
                            sec_y<=$signed(quotient_next[31:0]);state<=GAIN_PRODUCT;
                        end
                    end else bit_count<=bit_count+1'b1;
                end
                GAIN_PRODUCT: begin
                    tan_p_product<=sp*sec_p;square_p_product<=sec_p*sec_p;
                    tan_y_product<=sy*sec_y;square_y_product<=sec_y*sec_y;
                    state<=GAIN_SCALE;
                end
                GAIN_SCALE: begin
                    tan_p<=tan_p_product>>>16;sec2_p<=square_p_product>>>16;
                    tan_y<=tan_y_product>>>16;sec2_y<=square_y_product>>>16;
                    state<=B_PRODUCT;
                end
                B_PRODUCT: begin
                    p00<=sec_p*cr;p01<=sec_p*sr;p10<=sec2_p*sr;
                    p11<=sec2_p*cr;p12<=FY_Q16*tan_p;state<=B_SCALE;
                end
                B_SCALE: begin
                    // Preserve the previous dual-axis rounding exactly.
                    b00<=p00>>>16;b01<=(-p01)>>>16;
                    b10<=p10>>>16;b11<=p11>>>16;b12<=(-p12)>>>16;
                    state<=Y_PRODUCT;
                end
                Y_PRODUCT: begin
                    y00<=sec2_y*b00;y01<=sec2_y*b01;y02<=FX_Q16*tan_y;
                    y10<=sec_y*b10;y11<=sec_y*b11;y12<=sec_y*b12;
                    state<=COMMIT;
                end
                COMMIT: begin
                    a00<=y00>>>16;a01<=y01>>>16;a02<=y02>>>16;
                    a10<=y10>>>16;a11<=y11>>>16;a12<=y12>>>16;
                    busy<=1'b0;done<=1'b1;error<=1'b0;state<=IDLE;
                end
                REJECT: begin
                    a00<=32'sd65536;a01<=0;a02<=0;
                    a10<=0;a11<=32'sd65536;a12<=0;
                    busy<=1'b0;done<=1'b1;error<=1'b1;state<=IDLE;
                end
                default: begin state<=IDLE;busy<=1'b0;error<=1'b1;end
            endcase
        end
    end
endmodule
`default_nettype wire
