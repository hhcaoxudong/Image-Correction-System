`timescale 1ns/1ps
`default_nettype wire
`include "source/undistort/brown_calibration.vh"

// Inverse image sampling: ideal output -> distorted source, Brown k1/k2/p1/p2.
// Q24 arithmetic follows the Qt fixedCoordinate operation order (k3 = 0).
// This calibrated 1080p profile stays in signed32 range at every Q24 node.
module brown_map_stream #(
    parameter integer WIDTH=1920, HEIGHT=1080, START_ROW=0, ROWS=HEIGHT,
    parameter integer SIDE_BY_SIDE=0,
    parameter signed [31:0] FX_Q16=`V132_FX_Q16, FY_Q16=`V132_FY_Q16,
    parameter signed [31:0] CX_Q16=`V132_CX_Q16, CY_Q16=`V132_CY_Q16,
    parameter signed [31:0] INV_FX_Q30=`V132_INV_FX_Q30,
    parameter signed [31:0] INV_FY_Q30=`V132_INV_FY_Q30,
    parameter signed [31:0] K1_Q24=`V132_K1_Q24, K2_Q24=`V132_K2_Q24,
    parameter signed [31:0] P1_Q24=`V132_P1_Q24, P2_Q24=`V132_P2_Q24
)(
    input wire clk, rst_n, start,
    input wire affine_enable,
    input wire signed [31:0] affine_a00,affine_a01,affine_a02,
    input wire signed [31:0] affine_a10,affine_a11,affine_a12,
    input wire signed [31:0] homography_h00,homography_h01,homography_h02,
    input wire signed [31:0] homography_h10,homography_h11,homography_h12,
    input wire signed [31:0] homography_h20,homography_h21,homography_h22,
    output reg busy, done, error,
    output wire [31:0] map_u, map_v,
    output wire map_valid,
    input wire map_ready,
    output wire map_user, map_last,
    output wire [15:0] map_x, map_y
);
    // Keep source coordinates aligned with the destination metadata after the
    // reciprocal interpolation stage.
    localparam integer STAGES=39;
    localparam signed [31:0] ONE_Q24=32'sd16777216;
    localparam signed [31:0] ONE_Q16=32'sd65536;
    localparam signed [31:0] U_LIMIT=WIDTH*65536, V_LIMIT=HEIGHT*65536;
    reg [STAGES-1:0] valid;
    reg issuing;
    reg [15:0] issue_x, issue_y;
    reg [15:0] px[0:STAGES-1], py[0:STAGES-1];
    reg signed [31:0] xn[1:12], yn[1:12];
    reg signed [63:0] nx0, ny0;
    reg signed [31:0] centered_x_reg, centered_y_reg;
    reg signed [63:0] homography_num_x_reg,homography_num_y_reg,homography_den_reg;
    reg signed [31:0] affine_x_q16_reg, affine_y_q16_reg;
    reg signed [31:0] dx_reg, dy_reg;
    wire signed [63:0] x2p4,y2p4,xyp4,r4p9,xradp15,yradp15;
    reg signed [31:0] x2s5,y2s5,xys5;
    reg signed [31:0] x2s6,y2s6,xys6,r2s6;
    reg signed [31:0] x2d[7:9],y2d[7:9],xyd[7:9],r2d[7:9];
    reg signed [31:0] r4s10,r2s10,xys10,sumx10,sumy10;
    reg signed [63:0] k1p11,k2p11,p1xyp11,p2xyp11,p2xp11,p1yp11;
    reg signed [31:0] radial12,tanx12,tany12;
    reg signed [31:0] tanxd[13:15],tanyd[13:15];
    reg signed [31:0] xd16,yd16;
    wire signed [63:0] up17,vp17;
    reg signed [31:0] u18,v18;
    reg signed [31:0] u18_d0,u18_d1,u18_d2,u18_d3;
    reg signed [31:0] v18_d0,v18_d1,v18_d2,v18_d3;
    reg [33:0] invalid_pipe;
    wire advance=!valid[STAGES-1] || map_ready;
    wire ce=advance;
    // Evaluate the unchanged full-resolution calibration at decimated coordinates.
    // px/py still carry destination coordinates and frame/line markers.
    localparam integer PICTURE_TOP=HEIGHT/4, PICTURE_BOTTOM=HEIGHT*3/4;
    wire issue_picture=issue_y>=PICTURE_TOP && issue_y<PICTURE_BOTTOM;
    wire [15:0] half_x=issue_x<WIDTH/2 ? issue_x : issue_x-WIDTH/2;
    wire [15:0] sample_x=SIDE_BY_SIDE ? (half_x<<1) : issue_x;
    wire [15:0] sample_y=SIDE_BY_SIDE ? (issue_picture ? ((issue_y-PICTURE_TOP)<<1) : 16'd0) : issue_y;
    wire signed [31:0] sample_x_q16=$signed({sample_x,16'd0});
    wire signed [31:0] sample_y_q16=$signed({sample_y,16'd0});
    wire signed [31:0] centered_x=sample_x_q16-CX_Q16;
    wire signed [31:0] centered_y=sample_y_q16-CY_Q16;
    // Full projective inverse map in centred pixel coordinates. Numerators and
    // denominator are Q32 while coefficients and coordinates are Q16.
    wire signed [63:0] hom_x00,hom_x01,hom_y10,hom_y11,hom_d20,hom_d21;
    // The frame controller selects the matrix before a frame starts. Keeping
    // this selection out of the pixel path avoids a high-fanout enable net.
    wire signed [31:0] map_h00=homography_h00;
    wire signed [31:0] map_h01=homography_h01;
    wire signed [31:0] map_h02=homography_h02;
    wire signed [31:0] map_h10=homography_h10;
    wire signed [31:0] map_h11=homography_h11;
    wire signed [31:0] map_h12=homography_h12;
    wire signed [31:0] map_h20=homography_h20;
    wire signed [31:0] map_h21=homography_h21;
    wire signed [31:0] map_h22=homography_h22;
    brown_mul32_pipe hom_mul_x00(.clk(clk),.ce(ce),.a(map_h00),.b(centered_x_reg),.p(hom_x00));
    brown_mul32_pipe hom_mul_x01(.clk(clk),.ce(ce),.a(map_h01),.b(centered_y_reg),.p(hom_x01));
    brown_mul32_pipe hom_mul_y10(.clk(clk),.ce(ce),.a(map_h10),.b(centered_x_reg),.p(hom_y10));
    brown_mul32_pipe hom_mul_y11(.clk(clk),.ce(ce),.a(map_h11),.b(centered_y_reg),.p(hom_y11));
    brown_mul32_pipe hom_mul_d20(.clk(clk),.ce(ce),.a(map_h20),.b(centered_x_reg),.p(hom_d20));
    brown_mul32_pipe hom_mul_d21(.clk(clk),.ce(ce),.a(map_h21),.b(centered_y_reg),.p(hom_d21));
    // PDS cannot map a variable signed '/' in this one-pixel-per-cycle path.
    // Use a fixed-point reciprocal with two Newton refinements instead.  The
    // reciprocal is Q16 and the numerator is first reduced from Q32 to Q16,
    // so the result below is a Q16 source coordinate.
    wire signed [31:0] hom_den_q16 = homography_den_reg[47:16];
    wire signed [31:0] hom_num_x_q16 = homography_num_x_reg[47:16];
    wire signed [31:0] hom_num_y_q16 = homography_num_y_reg[47:16];
    reg [4:0] hom_recip_interval_d1;
    reg [9:0] hom_recip_fraction_d1,hom_recip_fraction_d2;
    reg hom_recip_scale_half_d1,hom_recip_scale_half_d2;
    reg [2:0] hom_recip_shift_left_d1,hom_recip_shift_left_d2,hom_recip_shift_left_d3;
    reg hom_recip_special_d1,hom_recip_special_d2;
    reg signed [31:0] hom_recip_special_value_d1,hom_recip_special_value_d2;
    reg [17:0] hom_recip_base_d2,hom_recip_base_d3;
    reg signed [12:0] hom_recip_step_d2;
    reg signed [23:0] hom_recip_interpolation_product_d3;
    reg hom_recip_scale_half_d3,hom_recip_special_d3;
    reg signed [31:0] hom_recip_special_value_d3;
    reg signed [31:0] hom_num_x_q16_d1,hom_num_y_q16_d1;
    reg signed [31:0] hom_num_x_q16_d2,hom_num_y_q16_d2;
    reg signed [31:0] hom_num_x_q16_d3,hom_num_y_q16_d3;
    reg signed [31:0] hom_num_x_q16_d4,hom_num_y_q16_d4;
    reg signed [31:0] hom_recip_q16_reg;
    reg hom_den_positive_d1,hom_den_positive_d2,hom_den_positive_d3;
    reg hom_den_positive_d4,hom_den_positive_d5,hom_den_positive_d6;
    reg hom_den_positive_d7;
    reg [31:0] hom_recip_mantissa_comb;
    reg [4:0] hom_recip_interval_comb;
    reg [9:0] hom_recip_fraction_comb;
    reg hom_recip_scale_half_comb,hom_recip_special_comb;
    reg signed [31:0] hom_recip_special_value_comb;
    reg [2:0] hom_recip_shift_left_comb;
    reg [17:0] hom_recip_base_comb;
    reg signed [12:0] hom_recip_step_comb;
    // Register the lookup product before the reciprocal feeds the divider DSP.
    wire signed [23:0] hom_recip_interpolation_product = hom_recip_interpolation_product_d3;
    wire signed [31:0] hom_recip_interpolated =
        $signed({1'b0,hom_recip_base_d3}) + (hom_recip_interpolation_product >>> 10);
    wire signed [31:0] hom_recip_normalized = hom_recip_scale_half_d3 ?
        (hom_recip_interpolated >>> 1) : hom_recip_interpolated;
    wire signed [31:0] hom_recip_q16_for_mul = hom_recip_special_d3 ? hom_recip_special_value_d3 :
        (hom_recip_normalized <<< hom_recip_shift_left_d3);
    wire signed [63:0] hom_x_product,hom_y_product;
    // Isolate interpolation/normalization from the DSP input setup path.
    brown_mul32_pipe hom_div_mul_x(.clk(clk),.ce(ce),.a(hom_num_x_q16_d4),
                                   .b(hom_recip_q16_reg),.p(hom_x_product));
    brown_mul32_pipe hom_div_mul_y(.clk(clk),.ce(ce),.a(hom_num_y_q16_d4),
                                   .b(hom_recip_q16_reg),.p(hom_y_product));
    wire signed [63:0] hom_x_div = hom_den_positive_d7 ?
                                    (hom_x_product >>> 16) : 64'sd0;
    wire signed [63:0] hom_y_div = hom_den_positive_d7 ?
                                    (hom_y_product >>> 16) : 64'sd0;
    wire signed [31:0] dx=dx_reg;
    wire signed [31:0] dy=dy_reg;
    // 2026-10-22: when the pose pushes the sample point outside the captured
    // frame, saturate it to the last safe 2x2 bilinear position instead of
    // marking it invalid.  The border then repeats the edge pixels, which is
    // what the 50K reference (V13_21 SAFE_X_MAX/SAFE_Y_MAX) does: no black
    // wedge sweeps across the picture while the camera rotates.  Genuinely
    // illegal samples (non-positive denominator, overflow) stay invalid.
    localparam signed [31:0] U_SAFE_MAX=(WIDTH-2)*65536;
    localparam signed [31:0] V_SAFE_MAX=(HEIGHT-2)*65536;
    wire signed [31:0] u_clamped=(u18_d3<0) ? 32'sd0 :
                                  ((u18_d3>U_SAFE_MAX) ? U_SAFE_MAX : u18_d3);
    wire signed [31:0] v_clamped=(v18_d3<0) ? 32'sd0 :
                                  ((v18_d3>V_SAFE_MAX) ? V_SAFE_MAX : v18_d3);
    wire coord_ok_d=!invalid_pipe[33];
    brown_mul32_pipe final_mul_x(.clk(clk),.ce(ce),.a(FX_Q16),.b(xd16),.p(up17));
    brown_mul32_pipe final_mul_y(.clk(clk),.ce(ce),.a(FY_Q16),.b(yd16),.p(vp17));
    assign map_u=coord_ok_d ? u_clamped : 32'hffffffff;
    assign map_v=coord_ok_d ? v_clamped : 32'hffffffff;
    assign map_valid=rst_n && valid[STAGES-1];
    assign map_x=px[STAGES-1];
    assign map_y=py[STAGES-1];
    assign map_user=(map_x==0)&&(map_y==START_ROW);
    assign map_last=(map_x==WIDTH-1);
    brown_mul32_pipe square_x(.clk(clk),.ce(ce),.a(xn[1]),.b(xn[1]),.p(x2p4));
    brown_mul32_pipe square_y(.clk(clk),.ce(ce),.a(yn[1]),.b(yn[1]),.p(y2p4));
    brown_mul32_pipe cross_xy(.clk(clk),.ce(ce),.a(xn[1]),.b(yn[1]),.p(xyp4));
    brown_mul32_pipe square_r2(.clk(clk),.ce(ce),.a(r2s6),.b(r2s6),.p(r4p9));
    brown_mul32_pipe radial_x(.clk(clk),.ce(ce),.a(xn[12]),.b(radial12),.p(xradp15));
    brown_mul32_pipe radial_y(.clk(clk),.ce(ce),.a(yn[12]),.b(radial12),.p(yradp15));

    // Normalize the denominator first, then register the interval table and
    // interpolation result on separate cycles.
    always @* begin
        hom_recip_mantissa_comb = 0;
        hom_recip_interval_comb = 0;
        hom_recip_fraction_comb = 0;
        hom_recip_scale_half_comb = 0;
        hom_recip_shift_left_comb = 0;
        hom_recip_special_comb = 0;
        hom_recip_special_value_comb = 0;
        if (hom_den_q16 <= 0) begin
            hom_recip_special_comb = 1;
            hom_recip_special_value_comb = 0;
        end else if (hom_den_q16 < 32'sd32768) begin
            if (hom_den_q16 >= 32'sd16384) begin
                hom_recip_shift_left_comb = 1;
                hom_recip_mantissa_comb = $unsigned(hom_den_q16) << 1;
            end else if (hom_den_q16 >= 32'sd8192) begin
                hom_recip_shift_left_comb = 2;
                hom_recip_mantissa_comb = $unsigned(hom_den_q16) << 2;
            end else if (hom_den_q16 >= 32'sd4096) begin
                hom_recip_shift_left_comb = 3;
                hom_recip_mantissa_comb = $unsigned(hom_den_q16) << 3;
            end else if (hom_den_q16 >= 32'sd2048) begin
                hom_recip_shift_left_comb = 4;
                hom_recip_mantissa_comb = $unsigned(hom_den_q16) << 4;
            end else if (hom_den_q16 >= 32'sd1024) begin
                hom_recip_shift_left_comb = 5;
                hom_recip_mantissa_comb = $unsigned(hom_den_q16) << 5;
            end else begin
                // Below 1/64, projected coordinates are outside the source
                // frame. Saturate the reciprocal before the bounds check.
                hom_recip_special_comb = 1;
                hom_recip_special_value_comb = 32'sd4194304;
            end
        end else if (hom_den_q16 >= 32'sd131072) begin
            hom_recip_special_comb = 1;
            hom_recip_special_value_comb = 32'sd32768;
        end else begin
            hom_recip_scale_half_comb = hom_den_q16 >= 32'sd65536;
            hom_recip_mantissa_comb = hom_recip_scale_half_comb ?
                ($unsigned(hom_den_q16) >> 1) : $unsigned(hom_den_q16);
        end
        if (!hom_recip_special_comb) begin
            hom_recip_interval_comb = hom_recip_mantissa_comb[14:10];
            hom_recip_fraction_comb = hom_recip_mantissa_comb[9:0];
        end
    end

    always @* begin
        hom_recip_base_comb = 18'd65536;
        hom_recip_step_comb = 0;
        case (hom_recip_interval_d1)
            0: begin hom_recip_base_comb=18'd131072; hom_recip_step_comb=-13'sd3972; end
            1: begin hom_recip_base_comb=18'd127100; hom_recip_step_comb=-13'sd3738; end
            2: begin hom_recip_base_comb=18'd123362; hom_recip_step_comb=-13'sd3525; end
            3: begin hom_recip_base_comb=18'd119837; hom_recip_step_comb=-13'sd3329; end
            4: begin hom_recip_base_comb=18'd116508; hom_recip_step_comb=-13'sd3148; end
            5: begin hom_recip_base_comb=18'd113360; hom_recip_step_comb=-13'sd2984; end
            6: begin hom_recip_base_comb=18'd110376; hom_recip_step_comb=-13'sd2830; end
            7: begin hom_recip_base_comb=18'd107546; hom_recip_step_comb=-13'sd2688; end
            8: begin hom_recip_base_comb=18'd104858; hom_recip_step_comb=-13'sd2558; end
            9: begin hom_recip_base_comb=18'd102300; hom_recip_step_comb=-13'sd2436; end
            10: begin hom_recip_base_comb=18'd99864; hom_recip_step_comb=-13'sd2322; end
            11: begin hom_recip_base_comb=18'd97542; hom_recip_step_comb=-13'sd2217; end
            12: begin hom_recip_base_comb=18'd95325; hom_recip_step_comb=-13'sd2118; end
            13: begin hom_recip_base_comb=18'd93207; hom_recip_step_comb=-13'sd2026; end
            14: begin hom_recip_base_comb=18'd91181; hom_recip_step_comb=-13'sd1940; end
            15: begin hom_recip_base_comb=18'd89241; hom_recip_step_comb=-13'sd1860; end
            16: begin hom_recip_base_comb=18'd87381; hom_recip_step_comb=-13'sd1783; end
            17: begin hom_recip_base_comb=18'd85598; hom_recip_step_comb=-13'sd1712; end
            18: begin hom_recip_base_comb=18'd83886; hom_recip_step_comb=-13'sd1645; end
            19: begin hom_recip_base_comb=18'd82241; hom_recip_step_comb=-13'sd1581; end
            20: begin hom_recip_base_comb=18'd80660; hom_recip_step_comb=-13'sd1522; end
            21: begin hom_recip_base_comb=18'd79138; hom_recip_step_comb=-13'sd1466; end
            22: begin hom_recip_base_comb=18'd77672; hom_recip_step_comb=-13'sd1412; end
            23: begin hom_recip_base_comb=18'd76260; hom_recip_step_comb=-13'sd1362; end
            24: begin hom_recip_base_comb=18'd74898; hom_recip_step_comb=-13'sd1314; end
            25: begin hom_recip_base_comb=18'd73584; hom_recip_step_comb=-13'sd1268; end
            26: begin hom_recip_base_comb=18'd72316; hom_recip_step_comb=-13'sd1226; end
            27: begin hom_recip_base_comb=18'd71090; hom_recip_step_comb=-13'sd1185; end
            28: begin hom_recip_base_comb=18'd69905; hom_recip_step_comb=-13'sd1146; end
            29: begin hom_recip_base_comb=18'd68759; hom_recip_step_comb=-13'sd1109; end
            30: begin hom_recip_base_comb=18'd67650; hom_recip_step_comb=-13'sd1074; end
            31: begin hom_recip_base_comb=18'd66576; hom_recip_step_comb=-13'sd1040; end
            default: begin hom_recip_base_comb=18'd65536; hom_recip_step_comb=0; end
        endcase
    end
    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            busy<=0;done<=0;error<=0;issuing<=0;valid<=0;
            issue_x<=0;issue_y<=START_ROW;
            centered_x_reg<=0;centered_y_reg<=0;
            homography_num_x_reg<=0;homography_num_y_reg<=0;homography_den_reg<=0;
            hom_recip_interval_d1<=0;hom_recip_fraction_d1<=0;
            hom_recip_scale_half_d1<=0;hom_recip_special_d1<=0;
            hom_recip_shift_left_d1<=0;hom_recip_shift_left_d2<=0;hom_recip_shift_left_d3<=0;
            hom_recip_special_value_d1<=0;
            hom_recip_base_d2<=0;hom_recip_step_d2<=0;hom_recip_fraction_d2<=0;
            hom_recip_scale_half_d2<=0;hom_recip_special_d2<=0;
            hom_recip_special_value_d2<=0;
            hom_recip_base_d3<=0;hom_recip_interpolation_product_d3<=0;
            hom_recip_scale_half_d3<=0;hom_recip_special_d3<=0;
            hom_recip_special_value_d3<=0;
            hom_num_x_q16_d1<=0;hom_num_y_q16_d1<=0;
            hom_num_x_q16_d2<=0;hom_num_y_q16_d2<=0;
            hom_num_x_q16_d3<=0;hom_num_y_q16_d3<=0;
            hom_num_x_q16_d4<=0;hom_num_y_q16_d4<=0;hom_recip_q16_reg<=0;
            hom_den_positive_d1<=0;hom_den_positive_d2<=0;
            hom_den_positive_d3<=0;hom_den_positive_d4<=0;
            hom_den_positive_d5<=0;hom_den_positive_d6<=0;
            hom_den_positive_d7<=0;
            affine_x_q16_reg<=0;affine_y_q16_reg<=0;
            dx_reg<=0;dy_reg<=0;
            u18_d0<=0;u18_d1<=0;u18_d2<=0;u18_d3<=0;
            v18_d0<=0;v18_d1<=0;v18_d2<=0;v18_d3<=0;
            invalid_pipe<=0;
        end else begin
            done<=0;
            if (!busy && start) begin
                valid<=0;error<=0;issue_x<=0;issue_y<=START_ROW;
                if(WIDTH<1 || WIDTH>1920 || HEIGHT<1 || HEIGHT>1080 ||
                   START_ROW<0 || ROWS<1 || START_ROW+ROWS>HEIGHT) begin
                    busy<=0;issuing<=0;done<=1;error<=1;
                end else begin busy<=1;issuing<=1;end
            end else if(ce) begin
                valid<={valid[STAGES-2:0],issuing};
                px[0]<=issue_x;py[0]<=issue_y;
                for(i=1;i<STAGES;i=i+1) begin px[i]<=px[i-1];py[i]<=py[i-1];end
                for(i=2;i<=12;i=i+1) begin xn[i]<=xn[i-1];yn[i]<=yn[i-1];end
                // Break the long issue_y/issue_x -> affine -> normalization
                // path.  Separate registers hold centered coordinates, the
                // affine multiply result, and the affine output coordinate.
                centered_x_reg<=centered_x;
                centered_y_reg<=centered_y;
                homography_num_x_reg<=hom_x00+hom_x01+($signed(map_h02)<<<16);
                homography_num_y_reg<=hom_y10+hom_y11+($signed(map_h12)<<<16);
                // h20/h21 arrive in Q24 (see imu_matrix_affine); their product
                // with the Q16 centred coordinate is Q40, so shift by 8 to land
                // in the same Q32 as h22<<16 and the numerators.
                homography_den_reg<=hom_d20+hom_d21+($signed(map_h22)<<<16);
                hom_recip_interval_d1<=hom_recip_interval_comb;
                hom_recip_fraction_d1<=hom_recip_fraction_comb;
                hom_recip_scale_half_d1<=hom_recip_scale_half_comb;
                hom_recip_shift_left_d1<=hom_recip_shift_left_comb;
                hom_recip_special_d1<=hom_recip_special_comb;
                hom_recip_special_value_d1<=hom_recip_special_value_comb;
                hom_recip_base_d2<=hom_recip_base_comb;
                hom_recip_step_d2<=hom_recip_step_comb;
                hom_recip_fraction_d2<=hom_recip_fraction_d1;
                hom_recip_scale_half_d2<=hom_recip_scale_half_d1;
                hom_recip_shift_left_d2<=hom_recip_shift_left_d1;
                hom_recip_special_d2<=hom_recip_special_d1;
                hom_recip_special_value_d2<=hom_recip_special_value_d1;
                hom_recip_base_d3<=hom_recip_base_d2;
                hom_recip_interpolation_product_d3<=
                    hom_recip_step_d2 * $signed({1'b0,hom_recip_fraction_d2});
                hom_recip_scale_half_d3<=hom_recip_scale_half_d2;
                hom_recip_shift_left_d3<=hom_recip_shift_left_d2;
                hom_recip_special_d3<=hom_recip_special_d2;
                hom_recip_special_value_d3<=hom_recip_special_value_d2;
                hom_num_x_q16_d1<=hom_num_x_q16;
                hom_num_y_q16_d1<=hom_num_y_q16;
                hom_num_x_q16_d2<=hom_num_x_q16_d1;
                hom_num_y_q16_d2<=hom_num_y_q16_d1;
                hom_num_x_q16_d3<=hom_num_x_q16_d2;
                hom_num_y_q16_d3<=hom_num_y_q16_d2;
                hom_num_x_q16_d4<=hom_num_x_q16_d3;
                hom_num_y_q16_d4<=hom_num_y_q16_d3;
                hom_recip_q16_reg<=hom_recip_q16_for_mul;
                hom_den_positive_d1<=homography_den_reg>0;
                hom_den_positive_d2<=hom_den_positive_d1;
                hom_den_positive_d3<=hom_den_positive_d2;
                hom_den_positive_d4<=hom_den_positive_d3;
                hom_den_positive_d5<=hom_den_positive_d4;
                hom_den_positive_d6<=hom_den_positive_d5;
                hom_den_positive_d7<=hom_den_positive_d6;
                affine_x_q16_reg<=hom_x_div[31:0];
                affine_y_q16_reg<=hom_y_div[31:0];
                dx_reg<=affine_x_q16_reg;
                dy_reg<=affine_y_q16_reg;
                nx0<=dx_reg*INV_FX_Q30;ny0<=dy_reg*INV_FY_Q30;
                xn[1]<=nx0>>>22;yn[1]<=ny0>>>22;
                x2s5<=x2p4>>>24;y2s5<=y2p4>>>24;xys5<=xyp4>>>24;
                r2s6<=x2s5+y2s5;x2s6<=x2s5;y2s6<=y2s5;xys6<=xys5;
                x2d[7]<=x2s6;y2d[7]<=y2s6;xyd[7]<=xys6;r2d[7]<=r2s6;
                for(i=8;i<=9;i=i+1) begin
                    x2d[i]<=x2d[i-1];y2d[i]<=y2d[i-1];xyd[i]<=xyd[i-1];r2d[i]<=r2d[i-1];
                end
                r4s10<=r4p9>>>24;r2s10<=r2d[9];xys10<=xyd[9];
                sumx10<=r2d[9]+(x2d[9]<<<1);sumy10<=r2d[9]+(y2d[9]<<<1);
                k1p11<=K1_Q24*r2s10;k2p11<=K2_Q24*r4s10;
                p1xyp11<=P1_Q24*xys10;p2xyp11<=P2_Q24*xys10;
                p2xp11<=P2_Q24*sumx10;p1yp11<=P1_Q24*sumy10;
                radial12<=ONE_Q24+(k1p11>>>24)+(k2p11>>>24);
                tanx12<=((p1xyp11>>>24)<<<1)+(p2xp11>>>24);
                tany12<=(p1yp11>>>24)+((p2xyp11>>>24)<<<1);
                tanxd[13]<=tanx12;tanyd[13]<=tany12;
                for(i=14;i<=15;i=i+1) begin tanxd[i]<=tanxd[i-1];tanyd[i]<=tanyd[i-1];end
                xd16<=(xradp15>>>24)+tanxd[15];yd16<=(yradp15>>>24)+tanyd[15];
                u18<=(up17>>>24)+CX_Q16;v18<=(vp17>>>24)+CY_Q16;
                u18_d0<=u18;u18_d1<=u18_d0;u18_d2<=u18_d1;u18_d3<=u18_d2;
                v18_d0<=v18;v18_d1<=v18_d0;v18_d2<=v18_d1;v18_d3<=v18_d2;
                invalid_pipe[0]<=homography_den_reg<=0;
                for(i=1;i<=33;i=i+1) invalid_pipe[i]<=invalid_pipe[i-1];
                // A wide projective coordinate must not wrap into the sensor
                // area when narrowed. Carry each overflow with the same pixel.
                invalid_pipe[7]<=invalid_pipe[6] ||
                    (hom_x_div[63:31]!={33{hom_x_div[31]}}) ||
                    (hom_y_div[63:31]!={33{hom_y_div[31]}});
                invalid_pipe[10]<=invalid_pipe[9] ||
                    (nx0[63:53]!={11{nx0[53]}}) ||
                    (ny0[63:53]!={11{ny0[53]}});
                // Squared Q24 values are nonnegative. Bit 55 is the sign
                // bit after reduction to signed32; any high bit is overflow.
                invalid_pipe[14]<=invalid_pipe[13] ||
                    ((K1_Q24!=0 || K2_Q24!=0 || P1_Q24!=0 || P2_Q24!=0) &&
                     ((|x2p4[63:55]) || (|y2p4[63:55])));
                invalid_pipe[16]<=invalid_pipe[15] ||
                    ((K1_Q24!=0 || K2_Q24!=0 || P1_Q24!=0 || P2_Q24!=0) && r2s6[31]);
                invalid_pipe[19]<=invalid_pipe[18] ||
                    ((K2_Q24!=0) && (|r4p9[63:55]));
                if(issuing) begin
                    if(issue_x==WIDTH-1) begin
                        issue_x<=0;
                        if(issue_y==START_ROW+ROWS-1) issuing<=0;
                        else issue_y<=issue_y+1'b1;
                    end else issue_x<=issue_x+1'b1;
                end
                if(valid[STAGES-1] && map_ready && map_last && map_y==START_ROW+ROWS-1) begin
                    busy<=0;issuing<=0;valid<=0;done<=1;
                end
            end
        end
    end
endmodule
// Exact signed 32x32 product, three clock enables of latency. Register four
// 16-bit partial products before recombination to avoid a long DSP cascade.
module brown_mul32_pipe(
    input wire clk,ce,
    input wire signed [31:0] a,b,
    output reg signed [63:0] p
);
    wire signed [16:0] al=$signed({1'b0,a[15:0]}),bl=$signed({1'b0,b[15:0]});
    wire signed [15:0] ah=a[31:16],bh=b[31:16];
    reg [31:0] ll;
    reg signed [32:0] lh,hl;
    reg signed [31:0] hh,hh_d;
    reg signed [34:0] middle;
    reg [15:0] low_d;
    wire signed [47:0] upper=($signed({{16{hh_d[31]}},hh_d})<<<16)+
                                   $signed({{13{middle[34]}},middle});
    always @(posedge clk) if(ce) begin
        ll<=al*bl;lh<=al*bh;hl<=ah*bl;hh<=ah*bh;
        middle<=$signed({{2{lh[32]}},lh})+$signed({{2{hl[32]}},hl})+
                $signed({19'd0,ll[31:16]});
        hh_d<=hh;low_d<=ll[15:0];
        p<={upper,low_d};
    end
endmodule
`default_nettype wire