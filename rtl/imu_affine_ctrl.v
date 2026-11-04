`timescale 1ns/1ps
`default_nettype none

// WIT angle receiver, KEY1 reference capture, and exact one-angle rotation.
// Six Q16.16 outputs preserve the existing map interface; only cos and sin
// vary. The map rotates around the calibrated optical principal point.
module imu_affine_ctrl #(
    parameter integer CLK_HZ=10000000,
    parameter integer BAUD=115200,
    parameter integer ROLL_SIGN=1
)(
    input wire clk,
    input wire rst_n,
    input wire sensor_rx,
    input wire key1_n,
    output reg affine_enable,
    output reg signed [31:0] affine_a00,
    output reg signed [31:0] affine_a01,
    output reg signed [31:0] affine_a02,
    output reg signed [31:0] affine_a10,
    output reg signed [31:0] affine_a11,
    output reg signed [31:0] affine_a12,
    output reg frame_valid,
    output reg pose_valid_latched,
    output reg pose_legal_latched,
    output wire signed [15:0] debug_roll_raw,debug_pitch_raw,debug_yaw_raw,debug_delta_roll_raw,
    output wire debug_fresh
);
    localparam integer BAUD_DIV=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer HALF_DIV=BAUD_DIV/2;
    localparam integer KEY_DEBOUNCE=CLK_HZ/100;
    localparam signed [31:0] ONE_Q16=32'sd65536;
    // Pitch/yaw beyond about 15 degrees warn that pure roll compensation
    // cannot describe the motion. This only drives the status indicator.
    localparam signed [16:0] MAX_DELTA_RAW=17'sd2731;

    reg rx_meta,rx_sync,rx_busy;
    reg [15:0] rx_count;
    reg [3:0] rx_bit;
    reg [7:0] rx_shift;
    reg byte_valid;
    reg [7:0] byte_data;

    always @(posedge clk) begin
        if(!rst_n) begin
            rx_meta<=1'b1; rx_sync<=1'b1; rx_busy<=1'b0;
            rx_count<=0; rx_bit<=0; rx_shift<=0;
            byte_valid<=1'b0; byte_data<=0;
        end else begin
            rx_meta<=sensor_rx;
            rx_sync<=rx_meta;
            byte_valid<=1'b0;
            if(!rx_busy) begin
                if(!rx_sync) begin
                    rx_busy<=1'b1;
                    rx_count<=HALF_DIV;
                    rx_bit<=0;
                end
            end else if(rx_count!=0) begin
                rx_count<=rx_count-1'b1;
            end else if(rx_bit==0) begin
                if(rx_sync) rx_busy<=1'b0;
                else begin
                    rx_bit<=1;
                    rx_count<=BAUD_DIV-1;
                end
            end else if(rx_bit<=8) begin
                rx_shift[rx_bit-1]<=rx_sync;
                if(rx_bit==8) begin
                    rx_bit<=9;
                end else begin
                    rx_bit<=rx_bit+1'b1;
                end
                rx_count<=BAUD_DIV-1;
            end else begin
                rx_busy<=1'b0;
                byte_valid<=rx_sync; // Reject a low stop bit.
                byte_data<=rx_shift;
            end
        end
    end

    reg [3:0] frame_index;
    reg [7:0] frame_byte[0:10];
    reg signed [15:0] roll_raw,pitch_raw,yaw_raw;
    reg pose_valid;
    reg pose_strobe;
    reg [7:0] checksum;
    // A KEY1 press is accepted only after at least one checksum-valid WIT
    // attitude frame has been received.  Without this guard, pressing KEY1
    // during startup stores the reset value (0,0,0) as the reference pose.
    reg base_valid;
    assign debug_roll_raw=roll_raw;
    assign debug_pitch_raw=pitch_raw;
    assign debug_yaw_raw=yaw_raw;
    assign debug_delta_roll_raw=base_valid ? rotation_raw : 16'sd0;
    assign debug_fresh=pose_fresh;
    reg [31:0] frame_idle;
    integer i;
    always @(posedge clk) begin
        if(!rst_n) begin
            frame_idle<=0; frame_index<=0; roll_raw<=0; pitch_raw<=0; yaw_raw<=0;
            pose_valid<=1'b0; pose_strobe<=0; checksum<=0;
            pose_valid_latched<=1'b0;
            for(i=0;i<11;i=i+1) frame_byte[i]<=0;
        end else begin
            pose_strobe<=0;
            if(byte_valid) begin
            frame_idle<=0;
            if(frame_index==0) begin
                if(byte_data==8'h55) begin frame_byte[0]<=byte_data; checksum<=byte_data; frame_index<=1; end
            end else if(frame_index==1) begin
                if(byte_data==8'h53) begin frame_byte[1]<=byte_data; checksum<=checksum+byte_data; frame_index<=2; end
                else if(byte_data==8'h55) begin frame_byte[0]<=byte_data; checksum<=byte_data; frame_index<=1; end
                else frame_index<=0;
            end else begin
                frame_byte[frame_index]<=byte_data;
                checksum<=checksum+byte_data;
                if(frame_index==10) begin
                    frame_index<=0;
                    if(byte_data==checksum) begin
                        roll_raw<=$signed({frame_byte[3],frame_byte[2]});
                        pitch_raw<=$signed({frame_byte[5],frame_byte[4]});
                        yaw_raw<=$signed({frame_byte[7],frame_byte[6]});
                        pose_valid<=1'b1; pose_strobe<=1;
                        pose_valid_latched<=1'b1;
                    end
                end else frame_index<=frame_index+1'b1;
            end
            end else if(frame_idle<BAUD_DIV*40) frame_idle<=frame_idle+1'b1;
            else frame_index<=0;
        end
    end

    reg key_meta,key_sync,key_stable,key_press;
    reg [31:0] key_count;
    always @(posedge clk) begin
        if(!rst_n) begin
            key_meta<=1'b1; key_sync<=1'b1; key_stable<=1'b1;
            key_count<=0; key_press<=1'b0;
        end else begin
            key_meta<=key1_n;
            key_sync<=key_meta;
            key_press<=1'b0;
            if(key_sync==key_stable) key_count<=0;
            else if(key_count<KEY_DEBOUNCE-1) key_count<=key_count+1'b1;
            else begin
                key_stable<=key_sync; key_count<=0;
                if(!key_sync) key_press<=1'b1;
            end
        end
    end

    // V6.2: one angle controls exact optical-axis rotation. Pitch/yaw only
    // drive the validity indicator; there are no image translations.
    reg signed [15:0] base_roll,base_pitch,base_yaw;
    // Sixteen-bit subtraction intentionally wraps at +/-180 degrees.
    wire signed [15:0] d_roll=roll_raw-base_roll;
    wire signed [15:0] d_pitch=pitch_raw-base_pitch;
    wire signed [15:0] d_yaw=yaw_raw-base_yaw;
    wire signed [16:0] roll_wide={d_roll[15],d_roll};
    wire signed [16:0] pitch_wide={d_pitch[15],d_pitch};
    wire signed [16:0] yaw_wide={d_yaw[15],d_yaw};
    wire signed [16:0] roll_abs=roll_wide<0 ? -roll_wide : roll_wide;
    wire signed [16:0] pitch_abs=pitch_wide<0 ? -pitch_wide : pitch_wide;
    wire signed [16:0] yaw_abs=yaw_wide<0 ? -yaw_wide : yaw_wide;
    wire signed [15:0] rotation_raw=ROLL_SIGN<0 ? -d_roll : d_roll;
    reg rotation_start;
    wire rotation_busy,rotation_done;
    wire signed [31:0] rotation_cos,rotation_sin;
    rotation_sincos rotation_math(
        .clk(clk),.rst_n(rst_n),.start(rotation_start),.angle_raw(rotation_raw),
        .busy(rotation_busy),.done(rotation_done),
        .cos_q16(rotation_cos),.sin_q16(rotation_sin));

    // Freshness is independent of whether any frame was ever received.
    // Half a second without an angle frame prevents stale calibration.
    reg [31:0] pose_age;
    wire pose_fresh=pose_valid && pose_age<CLK_HZ/2;
    always @(posedge clk) begin
        if(!rst_n) pose_age<=CLK_HZ/2;
        else if(pose_strobe) pose_age<=0;
        else if(pose_age<CLK_HZ/2) pose_age<=pose_age+1'b1;
    end

    always @(posedge clk) begin
        if(!rst_n) begin
            affine_enable<=0; base_valid<=0; frame_valid<=0;
            affine_a00<=ONE_Q16; affine_a01<=0; affine_a02<=0;
            affine_a10<=0; affine_a11<=ONE_Q16; affine_a12<=0;
            base_roll<=0; base_pitch<=0; base_yaw<=0;
            pose_legal_latched<=0; rotation_start<=0;
        end else begin
            rotation_start<=0;
            frame_valid<=0;
            pose_legal_latched<=pose_fresh && base_valid &&
                roll_abs<=17'sd5461 && pitch_abs<=MAX_DELTA_RAW && yaw_abs<=MAX_DELTA_RAW;
            // KEY1 has priority over completion of a previous calculation.
            if(key_press) begin
                if(affine_enable) begin
                    affine_enable<=0; base_valid<=0;
                    affine_a00<=ONE_Q16; affine_a01<=0; affine_a02<=0;
                    affine_a10<=0; affine_a11<=ONE_Q16; affine_a12<=0;
                    frame_valid<=1;
                end else if(pose_fresh) begin
                    base_roll<=roll_raw; base_pitch<=pitch_raw; base_yaw<=yaw_raw;
                    base_valid<=1; affine_enable<=1;
                    affine_a00<=ONE_Q16; affine_a01<=0; affine_a02<=0;
                    affine_a10<=0; affine_a11<=ONE_Q16; affine_a12<=0;
                    frame_valid<=1;
                end
            end else if(affine_enable && base_valid) begin
                if(pose_strobe && !rotation_busy && !rotation_start)
                    rotation_start<=1;
                if(rotation_done) begin
                    affine_a00<=rotation_cos; affine_a11<=rotation_cos;
                    affine_a01<=-rotation_sin; affine_a10<=rotation_sin;
                    affine_a02<=0; affine_a12<=0;
                    frame_valid<=1;
                end
            end
        end
    end
endmodule

// Sequential CORDIC: 20 shift/add iterations at the configuration clock.
// Input: signed WIT units (32768 = 180 degrees). Outputs: signed Q16.16.
// atan constants and gain are generated from mathematical definitions.
module rotation_sincos(
    input wire clk,rst_n,start,
    input wire signed [15:0] angle_raw,
    output reg busy,done,
    output reg signed [31:0] cos_q16,sin_q16
);
    reg signed [31:0] x,y,z;
    reg [4:0] step;
    reg signed [31:0] atan_step;
    wire signed [31:0] phase={angle_raw,16'd0};
    always @* begin
        case(step)
            5'd0: atan_step=32'sd536870912;
            5'd1: atan_step=32'sd316933406;
            5'd2: atan_step=32'sd167458907;
            5'd3: atan_step=32'sd85004756;
            5'd4: atan_step=32'sd42667331;
            5'd5: atan_step=32'sd21354465;
            5'd6: atan_step=32'sd10679838;
            5'd7: atan_step=32'sd5340245;
            5'd8: atan_step=32'sd2670163;
            5'd9: atan_step=32'sd1335087;
            5'd10: atan_step=32'sd667544;
            5'd11: atan_step=32'sd333772;
            5'd12: atan_step=32'sd166886;
            5'd13: atan_step=32'sd83443;
            5'd14: atan_step=32'sd41722;
            5'd15: atan_step=32'sd20861;
            5'd16: atan_step=32'sd10430;
            5'd17: atan_step=32'sd5215;
            5'd18: atan_step=32'sd2608;
            5'd19: atan_step=32'sd1304;
            default: atan_step=0;
        endcase
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            busy<=0;done<=0;cos_q16<=65536;sin_q16<=0;
            x<=0;y<=0;z<=0;step<=0;
        end else begin
            done<=0;
            if(!busy && start) begin
                if(angle_raw==0) begin
                    cos_q16<=65536;sin_q16<=0;done<=1;
                end else begin
                    busy<=1;step<=0;y<=0;
                    if(angle_raw>16'sd16384 || angle_raw< -16'sd16384) begin
                        x<=-32'sd652032874;
                        z<=phase ^ 32'h80000000;
                    end else begin
                        x<=32'sd652032874;z<=phase;
                    end
                end
            end else if(busy) begin
                if(step==20) begin
                    cos_q16<=(x+32'sd8192)>>>14;
                    sin_q16<=(y+32'sd8192)>>>14;
                    busy<=0;done<=1;
                end else begin
                    if(z>=0) begin
                        x<=x-(y>>>step);y<=y+(x>>>step);z<=z-atan_step;
                    end else begin
                        x<=x+(y>>>step);y<=y-(x>>>step);z<=z+atan_step;
                    end
                    step<=step+1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
