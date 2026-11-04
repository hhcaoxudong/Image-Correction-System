`timescale 1ns/1ps
`default_nettype none

// WT901BC -> current/reference matrices -> centre affine inverse map.
// Raw Euler differences below are diagnostics only, never map coefficients.
// V1.3.2.6.6: when a pose exceeds the local-affine validity range, retain the
// last valid coefficients. This prevents a visible jump back to the identity
// map. pose_legal is cleared so the LED/debug output still reports the limit.
// A short UART freshness gap also retains the last map. A continuous gap of
// about two seconds still disables correction and falls back to identity.
module imu_sensor_control #(
    parameter integer CLK_HZ=10000000,
    parameter integer BAUD=115200,
    parameter integer STALE_MS=500,
    parameter integer STALE_HOLD_MS=2000
)(
    input wire clk,rst_n,sensor_rx,key1_n,
    output reg enabled,command_valid,calibrated,pose_legal,
    output wire fresh,
    output reg signed [31:0] a00,a01,a02,a10,a11,a12,
    output reg signed [31:0] h00,h01,h02,h10,h11,h12,h20,h21,h22,
    output reg signed [15:0] roll_raw,pitch_raw,yaw_raw,
    output reg signed [15:0] delta_roll_raw,delta_pitch_raw,delta_yaw_raw
);
    wire sample_valid,ever_received;
    wire signed [15:0] sample_roll,sample_pitch,sample_yaw;
    wit_pose_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.STALE_MS(STALE_MS)) receiver(
        .clk(clk),.rst_n(rst_n),.sensor_rx(sensor_rx),
        .frame_valid(sample_valid),.roll_raw(sample_roll),
        .pitch_raw(sample_pitch),.yaw_raw(sample_yaw),
        .received_latched(ever_received),.fresh(fresh));

    reg key_meta,key_sync,key_stable,key_press,key_armed;
    reg [31:0] key_count;
    always @(posedge clk) begin
        if(!rst_n) begin
            key_meta<=1;key_sync<=1;key_stable<=1;key_press<=0;key_armed<=0;key_count<=0;
        end else begin
            key_meta<=key1_n;key_sync<=key_meta;key_press<=0;
            if(key_sync==key_stable) begin
                key_count<=0;
                // KEY1 is active-low. A board that powers up with the key
                // released must be armed without requiring a dummy press.
                // If it powers up pressed, key_armed stays low until a
                // complete released debounce interval is observed.
                if(key_sync) key_armed<=1'b1;
            end
            else if(key_count<CLK_HZ/100-1)key_count<=key_count+1'b1;
            else begin
                key_count<=0;key_stable<=key_sync;
                // SW4/KEY1 is active-low. Do not accept a low level seen
                // during startup as a press. The key must first be observed
                // released (stable high), then a later stable low is valid.
                if(key_sync)key_armed<=1'b1;
                else if(key_armed)key_press<=1'b1;
            end
        end
    end

    reg signed [15:0] latest_roll,latest_pitch,latest_yaw;
    reg signed [15:0] base_roll,base_pitch,base_yaw;
    reg signed [15:0] work_roll,work_pitch,work_yaw;
    reg signed [15:0] work_base_roll,work_base_pitch,work_base_yaw;
    reg calibration_request,dirty,inflight,math_start,was_fresh;
    reg [31:0] stale_hold_count;
    localparam integer STALE_HOLD_CYCLES=(CLK_HZ/1000)*STALE_HOLD_MS;
    // 2026-10-22: 46-count (0.25 deg) dead band copied from the 50K reference.
    // Without it every 100 Hz sample recomputes the matrix and tiny IMU noise
    // keeps nudging the picture instead of letting it settle.
    function moved_enough;
        input signed [15:0] nr,np,ny,ar_,ap_,ay_;
        reg signed [16:0] dr_,dp_,dy_;
        reg [16:0] qr,qp,qy;
        begin
            dr_=$signed({nr[15],nr}) - $signed({ar_[15],ar_});
            dp_=$signed({np[15],np}) - $signed({ap_[15],ap_});
            dy_=$signed({ny[15],ny}) - $signed({ay_[15],ay_});
            qr = dr_[16] ? -dr_ : dr_;
            qp = dp_[16] ? -dp_ : dp_;
            qy = dy_[16] ? -dy_ : dy_;
            moved_enough = (qr>=17'd46)||(qp>=17'd46)||(qy>=17'd46);
        end
    endfunction
    reg [15:0] generation,work_generation;
    wire math_busy,math_done,math_error,math_legal;
    wire signed [31:0] math_a00,math_a01,math_a02,math_a10,math_a11,math_a12;
    wire signed [31:0] math_h00,math_h01,math_h02,math_h10,math_h11,math_h12,math_h20,math_h21,math_h22;
    wire signed [31:0] math_r22,math_trace;
    imu_matrix_affine math(
        .clk(clk),.rst_n(rst_n),.start(math_start),
        .current_roll(work_roll),.current_pitch(work_pitch),.current_yaw(work_yaw),
        .base_roll(work_base_roll),.base_pitch(work_base_pitch),.base_yaw(work_base_yaw),
        .busy(math_busy),.done(math_done),.error(math_error),.legal(math_legal),
        .a00(math_a00),.a01(math_a01),.a02(math_a02),
        .a10(math_a10),.a11(math_a11),.a12(math_a12),
        .h00(math_h00),.h01(math_h01),.h02(math_h02),.h10(math_h10),.h11(math_h11),.h12(math_h12),
        .h20(math_h20),.h21(math_h21),.h22(math_h22),
        .rel_r22(math_r22),.rel_trace(math_trace));

    always @(posedge clk) begin
        if(!rst_n) begin
            enabled<=0;command_valid<=0;calibrated<=0;pose_legal<=0;
            a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
            h00<=65536;h01<=0;h02<=0;h10<=0;h11<=65536;h12<=0;h20<=0;h21<=0;h22<=65536;
            roll_raw<=0;pitch_raw<=0;yaw_raw<=0;
            delta_roll_raw<=0;delta_pitch_raw<=0;delta_yaw_raw<=0;
            latest_roll<=0;latest_pitch<=0;latest_yaw<=0;
            base_roll<=0;base_pitch<=0;base_yaw<=0;
            work_roll<=0;work_pitch<=0;work_yaw<=0;
            work_base_roll<=0;work_base_pitch<=0;work_base_yaw<=0;
            calibration_request<=0;dirty<=0;inflight<=0;math_start<=0;was_fresh<=0;
            generation<=0;work_generation<=0;stale_hold_count<=0;
        end else begin
            command_valid<=0;math_start<=0;was_fresh<=fresh;
            if(fresh) stale_hold_count<=0;
            else if(stale_hold_count<STALE_HOLD_CYCLES) stale_hold_count<=stale_hold_count+1'b1;
            if(math_done)inflight<=0;
            if(sample_valid) begin
                latest_roll<=sample_roll;latest_pitch<=sample_pitch;latest_yaw<=sample_yaw;
            end

            // A fresh key captures the most recent complete sample; an early
            // key remains pending until the first complete checked sample.
            if((key_press && fresh) || (calibration_request && sample_valid)) begin
                base_roll<=sample_valid ? sample_roll : latest_roll;
                base_pitch<=sample_valid ? sample_pitch : latest_pitch;
                base_yaw<=sample_valid ? sample_yaw : latest_yaw;
                roll_raw<=sample_valid ? sample_roll : latest_roll;
                pitch_raw<=sample_valid ? sample_pitch : latest_pitch;
                yaw_raw<=sample_valid ? sample_yaw : latest_yaw;
                delta_roll_raw<=0;delta_pitch_raw<=0;delta_yaw_raw<=0;
                a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                h00<=65536;h01<=0;h02<=0;h10<=0;h11<=65536;h12<=0;h20<=0;h21<=0;h22<=65536;
                calibrated<=1;pose_legal<=1;enabled<=1;command_valid<=1;
                calibration_request<=0;dirty<=0;generation<=generation+1'b1;
            end else if(key_press) begin
                calibration_request<=1;dirty<=0;generation<=generation+1'b1;
                enabled<=0;pose_legal<=0;command_valid<=1;
                a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                h00<=65536;h01<=0;h02<=0;h10<=0;h11<=65536;h12<=0;h20<=0;h21<=0;h22<=65536;
            end else if(!fresh) begin
                // Brief UART gaps are common during movement. Keep the last
                // complete map so a sub-second gap cannot flash the identity
                // image. This hold is legal only after KEY1 calibration;
                // startup must never assert EN before a reference exists.
                dirty<=0;pose_legal<=0;
                if(was_fresh || stale_hold_count==STALE_HOLD_CYCLES-1)
                    command_valid<=1;
                if(calibrated && stale_hold_count<STALE_HOLD_CYCLES-1) begin
                    enabled<=1;
                end else begin
                    enabled<=0;
                    a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                end
            end else if(sample_valid) begin
                generation<=generation+1'b1;
                // 2026-10-22: the dead band must only SKIP the recompute, it
                // must not fall into the "not calibrated" branch below, which
                // forces enabled<=0.  Falling through there made EN blink 1/0
                // on every steady sample and toggled the correction on and off.
                if(calibrated && !calibration_request) begin
                    if(moved_enough(latest_roll,latest_pitch,latest_yaw,work_roll,work_pitch,work_yaw))
                        dirty<=1;
                end else begin
                    // A valid sensor frame alone does not enable correction.
                    // The reference is created only by an armed KEY1 press.
                    enabled<=0;pose_legal<=0;
                    roll_raw<=sample_roll;pitch_raw<=sample_pitch;yaw_raw<=sample_yaw;
                    delta_roll_raw<=0;delta_pitch_raw<=0;delta_yaw_raw<=0;
                end
            end else if(inflight && math_done) begin
                if(work_generation==generation && !dirty && fresh && calibrated && !calibration_request) begin
                    roll_raw<=work_roll;pitch_raw<=work_pitch;yaw_raw<=work_yaw;
                    // Signed 16-bit wrap is only for human-readable diagnostics.
                    delta_roll_raw<=work_roll-base_roll;
                    delta_pitch_raw<=work_pitch-base_pitch;
                    delta_yaw_raw<=work_yaw-base_yaw;
                    command_valid<=1;
                    if(math_legal && !math_error) begin
                        enabled<=1;pose_legal<=1;
                        a00<=math_a00;a01<=math_a01;a02<=math_a02;
                        a10<=math_a10;a11<=math_a11;a12<=math_a12;
                        h00<=math_h00;h01<=math_h01;h02<=math_h02;
                        h10<=math_h10;h11<=math_h11;h12<=math_h12;
                        h20<=math_h20;h21<=math_h21;h22<=math_h22;
                    end else begin
                        // Keep affine_enable asserted and retain a00..a12.
                        // The previous implementation replaced them with the
                        // identity matrix, which caused the right image to
                        // jump suddenly when r22/trace crossed the limit.
                        enabled<=1;pose_legal<=0;
                    end
                end
            end else if(dirty && fresh && calibrated && !calibration_request && !inflight && !math_busy && !math_start) begin
                work_roll<=latest_roll;work_pitch<=latest_pitch;work_yaw<=latest_yaw;
                work_base_roll<=base_roll;work_base_pitch<=base_pitch;work_base_yaw<=base_yaw;
                work_generation<=generation;math_start<=1;inflight<=1;dirty<=0;
            end
        end
    end
endmodule
`default_nettype wire