`timescale 1ns/1ps
`default_nettype none

module imu_dynamic_affine #(
    parameter integer CLK_HZ = 100000000,
    parameter integer UPDATE_PERIOD_MS = 100,
    parameter integer DEADBAND_RAW = 46
)(
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire imu_alive,
    input wire signed [15:0] roll_raw,
    input wire signed [15:0] pitch_raw,
    input wire signed [15:0] yaw_raw,
    input wire [1:0] model,
    input wire signed [31:0] fx_q16,
    input wire signed [31:0] fy_q16,
    input wire zero_key_n,
    input wire commit_ready,
    input wire zero_request,
    output reg affine_enable,
    output reg signed [31:0] a00,a01,a02,a10,a11,a12,
    output reg affine_rebuild,
    output reg calibrated,
    output reg pose_legal,
    output wire signed [15:0] latest_roll_raw,latest_pitch_raw,latest_yaw_raw,
    output wire signed [15:0] base_roll_raw,base_pitch_raw,base_yaw_raw
);
    localparam signed [31:0] ONE_Q16 = 32'sd65536;
    localparam integer DEBOUNCE_CYCLES = CLK_HZ / 100;
    localparam integer UPDATE_CYCLES = (CLK_HZ / 1000) * UPDATE_PERIOD_MS;
    localparam [31:0] UPDATE_LIMIT = (UPDATE_CYCLES > 0) ? UPDATE_CYCLES : 1;

    reg key_meta,key_sync,key_stable,key_press;
    reg [31:0] key_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta<=1; key_sync<=1; key_stable<=1; key_press<=0; key_count<=0;
        end else begin
            key_meta<=zero_key_n; key_sync<=key_meta; key_press<=0;
            if (key_sync==key_stable) key_count<=0;
            else if (key_count<DEBOUNCE_CYCLES-1) key_count<=key_count+1'b1;
            else begin
                key_count<=0; key_stable<=key_sync;
                if (!key_sync) key_press<=1;
            end
        end
    end

    function pose_changed;
        input signed [15:0] r0,p0,y0,r1,p1,y1;
        reg signed [16:0] dr,dp,dy;
        reg [16:0] ar,ap,ay;
        begin
            dr=$signed({r0[15],r0})-$signed({r1[15],r1});
            dp=$signed({p0[15],p0})-$signed({p1[15],p1});
            dy=$signed({y0[15],y0})-$signed({y1[15],y1});
            ar=dr[16] ? -dr : dr;
            ap=dp[16] ? -dp : dp;
            ay=dy[16] ? -dy : dy;
            pose_changed=(ar>=DEADBAND_RAW)||(ap>=DEADBAND_RAW)||(ay>=DEADBAND_RAW);
        end
    endfunction

    reg signed [15:0] latest_roll,latest_pitch,latest_yaw;
    reg signed [15:0] base_roll,base_pitch,base_yaw;
    reg signed [15:0] applied_roll,applied_pitch,applied_yaw;
    reg signed [15:0] work_roll,work_pitch,work_yaw;
    reg [1:0] applied_model,work_model;
    reg signed [31:0] applied_fx,applied_fy,work_fx,work_fy;
    reg signed [31:0] candidate[0:5];
    reg [31:0] update_age;
    reg start_pending,math_start,candidate_pending,was_alive;
    reg candidate_enable;
    reg key_press_d;
    reg [15:0] reference_epoch,math_epoch;
    wire update_due=(update_age>=UPDATE_LIMIT);
    wire math_busy,math_done,math_error,math_legal;
    wire signed [31:0] math_a00,math_a01,math_a02,math_a10,math_a11,math_a12;
    wire signed [31:0] math_r22,math_trace;
    wire signed [31:0] work_tilt_limit=(work_model==2'd2)?32'sd50211:32'sd53690;
    // Pose calculations use only the registered latest sample. This removes
    // the high-fanout sample_valid mux from the matrix decision path.
    wire signed [15:0] newest_roll=latest_roll;
    wire signed [15:0] newest_pitch=latest_pitch;
    wire signed [15:0] newest_yaw=latest_yaw;

    imu_matrix_affine math(
        .clk(clk),.rst_n(rst_n),.start(math_start),.fx_q16(work_fx),.fy_q16(work_fy),.tilt_limit_q16(work_tilt_limit),
        .current_roll(work_roll),.current_pitch(work_pitch),.current_yaw(work_yaw),
        .base_roll(base_roll),.base_pitch(base_pitch),.base_yaw(base_yaw),
        .busy(math_busy),.done(math_done),.error(math_error),.legal(math_legal),
        .a00(math_a00),.a01(math_a01),.a02(math_a02),
        .a10(math_a10),.a11(math_a11),.a12(math_a12),
        .rel_r22(math_r22),.rel_trace(math_trace)
    );

    assign latest_roll_raw=latest_roll;
    assign latest_pitch_raw=latest_pitch;
    assign latest_yaw_raw=latest_yaw;
    assign base_roll_raw=base_roll;
    assign base_pitch_raw=base_pitch;
    assign base_yaw_raw=base_yaw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latest_roll<=0;latest_pitch<=0;latest_yaw<=0;
            base_roll<=0;base_pitch<=0;base_yaw<=0;
            applied_roll<=0;applied_pitch<=0;applied_yaw<=0;
            work_roll<=0;work_pitch<=0;work_yaw<=0;
            applied_model<=2'd1;work_model<=2'd1;
            applied_fx<=0;applied_fy<=0;work_fx<=0;work_fy<=0;
            update_age<=UPDATE_LIMIT;
            start_pending<=0;math_start<=0;candidate_pending<=0;was_alive<=0;
            candidate_enable<=0;
            key_press_d<=0;reference_epoch<=0;math_epoch<=0;
            affine_enable<=0;calibrated<=0;pose_legal<=0;affine_rebuild<=0;
            a00<=ONE_Q16;a01<=0;a02<=0;a10<=0;a11<=ONE_Q16;a12<=0;
            candidate[0]<=ONE_Q16;candidate[1]<=0;candidate[2]<=0;
            candidate[3]<=0;candidate[4]<=ONE_Q16;candidate[5]<=0;
        end else begin
            math_start<=0;
            affine_rebuild<=0;
            was_alive<=imu_alive;
            key_press_d<=key_press || zero_request;
            if (update_age<UPDATE_LIMIT) update_age<=update_age+1'b1;

            if (sample_valid) begin
                latest_roll<=roll_raw;latest_pitch<=pitch_raw;latest_yaw<=yaw_raw;
            end

            // Even first zero capture waits for ownership and a safe map boundary.
            if (key_press_d && imu_alive) begin
                base_roll<=newest_roll;base_pitch<=newest_pitch;base_yaw<=newest_yaw;
                reference_epoch<=reference_epoch+1'b1;
                calibrated<=1;pose_legal<=1;update_age<=UPDATE_LIMIT;
                start_pending<=0;
                begin
                    candidate[0]<=ONE_Q16;candidate[1]<=0;candidate[2]<=0;
                    candidate[3]<=0;candidate[4]<=ONE_Q16;candidate[5]<=0;
                    work_roll<=newest_roll;work_pitch<=newest_pitch;work_yaw<=newest_yaw;
                    work_model<=model;work_fx<=fx_q16;work_fy<=fy_q16;
                    candidate_enable<=1;candidate_pending<=1;
                end
            end else if (!imu_alive && was_alive && calibrated) begin
                reference_epoch<=reference_epoch+1'b1;start_pending<=0;
                candidate[0]<=ONE_Q16;candidate[1]<=0;candidate[2]<=0;
                candidate[3]<=0;candidate[4]<=ONE_Q16;candidate[5]<=0;
                work_roll<=applied_roll;work_pitch<=applied_pitch;work_yaw<=applied_yaw;
                work_model<=applied_model;work_fx<=applied_fx;work_fy<=applied_fy;
                candidate_enable<=0;candidate_pending<=1;pose_legal<=0;
            end

            // At most one new matrix calculation per period. Every launch
            // snapshots the newest IMU sample, coalescing intermediate poses.
            if (!key_press_d && !sample_valid && calibrated && imu_alive && update_due && !math_busy &&
                !start_pending && !candidate_pending &&
                (model!=applied_model || fx_q16!=applied_fx || fy_q16!=applied_fy || pose_changed(newest_roll,newest_pitch,newest_yaw,
                             applied_roll,applied_pitch,applied_yaw))) begin
                work_roll<=newest_roll;work_pitch<=newest_pitch;work_yaw<=newest_yaw;
                work_model<=model;work_fx<=fx_q16;work_fy<=fy_q16;
                start_pending<=1;update_age<=0;
            end

            if (start_pending && !math_busy && !key_press_d && imu_alive) begin
                math_epoch<=reference_epoch;
                math_start<=1;start_pending<=0;
            end

            if (!key_press_d && math_done && imu_alive && math_epoch==reference_epoch) begin
                if (work_model==model && work_fx==fx_q16 && work_fy==fy_q16) begin
                    candidate_pending<=1;
                    if (!math_error && math_legal) begin
                        candidate[0]<=math_a00;candidate[1]<=math_a01;candidate[2]<=math_a02;
                        candidate[3]<=math_a10;candidate[4]<=math_a11;candidate[5]<=math_a12;
                        candidate_enable<=1;pose_legal<=1;
                    end else begin
                        // Do not keep applying an old pose matrix after the
                        // camera leaves this model's validated envelope.
                        candidate[0]<=ONE_Q16;candidate[1]<=0;candidate[2]<=0;
                        candidate[3]<=0;candidate[4]<=ONE_Q16;candidate[5]<=0;
                        candidate_enable<=0;pose_legal<=0;
                    end
                end else begin
                    candidate_pending<=0;pose_legal<=0;
                end
            end

            // Reject a candidate only when its lens configuration changed.
            // Motion after the sample snapshot must not starve the 10 Hz control loop.
            if (!key_press_d && candidate_pending && candidate_enable &&
                (work_model!=model || work_fx!=fx_q16 || work_fy!=fy_q16))
                candidate_pending<=0;
            else if (!key_press_d && candidate_pending && commit_ready && !math_busy && !start_pending) begin
                a00<=candidate[0];a01<=candidate[1];a02<=candidate[2];
                a10<=candidate[3];a11<=candidate[4];a12<=candidate[5];
                applied_roll<=work_roll;applied_pitch<=work_pitch;applied_yaw<=work_yaw;
                applied_model<=work_model;applied_fx<=work_fx;applied_fy<=work_fy;
                affine_enable<=candidate_enable;
                candidate_pending<=0;affine_rebuild<=1;update_age<=0;
            end
        end
    end
endmodule

`default_nettype wire
