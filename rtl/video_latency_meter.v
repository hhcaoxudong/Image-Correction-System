`timescale 1ns/1ps
`default_nettype none

// 帧端到端时间：采集首像素到该结果帧首次输出首像素。
// 公共 Gray 时间戳每 10 us 加一；显示单位为 0.01 ms。
// 两个采集槽和三个结果槽各自保存来源时间戳，重复扫描不重新计时。
module video_latency_meter #(
    parameter integer CORE_CLK_HZ = 125000000,
    parameter integer STALE_TICKS = 100000
)(
    input wire core_clk, core_rst_n,
    input wire vin_clk, vin_first,
    input wire capture_arm, capture_arm_bank, capture_bank,
    input wire process_take, process_source_bank,
    input wire [1:0] process_result_bank,
    input wire display_take,
    input wire [1:0] display_result_bank,
    input wire vout_clk, vout_vs, vout_de,
    output reg [19:0] latency_bcd,
    output reg latency_valid, latency_overflow,
    output reg [31:0] latency_ticks,
    output reg latency_update
);
    localparam integer TICK_CYCLES = CORE_CLK_HZ / 100000;
    localparam integer DIV_BITS = (TICK_CYCLES > 1) ? $clog2(TICK_CYCLES) : 1;
    localparam [DIV_BITS-1:0] DIV_LAST = TICK_CYCLES-1;
    reg [DIV_BITS-1:0] tick_div;
    reg [31:0] time_binary, time_gray;
    wire [31:0] next_time = time_binary + 32'd1;
    always @(posedge core_clk) begin
        if (!core_rst_n) begin
            tick_div <= 0; time_binary <= 0; time_gray <= 0;
        end else if (tick_div == DIV_LAST) begin
            tick_div <= 0;
            time_binary <= next_time;
            time_gray <= next_time ^ (next_time >> 1);
        end else tick_div <= tick_div + 1'b1;
    end

    reg vin_rst_meta=0, vin_rst_n=0, out_rst_meta=0, out_rst_n=0;
    always @(posedge vin_clk) begin
        vin_rst_meta <= core_rst_n;
        vin_rst_n <= vin_rst_meta;
    end
    always @(posedge vout_clk) begin
        out_rst_meta <= core_rst_n;
        out_rst_n <= out_rst_meta;
    end

    reg [31:0] vin_gray_meta, vin_gray_sync;
    wire [31:0] vin_ticks;
    assign vin_ticks[31] = vin_gray_sync[31];
    genvar g;
    generate for (g=0; g<31; g=g+1) begin : decode_input_gray
        assign vin_ticks[g] = ^vin_gray_sync[31:g];
    end endgenerate

    reg [31:0] input_mailbox;
    reg input_request;
    reg input_ack;
    reg [1:0] input_ack_sync;
    always @(posedge vin_clk) begin
        if (!vin_rst_n) begin
            vin_gray_meta <= 0; vin_gray_sync <= 0;
            input_mailbox <= 0; input_request <= 0; input_ack_sync <= 0;
        end else begin
            vin_gray_meta <= time_gray; vin_gray_sync <= vin_gray_meta;
            input_ack_sync <= {input_ack_sync[0],input_ack};
            // 数据保持到 core 确认；实际输入帧间隔远大于握手时间。
            if (vin_first && input_ack_sync[1] == input_request) begin
                input_mailbox <= vin_ticks;
                input_request <= !input_request;
            end
        end
    end

    reg [31:0] input_data_meta, input_data_sync;
    reg [2:0] input_request_sync;
    reg [31:0] capture_time[0:1];
    reg [1:0] capture_valid;
    reg [31:0] result_time[0:2];
    reg [2:0] result_valid;
    reg [32:0] output_mailbox;
    reg output_request;
    reg output_ack;
    reg [1:0] output_ack_sync;
    integer bank;
    always @(posedge core_clk) begin
        if (!core_rst_n) begin
            input_data_meta <= 0; input_data_sync <= 0;
            input_request_sync <= 0; input_ack <= 0;
            capture_time[0] <= 0; capture_time[1] <= 0; capture_valid <= 0;
            for (bank=0; bank<3; bank=bank+1) result_time[bank] <= 0;
            result_valid <= 0;
            output_mailbox <= 0; output_request <= 0; output_ack_sync <= 0;
        end else begin
            input_data_meta <= input_mailbox; input_data_sync <= input_data_meta;
            input_request_sync <= {input_request_sync[1:0],input_request};
            output_ack_sync <= {output_ack_sync[0],output_ack};
            if (input_request_sync[2] != input_ack) begin
                input_ack <= input_request_sync[2];
                capture_time[capture_bank] <= input_data_sync;
                capture_valid[capture_bank] <= 1'b1;
            end
            // 新采集请求先清有效位，丢失时间戳时不能复用上一帧数据。
            if (capture_arm) capture_valid[capture_arm_bank] <= 1'b0;
            if (process_take && process_result_bank < 3) begin
                result_time[process_result_bank] <= capture_time[process_source_bank];
                result_valid[process_result_bank] <= capture_valid[process_source_bank];
            end
            if (display_take && display_result_bank < 3 && output_ack_sync[1] == output_request) begin
                output_mailbox <= {result_valid[display_result_bank],result_time[display_result_bank]};
                output_request <= !output_request;
            end
        end
    end

    reg [31:0] out_gray_meta, out_gray_sync, output_ticks;
    wire [31:0] out_decoded_ticks;
    assign out_decoded_ticks[31] = out_gray_sync[31];
    generate for (g=0; g<31; g=g+1) begin : decode_output_gray
        assign out_decoded_ticks[g] = ^out_gray_sync[31:g];
    end endgenerate
    reg [32:0] output_data_meta, output_data_sync;
    reg [2:0] output_request_sync;
    reg vs_d, first_pixel_seen, pending_valid;
    reg [31:0] pending_origin, last_sample_time, delta_hold;
    reg sample_pending, bcd_busy;
    reg [19:0] bcd_work, bcd_adjusted;
    reg [16:0] binary_work;
    reg [4:0] bcd_step;
    wire [31:0] sample_delta = output_ticks - pending_origin;
    wire [31:0] sample_age = output_ticks - last_sample_time;
    wire output_event = output_request_sync[2] != output_ack;
    wire invalid_metadata = output_event && !output_data_sync[32];
    integer digit;
    // 每拍转换一位，不在像素时钟路径中放除法器。
    always @* begin
        bcd_adjusted = bcd_work;
        for (digit=0; digit<5; digit=digit+1)
            if (bcd_work[digit*4 +: 4] >= 4'd5)
                bcd_adjusted[digit*4 +: 4] = bcd_work[digit*4 +: 4] + 4'd3;
    end
    always @(posedge vout_clk) begin
        if (!out_rst_n) begin
            out_gray_meta <= 0; out_gray_sync <= 0; output_ticks <= 0;
            output_data_meta <= 0; output_data_sync <= 0;
            output_request_sync <= 0; output_ack <= 0;
            vs_d <= 0; first_pixel_seen <= 1; pending_valid <= 0;
            pending_origin <= 0; last_sample_time <= 0; delta_hold <= 0;
            sample_pending <= 0; bcd_busy <= 0;
            bcd_work <= 0; binary_work <= 0; bcd_step <= 0;
            latency_bcd <= 0; latency_valid <= 0; latency_overflow <= 0;
            latency_ticks <= 0; latency_update <= 0;
        end else begin
            out_gray_meta <= time_gray; out_gray_sync <= out_gray_meta;
            output_ticks <= out_decoded_ticks;
            output_data_meta <= output_mailbox; output_data_sync <= output_data_meta;
            output_request_sync <= {output_request_sync[1:0],output_request};
            vs_d <= vout_vs;
            if (latency_valid && sample_age >= STALE_TICKS) latency_valid <= 0;

            if (vout_vs && !vs_d) begin
                first_pixel_seen <= 0;
                pending_valid <= 0;
            end
            if (output_event) begin
                output_ack <= output_request_sync[2];
                pending_origin <= output_data_sync[31:0];
                // 元数据必须在本帧有效图像开始前送达；迟到时不伪造样本。
                pending_valid <= output_data_sync[32] && !first_pixel_seen;
            end
            // 无效新帧必须取消旧转换，不能被同拍完成的旧样本重新置有效。
            if (invalid_metadata) begin
                latency_valid <= 0; latency_overflow <= 0;
                bcd_busy <= 0; sample_pending <= 0; pending_valid <= 0;
            end else begin
            if (vout_de && !first_pixel_seen) begin
                first_pixel_seen <= 1;
                pending_valid <= 0;
                if (pending_valid) begin
                    delta_hold <= sample_delta;
                    last_sample_time <= output_ticks;
                    sample_pending <= 1;
                end
            end
            if (bcd_busy) begin
                bcd_work <= {bcd_adjusted[18:0],binary_work[16]};
                binary_work <= {binary_work[15:0],1'b0};
                bcd_step <= bcd_step + 1'b1;
                if (bcd_step == 5'd16) begin
                    latency_bcd <= {bcd_adjusted[18:0],binary_work[16]};
                    latency_valid <= 1; latency_overflow <= 0;
                    latency_update <= !latency_update; bcd_busy <= 0;
                end
            end
            if (sample_pending) begin
                sample_pending <= 0;
                latency_ticks <= delta_hold;
                if (delta_hold > 32'd99999) begin
                    latency_bcd <= 20'h99999;
                    latency_valid <= 1; latency_overflow <= 1;
                    latency_update <= !latency_update; bcd_busy <= 0;
                end else begin
                    bcd_work <= 0; binary_work <= delta_hold[16:0];
                    bcd_step <= 0; bcd_busy <= 1;
                end
            end
            end
        end
    end
endmodule
`default_nettype wire
