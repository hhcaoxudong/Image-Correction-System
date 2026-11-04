`timescale 1ns/1ps
`default_nettype none

// Performance telemetry. One core tick is 0.01 ms (100 kHz).
// diag_bcd is {item7,...,item0}; every item is five BCD digits.
module video_perf_diag #(
    parameter integer CORE_CLK_HZ = 125000000,
    parameter integer SNAPSHOT_TICKS = 10000
)(
    input wire core_clk,
    input wire rst_n,
    input wire capture_arm,
    input wire capture_done,
    input wire process_start,
    input wire process_done,
    input wire publish_ready,
    input wire display_take,
    input wire display_frame_start,
    input wire process_active,
    input wire cap_wait,
    input wire bank_wait,
    input wire queue_wait,
    output reg [159:0] diag_bcd,
    output reg [7:0] diag_valid,
    input wire vout_clk,
    input wire vout_rst_n,
    output reg [159:0] diag_bcd_pixel,
    output reg [7:0] diag_valid_pixel
);
    localparam integer TICK_CYCLES = (CORE_CLK_HZ >= 100000) ? (CORE_CLK_HZ / 100000) : 1;
    localparam integer SNAP_PERIOD = (SNAPSHOT_TICKS > 0) ? SNAPSHOT_TICKS : 10000;
    localparam integer DIV_BITS = (TICK_CYCLES > 1) ? $clog2(TICK_CYCLES) : 1;
    localparam integer SNAP_BITS = (SNAP_PERIOD > 1) ? $clog2(SNAP_PERIOD) : 1;
    localparam [16:0] MAX_COUNT = 17'd99999;

    reg [DIV_BITS-1:0] tick_div;
    reg [SNAP_BITS-1:0] snap_tick_count;
    wire tick_ce = (TICK_CYCLES <= 1) || (tick_div == TICK_CYCLES-1);
    wire snap_tick_ce = (SNAP_PERIOD <= 1) || (snap_tick_count == SNAP_PERIOD-1);

    reg cap_active, pro_active, rdy_active, swp_active;
    reg [16:0] cap_count, pro_count, rdy_count, swp_count;
    reg [16:0] cap_metric, pro_metric, rdy_metric, swp_metric, period_metric;
    reg [7:0] metric_valid;

    reg process_interval_active;
    reg [16:0] period_count;
    reg [16:0] cap_wait_count, bank_wait_count, queue_wait_count;
    reg [16:0] cap_wait_metric, bank_wait_metric, queue_wait_metric;
    // Compatibility aliases retained for existing benches and integration probes.
    reg [16:0] cap_wait_latched, bank_wait_latched, queue_wait_latched;

    reg [16:0] snap_metric [0:7];
    reg [7:0] snap_valid;

    reg [19:0] conv_result [0:7];
    reg [23:0] conv_work;
    reg [19:0] conv_bcd;
    reg [3:0] conv_item;
    reg [4:0] conv_step;
    reg conv_busy, conv_pack;
    reg [19:0] bcd_adjust;
    integer comb_digit;
    integer reset_item;

    reg [159:0] mailbox_bcd;
    reg [7:0] mailbox_valid;
    reg mailbox_req;
    reg mailbox_pending;
    reg [159:0] pending_bcd;
    reg [7:0] pending_valid;
    reg mailbox_ack_meta, mailbox_ack_sync;

    reg [159:0] pixel_bcd_meta, pixel_bcd_sync;
    reg [7:0] pixel_valid_meta, pixel_valid_sync;
    reg pixel_req_meta, pixel_req_sync, pixel_req_seen;
    reg pixel_ack;

    always @* begin
        bcd_adjust = conv_bcd;
        for (comb_digit = 0; comb_digit < 5; comb_digit = comb_digit + 1) begin
            if (conv_bcd[comb_digit*4 +: 4] >= 4'd5)
                bcd_adjust[comb_digit*4 +: 4] = conv_bcd[comb_digit*4 +: 4] + 4'd3;
        end
    end

    always @(posedge core_clk) begin
        if (!rst_n) begin
            tick_div <= 0;
            snap_tick_count <= 0;
            cap_active <= 0; pro_active <= 0; rdy_active <= 0; swp_active <= 0;
            cap_count <= 0; pro_count <= 0; rdy_count <= 0; swp_count <= 0;
            cap_metric <= 0; pro_metric <= 0; rdy_metric <= 0; swp_metric <= 0;
            period_metric <= 0; metric_valid <= 0;
            process_interval_active <= 0; period_count <= 0;
            cap_wait_count <= 0; bank_wait_count <= 0; queue_wait_count <= 0;
            cap_wait_metric <= 0; bank_wait_metric <= 0; queue_wait_metric <= 0;
            cap_wait_latched <= 0; bank_wait_latched <= 0; queue_wait_latched <= 0;
            snap_valid <= 0;
            conv_work <= 0; conv_bcd <= 0; conv_item <= 0; conv_step <= 0;
            conv_busy <= 0; conv_pack <= 0;
            mailbox_bcd <= 0; mailbox_valid <= 0; mailbox_req <= 0;
            mailbox_pending <= 0; pending_bcd <= 0; pending_valid <= 0;
            mailbox_ack_meta <= 0; mailbox_ack_sync <= 0;
            diag_bcd <= 0; diag_valid <= 0;
            for (reset_item = 0; reset_item < 8; reset_item = reset_item + 1) begin
                snap_metric[reset_item] <= 0;
                conv_result[reset_item] <= 0;
            end
        end else begin
            mailbox_ack_meta <= pixel_ack;
            mailbox_ack_sync <= mailbox_ack_meta;

            if (tick_ce) begin
                tick_div <= 0;
                if (cap_active && cap_count < MAX_COUNT) cap_count <= cap_count + 1'b1;
                if (pro_active && pro_count < MAX_COUNT) pro_count <= pro_count + 1'b1;
                if (rdy_active && rdy_count < MAX_COUNT) rdy_count <= rdy_count + 1'b1;
                if (swp_active && swp_count < MAX_COUNT) swp_count <= swp_count + 1'b1;
                if (process_interval_active && period_count < MAX_COUNT)
                    period_count <= period_count + 1'b1;
                if (process_interval_active) begin
                    if (cap_wait && cap_wait_count < MAX_COUNT) cap_wait_count <= cap_wait_count + 1'b1;
                    if (bank_wait && bank_wait_count < MAX_COUNT) bank_wait_count <= bank_wait_count + 1'b1;
                    if (queue_wait && queue_wait_count < MAX_COUNT) queue_wait_count <= queue_wait_count + 1'b1;
                end
            end else if (TICK_CYCLES > 1) begin
                tick_div <= tick_div + 1'b1;
            end

            if (capture_arm) begin
                cap_count <= 0;
                cap_active <= 1;
            end
            if (capture_done && (cap_active || capture_arm)) begin
                cap_metric <= capture_arm ? 0 : cap_count;
                metric_valid[0] <= 1;
                cap_active <= 0;
            end

            if (process_start) begin
                if (process_interval_active) begin
                    period_metric <= period_count;
                    cap_wait_metric <= cap_wait_count;
                    bank_wait_metric <= bank_wait_count;
                    queue_wait_metric <= queue_wait_count;
                    cap_wait_latched <= cap_wait_count;
                    bank_wait_latched <= bank_wait_count;
                    queue_wait_latched <= queue_wait_count;
                    metric_valid[4] <= 1;
                    metric_valid[5] <= 1;
                    metric_valid[6] <= 1;
                    metric_valid[7] <= 1;
                end
                process_interval_active <= 1;
                period_count <= 0;
                cap_wait_count <= 0;
                bank_wait_count <= 0;
                queue_wait_count <= 0;
                pro_count <= 0;
                pro_active <= 1;
            end
            if (process_done && (pro_active || process_start)) begin
                pro_metric <= process_start ? 0 : pro_count;
                metric_valid[1] <= 1;
                pro_active <= 0;
                rdy_count <= 0;
                rdy_active <= 1;
            end

            if (publish_ready && (rdy_active || (process_done && (pro_active || process_start)))) begin
                rdy_metric <= (process_done && !rdy_active) ? 0 : rdy_count;
                metric_valid[2] <= 1;
                rdy_active <= 0;
                swp_count <= 0;
                swp_active <= 1;
            end
            if (display_take && (swp_active || (publish_ready && (rdy_active || (process_done && (pro_active || process_start)))))) begin
                swp_metric <= (publish_ready && !swp_active) ? 0 : swp_count;
                metric_valid[3] <= 1;
                swp_active <= 0;
            end

            if (tick_ce) begin
            if (snap_tick_ce) begin
                snap_tick_count <= 0;
                if (!conv_busy && !conv_pack) begin
                    snap_metric[0] <= (cap_metric > MAX_COUNT) ? MAX_COUNT : cap_metric;
                    snap_metric[1] <= (pro_metric > MAX_COUNT) ? MAX_COUNT : pro_metric;
                    snap_metric[2] <= (rdy_metric > MAX_COUNT) ? MAX_COUNT : rdy_metric;
                    snap_metric[3] <= (swp_metric > MAX_COUNT) ? MAX_COUNT : swp_metric;
                    snap_metric[4] <= (period_metric > MAX_COUNT) ? MAX_COUNT : period_metric;
                    snap_metric[5] <= (cap_wait_latched > MAX_COUNT) ? MAX_COUNT : cap_wait_latched;
                    snap_metric[6] <= (bank_wait_latched > MAX_COUNT) ? MAX_COUNT : bank_wait_latched;
                    snap_metric[7] <= (queue_wait_latched > MAX_COUNT) ? MAX_COUNT : queue_wait_latched;
                    snap_valid <= metric_valid;
                    conv_item <= 0;
                    conv_step <= 0;
                    conv_bcd <= 0;
                    conv_work <= {7'd0, ((cap_metric > MAX_COUNT) ? MAX_COUNT : cap_metric)};
                    conv_busy <= 1;
                end
            end else if (SNAP_PERIOD > 1) begin
                snap_tick_count <= snap_tick_count + 1'b1;
            end
            end

            if (conv_busy) begin
                conv_bcd <= {bcd_adjust[18:0], conv_work[23]};
                conv_work <= {conv_work[22:0], 1'b0};
                conv_step <= conv_step + 1'b1;
                if (conv_step == 5'd23) begin
                    conv_result[conv_item] <= {bcd_adjust[18:0], conv_work[23]};
                    if (conv_item == 4'd7) begin
                        conv_busy <= 0;
                        conv_pack <= 1;
                    end else begin
                        conv_item <= conv_item + 1'b1;
                        conv_step <= 0;
                        conv_bcd <= 0;
                        conv_work <= {7'd0, snap_metric[conv_item + 1'b1]};
                    end
                end
            end else if (conv_pack) begin
                diag_bcd <= {conv_result[7],conv_result[6],conv_result[5],conv_result[4],
                             conv_result[3],conv_result[2],conv_result[1],conv_result[0]};
                diag_valid <= snap_valid;
                pending_bcd <= {conv_result[7],conv_result[6],conv_result[5],conv_result[4],
                                conv_result[3],conv_result[2],conv_result[1],conv_result[0]};
                pending_valid <= snap_valid;
                mailbox_pending <= 1;
                conv_pack <= 0;
            end

            if (mailbox_pending && (mailbox_ack_sync == mailbox_req) && !conv_pack) begin
                mailbox_bcd <= pending_bcd;
                mailbox_valid <= pending_valid;
                mailbox_req <= ~mailbox_req;
                mailbox_pending <= 0;
            end
        end
    end

    always @(posedge vout_clk) begin
        if (!vout_rst_n) begin
            pixel_bcd_meta <= 0; pixel_bcd_sync <= 0;
            pixel_valid_meta <= 0; pixel_valid_sync <= 0;
            pixel_req_meta <= 0; pixel_req_sync <= 0; pixel_req_seen <= 0;
            pixel_ack <= 0;
            diag_bcd_pixel <= 0; diag_valid_pixel <= 0;
        end else begin
            pixel_bcd_meta <= mailbox_bcd;
            pixel_bcd_sync <= pixel_bcd_meta;
            pixel_valid_meta <= mailbox_valid;
            pixel_valid_sync <= pixel_valid_meta;
            pixel_req_meta <= mailbox_req;
            pixel_req_sync <= pixel_req_meta;
            if (pixel_req_sync != pixel_req_seen) begin
                diag_bcd_pixel <= pixel_bcd_sync;
                diag_valid_pixel <= pixel_valid_sync;
                pixel_req_seen <= pixel_req_sync;
                pixel_ack <= pixel_req_sync;
            end
        end
    end
endmodule
`default_nettype wire
