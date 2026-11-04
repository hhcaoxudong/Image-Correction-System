`timescale 1ns / 1ps
`default_nettype none

// Measures elapsed time across eight actual frame publication intervals.  The
// counter is intentionally placed in the low-speed configuration clock domain
// so the pixel and DDR timing paths do not contain a one-second divider.
module fps_meter_cfg #(
    parameter integer CLK_HZ = 10000000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       frame_clk,
    input  wire       frame_rst_n,
    input  wire [31:0] frame_count,
    output reg  [7:0] fps_value,
    output reg  [3:0] fps_tens,
    output reg  [3:0] fps_ones,
    output reg [3:0] fps_frac_tens, fps_frac_ones,
    output reg        fps_update
);
    reg [31:0] ticks, idle_ticks, window_frames;
    reg anchor_valid, baseline_valid, dividing;
    reg [63:0] dividend, quotient;
    reg [32:0] remainder;
    reg [31:0] divisor;
    reg [6:0] divide_steps;
    wire [32:0] trial = {remainder[31:0],dividend[63]};
    wire subtract_ok = trial >= {1'b0,divisor};
    wire [63:0] next_quotient = {quotient[62:0],subtract_ok};
    wire [13:0] bounded_result = next_quotient > 64'd9999 ? 14'd9999 : next_quotient[13:0];
    // Frame counter is Gray coded in its own domain; decode it back here.
    // Declared before use: some simulators reject a forward reference.
    reg [31:0] count_gray;
    reg [31:0] gray_meta, gray_sync, count_prev;
    reg ready_meta, ready_sync;
    wire [31:0] count_sync;
    wire [31:0] frame_delta = count_sync - count_prev;
    wire [31:0] elapsed_ticks = ticks + 1'b1;
    wire [31:0] measured_frames = window_frames + frame_delta;
    localparam [63:0] RATE_SCALE = CLK_HZ * 64'd100;
    genvar bit_index;
    generate for (bit_index=0; bit_index<32; bit_index=bit_index+1) begin: decode_gray
        assign count_sync[bit_index] = ^gray_sync[31:bit_index];
    end endgenerate

    // Register Gray code in the counter's own domain before crossing clocks.
    always @(posedge frame_clk) begin
        if (!frame_rst_n) count_gray <= 0;
        else count_gray <= frame_count ^ (frame_count >> 1);
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            ready_meta <= 0; ready_sync <= 0;
            gray_meta <= 0; gray_sync <= 0;
        end else begin
            ready_meta <= frame_rst_n; ready_sync <= ready_meta;
            gray_meta <= count_gray; gray_sync <= gray_meta;
        end
    end

    // Divide sequentially in the configuration clock domain. Eight intervals
    // average publication jitter; one second without a new frame reports zero.
    always @(posedge clk) begin
        if (!rst_n) begin
            ticks<=0; idle_ticks<=0; window_frames<=0; count_prev<=0;
            anchor_valid<=0; baseline_valid<=0; dividing<=0;
            dividend<=0; quotient<=0; remainder<=0; divisor<=1; divide_steps<=0;
            fps_value<=0; fps_tens<=0; fps_ones<=0;
            fps_frac_tens<=0; fps_frac_ones<=0; fps_update<=0;
        end else if (!ready_sync) begin
            ticks<=0; idle_ticks<=0; window_frames<=0; count_prev<=0;
            anchor_valid<=0; baseline_valid<=0; dividing<=0;
            dividend<=0; quotient<=0; remainder<=0; divisor<=1; divide_steps<=0;
            fps_value<=0; fps_tens<=0; fps_ones<=0;
            fps_frac_tens<=0; fps_frac_ones<=0;
            // A source reset must send one zero-value update to the pixel domain.
            // Clearing the toggle can silently retain stale overlay digits.
            if (baseline_valid || anchor_valid || dividing) fps_update<=~fps_update;
        end else if (!baseline_valid) begin
            count_prev<=count_sync; baseline_valid<=1;
        end else begin
            count_prev<=count_sync;
            if (anchor_valid) ticks<=ticks+1'b1;
            if (idle_ticks < CLK_HZ) idle_ticks<=idle_ticks+1'b1;
            if (frame_delta!=0) begin
                idle_ticks<=0;
                if (!anchor_valid) begin
                    anchor_valid<=1; ticks<=0; window_frames<=0;
                end else if (measured_frames>=8 && !dividing) begin
                    dividend<=RATE_SCALE*measured_frames+{32'd0,elapsed_ticks}/2;
                    divisor<=elapsed_ticks; quotient<=0; remainder<=0;
                    divide_steps<=64; dividing<=1; ticks<=0; window_frames<=0;
                end else window_frames<=measured_frames;
            end else if (idle_ticks==CLK_HZ-1) begin
                anchor_valid<=0; ticks<=0; window_frames<=0; dividing<=0;
                fps_value<=0; fps_tens<=0; fps_ones<=0;
                fps_frac_tens<=0; fps_frac_ones<=0; fps_update<=~fps_update;
            end
            if (dividing && !(frame_delta==0 && idle_ticks==CLK_HZ-1)) begin
                dividend<={dividend[62:0],1'b0}; quotient<=next_quotient;
                remainder<=subtract_ok ? trial-{1'b0,divisor} : trial;
                divide_steps<=divide_steps-1'b1;
                if (divide_steps==1) begin
                    dividing<=0;
                    fps_value<=bounded_result/100;
                    fps_tens<=bounded_result/1000;
                    fps_ones<=(bounded_result/100)%10;
                    fps_frac_tens<=(bounded_result/10)%10;
                    fps_frac_ones<=bounded_result%10;
                    fps_update<=~fps_update;
                end
            end
        end
    end
endmodule
`default_nettype wire
