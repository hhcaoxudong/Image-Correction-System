`timescale 1ns/1ps
`default_nettype none

// Active-low KEY0 toggle. In V1.3.2.7, 0 enables Gamma and 1 bypasses it.
// The toggle bit is later synchronized into the DDR processing clock domain.
module image_mode_key #(
    parameter integer DEBOUNCE_CYCLES=100000  // 10 ms at the 10 MHz cfg clock
)(
    input wire clk,
    input wire rst_n,
    input wire key_n,
    output reg show_original
);
    localparam integer COUNT_WIDTH = 20;
    reg key_meta, key_sync, key_stable;
    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk) begin
        if (!rst_n) begin
            key_meta<=1'b1; key_sync<=1'b1; key_stable<=1'b1;
            count<=0; show_original<=1'b0;
        end else begin
            key_meta<=key_n;
            key_sync<=key_meta;
            if (key_sync==key_stable) begin
                count<=0;
            end else if (count<DEBOUNCE_CYCLES-1) begin
                count<=count+1'b1;
            end else begin
                key_stable<=key_sync;
                count<=0;
                if (!key_sync) show_original<=~show_original;
            end
        end
    end
endmodule
`default_nettype wire
