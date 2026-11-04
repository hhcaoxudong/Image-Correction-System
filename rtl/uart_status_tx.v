`timescale 1ns/1ps
`default_nettype none

// PARAM-V1 状态文本："send ok\r\n"（参数帧被接收）、"adjust ok\r\n"（新参数矫正的
// 第一帧已上屏）。事件先挂起，拿到 C24 授权后整行发出。
module uart_status_msg #(
    parameter integer CLK_HZ = 10000000,
    parameter integer BAUD   = 115200
)(
    input  wire clk, rst_n,
    input  wire send_evt,          // 单拍
    input  wire adjust_evt,        // 单拍
    output wire tx_req,
    input  wire tx_gnt,
    output wire line_busy,
    output reg  tx
);
    localparam integer DIV = (CLK_HZ + BAUD/2) / BAUD;
    reg send_pending, adjust_pending, active, kind;   // kind: 0 send ok, 1 adjust ok
    reg [3:0]  idx;
    reg [3:0]  bits;
    reg [15:0] cnt;
    reg [9:0]  shift;
    wire [3:0] last = kind ? 4'd10 : 4'd8;
    function [7:0] ch;
        input reg k; input reg [3:0] i;
        begin
            if (!k) case (i)
                0: ch="s"; 1: ch="e"; 2: ch="n"; 3: ch="d"; 4: ch=" ";
                5: ch="o"; 6: ch="k"; 7: ch=8'h0d; default: ch=8'h0a;
            endcase else case (i)
                0: ch="a"; 1: ch="d"; 2: ch="j"; 3: ch="u"; 4: ch="s"; 5: ch="t";
                6: ch=" "; 7: ch="o"; 8: ch="k"; 9: ch=8'h0d; default: ch=8'h0a;
            endcase
        end
    endfunction
    assign tx_req    = !active && (send_pending || adjust_pending);
    assign line_busy = active || send_pending || adjust_pending;
    always @(posedge clk) begin
        if (!rst_n) begin
            send_pending<=0; adjust_pending<=0; active<=0; kind<=0;
            idx<=0; bits<=0; cnt<=0; shift<=10'h3ff; tx<=1'b1;
        end else begin
            if (send_evt)   send_pending<=1'b1;
            if (adjust_evt) adjust_pending<=1'b1;
            if (!active) begin
                if (tx_gnt && (send_pending || adjust_pending)) begin
                    // send ok 先于 adjust ok：同时挂起时也保持事件发生的先后顺序
                    kind <= !send_pending;
                    if (send_pending) send_pending<=send_evt; else adjust_pending<=adjust_evt;
                    idx<=0; bits<=0; cnt<=0; active<=1'b1;
                    shift<={1'b1, ch(!send_pending, 4'd0), 1'b0}; tx<=1'b0;
                end
            end else if (cnt==DIV-1) begin
                cnt<=0; shift<={1'b1, shift[9:1]};
                if (bits==9) begin
                    bits<=0; tx<=1'b1;
                    if (idx==last) active<=1'b0;
                    else begin
                        idx<=idx+1'b1;
                        shift<={1'b1, ch(kind, idx+4'd1), 1'b0}; tx<=1'b0;
                    end
                end else begin bits<=bits+1'b1; tx<=shift[1]; end
            end else cnt<=cnt+1'b1;
        end
    end
endmodule

// C24 发送仲裁：以"整帧/整行"为单位授权，任何一路开始后一直占用到发完，
// 二进制协议帧和文本行不会交错。优先级：状态文本 > PARAM-V1 回包 > 调试文本。
module uart_tx_arbiter (
    input  wire clk, rst_n,
    input  wire req0, busy0, tx0,   // 状态文本
    input  wire req1, busy1, tx1,   // PARAM-V1 回包
    input  wire req2, busy2, tx2,   // 调试文本
    output wire gnt0, gnt1, gnt2,
    output reg  tx
);
    reg [1:0] owner;                // 0 空闲，1/2/3 对应 0/1/2 路
    assign gnt0 = owner==2'd1;
    assign gnt1 = owner==2'd2;
    assign gnt2 = owner==2'd3;
    always @(posedge clk) begin
        if (!rst_n) begin owner<=2'd0; tx<=1'b1; end
        else begin
            case (owner)
                2'd0: owner <= req0 ? 2'd1 : req1 ? 2'd2 : req2 ? 2'd3 : 2'd0;
                2'd1: if (!busy0) owner<=2'd0;
                2'd2: if (!busy1) owner<=2'd0;
                default: if (!busy2) owner<=2'd0;
            endcase
            tx <= owner==2'd1 ? tx0 : owner==2'd2 ? tx1 : owner==2'd3 ? tx2 : 1'b1;
        end
    end
endmodule
`default_nettype wire
