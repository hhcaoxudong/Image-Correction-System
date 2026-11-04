`timescale 1ns/1ps
`default_nettype none

// RGMII 收发物理层，按盘古 100K/200K MINI 官方 2_11_ethernet_test（rgmii_interface.v）的
// 已验证方案：RXC 只进 GTP_CLKBUFG（不经 PLL）；RX_CTL/RXD 各经 IODELAY 247 档（约 2.5 ns）
// 再进 IDDR；TXC 由 ODDR 产生并经 IODELAY 100 档（约 1 ns）。
// RX 不输出 RGMII 错误位：帧好坏由上层计算 FCS 判定（旧工程用错误位做门槛，帧尾必误判一次）。
module eth_rgmii_rx (
    input  wire rgmii_rxc,
    input  wire rgmii_rx_ctl,
    input  wire [3:0] rgmii_rxd,
    output wire rx_clk,
    output reg  gmii_rx_dv,
    output reg  [7:0] gmii_rxd
);
    localparam [7:0] DELAY_STEP = 8'd247;                    // 0~247，10 ps/档
    wire [7:0] delay_gray = (DELAY_STEP >> 1) ^ DELAY_STEP;  // IODELAY 只接受格雷码

    GTP_CLKBUFG rxc_bufg (.CLKIN(rgmii_rxc), .CLKOUT(rx_clk));

    wire ctl_ibuf, ctl_dly, ctl_q0, ctl_q1;
    GTP_INBUF #(.IOSTANDARD("LVCMOS33"), .TERM_DDR()) ctl_inbuf (.O(ctl_ibuf), .I(rgmii_rx_ctl));
    GTP_IODELAY_E2 #(.DELAY_STEP_VALUE(8'hE6), .DELAY_STEP_SEL("PORT"), .TDELAY_EN("FALSE")) ctl_delay (
        .DELAY_STEP(delay_gray), .DO(ctl_dly), .DELAY_SEL(1'b1), .DI(ctl_ibuf), .EN_N(1'b0));
    GTP_IDDR_E1 #(.GRS_EN("TRUE"), .IDDR_MODE("SAME_PIPELINED"), .RS_TYPE("SYNC_RESET")) ctl_iddr (
        .Q0(ctl_q0), .Q1(ctl_q1), .CE(1'b1), .CLK(rx_clk), .D(ctl_dly), .RS(1'b0));

    wire [3:0] d_ibuf, d_dly, d_q0, d_q1;
    genvar j;
    generate for (j=0; j<4; j=j+1) begin : rxd_lane
        GTP_INBUF #(.IOSTANDARD("LVCMOS33"), .TERM_DDR()) d_inbuf (.O(d_ibuf[j]), .I(rgmii_rxd[j]));
        GTP_IODELAY_E2 #(.DELAY_STEP_VALUE(8'hE6), .DELAY_STEP_SEL("PORT"), .TDELAY_EN("FALSE")) d_delay (
            .DELAY_STEP(delay_gray), .DO(d_dly[j]), .DELAY_SEL(1'b1), .DI(d_ibuf[j]), .EN_N(1'b0));
        GTP_IDDR_E1 #(.GRS_EN("TRUE"), .IDDR_MODE("SAME_PIPELINED"), .RS_TYPE("ASYNC_RESET")) d_iddr (
            .Q0(d_q0[j]), .Q1(d_q1[j]), .CE(1'b1), .CLK(rx_clk), .D(d_dly[j]), .RS(1'b0));
    end endgenerate

    always @(posedge rx_clk) begin
        gmii_rx_dv <= ctl_q0;
        gmii_rxd   <= {d_q1, d_q0};
    end
endmodule

module eth_rgmii_tx (
    input  wire tx_clk,
    input  wire rst_n,
    input  wire [7:0] gmii_txd,
    input  wire gmii_tx_en,
    output wire rgmii_txc,
    output wire rgmii_tx_ctl,
    output wire [3:0] rgmii_txd
);
    localparam [7:0] TXC_STEP = 8'd100;                      // 约 1 ns，与官方 demo 相同
    wire [7:0] txc_gray = (TXC_STEP >> 1) ^ TXC_STEP;
    wire [7:0] d  = rst_n ? gmii_txd : 8'd0;
    wire       en = rst_n && gmii_tx_en;
    wire [3:0] d_ddr; wire ctl_ddr, txc_ddr, txc_dly;
    genvar i;
    generate for (i=0; i<4; i=i+1) begin : txd_lane
        GTP_ODDR_E1 #(.GRS_EN("TRUE"), .ODDR_MODE("SAME_EDGE"), .RS_TYPE("ASYNC_RESET")) d_oddr (
            .Q(d_ddr[i]), .CE(1'b1), .CLK(tx_clk), .D0(d[i]), .D1(d[i+4]), .RS(1'b0));
        GTP_OUTBUF #(.IOSTANDARD("LVCMOS33"), .SLEW_RATE("FAST"), .DRIVE_STRENGTH("4")) d_obuf (
            .I(d_ddr[i]), .O(rgmii_txd[i]));
    end endgenerate
    GTP_ODDR_E1 #(.GRS_EN("TRUE"), .ODDR_MODE("SAME_EDGE"), .RS_TYPE("ASYNC_RESET")) ctl_oddr (
        .Q(ctl_ddr), .CE(1'b1), .CLK(tx_clk), .D0(en), .D1(en), .RS(1'b0));
    GTP_OUTBUF #(.IOSTANDARD("LVCMOS33"), .SLEW_RATE("FAST"), .DRIVE_STRENGTH("4")) ctl_obuf (
        .I(ctl_ddr), .O(rgmii_tx_ctl));
    GTP_ODDR_E1 #(.GRS_EN("TRUE"), .ODDR_MODE("SAME_EDGE"), .RS_TYPE("ASYNC_RESET")) txc_oddr (
        .Q(txc_ddr), .CE(1'b1), .CLK(tx_clk), .D0(1'b1), .D1(1'b0), .RS(1'b0));
    GTP_IODELAY_E2 #(.DELAY_STEP_SEL("PORT"), .DELAY_STEP_VALUE()) txc_delay (
        .DI(txc_ddr), .DELAY_SEL(1'b1), .DELAY_STEP(txc_gray), .DO(txc_dly), .EN_N(1'b0));
    GTP_OUTBUF #(.IOSTANDARD("LVCMOS33"), .SLEW_RATE("FAST"), .DRIVE_STRENGTH("4")) txc_obuf (
        .I(txc_dly), .O(rgmii_txc));
endmodule
`default_nettype wire
