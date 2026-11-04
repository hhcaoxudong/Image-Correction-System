`timescale 1ns/1ps
`default_nettype none

// 以太网同帧采集：GMII 级核心（仿真直接用它）+ RGMII 顶层封装。
// 板端：IP 192.168.1.20，MAC 02:00:00:00:00:01，UDP 6002；应答 ARP 与 ping。
module eth_core #(
    parameter [47:0] MY_MAC = 48'h02_00_00_00_00_01,
    parameter [31:0] MY_IP  = 32'hC0A80114,
    parameter [15:0] MY_PORT = 16'd6002,
    parameter integer IMG_W = 1920, IMG_H = 1080,
    parameter integer PACE_US_DEFAULT = 15,
    parameter integer SETTLE_CYC = 1024,
    parameter integer FREEZE_TIMEOUT_CYC = 250000000,
    parameter integer HOLD_TIMEOUT_CYC = 1250000000
)(
    input  wire clk, rst_n,             // core_clk 域
    input  wire rx_clk, rx_rst_n,       // RGMII 接收时钟域
    input  wire gmii_rx_dv, input wire [7:0] gmii_rxd,
    output wire gmii_tx_en, output wire [7:0] gmii_txd,
    output wire freeze_req, input wire frozen,
    input  wire [27:0] frozen_raw_base, frozen_res_base,
    input  wire [2:0] frozen_gamma_curve,
    output wire [7:0] coord_read_addr, input wire [63:0] coord_read_data,
    input wire coord_fisheye,
    input wire [383:0] coord_calibration, input wire [191:0] coord_affine,
    input wire coord_affine_enabled, coord_complete,
    input wire [7:0] coord_sample_count, coord_generation, input wire [3:0] coord_gamma,
    output wire ext_own,
    output wire [27:0] ext_addr, output wire [3:0] ext_len, output wire ext_valid,
    input  wire ext_ready,
    input  wire [255:0] ext_rdata, input wire ext_rvalid, ext_rlast,
    output wire [15:0] dbg_rx_frames, dbg_rx_fcs_bad, dbg_rx_cmd
);
    localparam integer NPKT = IMG_W * IMG_H / 384;
    wire arp_req, arp_ack, cmd_req, cmd_ack, icmp_req, icmp_ack;
    wire [47:0] arp_mac, cmd_mac, icmp_mac;
    wire [31:0] arp_ip, cmd_ip, icmp_ip, cmd_seq;
    wire [15:0] cmd_port, icmp_csum, icmp_id, icmp_seqn;
    wire [7:0]  cmd_code, cmd_arg0, cmd_arg1;
    wire [255:0] cmd_ranges;
    wire [6:0]  icmp_len;
    wire [511:0] icmp_data;
    eth_rx_parser #(.MY_MAC(MY_MAC),.MY_IP(MY_IP),.MY_PORT(MY_PORT)) rx (
        .clk(rx_clk),.rst_n(rx_rst_n),.gmii_rx_dv(gmii_rx_dv),.gmii_rxd(gmii_rxd),
        .arp_req(arp_req),.arp_ack(arp_ack),.arp_mac(arp_mac),.arp_ip(arp_ip),
        .cmd_req(cmd_req),.cmd_ack(cmd_ack),.cmd_mac(cmd_mac),.cmd_ip(cmd_ip),.cmd_port(cmd_port),
        .cmd_code(cmd_code),.cmd_seq(cmd_seq),.cmd_arg0(cmd_arg0),.cmd_arg1(cmd_arg1),.cmd_ranges(cmd_ranges),
        .icmp_req(icmp_req),.icmp_ack(icmp_ack),.icmp_mac(icmp_mac),.icmp_ip(icmp_ip),
        .icmp_csum(icmp_csum),.icmp_id(icmp_id),.icmp_seqn(icmp_seqn),.icmp_len(icmp_len),.icmp_data(icmp_data),
        .cnt_frames(dbg_rx_frames),.cnt_fcs_bad(dbg_rx_fcs_bad),.cnt_cmd(dbg_rx_cmd));
    wire tx_start, tx_busy, tx_done, pay_adv;
    wire [1:0] tx_kind;
    wire [47:0] tx_mac; wire [31:0] tx_ip; wire [15:0] tx_port;
    wire [31:0] st_seq, st_frame, im_frame, im_crc;
    wire [7:0] st_flags, st_cmd, im_ch, pay_byte;
    wire [15:0] st_fcs_bad, im_tag, im_idx;
    eth_capture_ctrl #(.NPKT(NPKT),.PACE_US_DEFAULT(PACE_US_DEFAULT),.SETTLE_CYC(SETTLE_CYC),
        .FREEZE_TIMEOUT_CYC(FREEZE_TIMEOUT_CYC),.HOLD_TIMEOUT_CYC(HOLD_TIMEOUT_CYC)) ctrl (
        .clk(clk),.rst_n(rst_n),
        .arp_req(arp_req),.arp_ack(arp_ack),.arp_mac(arp_mac),.arp_ip(arp_ip),
        .icmp_req(icmp_req),.icmp_ack(icmp_ack),.icmp_mac(icmp_mac),.icmp_ip(icmp_ip),
        .cmd_req(cmd_req),.cmd_ack(cmd_ack),.cmd_mac(cmd_mac),.cmd_ip(cmd_ip),.cmd_port(cmd_port),
        .cmd_code(cmd_code),.cmd_seq(cmd_seq),.cmd_arg0(cmd_arg0),.cmd_arg1(cmd_arg1),.cmd_ranges(cmd_ranges),
        .fcs_bad_rx(dbg_rx_fcs_bad),
        .tx_start(tx_start),.tx_kind(tx_kind),.tx_busy(tx_busy),.tx_done(tx_done),
        .tx_mac(tx_mac),.tx_ip(tx_ip),.tx_port(tx_port),
        .st_seq(st_seq),.st_flags(st_flags),.st_cmd(st_cmd),.st_fcs_bad(st_fcs_bad),.st_frame(st_frame),
        .im_ch(im_ch),.im_tag(im_tag),.im_frame(im_frame),.im_idx(im_idx),.im_crc(im_crc),
        .pay_byte(pay_byte),.pay_adv(pay_adv),
        .freeze_req(freeze_req),.frozen(frozen),.frozen_raw_base(frozen_raw_base),.frozen_res_base(frozen_res_base),.frozen_gamma_curve(frozen_gamma_curve),
        .coord_read_addr(coord_read_addr),.coord_read_data(coord_read_data),
        .coord_fisheye(coord_fisheye),.coord_calibration(coord_calibration),.coord_affine(coord_affine),
        .coord_affine_enabled(coord_affine_enabled),.coord_complete(coord_complete),
        .coord_sample_count(coord_sample_count),.coord_generation(coord_generation),.coord_gamma(coord_gamma),
        .ext_own(ext_own),.ext_addr(ext_addr),.ext_len(ext_len),.ext_valid(ext_valid),.ext_ready(ext_ready),
        .ext_rdata(ext_rdata),.ext_rvalid(ext_rvalid),.ext_rlast(ext_rlast));
    eth_tx_engine #(.MY_MAC(MY_MAC),.MY_IP(MY_IP),.MY_PORT(MY_PORT),
        .IMG_W(IMG_W[15:0]),.IMG_H(IMG_H[15:0]),.NPKT(NPKT[15:0])) tx (
        .clk(clk),.rst_n(rst_n),.start(tx_start),.kind(tx_kind),.busy(tx_busy),.done(tx_done),
        .dst_mac(tx_mac),.dst_ip(tx_ip),.dst_port(tx_port),
        .icmp_csum(icmp_csum),.icmp_id(icmp_id),.icmp_seqn(icmp_seqn),.icmp_len(icmp_len),.icmp_data(icmp_data),
        .st_seq(st_seq),.st_flags(st_flags),.st_cmd(st_cmd),.st_fcs_bad(st_fcs_bad),.st_frame(st_frame),
        .im_ch(im_ch),.im_tag(im_tag),.im_frame(im_frame),.im_idx(im_idx),.im_crc(im_crc),
        .pay_byte(pay_byte),.pay_adv(pay_adv),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en));
endmodule

module eth_top (
    input  wire clk, rst_n,
    input  wire rgmii_rxc, rgmii_rx_ctl, input wire [3:0] rgmii_rxd,
    output wire rgmii_txc, rgmii_tx_ctl, output wire [3:0] rgmii_txd,
    output wire freeze_req, input wire frozen,
    input  wire [27:0] frozen_raw_base, frozen_res_base,
    input  wire [2:0] frozen_gamma_curve,
    output wire [7:0] coord_read_addr, input wire [63:0] coord_read_data,
    input wire coord_fisheye,
    input wire [383:0] coord_calibration, input wire [191:0] coord_affine,
    input wire coord_affine_enabled, coord_complete,
    input wire [7:0] coord_sample_count, coord_generation, input wire [3:0] coord_gamma,
    output wire ext_own,
    output wire [27:0] ext_addr, output wire [3:0] ext_len, output wire ext_valid,
    input  wire ext_ready,
    input  wire [255:0] ext_rdata, input wire ext_rvalid, ext_rlast,
    output wire [15:0] dbg_rx_frames, dbg_rx_fcs_bad, dbg_rx_cmd
);
    wire rx_clk, rx_dv; wire [7:0] rxd;
    wire tx_en; wire [7:0] txd;
    reg [3:0] rx_rst_sr;
    always @(posedge rx_clk or negedge rst_n)
        if (!rst_n) rx_rst_sr <= 4'd0; else rx_rst_sr <= {rx_rst_sr[2:0], 1'b1};
    eth_rgmii_rx phy_rx (.rgmii_rxc(rgmii_rxc),.rgmii_rx_ctl(rgmii_rx_ctl),.rgmii_rxd(rgmii_rxd),
        .rx_clk(rx_clk),.gmii_rx_dv(rx_dv),.gmii_rxd(rxd));
    eth_rgmii_tx phy_tx (.tx_clk(clk),.rst_n(rst_n),.gmii_txd(txd),.gmii_tx_en(tx_en),
        .rgmii_txc(rgmii_txc),.rgmii_tx_ctl(rgmii_tx_ctl),.rgmii_txd(rgmii_txd));
    eth_core core (.clk(clk),.rst_n(rst_n),.rx_clk(rx_clk),.rx_rst_n(rx_rst_sr[3]),
        .gmii_rx_dv(rx_dv),.gmii_rxd(rxd),.gmii_tx_en(tx_en),.gmii_txd(txd),
        .freeze_req(freeze_req),.frozen(frozen),.frozen_raw_base(frozen_raw_base),.frozen_res_base(frozen_res_base),.frozen_gamma_curve(frozen_gamma_curve),
        .coord_read_addr(coord_read_addr),.coord_read_data(coord_read_data),
        .coord_fisheye(coord_fisheye),.coord_calibration(coord_calibration),.coord_affine(coord_affine),
        .coord_affine_enabled(coord_affine_enabled),.coord_complete(coord_complete),
        .coord_sample_count(coord_sample_count),.coord_generation(coord_generation),.coord_gamma(coord_gamma),
        .ext_own(ext_own),.ext_addr(ext_addr),.ext_len(ext_len),.ext_valid(ext_valid),.ext_ready(ext_ready),
        .ext_rdata(ext_rdata),.ext_rvalid(ext_rvalid),.ext_rlast(ext_rlast),
        .dbg_rx_frames(dbg_rx_frames),.dbg_rx_fcs_bad(dbg_rx_fcs_bad),.dbg_rx_cmd(dbg_rx_cmd));
endmodule
`default_nettype wire
