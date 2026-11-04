`timescale 1ns/1ps
`default_nettype none

// 以太网发送引擎（core_clk 域，GMII 字节流）。四种帧：
//   K_ARP 0  ARP 应答                         60 字节（含填充）
//   K_ICMP 1 ICMP 回显应答（数据 <=64）        max(60, 42+n)
//   K_STAT 2 UDP "FQAR" 状态，18 字节载荷       60
//   K_IMG 3  UDP "FQAP" 图像块，26+1152 字节    1220
// 两级流水：第 1 级按 (帧类型, 字节号) 组合选出字节并寄存；第 2 级出到 GMII 并同步算 FCS。
// 目的地址、状态/图像字段在 start 时锁存，帧内不受上游改动影响。
module eth_tx_engine #(
    parameter [47:0] MY_MAC  = 48'h02_00_00_00_00_01,
    parameter [31:0] MY_IP   = 32'hC0A80114,
    parameter [15:0] MY_PORT = 16'd6002,
    parameter [15:0] IMG_W   = 16'd1920,
    parameter [15:0] IMG_H   = 16'd1080,
    parameter [15:0] NPKT    = 16'd5400
)(
    input  wire clk, rst_n,
    input  wire start,
    input  wire [1:0] kind,
    output wire busy,
    output reg  done,
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,
    // ICMP：来自接收邮箱，回显期间保持不变
    input  wire [15:0] icmp_csum, icmp_id, icmp_seqn,
    input  wire [6:0]  icmp_len,
    input  wire [511:0] icmp_data,
    // FQAR
    input  wire [31:0] st_seq,
    input  wire [7:0]  st_flags, st_cmd,
    input  wire [15:0] st_fcs_bad,
    input  wire [31:0] st_frame,
    // FQAP
    input  wire [7:0]  im_ch,
    input  wire [15:0] im_tag,
    input  wire [31:0] im_frame,
    input  wire [15:0] im_idx,
    input  wire [31:0] im_crc,
    input  wire [7:0]  pay_byte,
    output wire pay_adv,
    output reg  [7:0] gmii_txd,
    output reg  gmii_tx_en
);
    localparam [1:0] K_ARP=0, K_ICMP=1, K_STAT=2, K_IMG=3;
    function [31:0] crc8;
        input reg [31:0] c;
        input reg [7:0]  d;
        integer k; reg [31:0] x;
        begin
            x = c;
            for (k=0; k<8; k=k+1) x = (x >> 1) ^ ((x[0] ^ d[k]) ? 32'hEDB88320 : 32'd0);
            crc8 = x;
        end
    endfunction
    // ---------------- start 时锁存 ----------------
    reg [1:0]  k;
    reg [47:0] d_mac; reg [31:0] d_ip; reg [15:0] d_port;
    reg [31:0] l_seq, l_sframe, l_iframe, l_crc;
    reg [7:0]  l_flags, l_cmd, l_ch;
    reg [15:0] l_fbad, l_tag, l_idx, l_icsum;
    reg [15:0] ip_id;
    reg [10:0] flen;          // 帧长（不含 FCS）
    reg [15:0] ip_tot;
    reg [17:0] ck_acc;        // IP 头校验和累加
    reg [15:0] ip_ck;
    localparam integer FL_IMG = 14 + 20 + 8 + 26 + 1152;   // 1220
    always @(posedge clk) begin
        if (start) begin
            k <= kind; d_mac <= dst_mac; d_ip <= dst_ip; d_port <= dst_port;
            l_seq <= st_seq; l_flags <= st_flags; l_cmd <= st_cmd; l_fbad <= st_fcs_bad; l_sframe <= st_frame;
            l_ch <= im_ch; l_tag <= im_tag; l_iframe <= im_frame; l_idx <= im_idx; l_crc <= im_crc;
            // ICMP 应答只把 type 8 改成 0：校验和 +0x0800（反码加法，进位回绕）
            l_icsum <= icmp_csum + 16'h0800 + ((icmp_csum > 16'hF7FF) ? 16'd1 : 16'd0);
            case (kind)
            K_ARP:  begin flen <= 11'd60; ip_tot <= 16'd0; end
            K_ICMP: begin flen <= (icmp_len <= 7'd18) ? 11'd60 : 11'd42 + icmp_len; ip_tot <= 16'd28 + icmp_len; end
            K_STAT: begin flen <= 11'd60; ip_tot <= 16'd46; end
            default:begin flen <= FL_IMG; ip_tot <= 16'd1206; end
            endcase
        end
    end
    // IP 头 10 个 16 位字中的常量部分：4500, 4000(DF), 40xx(TTL=64,proto), 源 IP 两半
    wire [7:0]  proto = (k == K_ICMP) ? 8'h01 : 8'h11;
    wire [17:0] ck_const = 18'h04500 + 18'h04000 + {2'd0, 8'h40, proto} + MY_IP[31:16] + MY_IP[15:0];

    // ---------------- 状态机 ----------------
    localparam [2:0] T_IDLE=0, T_PRE=1, T_DATA=2, T_FCS=3, T_IFG=4;
    reg [2:0]  ts;
    reg [3:0]  cnt;
    reg [10:0] idx;
    reg payload_phase;
    assign busy = (ts != T_IDLE);
    assign pay_adv = (ts == T_DATA) && (k == K_IMG) && payload_phase;
    // 第 1 级寄存：类型 0 空闲 / 1 前导 / 2 数据 / 3 FCS
    reg [1:0] s1_kind; reg [7:0] s1_byte; reg [1:0] s1_fcsn;
    reg [7:0] byte_c;
    reg [31:0] crc;
    always @(posedge clk) begin
        if (!rst_n) begin
            ts <= T_IDLE; cnt <= 0; idx <= 0; done <= 0; ip_id <= 16'd0; payload_phase <= 0;
            s1_kind <= 0; gmii_tx_en <= 0; gmii_txd <= 0; crc <= 32'hFFFFFFFF;
        end else begin
            done <= 1'b0;
            s1_kind <= 2'd0;
            case (ts)
            T_IDLE: begin payload_phase <= 0; if (start) begin ts <= T_PRE; cnt <= 0; end end
            T_PRE: begin
                // 7 个 55 + D5；同时分 3 拍算 IP 头校验和
                s1_kind <= 2'd1; s1_byte <= (cnt == 4'd7) ? 8'hD5 : 8'h55;
                case (cnt)
                4'd0: ck_acc <= ck_const + ip_tot;
                4'd1: ck_acc <= ck_acc + ip_id + d_ip[31:16];
                4'd2: ck_acc <= ck_acc + d_ip[15:0];
                4'd3: ck_acc <= {2'b00, ck_acc[15:0]} + ck_acc[17:16];
                4'd4: ip_ck  <= ~(ck_acc[15:0] + ck_acc[16]);
                default: ;
                endcase
                if (cnt == 4'd7) begin ts <= T_DATA; idx <= 0; end
                cnt <= cnt + 1'b1;
            end
            T_DATA: begin
                s1_kind <= 2'd2; s1_byte <= byte_c;
                idx <= idx + 1'b1;
                if (idx == 11'd67) payload_phase <= 1;
                if (idx == flen - 1'b1) begin ts <= T_FCS; cnt <= 0; end
            end
            T_FCS: begin
                s1_kind <= 2'd3; s1_fcsn <= cnt[1:0];
                cnt <= cnt + 1'b1;
                if (cnt == 4'd3) begin ts <= T_IFG; cnt <= 0; end
            end
            T_IFG: begin
                cnt <= cnt + 1'b1;
                if (cnt == 4'd13) begin ts <= T_IDLE; done <= 1'b1; ip_id <= ip_id + 1'b1; end
            end
            default: ts <= T_IDLE;
            endcase
            // 第 2 级：GMII 输出 + FCS
            case (s1_kind)
            2'd1: begin gmii_tx_en <= 1; gmii_txd <= s1_byte; crc <= 32'hFFFFFFFF; end
            2'd2: begin gmii_tx_en <= 1; gmii_txd <= s1_byte; crc <= crc8(crc, s1_byte); end
            2'd3: begin gmii_tx_en <= 1; gmii_txd <= ~crc[{s1_fcsn,3'b000} +: 8]; end
            default: begin gmii_tx_en <= 0; gmii_txd <= 8'h00; end
            endcase
        end
    end
    // ---------------- 按 (帧类型, 字节号) 选字节 ----------------
    wire [15:0] udp_len = ip_tot - 16'd20;
    wire [10:0] di = idx - 11'd42;
    always @* begin
        byte_c = 8'h00;
        case (idx)
        // 以太网头
        11'd0: byte_c = d_mac[47:40];  11'd1: byte_c = d_mac[39:32];  11'd2: byte_c = d_mac[31:24];
        11'd3: byte_c = d_mac[23:16];  11'd4: byte_c = d_mac[15:8];   11'd5: byte_c = d_mac[7:0];
        11'd6: byte_c = MY_MAC[47:40]; 11'd7: byte_c = MY_MAC[39:32]; 11'd8: byte_c = MY_MAC[31:24];
        11'd9: byte_c = MY_MAC[23:16]; 11'd10: byte_c = MY_MAC[15:8]; 11'd11: byte_c = MY_MAC[7:0];
        11'd12: byte_c = 8'h08;
        11'd13: byte_c = (k == K_ARP) ? 8'h06 : 8'h00;
        default: ;
        endcase
        if (k == K_ARP) begin
            case (idx)
            11'd14: byte_c = 8'h00; 11'd15: byte_c = 8'h01; 11'd16: byte_c = 8'h08; 11'd17: byte_c = 8'h00;
            11'd18: byte_c = 8'h06; 11'd19: byte_c = 8'h04; 11'd20: byte_c = 8'h00; 11'd21: byte_c = 8'h02;
            11'd22: byte_c = MY_MAC[47:40]; 11'd23: byte_c = MY_MAC[39:32]; 11'd24: byte_c = MY_MAC[31:24];
            11'd25: byte_c = MY_MAC[23:16]; 11'd26: byte_c = MY_MAC[15:8];  11'd27: byte_c = MY_MAC[7:0];
            11'd28: byte_c = MY_IP[31:24];  11'd29: byte_c = MY_IP[23:16];  11'd30: byte_c = MY_IP[15:8];
            11'd31: byte_c = MY_IP[7:0];
            11'd32: byte_c = d_mac[47:40];  11'd33: byte_c = d_mac[39:32];  11'd34: byte_c = d_mac[31:24];
            11'd35: byte_c = d_mac[23:16];  11'd36: byte_c = d_mac[15:8];   11'd37: byte_c = d_mac[7:0];
            11'd38: byte_c = d_ip[31:24];   11'd39: byte_c = d_ip[23:16];   11'd40: byte_c = d_ip[15:8];
            11'd41: byte_c = d_ip[7:0];
            default: ;
            endcase
        end else begin
            // IPv4 头（14..33）
            case (idx)
            11'd14: byte_c = 8'h45; 11'd15: byte_c = 8'h00;
            11'd16: byte_c = ip_tot[15:8]; 11'd17: byte_c = ip_tot[7:0];
            11'd18: byte_c = ip_id[15:8];  11'd19: byte_c = ip_id[7:0];
            11'd20: byte_c = 8'h40; 11'd21: byte_c = 8'h00; 11'd22: byte_c = 8'h40; 11'd23: byte_c = proto;
            11'd24: byte_c = ip_ck[15:8];  11'd25: byte_c = ip_ck[7:0];
            11'd26: byte_c = MY_IP[31:24]; 11'd27: byte_c = MY_IP[23:16]; 11'd28: byte_c = MY_IP[15:8]; 11'd29: byte_c = MY_IP[7:0];
            11'd30: byte_c = d_ip[31:24];  11'd31: byte_c = d_ip[23:16];  11'd32: byte_c = d_ip[15:8];  11'd33: byte_c = d_ip[7:0];
            default: ;
            endcase
            if (k == K_ICMP) begin
                case (idx)
                11'd34: byte_c = 8'h00; 11'd35: byte_c = 8'h00;
                11'd36: byte_c = l_icsum[15:8];  11'd37: byte_c = l_icsum[7:0];
                11'd38: byte_c = icmp_id[15:8];  11'd39: byte_c = icmp_id[7:0];
                11'd40: byte_c = icmp_seqn[15:8]; 11'd41: byte_c = icmp_seqn[7:0];
                default: if (idx >= 11'd42 && di < {4'd0, icmp_len}) byte_c = icmp_data[{di[5:0],3'b000} +: 8];
                endcase
            end else begin
                // UDP 头（34..41），校验和填 0
                case (idx)
                11'd34: byte_c = MY_PORT[15:8]; 11'd35: byte_c = MY_PORT[7:0];
                11'd36: byte_c = d_port[15:8];  11'd37: byte_c = d_port[7:0];
                11'd38: byte_c = udp_len[15:8]; 11'd39: byte_c = udp_len[7:0];
                11'd40: byte_c = 8'h00; 11'd41: byte_c = 8'h00;
                default: ;
                endcase
                if (k == K_STAT) begin
                    // FQAR（42..59）："FQAR" 01 01 seq[4] flags cmd fcs_bad[2] frame[4]
                    case (idx)
                    11'd42: byte_c = 8'h46; 11'd43: byte_c = 8'h51; 11'd44: byte_c = 8'h41; 11'd45: byte_c = 8'h52;
                    11'd46: byte_c = 8'h01; 11'd47: byte_c = 8'h01;
                    11'd48: byte_c = l_seq[7:0];  11'd49: byte_c = l_seq[15:8];
                    11'd50: byte_c = l_seq[23:16]; 11'd51: byte_c = l_seq[31:24];
                    11'd52: byte_c = l_flags; 11'd53: byte_c = l_cmd;
                    11'd54: byte_c = l_fbad[7:0]; 11'd55: byte_c = l_fbad[15:8];
                    11'd56: byte_c = l_sframe[7:0];  11'd57: byte_c = l_sframe[15:8];
                    11'd58: byte_c = l_sframe[23:16]; 11'd59: byte_c = l_sframe[31:24];
                    default: ;
                    endcase
                end else begin
                    // FQAP v2 三通道头（42..67）："FQAP" 02 ch tag[2] frame[4] w[2] h[2] idx[2] cnt[2] size[2] crc[4]
                    case (idx)
                    11'd42: byte_c = 8'h46; 11'd43: byte_c = 8'h51; 11'd44: byte_c = 8'h41; 11'd45: byte_c = 8'h50;
                    11'd46: byte_c = 8'h02; 11'd47: byte_c = l_ch;
                    11'd48: byte_c = l_tag[7:0]; 11'd49: byte_c = l_tag[15:8];
                    11'd50: byte_c = l_iframe[7:0];  11'd51: byte_c = l_iframe[15:8];
                    11'd52: byte_c = l_iframe[23:16]; 11'd53: byte_c = l_iframe[31:24];
                    11'd54: byte_c = IMG_W[7:0]; 11'd55: byte_c = IMG_W[15:8];
                    11'd56: byte_c = IMG_H[7:0]; 11'd57: byte_c = IMG_H[15:8];
                    11'd58: byte_c = l_idx[7:0]; 11'd59: byte_c = l_idx[15:8];
                    11'd60: byte_c = l_ch==3 ? 8'd3 : NPKT[7:0];  11'd61: byte_c = l_ch==3 ? 8'd0 : NPKT[15:8];
                    11'd62: byte_c = 8'h80; 11'd63: byte_c = 8'h04;          // 1152
                    11'd64: byte_c = l_crc[7:0];  11'd65: byte_c = l_crc[15:8];
                    11'd66: byte_c = l_crc[23:16]; 11'd67: byte_c = l_crc[31:24];
                    default: if (idx >= 11'd68) byte_c = pay_byte;
                    endcase
                end
            end
        end
    end
endmodule
`default_nettype wire
