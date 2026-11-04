`timescale 1ns/1ps
`default_nettype none

// 以太网接收解析（rx_clk 域，GMII 字节流）。
// 识别：ARP 请求（目标 IP 为本机）、ICMP 回显请求（本机 IP，数据 <=64 字节）、
//       UDP 6002 端口的 FQAC 命令。帧好坏只看 FCS（CRC32 余数 0xDEBB20E3），不看 RGMII 错误位。
// 结果放进三个"邮箱"：字段在帧内边收边写（仅当该邮箱空闲），帧尾 FCS 正确才翻转 req；
// core 侧处理完再翻转 ack。req!=ack 期间字段保持不变，core 侧可直接读取。
module eth_rx_parser #(
    parameter [47:0] MY_MAC  = 48'h02_00_00_00_00_01,
    parameter [31:0] MY_IP   = 32'hC0A80114,      // 192.168.1.20
    parameter [15:0] MY_PORT = 16'd6002
)(
    input  wire clk, rst_n,
    input  wire gmii_rx_dv,
    input  wire [7:0] gmii_rxd,
    output reg  arp_req,  input wire arp_ack,
    output reg  [47:0] arp_mac,
    output reg  [31:0] arp_ip,
    output reg  cmd_req,  input wire cmd_ack,
    output reg  [47:0] cmd_mac,
    output reg  [31:0] cmd_ip,
    output reg  [15:0] cmd_port,
    output reg  [7:0]  cmd_code,
    output reg  [31:0] cmd_seq,
    output reg  [7:0]  cmd_arg0, cmd_arg1,
    output reg  [255:0] cmd_ranges,
    output reg  icmp_req, input wire icmp_ack,
    output reg  [47:0] icmp_mac,
    output reg  [31:0] icmp_ip,
    output reg  [15:0] icmp_csum, icmp_id, icmp_seqn,
    output reg  [6:0]  icmp_len,
    output reg  [511:0] icmp_data,
    output reg  [15:0] cnt_frames, cnt_fcs_bad, cnt_cmd
);
    reg [1:0] arp_ack_s, cmd_ack_s, icmp_ack_s;
    always @(posedge clk) begin
        arp_ack_s  <= {arp_ack_s[0],  arp_ack};
        cmd_ack_s  <= {cmd_ack_s[0],  cmd_ack};
        icmp_ack_s <= {icmp_ack_s[0], icmp_ack};
    end
    wire arp_free  = (arp_req  == arp_ack_s[1]);
    wire cmd_free  = (cmd_req  == cmd_ack_s[1]);
    wire icmp_free = (icmp_req == icmp_ack_s[1]);

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
    localparam [1:0] S_IDLE=0, S_PRE=1, S_DATA=2, S_DROP=3;
    reg [1:0]  st;
    reg [10:0] idx;                 // 目的 MAC 第 0 字节为 0，含 FCS
    reg [31:0] crc;
    reg mac_ok, is_arp, is_ip, arp_ok, ip_ok, ip_uni, udp_ok, fq_ok, icmp_ok;
    reg [15:0] ip_total;
    reg w_arp, w_cmd, w_icmp;       // 帧开始时锁存：本帧可写哪些邮箱
    wire [7:0] b = gmii_rxd;
    wire [6:0] icmp_di = idx - 11'd42;   // ICMP 数据字节序号

    always @(posedge clk) begin
        if (!rst_n) begin
            st<=S_IDLE; idx<=0; crc<=32'hFFFFFFFF;
            arp_req<=0; cmd_req<=0; icmp_req<=0;
            cnt_frames<=0; cnt_fcs_bad<=0; cnt_cmd<=0;
        end else begin
            case (st)
            S_IDLE: if (gmii_rx_dv) st <= (b==8'h55) ? S_PRE : (b==8'hD5) ? S_DATA : S_DROP;
            S_PRE:  if (!gmii_rx_dv) st <= S_IDLE;
                    else if (b==8'hD5) st <= S_DATA;
                    else if (b!=8'h55) st <= S_DROP;
            S_DROP: if (!gmii_rx_dv) st <= S_IDLE;
            S_DATA: if (gmii_rx_dv) begin
                        idx <= idx + 1'b1;
                        crc <= crc8(crc, b);
                    end else begin
                        // ---------------- 帧结束：FCS 判定 ----------------
                        st <= S_IDLE;
                        cnt_frames <= cnt_frames + 1'b1;
                        if (crc != 32'hDEBB20E3) cnt_fcs_bad <= cnt_fcs_bad + 1'b1;
                        else if (mac_ok && idx >= 11'd64) begin
                            if (is_arp && arp_ok && w_arp) arp_req <= ~arp_req;
                            if (is_ip && ip_ok && udp_ok && fq_ok && w_cmd) begin
                                cmd_req <= ~cmd_req; cnt_cmd <= cnt_cmd + 1'b1;
                            end
                            if (is_ip && ip_ok && ip_uni && icmp_ok && w_icmp &&
                                ip_total >= 16'd28 && ip_total <= 16'd92) begin
                                icmp_len <= ip_total - 16'd28;
                                icmp_req <= ~icmp_req;
                            end
                        end
                    end
            endcase
            if (st != S_DATA) begin
                idx <= 0; crc <= 32'hFFFFFFFF;
                w_arp <= arp_free; w_cmd <= cmd_free; w_icmp <= icmp_free;
                mac_ok<=1; is_arp<=0; is_ip<=0; arp_ok<=1; ip_ok<=1; ip_uni<=1;
                udp_ok<=1; fq_ok<=1; icmp_ok<=1; ip_total<=0;
            end
            // ---------------- 逐字节检查与字段采集 ----------------
            if (st == S_DATA && gmii_rx_dv) begin
                // 目的 MAC：本机或广播（逐字节，每字节是本机值或 FF）
                case (idx)
                11'd0: if (b != MY_MAC[47:40] && b != 8'hFF) mac_ok <= 0;
                11'd1: if (b != MY_MAC[39:32] && b != 8'hFF) mac_ok <= 0;
                11'd2: if (b != MY_MAC[31:24] && b != 8'hFF) mac_ok <= 0;
                11'd3: if (b != MY_MAC[23:16] && b != 8'hFF) mac_ok <= 0;
                11'd4: if (b != MY_MAC[15:8]  && b != 8'hFF) mac_ok <= 0;
                11'd5: if (b != MY_MAC[7:0]   && b != 8'hFF) mac_ok <= 0;
                default: ;
                endcase
                // 源 MAC -> 各邮箱（仅空闲邮箱）
                if (idx >= 11'd6 && idx <= 11'd11) begin
                    if (w_arp)  arp_mac  <= {arp_mac[39:0],  b};
                    if (w_cmd)  cmd_mac  <= {cmd_mac[39:0],  b};
                    if (w_icmp) icmp_mac <= {icmp_mac[39:0], b};
                end
                if (idx == 11'd12) begin is_arp <= (b == 8'h08); is_ip <= (b == 8'h08); end
                if (idx == 11'd13) begin is_arp <= is_arp && (b == 8'h06); is_ip <= is_ip && (b == 8'h00); end
                // ---- ARP（14 起）：htype=1 ptype=0800 hlen=6 plen=4 op=1，目标 IP=本机 ----
                case (idx)
                11'd14: if (b != 8'h00) arp_ok <= 0;   11'd15: if (b != 8'h01) arp_ok <= 0;
                11'd16: if (b != 8'h08) arp_ok <= 0;   11'd17: if (b != 8'h00) arp_ok <= 0;
                11'd18: if (b != 8'h06) arp_ok <= 0;   11'd19: if (b != 8'h04) arp_ok <= 0;
                11'd20: if (b != 8'h00) arp_ok <= 0;   11'd21: if (b != 8'h01) arp_ok <= 0;
                11'd38: if (b != MY_IP[31:24]) arp_ok <= 0;  11'd39: if (b != MY_IP[23:16]) arp_ok <= 0;
                11'd40: if (b != MY_IP[15:8])  arp_ok <= 0;  11'd41: if (b != MY_IP[7:0])   arp_ok <= 0;
                default: ;
                endcase
                if (idx >= 11'd28 && idx <= 11'd31 && w_arp) arp_ip <= {arp_ip[23:0], b};
                // ---- IPv4（14 起）：版本4、头长20、不分片 ----
                if (idx == 11'd14 && b != 8'h45) ip_ok <= 0;
                if (idx == 11'd16) ip_total[15:8] <= b;
                if (idx == 11'd17) ip_total[7:0]  <= b;
                if (idx == 11'd20 && (b[5] || b[4:0] != 0)) ip_ok <= 0;   // MF 或片偏移
                if (idx == 11'd21 && b != 8'h00) ip_ok <= 0;
                if (idx == 11'd23) begin udp_ok <= (b == 8'h11); icmp_ok <= (b == 8'h01); end
                if (idx >= 11'd26 && idx <= 11'd29) begin
                    if (w_cmd)  cmd_ip  <= {cmd_ip[23:0],  b};
                    if (w_icmp) icmp_ip <= {icmp_ip[23:0], b};
                end
                // 目的 IP：本机单播，或 192.168.1.255 / 255.255.255.255 广播（ICMP 只认单播）
                if (idx == 11'd30 && b != MY_IP[31:24] && b != 8'hFF) ip_ok <= 0;
                if (idx == 11'd31 && b != MY_IP[23:16] && b != 8'hFF) ip_ok <= 0;
                if (idx == 11'd32 && b != MY_IP[15:8]  && b != 8'hFF) ip_ok <= 0;
                if (idx == 11'd33 && b != MY_IP[7:0]   && b != 8'hFF) ip_ok <= 0;
                if (idx == 11'd33 && b != MY_IP[7:0]) ip_uni <= 0;
                // ---- UDP（34 起）：目的端口 6002 ----
                if (idx >= 11'd34 && idx <= 11'd35 && w_cmd) cmd_port <= {cmd_port[7:0], b};
                if (idx == 11'd36 && b != MY_PORT[15:8]) udp_ok <= 0;
                if (idx == 11'd37 && b != MY_PORT[7:0])  udp_ok <= 0;
                // ---- FQAC（42 起）："FQAC" 01 cmd seq[4] arg0 arg1 ranges[32] ----
                case (idx)
                11'd42: if (b != 8'h46) fq_ok <= 0;   11'd43: if (b != 8'h51) fq_ok <= 0;
                11'd44: if (b != 8'h41) fq_ok <= 0;   11'd45: if (b != 8'h43) fq_ok <= 0;
                11'd46: if (b != 8'h01) fq_ok <= 0;
                11'd47: begin if (b > 8'd3) fq_ok <= 0; if (w_cmd) cmd_code <= b; end
                default: ;
                endcase
                if (w_cmd) begin
                    if (idx >= 11'd48 && idx <= 11'd51) cmd_seq <= {b, cmd_seq[31:8]};     // 小端
                    if (idx == 11'd52) cmd_arg0 <= b;
                    if (idx == 11'd53) cmd_arg1 <= b;
                    if (idx >= 11'd54 && idx <= 11'd85) cmd_ranges <= {b, cmd_ranges[255:8]};
                end
                // ---- ICMP（34 起）：type=8 code=0，校验和/标识/序号，数据 42 起 ----
                if (idx == 11'd34 && b != 8'h08) icmp_ok <= 0;
                if (idx == 11'd35 && b != 8'h00) icmp_ok <= 0;
                if (w_icmp) begin
                    if (idx >= 11'd36 && idx <= 11'd37) icmp_csum <= {icmp_csum[7:0], b};
                    if (idx >= 11'd38 && idx <= 11'd39) icmp_id   <= {icmp_id[7:0],   b};
                    if (idx >= 11'd40 && idx <= 11'd41) icmp_seqn <= {icmp_seqn[7:0], b};
                    if (idx >= 11'd42 && idx <= 11'd105) icmp_data[{icmp_di[5:0],3'b000} +: 8] <= b;
                end
            end
        end
    end
endmodule
`default_nettype wire
