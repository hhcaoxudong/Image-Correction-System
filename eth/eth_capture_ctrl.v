`timescale 1ns/1ps
`default_nettype none

// 同帧采集控制（core_clk 域）。
// 命令（FQAC v1）：00 状态；01 采集（arg0=包间隔 us，0=默认）；02 补发（arg0=通道，8 组 起始/个数）；03 释放。
// 每条命令都回一个 FQAR（seq=命令序号，cmd=命令码）。
// 采集流程：冻结（fram_buf 停在当前已上屏的结果帧）-> 等稳 -> 扫两通道算整幅 CRC32
//          -> ch0原图/ch1矫正/ch2 Gamma/ch3同帧坐标参数 FQAP v2 -> 保持。
// 每个 FQAP 块 = 384 像素 = 1152 字节 RGB888 = DDR 中 24 个 256 bit 拍，按列边界拆突发。
module eth_capture_ctrl #(
    parameter integer NPKT = 5400,                       // 每通道块数 = W*H/384
    parameter integer PACE_US_DEFAULT = 15,
    parameter integer SETTLE_CYC = 1024,
    parameter integer FREEZE_TIMEOUT_CYC = 250000000,    // 2 s：一直等不到冻结（无视频输入）
    parameter integer HOLD_TIMEOUT_CYC = 1250000000      // 10 s：保持期间无命令则自动释放
)(
    input  wire clk, rst_n,
    // 接收邮箱（rx 域，req!=ack 期间保持）
    input  wire arp_req,  output reg arp_ack,
    input  wire [47:0] arp_mac, input wire [31:0] arp_ip,
    input  wire icmp_req, output reg icmp_ack,
    input  wire [47:0] icmp_mac, input wire [31:0] icmp_ip,
    input  wire cmd_req,  output reg cmd_ack,
    input  wire [47:0] cmd_mac, input wire [31:0] cmd_ip, input wire [15:0] cmd_port,
    input  wire [7:0]  cmd_code, input wire [31:0] cmd_seq,
    input  wire [7:0]  cmd_arg0, cmd_arg1,
    input  wire [255:0] cmd_ranges,
    input  wire [15:0] fcs_bad_rx,
    // 发送引擎
    output reg  tx_start, output reg [1:0] tx_kind,
    input  wire tx_busy, tx_done,
    output reg  [47:0] tx_mac, output reg [31:0] tx_ip, output reg [15:0] tx_port,
    output reg  [31:0] st_seq, output reg [7:0] st_flags, st_cmd, output reg [15:0] st_fcs_bad,
    output wire [31:0] st_frame,
    output reg  [7:0] im_ch, output reg [15:0] im_tag, output wire [31:0] im_frame,
    output reg  [15:0] im_idx, output reg [31:0] im_crc,
    output wire [7:0] pay_byte, input wire pay_adv,
    // fram_buf
    output reg  freeze_req, input wire frozen,
    input  wire [27:0] frozen_raw_base, frozen_res_base,
    input  wire [2:0] frozen_gamma_curve,
    output wire [7:0] coord_read_addr, input wire [63:0] coord_read_data,
    input wire coord_fisheye,
    input wire [383:0] coord_calibration, input wire [191:0] coord_affine,
    input wire coord_affine_enabled, coord_complete,
    input wire [7:0] coord_sample_count, coord_generation, input wire [3:0] coord_gamma,
    output reg  ext_own,
    output reg  [27:0] ext_addr, output reg [3:0] ext_len, output reg ext_valid,
    input  wire ext_ready,
    input  wire [255:0] ext_rdata, input wire ext_rvalid, ext_rlast
);
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
    // ---------------- 邮箱同步（rx -> core）----------------
    reg [1:0] arp_s, icmp_s, cmd_s;
    reg [15:0] fb_m, fb_s;
    always @(posedge clk) begin
        if (!rst_n) begin arp_s<=0; icmp_s<=0; cmd_s<=0; end
        else begin arp_s<={arp_s[0],arp_req}; icmp_s<={icmp_s[0],icmp_req}; cmd_s<={cmd_s[0],cmd_req}; end
        fb_m <= fcs_bad_rx; fb_s <= fb_m;
    end
    wire arp_pend  = (arp_s[1]  != arp_ack);
    wire icmp_pend = (icmp_s[1] != icmp_ack);
    wire cmd_pend  = (cmd_s[1]  != cmd_ack);
    // ---------------- A：命令接收、状态回包、发送仲裁 ----------------
    reg [47:0] peer_mac; reg [31:0] peer_ip; reg [15:0] peer_port; reg peer_ok;
    reg ev_cap, ev_rs, ev_rel;            // 给 B 的单拍事件
    reg [31:0] ev_seq; reg [7:0] ev_arg0, ev_arg1;
    reg [255:0] rs_ranges;
    reg [31:0] stp_seq; reg [7:0] stp_cmd; reg st_pend;
    reg [1:0] inflight;                   // 0 无 1 ARP 2 ICMP/STAT 3 IMG
    reg infl_icmp;
    reg img_go, img_started, img_sent;
    wire [7:0] flags;
    wire in_hold;
    always @(posedge clk) begin
        if (!rst_n) begin
            cmd_ack<=0; arp_ack<=0; icmp_ack<=0; peer_ok<=0;
            ev_cap<=0; ev_rs<=0; ev_rel<=0; st_pend<=0;
            inflight<=0; infl_icmp<=0; tx_start<=0; img_started<=0; img_sent<=0;
        end else begin
            ev_cap<=0; ev_rs<=0; ev_rel<=0; tx_start<=0; img_started<=0; img_sent<=0;
            // 发送完成
            if (tx_done) begin
                if (inflight==2'd1) arp_ack <= ~arp_ack;
                if (inflight==2'd2 && infl_icmp) icmp_ack <= ~icmp_ack;
                if (inflight==2'd3) img_sent <= 1'b1;
                inflight <= 2'd0;
            end
            // 仲裁：ARP > ICMP > 状态 > 图像。tx_done 当拍不发，等 ack 生效。
            if (!tx_busy && !tx_start && !tx_done && inflight==2'd0) begin
                if (arp_pend) begin
                    tx_kind<=2'd0; tx_mac<=arp_mac; tx_ip<=arp_ip; tx_port<=0;
                    tx_start<=1; inflight<=2'd1;
                end else if (icmp_pend) begin
                    tx_kind<=2'd1; tx_mac<=icmp_mac; tx_ip<=icmp_ip; tx_port<=0;
                    tx_start<=1; inflight<=2'd2; infl_icmp<=1;
                end else if (st_pend && peer_ok) begin
                    tx_kind<=2'd2; tx_mac<=peer_mac; tx_ip<=peer_ip; tx_port<=peer_port;
                    st_seq<=stp_seq; st_cmd<=stp_cmd; st_flags<=flags; st_fcs_bad<=fb_s;
                    tx_start<=1; inflight<=2'd2; infl_icmp<=0; st_pend<=0;
                end else if (img_go && peer_ok) begin
                    tx_kind<=2'd3; tx_mac<=peer_mac; tx_ip<=peer_ip; tx_port<=peer_port;
                    tx_start<=1; inflight<=2'd3; img_started<=1;
                end
            end
            // 收命令：记下对端，排一个状态回包，按命令码通知 B
            if (cmd_pend) begin
                cmd_ack <= ~cmd_ack;
                peer_mac<=cmd_mac; peer_ip<=cmd_ip; peer_port<=cmd_port; peer_ok<=1;
                stp_seq<=cmd_seq; stp_cmd<=cmd_code; st_pend<=1;
                ev_seq<=cmd_seq; ev_arg0<=cmd_arg0; ev_arg1<=cmd_arg1;
                case (cmd_code)
                8'd1: ev_cap <= 1;
                8'd2: if (in_hold && cmd_arg0 < 4) begin ev_rs <= 1; rs_ranges <= cmd_ranges; end
                8'd3: ev_rel <= 1;
                default: ;
                endcase
            end
        end
    end
    // ---------------- B：冻结、扫描 CRC、发块、补发 ----------------
    localparam [2:0] M_IDLE=0, M_FREEZE=1, M_SETTLE=2, M_SCAN=3, M_SEND=4, M_HOLD=5, M_RESEND=6, M_REL=7;
    localparam [3:0] P_IDLE=0, P_JOB=1, P_RLOAD=2, P_ARA=3, P_DA=4, P_ARB=5, P_DB=6,
                     P_PR0=7, P_PR1=8, P_RUN=9, P_RDY=10, P_TX=11, P_META_WAIT=12, P_META_BYTE=13;
    reg [15:0] j_k;
    reg [10:0] bcnt;
    reg [1023:0] csm_header;
    reg [511:0] csm_info;
    reg [31:0] ch_crc3;
    wire [12:0] meta_offset=({11'd0,j_k[1:0]}<<10)+({11'd0,j_k[1:0]}<<7)+{2'd0,bcnt};
    wire [12:0] sample_offset=meta_offset-13'd128;
    assign coord_read_addr=sample_offset[11:4];
    wire [15:0] sample_x=16'd64+{5'd0,coord_read_addr[3:1],8'd0}+{15'd0,coord_read_addr[0]};
    wire [15:0] sample_y=16'd28+{5'd0,coord_read_addr[7:4],7'd0};
    wire sample_valid=coord_read_data[31:0]!=32'hffffffff&&coord_read_data[63:32]!=32'hffffffff;
    wire [31:0] sample_flags={29'd0,
        sample_valid&&(coord_read_data[63:32]==0||coord_read_data[63:32]==32'd70647808),
        sample_valid&&(coord_read_data[31:0]==0||coord_read_data[31:0]==32'd125698048),sample_valid};
    wire [127:0] sample_record={sample_y,sample_x,sample_flags,coord_read_data};
    wire [7:0] meta_byte=meta_offset<128 ? csm_header[meta_offset[6:0]*8+:8] :
        meta_offset<2432 ? sample_record[meta_offset[3:0]*8+:8] :
        meta_offset<2496 ? csm_info[(meta_offset-2432)*8+:8] : 8'd0;
    reg [2:0]  ms;
    reg [3:0]  ps;
    reg [31:0] mtimer;
    reg [31:0] cap_seq, pend_seq;
    reg        cap_pend, abort, restart, frz_timeout, crc_ok;
    reg [7:0]  pace_us;
    reg [27:0] base_raw, base_res;
    reg [31:0] frame_no;
    reg [31:0] crc_acc, ch_crc0, ch_crc1, ch_crc2;
    reg [2:0] capture_gamma_curve;
    // 作业：0 扫描 CRC，1 全部发送，2 按区间补发
    reg [1:0]  j_mode;
    reg [1:0]  j_ch;
    reg        j_done;
    reg [15:0] j_end;
    reg [3:0]  j_ri;
    reg [255:0] j_ranges;
    // 块读取与像素生成
    reg [27:0] pe_addr;
    reg [4:0] read_remaining,read_chunk;
    wire [10:0] read_column_words=11'd1024-{1'b0,pe_addr[9:0]};
    wire [7:0] read_column_beats=read_column_words[10:3];
    wire [4:0] read_candidate=read_remaining>=16 ? 5'd16 : read_remaining;
    wire [4:0] read_pick=read_column_beats<read_candidate ? read_column_beats[4:0] : read_candidate;
    reg [4:0]  wb;                         // 写入拍号
    reg [9:0]  gpix;                       // 下一个装入 nxt 的像素号（0..383）
    reg [15:0] cur_pix, nxt_pix;
    reg [1:0]  comp;

    reg [15:0] pace_cnt;
    reg [255:0] pbuf [0:31];
    // Offline metadata has its own byte buffer; the image datapath stays unchanged.
    reg [7:0] meta_buf [0:1151];
    wire [255:0] prow = pbuf[gpix[8:4]];
    wire [15:0]  pval = prow[{gpix[3:0],4'b0000} +: 16];
    wire [7:0] r8 = {cur_pix[15:11], cur_pix[15:13]};
    wire [7:0] g8 = {cur_pix[10:5],  cur_pix[10:9]};
    wire [7:0] b8 = {cur_pix[4:0],   cur_pix[4:2]};
    wire gen_adv = j_ch!=3 && ((ps==P_RUN) || (ps==P_TX && pay_adv));
    // Synchronous ROM lookahead keeps one byte per clock, including stalls.
    wire [15:0] gamma_pixel = ((ps==P_PR1) || (gen_adv && comp==2'd2)) ? nxt_pix : cur_pix;
    wire [7:0] gamma_in_r = {gamma_pixel[15:11],gamma_pixel[15:13]};
    wire [7:0] gamma_in_g = {gamma_pixel[10:5],gamma_pixel[10:9]};
    wire [7:0] gamma_in_b = {gamma_pixel[4:0],gamma_pixel[4:2]};
    reg [7:0] gamma_lut_r [0:1791];
    reg [7:0] gamma_lut_g [0:1791];
    reg [7:0] gamma_lut_b [0:1791];
    reg [7:0] gamma_r, gamma_g, gamma_b;
    initial begin
        $readmemh("source/undistort/lut/gamma_8.hex",gamma_lut_r);
        $readmemh("source/undistort/lut/gamma_8.hex",gamma_lut_g);
        $readmemh("source/undistort/lut/gamma_8.hex",gamma_lut_b);
    end
    always @(posedge clk) begin
        gamma_r <= gamma_lut_r[{capture_gamma_curve,gamma_in_r}];
        gamma_g <= gamma_lut_g[{capture_gamma_curve,gamma_in_g}];
        gamma_b <= gamma_lut_b[{capture_gamma_curve,gamma_in_b}];
    end
    wire [7:0] send_r = j_ch==2 ? gamma_r : r8;
    wire [7:0] send_g = j_ch==2 ? gamma_g : g8;
    wire [7:0] send_b = j_ch==2 ? gamma_b : b8;
    assign pay_byte = j_ch==3 ? meta_buf[bcnt] : (comp==2'd0) ? send_r : (comp==2'd1) ? send_g : send_b;
    always @(posedge clk) begin
        if (ext_rvalid && (ps==P_DA || ps==P_DB)) pbuf[wb] <= ext_rdata;
        else if(ps==P_META_BYTE) meta_buf[bcnt]<=meta_byte;
    end
    wire [31:0] rng = j_ranges[{j_ri[2:0],5'b00000} +: 32];
    wire [16:0] rng_end = {1'b0, rng[15:0]} + {1'b0, rng[31:16]};
    wire [15:0] pace_cyc_raw = ({8'd0,pace_us} << 7) - ({8'd0,pace_us} << 1) - {8'd0,pace_us};   // *125
    wire [27:0] pkt_off = ({12'd0, j_k} << 7) + ({12'd0, j_k} << 6);                                 // *192 字
    assign in_hold = (ms == M_HOLD);
    assign st_frame = frame_no;
    assign im_frame = frame_no;
    assign flags = {peer_ok, frz_timeout, in_hold, crc_ok, frozen, ext_own, 1'b0, (ms != M_IDLE)};
    always @(posedge clk) begin
        if (!rst_n) begin
            ps<=P_IDLE; j_done<=0; ext_valid<=0; img_go<=0; pace_cnt<=0; wb<=0; gpix<=0; comp<=0; bcnt<=0;
            ms<=M_IDLE; freeze_req<=0; ext_own<=0; mtimer<=0; cap_pend<=0; abort<=0; restart<=0;
            frz_timeout<=0; crc_ok<=0; frame_no<=0; cap_seq<=0; pace_us<=0; capture_gamma_curve<=3;
        end else begin
            j_done <= 1'b0;
            if (pace_cnt != 0) pace_cnt <= pace_cnt - 1'b1;
            // 像素生成器：每 3 个字节换一个像素，nxt 提前一拍从缓冲区取
            if (gen_adv) begin
                if (comp == 2'd2) begin comp <= 0; cur_pix <= nxt_pix; nxt_pix <= pval; gpix <= gpix + 1'b1; end
                else comp <= comp + 1'b1;
            end
            case (ps)
            P_IDLE: ;
            P_JOB: begin
                if (abort) begin ps <= P_IDLE; j_done <= 1; end
                else if (j_ch==3 && j_k < j_end) begin bcnt<=0; ps<=P_META_WAIT; end
                else if (j_k < j_end) begin pe_addr <= (j_ch==1 ? base_res : base_raw) + pkt_off; read_remaining<=24; wb<=0; ps <= P_ARA; end
                else if (j_mode == 2'd2) begin
                    if (j_ri != 4'd7) begin j_ri <= j_ri + 1'b1; ps <= P_RLOAD; end
                    else begin ps <= P_IDLE; j_done <= 1; end
                end else if (j_ch < 3) begin
                    if (j_mode == 2'd0) begin
                        if (j_ch==0) ch_crc0 <= ~crc_acc;
                        else if(j_ch==1) ch_crc1 <= ~crc_acc;
                        else ch_crc2 <= ~crc_acc;
                        crc_acc <= 32'hFFFFFFFF;
                    end
                    j_ch <= j_ch+1'b1; j_k <= 0; j_end <= j_ch==2 ? 3 : NPKT;
                end else begin
                    if (j_mode == 2'd0) ch_crc3 <= ~crc_acc;
                    ps <= P_IDLE; j_done <= 1;
                end
            end
            P_META_WAIT: ps<=P_META_BYTE;
            P_META_BYTE: begin
                if(bcnt==1151)begin bcnt<=0;ps<=j_mode==0 ? P_RUN : P_RDY;end
                else begin bcnt<=bcnt+1'b1;ps<=P_META_WAIT;end
            end
            P_RLOAD: begin
                j_k   <= rng[15:0];
                j_end <= (rng_end > (j_ch==3 ? 3 : NPKT)) ? (j_ch==3 ? 16'd3 : NPKT[15:0]) : rng_end[15:0];
                ps <= P_JOB;
            end
            P_ARA: begin
                ext_addr <= pe_addr; ext_len <= read_pick[3:0]-4'd1; ext_valid <= 1; read_chunk<=read_pick;
                if (ext_valid && ext_ready) begin ext_valid <= 0; ps <= P_DA; end
            end
            P_DA: if (ext_rvalid) begin
                wb <= wb + 1'b1;
                if (ext_rlast) begin
                    if (read_remaining==read_chunk) begin ps<=P_PR0; gpix<=0; end
                    else begin
                        pe_addr<=pe_addr+{20'd0,read_chunk,3'd0};
                        read_remaining<=read_remaining-read_chunk; ps<=P_ARA;
                    end
                end
            end
            P_PR0: begin nxt_pix <= pval; gpix <= gpix + 1'b1; ps <= P_PR1; end
            P_PR1: begin
                cur_pix <= nxt_pix; nxt_pix <= pval; gpix <= gpix + 1'b1; comp <= 0; bcnt <= 0;
                ps <= (j_mode == 2'd0) ? P_RUN : P_RDY;
            end
            P_RUN: begin
                crc_acc <= crc8(crc_acc, pay_byte);
                bcnt <= bcnt + 1'b1;
                if (bcnt == 11'd1151) begin j_k <= j_k + 1'b1; ps <= P_JOB; end
            end
            P_RDY: begin
                im_ch <= {6'd0, j_ch}; im_idx <= j_k;
                im_crc <= j_ch==3 ? ch_crc3 : j_ch==2 ? ch_crc2 : j_ch==1 ? ch_crc1 : ch_crc0;
                im_tag <= cap_seq[15:0];
                if (abort) begin img_go <= 0; ps <= P_JOB; end
                else if (pace_cnt == 0 && !img_go) begin img_go <= 1; ps <= P_TX; end
            end
            P_TX: begin
                if(j_ch==3 && pay_adv) bcnt<=bcnt+1'b1;
                if (img_started) begin
                    img_go <= 0;
                    pace_cnt <= (pace_us == 0) ? (PACE_US_DEFAULT*125) : pace_cyc_raw;
                end
                if (img_sent) begin j_k <= j_k + 1'b1; ps <= P_JOB; end
            end
            default: ps <= P_IDLE;
            endcase
            // ---------------- 主状态机（放在块引擎之后：同拍写同一寄存器时以这里为准）----------------
            mtimer <= mtimer + 1'b1;
            case (ms)
            M_IDLE: begin
                abort <= 0; restart <= 0;
                if (cap_pend || ev_cap) begin
                    cap_seq <= ev_cap ? ev_seq : pend_seq;
                    if (ev_cap) pace_us <= ev_arg0;
                    cap_pend <= 0; frz_timeout <= 0; crc_ok <= 0;
                    freeze_req <= 1; mtimer <= 0; ms <= M_FREEZE;
                end
            end
            M_FREEZE: begin
                if (ev_rel) ms <= M_REL;
                else if (frozen) begin mtimer <= 0; ms <= M_SETTLE; end
                else if (mtimer >= FREEZE_TIMEOUT_CYC) begin frz_timeout <= 1; ms <= M_REL; end
            end
            M_SETTLE: begin
                // 等矫正通道 1 的读完全结束，再接管它的 DDR 读口
                if (ev_rel) ms <= M_REL;
                else if (mtimer == SETTLE_CYC/2) ext_own <= 1;
                else if (mtimer >= SETTLE_CYC && coord_complete && coord_sample_count==144) begin
                    base_raw <= frozen_raw_base; base_res <= frozen_res_base;
                    capture_gamma_curve <= frozen_gamma_curve<=6 ? frozen_gamma_curve : 3'd3;
                    frame_no <= frame_no + 1'b1;
                    csm_header<=0;
                    csm_header[31:0]<=32'h314d5343;
                    csm_header[63:32]<={16'd16,16'd144};csm_header[79:64]<=16'd16;
                    csm_header[95:80]<={14'd0,coord_complete,coord_affine_enabled};
                    csm_header[127:96]<=frame_no+1'b1;csm_header[159:128]<=coord_fisheye ? 0 : 1;csm_header[191:160]<=2;
                    csm_header[223:192]<={24'd0,coord_generation};csm_header[255:224]<=72;
                    csm_header[447:256]<=coord_affine;csm_header[479:448]<=125698048;csm_header[511:480]<=70647808;
                    csm_header[895:512]<=coord_calibration;csm_header[927:896]<={16'd1080,16'd1920};
                    csm_header[959:928]<=144;csm_header[991:960]<={28'd0,coord_gamma};
                    csm_info<={64'd0,32'd0,32'd4147200,16'd1080,16'd1920,
                        coord_calibration[127:64],32'd65536,coord_calibration[383:256],
                        coord_calibration[191:128],coord_calibration[63:0]};
                    crc_acc <= 32'hFFFFFFFF; j_mode <= 2'd0; j_ch <= 0; j_k <= 0; j_end <= NPKT;
                    ps <= P_JOB; ms <= M_SCAN;
                end
            end
            M_SCAN: if (j_done) begin
                if (abort) ms <= M_REL;
                else begin crc_ok <= 1; j_mode <= 2'd1; j_ch <= 0; j_k <= 0; j_end <= NPKT; ps <= P_JOB; ms <= M_SEND; end
            end
            M_SEND, M_RESEND: if (j_done) begin
                if (restart) begin restart <= 0; abort <= 0; j_mode <= 2'd1; j_ch <= 0; j_k <= 0; j_end <= NPKT; ps <= P_JOB; ms <= M_SEND; end
                else if (abort) ms <= M_REL;
                else begin mtimer <= 0; ms <= M_HOLD; end
            end
            M_HOLD: begin
                if (ev_rs) begin
                    j_mode <= 2'd2; j_ch <= ev_arg0[1:0]; j_ri <= 0; j_ranges <= rs_ranges;
                    ps <= P_RLOAD; ms <= M_RESEND;
                end else if (ev_cap && ev_seq == cap_seq) begin       // 同序号重发 = 整对重发
                    j_mode <= 2'd1; j_ch <= 0; j_k <= 0; j_end <= NPKT; ps <= P_JOB; ms <= M_SEND;
                end else if (ev_rel || ev_cap || mtimer >= HOLD_TIMEOUT_CYC) ms <= M_REL;
                if (ev_rs || ev_cap) mtimer <= 0;
            end
            M_REL: begin
                // 读口已空闲（作业结束才会到这里）：先交还读口，再撤冻结，等 fram_buf 离开 FROZEN
                if (ext_own) begin ext_own <= 0; mtimer <= 0; end
                else if (freeze_req) begin if (mtimer >= 32'd16) begin freeze_req <= 0; mtimer <= 0; end end
                else if (!frozen && mtimer >= 32'd64) ms <= M_IDLE;
            end
            default: ms <= M_IDLE;
            endcase
            // 进行中的事件
            if (ms == M_SCAN || ms == M_SEND || ms == M_RESEND) begin
                if (ev_rel) abort <= 1;
                if (ev_cap && ev_seq != cap_seq) begin abort <= 1; cap_pend <= 1; pend_seq <= ev_seq; pace_us <= ev_arg0; end
                if (ev_cap && ev_seq == cap_seq && ms != M_SCAN) begin abort <= 1; restart <= 1; end
            end
            if ((ms == M_FREEZE || ms == M_SETTLE) && ev_cap && ev_seq != cap_seq) begin
                cap_pend <= 1; pend_seq <= ev_seq; pace_us <= ev_arg0;
            end
            if (ms == M_HOLD && ev_cap && ev_seq != cap_seq) begin cap_pend <= 1; pend_seq <= ev_seq; pace_us <= ev_arg0; end
        end
    end
endmodule
`default_nettype wire
