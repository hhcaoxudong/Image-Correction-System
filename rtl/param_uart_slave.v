// ===========================================================================
// 本版已纳入综合，PG2L100H 时序通过（2026-10-28）。
// 早先"11 个种子无一通过"的真正原因不是本模块/DRAM 占用，而是 brown_map_stream
// 把运行期参数做裸 32x32 乘法（无寄存器 4 级 APM 级联）；已改用 brown_mul32_pipe。
// 2026-10-31：schema2 行列式校验改为64拍移位加减，不再使用APM乘法器。
// 本版资源以重新综合报告为准。协议细节见 烧录必读.txt 第三节。
// ===========================================================================
`timescale 1ns/1ps
`default_nettype none
// 出厂标定值：上电默认就是这一组，串口不下发也能正常出图。
`include "source/undistort/brown_calibration.vh"

// PARAM-V1 标定参数从机（对应上位机 parameterprotocol.h）
//
// 帧（小端）：
//   A5 5A | ver=01 | cmd | seq[1:0] | len[1:0] | payload[len] | crc32[3:0]
//   crc32 覆盖 ver..payload（不含 A5 5A 与 CRC 自身），多项式 0xEDB88320 反射，
//   初值 0xFFFFFFFF，末尾取反。
//
// 命令与回包（动态鱼眼扩展见 README_动态鱼眼.md）：
//   0x10 写入配置（payload 68 或 96 字节）
//        -> 0x90 [00][crc32(config)[3:0]][generation[3:0]]   9 字节
//        -> 无效配置 0x90 [02]                                1 字节
//   0x12 独立写仿射28字节(flags+6系数)，0x92应答CRC与代次；不修改相机参数。
//   0x13 读当前仿射，0x93返回status+generation+28字节。
//   0x11 读回配置（payload 必须为空）
//        -> 0x91 [00][generation[3:0]][config]                5+68 / 5+96 字节
//        -> payload 非空 [01]；尚未写入 [03]                  1 字节
//
// 配置块字段偏移：
//   0 schema(1/2)  2 model(0 Brown/1 鱼眼)  3 fractionBits(16/20/24)
//   4 width  6 height
//   8 12 16 20  输入 fx fy cx cy   (Q16)
//   24 28 32 36 输出 fx fy cx cy   (Q16)
//   40 44       输出逆 fx fy       (Q30)
//   48..64      Brown: k1/k2/p1/p2/k3；鱼眼: k1/k2/k3/k4/0 (Q fractionBits)
//   [schema 2] 68 flags, 72..92 逆仿射 b00 b01 b02 b10 b11 b12
//
// 支持 Brown 与动态鱼眼，尺寸固定1920x1080。鱼眼提交须先经0x20/0x21/0x22
// 完整上传并验证与本次D系数绑定的4096点表；模型和表银行随配置原子提交。
module param_uart_slave #(
    parameter integer CLK_HZ = 10000000,
    parameter integer BAUD   = 115200,
    parameter integer H_ACTIVE = 1920,
    parameter integer V_ACTIVE = 1080,
    parameter signed [31:0] DEF_FX_Q16 = `V132_FX_Q16,
    parameter signed [31:0] DEF_FY_Q16 = `V132_FY_Q16,
    parameter signed [31:0] DEF_CX_Q16 = `V132_CX_Q16,
    parameter signed [31:0] DEF_CY_Q16 = `V132_CY_Q16,
    parameter signed [31:0] DEF_INV_FX_Q30 = `V132_INV_FX_Q30,
    parameter signed [31:0] DEF_INV_FY_Q30 = `V132_INV_FY_Q30,
    parameter signed [31:0] DEF_K1_Q24 = `V132_K1_Q24,
    parameter signed [31:0] DEF_K2_Q24 = `V132_K2_Q24,
    parameter signed [31:0] DEF_P1_Q24 = `V132_P1_Q24,
    parameter signed [31:0] DEF_P2_Q24 = `V132_P2_Q24
)(
    input  wire clk,
    input  wire rst_n,
    input  wire rx,
    output reg  tx,
    output wire tx_busy,
    // C24 is shared with status text; a reply frame starts only when granted
    // and is then sent whole (see uart_tx_arbiter).
    output wire tx_req,
    input  wire tx_gnt,
    output reg p_fisheye,p_lut_bank,
    input wire frame_lut_bank,
    output reg lut_wr_en,
    output reg [12:0] lut_wr_addr,
    output reg [24:0] lut_wr_data,
    output reg  cfg_apply,                        // 单周期脉冲，cfg_clk 域
    output reg signed [31:0] p_ofx_q16,p_ofy_q16,
    output reg signed [31:0] p_fx_q16, p_fy_q16,
    output reg signed [31:0] p_icx_q16, p_icy_q16,
    output reg signed [31:0] p_ocx_q16, p_ocy_q16,
    output reg signed [31:0] p_ifx_q30, p_ify_q30,
    output reg signed [31:0] p_k1_q24, p_k2_q24,
    output reg signed [31:0] p_p1_q24, p_p2_q24,
    output reg pose_on_pulse,pose_off_pulse,affine_off_pulse,
    output reg p_affine_enable,
    output reg signed [31:0] p_b00_q24,p_b01_q24,p_b02_q16,
    output reg signed [31:0] p_b10_q24,p_b11_q24,p_b12_q16,
    output reg  [31:0] cfg_generation,
    output reg  [15:0] dbg_frames_ok,
    output reg  [15:0] dbg_frames_bad,
    output reg capture_start_toggle, capture_abort_toggle, capture_release_toggle
);
    localparam integer DIV = (CLK_HZ + BAUD/2) / BAUD;
    localparam [3:0] S_HUNT=0, S_HUNT2=1, S_HDR=2, S_PAY=3, S_CRC=4, S_EXEC=5,
                     S_DET=6, S_COMMIT=7, S_LUT_WRITE=8;
    reg [3:0]  st;


    // ---------------- UART 接收 ----------------
    reg rx_meta, rx_sync, rx_busy;
    reg [15:0] rx_cnt;
    reg [3:0]  rx_bit;
    reg [7:0]  rx_shift, rx_byte;
    reg        rx_valid;
    always @(posedge clk) begin
        if (!rst_n) begin
            rx_meta<=1; rx_sync<=1; rx_busy<=0; rx_cnt<=0; rx_bit<=0;
            rx_shift<=0; rx_byte<=0; rx_valid<=0;
        end else begin
            rx_meta<=rx; rx_sync<=rx_meta; rx_valid<=0;
            if (!rx_busy) begin
                if (!rx_sync) begin rx_busy<=1; rx_cnt<=DIV/2; rx_bit<=0; end
            end else if (rx_cnt!=0) rx_cnt<=rx_cnt-1'b1;
            else if (rx_bit==0) begin
                if (rx_sync) rx_busy<=0;
                else begin rx_bit<=1; rx_cnt<=DIV-1; end
            end else if (rx_bit<=8) begin
                rx_shift[rx_bit-1]<=rx_sync; rx_bit<=rx_bit+1'b1; rx_cnt<=DIV-1;
            end else begin
                rx_busy<=0; rx_valid<=1; rx_byte<=rx_shift;
            end
        end
    end

    // ASCII capture commands share the existing UART receiver. Binary PARAM-V1
    // payloads cannot trigger commands: only hunt-state bytes enter this parser.
    reg [127:0] capture_text;
    wire [127:0] capture_next={capture_text[119:0],rx_byte};
    always @(posedge clk) begin
        if (!rst_n) begin
            pose_on_pulse<=0;pose_off_pulse<=0;affine_off_pulse<=0;
            capture_text<=0; capture_start_toggle<=0;
            capture_abort_toggle<=0; capture_release_toggle<=0;
        end else begin
            pose_on_pulse<=0;pose_off_pulse<=0;affine_off_pulse<=0;
            if (rx_valid) begin
            if (st!=S_HUNT || rx_byte==8'hA5) capture_text<=0;
            else begin
                capture_text<=capture_next;
                if(capture_next[63:0]=="POSE ON;")begin pose_on_pulse<=1;capture_text<=0;end
                if(capture_next[71:0]=="POSE OFF;")begin pose_off_pulse<=1;capture_text<=0;end
                if(capture_next[87:0]=="AFFINE OFF;")begin affine_off_pulse<=1;capture_text<=0;end
                if(capture_next[111:0]=="CAPTURE FULL3;") begin
                    capture_start_toggle<=~capture_start_toggle; capture_text<=0;
                end
                if(capture_next[111:0]=="CAPTURE ABORT;") begin
                    capture_abort_toggle<=~capture_abort_toggle; capture_text<=0;
                end
                if(capture_next=="CAPTURE RELEASE;") begin
                    capture_release_toggle<=~capture_release_toggle; capture_text<=0;
                end
            end
        end
    end
    end

    // ---------------- CRC32 ----------------
    function [31:0] crc_byte;
        input [31:0] c;
        input [7:0]  d;
        integer i;
        reg [31:0] x;
        begin
            x = c;
            for (i=0;i<8;i=i+1)
                x = (x>>1) ^ ((x[0]^d[i]) ? 32'hEDB8_8320 : 32'd0);
            crc_byte = x;
        end
    endfunction

    // ---------------- 帧解析 ----------------
    reg [2:0]  hdr_idx;
    reg [15:0] pay_len, pay_idx;
    reg [31:0] crc_frame, crc_pay;
    reg [31:0] crc_rx;
    reg [1:0]  crc_idx;
    reg        crc_ok;
    reg [7:0]  in_cmd;
    reg [15:0] in_seq;
    reg [7:0]  cfg_buf [0:95];
    reg [7:0] active_buf [0:95];
    reg upload_session,upload_ready,upload_bank;
    reg [12:0] upload_next;
    reg [3:0] upload_word;
    reg [31:0] upload_crc,upload_crc_work,upload_expected_crc;
    reg signed [31:0] upload_d0,upload_d1,upload_d2,upload_d3;
    wire [15:0] upload_offset={cfg_buf[1],cfg_buf[0]};
    wire [6:0] upload_byte_offset={upload_word,2'b00}+7'd2;
    wire [31:0] upload_value={cfg_buf[upload_byte_offset+3],cfg_buf[upload_byte_offset+2],cfg_buf[upload_byte_offset+1],cfg_buf[upload_byte_offset]};
    integer check_i;
    reg chunk_values_ok;
    always @* begin
        chunk_values_ok=1;
        for(check_i=0;check_i<16;check_i=check_i+1)
            if((cfg_buf[5+check_i*4]&8'hfe)!=0)chunk_values_ok=0;
    end
    reg [7:0] affine_buf [0:27];
    reg [7:0]  cfg_size;                          // 0 = 尚未写入
    reg [31:0] crc_cfg, crc_affine;

    wire [15:0] schema = {cfg_buf[1], cfg_buf[0]};
    wire [7:0]  model  = cfg_buf[2];
    wire [7:0]  frac   = cfg_buf[3];
    wire [15:0] img_w  = {cfg_buf[5], cfg_buf[4]};
    wire [15:0] img_h  = {cfg_buf[7], cfg_buf[6]};
    wire signed [31:0] v_fx  = {cfg_buf[11],cfg_buf[10],cfg_buf[9], cfg_buf[8]};
    wire signed [31:0] v_fy  = {cfg_buf[15],cfg_buf[14],cfg_buf[13],cfg_buf[12]};
    wire signed [31:0] v_cx  = {cfg_buf[19],cfg_buf[18],cfg_buf[17],cfg_buf[16]};
    wire signed [31:0] v_cy  = {cfg_buf[23],cfg_buf[22],cfg_buf[21],cfg_buf[20]};
    wire signed [31:0] v_ofx = {cfg_buf[27],cfg_buf[26],cfg_buf[25],cfg_buf[24]};
    wire signed [31:0] v_ofy = {cfg_buf[31],cfg_buf[30],cfg_buf[29],cfg_buf[28]};
    wire signed [31:0] v_ocx = {cfg_buf[35],cfg_buf[34],cfg_buf[33],cfg_buf[32]};
    wire signed [31:0] v_ocy = {cfg_buf[39],cfg_buf[38],cfg_buf[37],cfg_buf[36]};
    wire signed [31:0] v_ifx = {cfg_buf[43],cfg_buf[42],cfg_buf[41],cfg_buf[40]};
    wire signed [31:0] v_ify = {cfg_buf[47],cfg_buf[46],cfg_buf[45],cfg_buf[44]};
    wire signed [31:0] v_k1  = {cfg_buf[51],cfg_buf[50],cfg_buf[49],cfg_buf[48]};
    wire signed [31:0] v_k2  = {cfg_buf[55],cfg_buf[54],cfg_buf[53],cfg_buf[52]};
    wire signed [31:0] v_p1  = {cfg_buf[59],cfg_buf[58],cfg_buf[57],cfg_buf[56]};
    wire signed [31:0] v_p2  = {cfg_buf[63],cfg_buf[62],cfg_buf[61],cfg_buf[60]};

    wire [31:0] v_flags=(in_cmd==8'h12) ? {affine_buf[3],affine_buf[2],affine_buf[1],affine_buf[0]} : {cfg_buf[71],cfg_buf[70],cfg_buf[69],cfg_buf[68]};
    wire signed [31:0] v_b00=(in_cmd==8'h12) ? {affine_buf[7],affine_buf[6],affine_buf[5],affine_buf[4]} : {cfg_buf[75],cfg_buf[74],cfg_buf[73],cfg_buf[72]};
    wire signed [31:0] v_b01=(in_cmd==8'h12) ? {affine_buf[11],affine_buf[10],affine_buf[9],affine_buf[8]} : {cfg_buf[79],cfg_buf[78],cfg_buf[77],cfg_buf[76]};
    wire signed [31:0] v_b02=(in_cmd==8'h12) ? {affine_buf[15],affine_buf[14],affine_buf[13],affine_buf[12]} : {cfg_buf[83],cfg_buf[82],cfg_buf[81],cfg_buf[80]};
    wire signed [31:0] v_b10=(in_cmd==8'h12) ? {affine_buf[19],affine_buf[18],affine_buf[17],affine_buf[16]} : {cfg_buf[87],cfg_buf[86],cfg_buf[85],cfg_buf[84]};
    wire signed [31:0] v_b11=(in_cmd==8'h12) ? {affine_buf[23],affine_buf[22],affine_buf[21],affine_buf[20]} : {cfg_buf[91],cfg_buf[90],cfg_buf[89],cfg_buf[88]};
    wire signed [31:0] v_b12=(in_cmd==8'h12) ? {affine_buf[27],affine_buf[26],affine_buf[25],affine_buf[24]} : {cfg_buf[95],cfg_buf[94],cfg_buf[93],cfg_buf[92]};
    // Keep Q16 conversion and centred translations in a bounded range.
    wire affine_range_ok=(v_b00>=-268435456 && v_b00<=268435456) &&
        (v_b01>=-268435456 && v_b01<=268435456) &&
        (v_b10>=-268435456 && v_b10<=268435456) &&
        (v_b11>=-268435456 && v_b11<=268435456) &&
        (v_b02>=-1073741824 && v_b02<=1073741824) &&
        (v_b12>=-1073741824 && v_b12<=1073741824);
    // 配置检查无需像素吞吐率：用64拍精确移位加减代替两个32x32乘法器。
    // 每个乘数的bit31权重为负；第二个乘积再整体取反，累计行列式。
    reg [63:0] det_acc, det_multiplicand;
    reg [31:0] det_multiplier;
    reg [4:0] det_count;
    reg det_phase, det_nonzero;
    wire [63:0] det_term = det_multiplier[0] ? det_multiplicand : 64'd0;
    wire det_subtract = det_phase ^ (det_count == 5'd31);
    wire [63:0] det_next = det_subtract ? det_acc - det_term : det_acc + det_term;
    wire affine_ok=(schema==1) || ((v_flags<=1) &&
        (!v_flags[0] || (affine_range_ok && det_nonzero)));
    wire size_ok = ((schema==16'd1) && (pay_len==16'd68)) ||
                   ((schema==16'd2) && (pay_len==16'd96));
    wire frac_ok = (frac==8'd16) || (frac==8'd20) || (frac==8'd24);
    wire geom_ok = (img_w==H_ACTIVE[15:0]) && (img_h==V_ACTIVE[15:0]);
    wire pos_ok  = (v_fx>0) && (v_fy>0) && (v_ofx>0) &&
                   (v_ofy>0) && (v_ifx>0) && (v_ify>0);
    reg [4:0] d_shift;
    wire signed [31:0] k1_q24 = v_k1 <<< d_shift;
    wire signed [31:0] k2_q24 = v_k2 <<< d_shift;
    wire signed [31:0] p1_q24 = v_p1 <<< d_shift;
    wire signed [31:0] p2_q24 = v_p2 <<< d_shift;
    wire fish_profile_ok=upload_ready && k1_q24==upload_d0 && k2_q24==upload_d1 && p1_q24==upload_d2 && p2_q24==upload_d3;
    wire model_ok=(model==0) || (model==1 && fish_profile_ok && {cfg_buf[67],cfg_buf[66],cfg_buf[65],cfg_buf[64]}==0);
    wire cfg_ok  = size_ok && frac_ok && geom_ok && pos_ok && model_ok && affine_ok;

    wire standalone_ok = pay_len==16'd28 && v_flags<=1 &&
        (!v_flags[0] || (affine_range_ok && det_nonzero));
    wire needs_det_check = crc_ok && v_flags==32'd1 && affine_range_ok &&
        ((in_cmd==8'h10 && size_ok && frac_ok && geom_ok && pos_ok &&
          model_ok && schema==16'd2) || (in_cmd==8'h12 && pay_len==16'd28));


    always @* begin
        case (frac)
            8'd16: d_shift = 5'd8;
            8'd20: d_shift = 5'd4;
            default: d_shift = 5'd0;
        endcase
    end

    // ---------------- 回包 ----------------
    // 发送侧：把整帧看成一个按索引取字节的函数，索引 0..last。
    // 帧头 8 字节 + 载荷 resp_plen + CRC 4 字节。
    reg        resp_pending;
    reg [2:0]  resp_kind; // 0..3 = 90..93, 4..6 = A0..A2
    reg [7:0]  resp_status;
    reg [15:0] resp_seq;
    reg [15:0] resp_plen;
    reg [15:0] plen_end;                          // 第一个 CRC 字节的索引
    reg [15:0] tx_index;
    reg [31:0] crc_tx;
    reg        tx_active;
    reg [9:0]  tx_shift;
    reg [3:0]  tx_bits;
    reg [15:0] tx_cnt;

    assign tx_busy = resp_pending || tx_active;
    // cfg_apply 那一拍让出一次，使同时产生的 "send ok" 先拿到 C24
    assign tx_req  = resp_pending && !tx_active && !cfg_apply;
    wire [15:0] tx_last = plen_end + 16'd3;

    // 2026-10-28：CRC 字节取 cval（已计入最后一个载荷字节的累加值）并做末尾取反，
    // 与上位机 crc() 一致。原先直接取 crc_tx：既漏了取反，装载第一个 CRC 字节时
    // 也还没计入最后一个载荷字节，上位机 Parser 会丢弃全部回包，表现为"等待回包超时"。
    function [7:0] byte_for;
        input [15:0] i;
        input [31:0] cval;
        begin
            if (i==16'd0)      byte_for = 8'hA5;
            else if (i==16'd1) byte_for = 8'h5A;
            else if (i==16'd2) byte_for = 8'h01;
            else if (i==16'd3) byte_for = resp_kind>=4 ? 8'ha0+resp_kind-3'd4 : 8'h90+{5'b0,resp_kind};
            else if (i==16'd4) byte_for = resp_seq[7:0];
            else if (i==16'd5) byte_for = resp_seq[15:8];
            else if (i==16'd6) byte_for = resp_plen[7:0];
            else if (i==16'd7) byte_for = resp_plen[15:8];
            else if (i < plen_end) begin
                if (i==16'd8) byte_for = resp_status;
                else if (resp_kind>=4) begin
                    if(i<=16'd12)byte_for=(upload_crc^32'hffffffff)>>((i-16'd9)*8);
                    else byte_for={19'd0,upload_next}>>((i-16'd13)*8);
                end else if (resp_kind==2'd3) begin
                    if(i<=16'd12) byte_for=cfg_generation[(i-16'd9)*8 +: 8];
                    else case((i-16'd13)>>2)
                        0: byte_for=({31'b0,p_affine_enable} >> ((i-16'd13)%4*8));
                        1: byte_for=p_b00_q24 >> ((i-16'd13)%4*8);
                        2: byte_for=p_b01_q24 >> ((i-16'd13)%4*8);
                        3: byte_for=p_b02_q16 >> ((i-16'd13)%4*8);
                        4: byte_for=p_b10_q24 >> ((i-16'd13)%4*8);
                        5: byte_for=p_b11_q24 >> ((i-16'd13)%4*8);
                        6: byte_for=p_b12_q16 >> ((i-16'd13)%4*8);
                        default: byte_for=0;
                    endcase
                end else if (i<=16'd12) byte_for = resp_kind==2'd1
                    ? cfg_generation[(i-16'd9)*8 +: 8] : (resp_kind==2'd2 ? crc_affine[(i-16'd9)*8 +: 8] : crc_cfg[(i-16'd9)*8 +: 8]);
                else byte_for = resp_kind==2'd1
                    ? active_buf[i-16'd13] : cfg_generation[(i-16'd13)*8 +: 8];
            end else if (i < tx_last + 16'd1)
                byte_for = ~cval[(i-plen_end)*8 +: 8];
            else byte_for = 8'h00;
        end
    endfunction

    wire [31:0] crc_tx_next = (tx_index>=16'd2 && tx_index<plen_end) ?
                              crc_byte(crc_tx, byte_for(tx_index, crc_tx)) : crc_tx;

    integer k;
    always @(posedge clk) begin
        if (!rst_n) begin
            st<=S_HUNT; hdr_idx<=0; pay_len<=0; pay_idx<=0;
            crc_frame<=32'hFFFFFFFF; crc_pay<=32'hFFFFFFFF;
            crc_rx<=0; crc_idx<=0; crc_ok<=0; in_cmd<=0; in_seq<=0;
            cfg_size<=0; crc_cfg<=0; crc_affine<=0; cfg_apply<=0; cfg_generation<=0;
            det_acc<=0; det_multiplicand<=0; det_multiplier<=0;
            det_count<=0; det_phase<=0; det_nonzero<=0;
            p_fisheye<=0;p_lut_bank<=0;lut_wr_en<=0;lut_wr_addr<=0;lut_wr_data<=0;
            upload_session<=0;upload_ready<=0;upload_bank<=1;upload_next<=0;upload_word<=0;
            upload_crc<=32'hffffffff;upload_crc_work<=32'hffffffff;upload_expected_crc<=0;
            upload_d0<=0;upload_d1<=0;upload_d2<=0;upload_d3<=0;
            p_fx_q16<=DEF_FX_Q16; p_fy_q16<=DEF_FY_Q16;
            p_ofx_q16<=DEF_FX_Q16;p_ofy_q16<=DEF_FY_Q16;
            p_icx_q16<=DEF_CX_Q16; p_icy_q16<=DEF_CY_Q16;
            p_ocx_q16<=DEF_CX_Q16; p_ocy_q16<=DEF_CY_Q16;
            p_ifx_q30<=DEF_INV_FX_Q30; p_ify_q30<=DEF_INV_FY_Q30;
            p_k1_q24<=DEF_K1_Q24; p_k2_q24<=DEF_K2_Q24;
            p_p1_q24<=DEF_P1_Q24; p_p2_q24<=DEF_P2_Q24;
            p_affine_enable<=0;p_b00_q24<=16777216;p_b01_q24<=0;p_b02_q16<=0;
            p_b10_q24<=0;p_b11_q24<=16777216;p_b12_q16<=0;
            resp_pending<=0; resp_kind<=0; resp_status<=0; resp_seq<=0; resp_plen<=0;
            plen_end<=0; tx_index<=0; crc_tx<=32'hFFFFFFFF;
            tx<=1'b1; tx_shift<=10'h3ff; tx_bits<=0; tx_cnt<=0; tx_active<=0;
            dbg_frames_ok<=0; dbg_frames_bad<=0;
            // cfg_buf 不复位：cfg_size==0 已经挡住未初始化数据的回读，
            // 让综合器把它推断成分布式 RAM，同时大幅减轻复位网扇出。
        end else begin
            cfg_apply<=1'b0;lut_wr_en<=0;

            // ---- 收字节 ----
            if (rx_valid) begin
                case (st)
                    S_HUNT:  st <= (rx_byte==8'hA5) ? S_HUNT2 : S_HUNT;
                    S_HUNT2: begin
                        // 进入新帧时必须复位头部索引与 CRC 累加器，
                        // 否则上一帧残留的 hdr_idx 会把版本字节当成别的字段。
                        if (rx_byte==8'h5A) begin
                            st<=S_HDR; hdr_idx<=3'd0; crc_frame<=32'hFFFFFFFF;
                        end else st <= (rx_byte==8'hA5) ? S_HUNT2 : S_HUNT;
                    end
                    S_HDR: begin
                        crc_frame <= crc_byte(crc_frame, rx_byte);
                        case (hdr_idx)
                            3'd0: if (rx_byte!=8'h01) st<=S_HUNT; else hdr_idx<=3'd1;
                            3'd1: begin in_cmd<=rx_byte; hdr_idx<=3'd2; end
                            3'd2: begin in_seq[7:0]<=rx_byte; hdr_idx<=3'd3; end
                            3'd3: begin in_seq[15:8]<=rx_byte; hdr_idx<=3'd4; end
                            3'd4: begin pay_len[7:0]<=rx_byte; hdr_idx<=3'd5; end
                            default: begin
                                if (rx_byte!=8'd0 || pay_len>16'd96) st<=S_HUNT;
                                else begin
                                    pay_idx<=0; crc_pay<=32'hFFFFFFFF;upload_crc_work<=upload_crc;
                                    if (pay_len==16'd0) begin
                                        crc_rx<=0; crc_idx<=0; st<=S_CRC;
                                    end else st<=S_PAY;
                                end
                            end
                        endcase
                    end
                    S_PAY: begin
                        if(in_cmd==8'h12) begin
                            if(pay_idx<28) affine_buf[pay_idx[4:0]] <= rx_byte;
                        end else cfg_buf[pay_idx[6:0]] <= rx_byte;
                        crc_frame <= crc_byte(crc_frame, rx_byte);
                        crc_pay   <= crc_byte(crc_pay,   rx_byte);
                        if(in_cmd==8'h21 && pay_idx>=2)upload_crc_work<=crc_byte(upload_crc_work,rx_byte);
                        if (pay_idx==pay_len-1) begin crc_rx<=0; crc_idx<=0; st<=S_CRC; end
                        else pay_idx <= pay_idx + 1'b1;
                    end
                    S_CRC: begin
                        // 小端：第一个字节是 CRC 的低字节
                        crc_rx <= {rx_byte, crc_rx[31:8]};
                        if (crc_idx==2'd3) begin
                            // 上位机 crc() = 原始累加值 ^ 0xFFFFFFFF
                            crc_ok <= ({rx_byte, crc_rx[31:8]} == (crc_frame ^ 32'hFFFF_FFFF));
                            st     <= S_EXEC;
                        end else crc_idx <= crc_idx + 1'b1;
                    end
                    // These states execute without consuming input bytes.
                    S_EXEC, S_DET, S_COMMIT, S_LUT_WRITE: begin end
                    default: st <= S_HUNT;
                endcase
            end

            // A chunk writes only after frame CRC, sequential offset and every value validate.
            if(st==S_LUT_WRITE)begin
                lut_wr_en<=1;
                lut_wr_addr<={upload_bank,upload_next[11:0]}+upload_word;
                lut_wr_data<=upload_value[24:0];
                if(upload_word==15)begin
                    upload_next<=upload_next+13'd16;upload_crc<=upload_crc_work;
                    st<=S_HUNT;resp_kind<=5;resp_status<=0;resp_plen<=9;resp_seq<=in_seq;resp_pending<=1;
                end else upload_word<=upload_word+1'b1;
            end
            // ---- 执行 ----
            if (st==S_EXEC && needs_det_check) begin
                det_acc <= 64'd0;
                det_multiplicand <= {{32{v_b00[31]}},v_b00};
                det_multiplier <= v_b11;
                det_count <= 5'd0;
                det_phase <= 1'b0;
                det_nonzero <= 1'b0;
                st <= S_DET;
            end
            if (st==S_DET) begin
                det_acc <= det_next;
                if (det_count==5'd31) begin
                    if (!det_phase) begin
                        det_multiplicand <= {{32{v_b01[31]}},v_b01};
                        det_multiplier <= v_b10;
                        det_count <= 5'd0;
                        det_phase <= 1'b1;
                    end else begin
                        det_nonzero <= (det_next!=64'd0);
                        st <= S_COMMIT;
                    end
                end else begin
                    det_multiplicand <= det_multiplicand << 1;
                    det_multiplier <= det_multiplier >> 1;
                    det_count <= det_count + 1'b1;
                end
            end
            // At 10 MHz the added check takes 6.5 us, below one UART byte.
            if ((st==S_EXEC && !needs_det_check) || st==S_COMMIT) begin
                st <= S_HUNT;
                if (!crc_ok) dbg_frames_bad <= dbg_frames_bad + 1'b1;
                else begin
                    dbg_frames_ok <= dbg_frames_ok + 1'b1;
                    if(in_cmd==8'h20)begin
                        resp_kind<=4;resp_seq<=in_seq;resp_pending<=1;resp_plen<=9;
                        if(pay_len==20 && p_lut_bank==frame_lut_bank)begin
                            upload_session<=1;upload_ready<=0;upload_bank<=~p_lut_bank;upload_next<=0;upload_crc<=32'hffffffff;
                            upload_d0<={cfg_buf[3],cfg_buf[2],cfg_buf[1],cfg_buf[0]};
                            upload_d1<={cfg_buf[7],cfg_buf[6],cfg_buf[5],cfg_buf[4]};
                            upload_d2<={cfg_buf[11],cfg_buf[10],cfg_buf[9],cfg_buf[8]};
                            upload_d3<={cfg_buf[15],cfg_buf[14],cfg_buf[13],cfg_buf[12]};
                            upload_expected_crc<={cfg_buf[19],cfg_buf[18],cfg_buf[17],cfg_buf[16]};resp_status<=0;
                        end else resp_status<=4;
                    end else if(in_cmd==8'h21)begin
                        if(upload_session && !upload_ready && pay_len==66 && upload_next<4096 && upload_offset=={3'd0,upload_next} && chunk_values_ok)begin
                            upload_word<=0;st<=S_LUT_WRITE;
                        end else begin resp_kind<=5;resp_status<=4;resp_plen<=9;resp_seq<=in_seq;resp_pending<=1;end
                    end else if(in_cmd==8'h22)begin
                        resp_kind<=6;resp_seq<=in_seq;resp_pending<=1;resp_plen<=9;
                        if(upload_session && pay_len==0 && upload_next==4096 && (upload_crc^32'hffffffff)==upload_expected_crc)begin upload_ready<=1;resp_status<=0;end
                        else resp_status<=4;
                    end else if (in_cmd==8'h10) begin
                        if (cfg_ok) begin
                            cfg_size       <= pay_len[7:0];
                            crc_cfg        <= crc_pay ^ 32'hFFFF_FFFF;
                            cfg_generation <= cfg_generation + 1'b1;
                            cfg_apply      <= 1'b1;
                            p_fisheye<=model==1;
                            if(model==1)begin p_lut_bank<=upload_bank;upload_ready<=0;upload_session<=0;end
                            for(k=0;k<96;k=k+1)active_buf[k]<=cfg_buf[k];
                            p_fx_q16<=v_fx;   p_fy_q16<=v_fy;
                            p_ofx_q16<=v_ofx;p_ofy_q16<=v_ofy;
                            p_icx_q16<=v_cx;  p_icy_q16<=v_cy;
                            p_ocx_q16<=v_ocx; p_ocy_q16<=v_ocy;
                            p_ifx_q30<=v_ifx; p_ify_q30<=v_ify;
                            p_k1_q24<=k1_q24; p_k2_q24<=k2_q24;
                            p_p1_q24<=p1_q24; p_p2_q24<=p2_q24;
                            p_affine_enable<=schema==2 && v_flags[0];
                            p_b00_q24<=v_b00;p_b01_q24<=v_b01;p_b02_q16<=v_b02;
                            p_b10_q24<=v_b10;p_b11_q24<=v_b11;p_b12_q16<=v_b12;
                            resp_kind<=1'b0; resp_status<=8'h00; resp_plen<=16'd9;
                        end else begin
                            // 配置非法：直接作废，避免半新半旧的数据被回读出去。
                            resp_kind<=1'b0; resp_status<=8'h02; resp_plen<=16'd1;
                        end
                        resp_seq<=in_seq; resp_pending<=1'b1;
                    end else if (in_cmd==8'h12) begin
                        resp_kind<=2'd2; resp_seq<=in_seq; resp_pending<=1'b1;
                        if(standalone_ok) begin
                            p_affine_enable<=v_flags[0];
                            p_b00_q24<=v_b00;p_b01_q24<=v_b01;p_b02_q16<=v_b02;
                            p_b10_q24<=v_b10;p_b11_q24<=v_b11;p_b12_q16<=v_b12;
                            cfg_generation<=cfg_generation+1'b1;cfg_apply<=1'b1;
                            crc_affine<=crc_pay ^ 32'hFFFFFFFF;
                            resp_status<=0;resp_plen<=9;
                        end else begin resp_status<=2;resp_plen<=1;end
                    end else if (in_cmd==8'h13) begin
                        resp_kind<=2'd3;resp_seq<=in_seq;resp_pending<=1'b1;
                        if(pay_len==0) begin resp_status<=0;resp_plen<=33;end
                        else begin resp_status<=1;resp_plen<=1;end
                    end else if (in_cmd==8'h11) begin
                        if (pay_len!=16'd0) begin
                            resp_kind<=1'b1; resp_status<=8'h01; resp_plen<=16'd1;
                        end else if (cfg_size==8'd0) begin
                            resp_kind<=1'b1; resp_status<=8'h03; resp_plen<=16'd1;
                        end else begin
                            resp_kind<=1'b1; resp_status<=8'h00;
                            resp_plen<=16'd5 + {8'd0, cfg_size};
                        end
                        resp_seq<=in_seq; resp_pending<=1'b1;
                    end
                end
            end

            // ---- 发送：索引 0..tx_last，每字节一个装载/移位循环 ----
            if (!tx_active) begin
                if (resp_pending && tx_gnt) begin
                    resp_pending <= 1'b0;
                    plen_end     <= 16'd8 + resp_plen;
                    tx_index     <= 16'd0;
                    crc_tx       <= 32'hFFFFFFFF;
                    tx_shift     <= {1'b1, 8'hA5, 1'b0};
                    tx           <= 1'b0;
                    tx_cnt       <= 0;
                    tx_bits      <= 0;
                    tx_active    <= 1'b1;
                end
            end else if (tx_cnt==DIV-1) begin
                tx_cnt    <= 0;
                tx_shift  <= {1'b1, tx_shift[9:1]};
                if (tx_bits==9) begin
                    tx_bits <= 0;
                    tx      <= 1'b1;
                    if (tx_index==tx_last) begin
                        tx_active <= 1'b0;
                    end else begin
                        // 先把刚发完的字节计入 CRC，再用更新后的值装载下一字节
                        crc_tx   <= crc_tx_next;
                        tx_index <= tx_index + 1'b1;
                        tx_shift <= {1'b1, byte_for(tx_index + 16'd1, crc_tx_next), 1'b0};
                        tx       <= 1'b0;
                    end
                end else begin
                    tx_bits <= tx_bits + 1'b1;
                    tx      <= tx_shift[1];
                end
            end else tx_cnt <= tx_cnt + 1'b1;
        end
    end
endmodule
`default_nettype wire
