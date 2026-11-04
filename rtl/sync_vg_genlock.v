`timescale 1ns/1ps
`default_nettype none

// 1080p 输出时序发生器 + 帧锁相（genlock）。
//
// 输出像素时钟来自板载 27 MHz 晶振，与 HDMI 输入不同源，结果发布到换屏之间的等待 W
// 原本在 0~16.7 ms 之间随机漂移。本模块时序与 sync_vg 完全一致，只是每帧总行数可以
// 微调（加长/缩短的是场前肩；同步、后肩、有效区不动），把每次"结果发布"锁在下一帧
// 开始之前 MARGIN_LINES 行，使 W 稳定在约 MARGIN_LINES 行。
//
// 相位误差 e = MARGIN - (本帧剩余行数)，单位为行。e>0 表示发布太晚，下一帧加长；
// e<0 表示等待太久，下一帧缩短。缩短最多 V_FP-1 行，误差大时改为反方向加长绕一圈
// 更快（SHORT_RANGE）。增益 1/2，带 ±DEADBAND 死区，无发布的帧保持标称行数。
// GENLOCK_EN=0 时与 sync_vg 逐周期一致。
// hold=1：不改帧长，冻结在当前行数。左侧原图/右侧矫正的对比模式和全屏原图模式下，
// 处理时长与正常模式差得多，锁相会把 ±3 行一路推、刷新率在 59.84~60.16 Hz 之间来回，
// 显示器一旦把这种行数变化当成换时序就会黑屏且反复重同步。诊断模式本来只看几何，
// 帧长冻结、宁可偶发一次撕裂，也不要黑屏。
module sync_vg_genlock #(
    parameter integer X_BITS=12, Y_BITS=12,
    parameter integer V_TOTAL=1125, V_FP=4, V_BP=36, V_SYNC=5, V_ACT=1080,
    parameter integer H_TOTAL=2200, H_FP=88, H_BP=148, H_SYNC=44, H_ACT=1920,
    parameter integer HV_OFFSET=0,
    parameter integer GENLOCK_EN=1,
    parameter integer MARGIN_LINES=48,       // 约 0.71 ms，吸收处理时间的帧间抖动
    // 每帧最多加长/缩短 3 行：刷新率始终在 59.84~60.16 Hz。原先加长上限 16 行（59.2 Hz），
    // 切换对比/原图时处理时间变短约 4 ms，锁相连续几十帧以 59.2 Hz 追相位，显示器当作
    // 换了时序而黑屏重新同步。限幅后重新锁定最长约 3 s，期间画面正常、只是 LAT 稍大。
    parameter integer MAX_LONG=3,
    parameter integer MAX_SHORT=(V_FP-1<3) ? V_FP-1 : 3,   // 场前肩至少留 1 行
    parameter integer DEADBAND=1
)(
    input  wire clk, rstn,
    input  wire publish_toggle,              // fram_buf 结果发布时翻转，core 时钟域
    input  wire hold,                        // 1：冻结当前帧行数（诊断模式用，见下方注释）
    output reg  vs_out, hs_out, de_out, de_re,
    output reg  [X_BITS-1:0] x_act,
    output reg  [Y_BITS-1:0] y_act,
    output reg  [11:0] frame_lines,          // 调试：当前帧总行数
    output reg  signed [11:0] phase_err,     // 调试：最近一次相位误差（行）
    output reg  locked                       // 调试：连续 8 帧 |e|<=16
);
    // 有效区结束行与 sync_vg 的 V_TOTAL-V_FP-1 相同，但不随帧长变化
    localparam integer V_ACT_END   = V_SYNC + V_BP + V_ACT - 1;
    localparam integer SHORT_RANGE = V_TOTAL*MAX_SHORT/(MAX_LONG+MAX_SHORT);

    reg [X_BITS-1:0] h_count;
    reg [Y_BITS-1:0] v_count;
    reg [11:0] cur_vtotal;
    wire line_end  = (h_count == H_TOTAL-1);
    wire frame_end = line_end && (v_count == cur_vtotal-1);

    always @(posedge clk) begin
        if (!rstn) h_count <= 0;
        else if (h_count < H_TOTAL-1) h_count <= h_count + 1'b1;
        else h_count <= 0;
    end
    always @(posedge clk) begin
        if (!rstn) v_count <= 0;
        else if (line_end) begin
            if (v_count == cur_vtotal-1) v_count <= 0;
            else v_count <= v_count + 1'b1;
        end
    end

    // ---------------- 相位检测与帧长调整 ----------------
    // 误差计算拆成 4 级寄存（一帧约 160 万个周期，晚几拍无所谓），位宽 13 位有符号。
    // 帧边界前 8 个周期内才到的发布当作没看到：这时流水还没算完，宁可本帧用标称行数。
    reg [2:0]  pub_sync;
    wire       pub_evt = pub_sync[2] ^ pub_sync[1];
    reg        pub_seen;
    reg [11:0] pub_line;
    reg [3:0]  settle_cnt;                   // 自上次发布后的周期数（饱和），>=8 表示流水已稳定
    reg [3:0]  lock_cnt;
    reg signed [12:0] e0, e1, a2, a3;
    wire signed [12:0] vt_s  = $signed({1'b0, cur_vtotal});
    wire signed [12:0] pl_s  = $signed({1'b0, pub_line});
    always @(posedge clk) begin
        if (!rstn) begin
            e0<=0; e1<=0; a2<=0; a3<=0;
        end else begin
            // 级 1：e = pub_line + MARGIN - vtotal
            e0 <= pl_s + MARGIN_LINES - vt_s;
            // 级 2：误差大且为负时，反方向加长绕一圈更快
            e1 <= (e0 < -SHORT_RANGE) ? e0 + vt_s : e0;
            // 级 3：增益 1/2（向零取整，与 (e+1)/2、-((1-e)/2) 相同）+ 死区
            a2 <= (e1 >  DEADBAND) ? ((e1 + 13'sd1) >>> 1) :
                  (e1 < -DEADBAND) ? -((13'sd1 - e1) >>> 1) : 13'sd0;
            // 级 4：限幅
            a3 <= (a2 >  MAX_LONG)  ?  MAX_LONG :
                  (a2 < -MAX_SHORT) ? -MAX_SHORT : a2;
        end
    end
    // hold 来自 core_clk 域（显示模式寄存器），这里做两级同步再用
    reg hold_meta, hold_sync;
    always @(posedge clk) begin
        if (!rstn) begin hold_meta <= 1'b0; hold_sync <= 1'b0; end
        else begin hold_meta <= hold; hold_sync <= hold_meta; end
    end
    wire use_adj = (GENLOCK_EN!=0) && pub_seen && (settle_cnt >= 4'd8);
    always @(posedge clk) begin
        if (!rstn) begin
            pub_sync<=3'b000; pub_seen<=1'b0; pub_line<=12'd0; settle_cnt<=4'd0;
            cur_vtotal<=V_TOTAL; frame_lines<=V_TOTAL;
            phase_err<=0; locked<=1'b0; lock_cnt<=4'd0;
        end else begin
            pub_sync <= {pub_sync[1:0], publish_toggle};
            if (pub_evt) settle_cnt <= 4'd0;
            else if (settle_cnt != 4'd15) settle_cnt <= settle_cnt + 1'b1;
            if (frame_end) begin
                if (hold_sync) begin
                    frame_lines <= cur_vtotal;
                    phase_err   <= e1[11:0];
                    lock_cnt    <= 4'd0; locked <= 1'b0;
                end else if (use_adj) begin
                    cur_vtotal  <= V_TOTAL + a3;
                    frame_lines <= V_TOTAL + a3;
                    phase_err   <= e1[11:0];
                    if (e1 <= 13'sd16 && e1 >= -13'sd16) begin
                        if (lock_cnt != 4'd15) lock_cnt <= lock_cnt + 1'b1;
                        locked <= (lock_cnt >= 4'd7);
                    end else begin lock_cnt <= 4'd0; locked <= 1'b0; end
                end else begin
                    cur_vtotal <= V_TOTAL; frame_lines <= V_TOTAL;
                end
                // 恰好与帧边界同拍的发布计入新帧第 0 行
                pub_seen <= pub_evt; pub_line <= 12'd0;
            end else if (pub_evt) begin
                pub_seen <= 1'b1; pub_line <= v_count;
            end
        end
    end

    // ---------------- 输出（与 sync_vg 相同）----------------
    always @(posedge clk) begin
        if (!rstn) hs_out <= 1'b0;
        else hs_out <= (h_count < H_SYNC);
    end
    always @(posedge clk) begin
        if (!rstn) vs_out <= 1'b0;
        else if ((v_count == 0) && (h_count == HV_OFFSET)) vs_out <= 1'b1;
        else if ((v_count == V_SYNC) && (h_count == HV_OFFSET)) vs_out <= 1'b0;
    end
    always @(posedge clk) begin
        if (!rstn) de_out <= 1'b0;
        else de_out <= ((v_count >= V_SYNC + V_BP) && (v_count <= V_ACT_END)) &&
                       ((h_count >= H_SYNC + H_BP) && (h_count <= H_TOTAL - H_FP - 1));
    end
    always @(posedge clk) begin
        if (!rstn) de_re <= 1'b0;
        else de_re <= ((v_count >= V_SYNC + V_BP) && (v_count <= V_ACT_END)) &&
                      ((h_count >= H_SYNC + H_BP - 2) && (h_count <= H_TOTAL - H_FP - 3));
    end
    always @(posedge clk) begin
        if (!rstn) x_act <= 0;
        else if (h_count > (H_SYNC + H_BP - 1)) x_act <= h_count - (H_SYNC + H_BP);
        else x_act <= 0;
    end
    always @(posedge clk) begin
        if (!rstn) y_act <= 0;
        else if (v_count > (V_SYNC + V_BP - 1)) y_act <= v_count - (V_SYNC + V_BP);
        else y_act <= 0;
    end
endmodule
`default_nettype wire
