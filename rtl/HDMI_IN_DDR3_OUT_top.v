`timescale 1ns / 1ps
`include "source/undistort/brown_calibration.vh"
module HDMI_IN_DDR3_OUT_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8,
    parameter [1:0] DEMO_MODE     = 2'd0,
    // 1 keeps the on-screen P/R/W/S telemetry, 0 removes it for the release
    // build so the 125 MHz domain matches the timing-clean baseline.
    // PARAM-V1 build: diagnostics are removed from the 125 MHz domain so the
    // UART parameter slave has the best chance of closing timing on PG2L100H.
    parameter integer DIAG_EN     = 0
)(
	input                                sys_clk              ,//27Mhz
    input                                clk_p ,
    input                                clk_n ,
    input                                rst_in ,
    input                                key1_in ,
    input                                key2_in ,
    input                                key3_in ,
    input                                sensor_uart_rx ,
    input                                pc_uart_rx ,
    output                               uart_tx ,

//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
    output                               init_over_rx              , // LED2: reference captured and rotation enabled
//MS72xx       
    output                               rstn_out                  ,
    output                               hd_scl                ,
    inout                                hd_sda                ,
    output                               hdmi_int_led              ,// LED3: affine correction enabled

    //HDMI_in
    input             pixclk_in    ,                            
    input             vs_in    , 
    input             hs_in    , 
    input             de_in    ,
    input     [7:0]   r_in    , 
    input     [7:0]   g_in    , 
    input     [7:0]   b_in    , 
//HDMI_OUT
    output            tmds_clk_n,  // 
    output            tmds_clk_p,  //
    output [2:0]      tmds_data_n, //
    output [2:0]      tmds_data_p, //
//ETH：RTL8211F RGMII（底板已布线；PHY 复位为 G25 rstn_out）
    output            rgmii_txc,
    output            rgmii_tx_ctl,
    output     [3:0]  rgmii_txd,
    input             rgmii_rxc,
    input             rgmii_rx_ctl,
    input      [3:0]  rgmii_rxd
);
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [17:0]                 rstn_1ms            ;   // 名字沿用；现为 20 ms（PHY 复位需 >=10 ms）
    wire[15:0]                  o_rgb565            ;

//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
/////////////////////////////////////////////////////////////////////////////////////

    reg  [7:0]                  r_out;
    reg  [7:0]                  g_out;
    reg  [7:0]                  b_out;
    reg                       vs_out;
    reg                       hs_out;
    reg                       de_out;
//PLL
pll pll_gen_clk (
    .clkin1   (  sys_clk    ),//27MHz
    .clkout1  (  pix_clk_5x    ),//742.5
    .clkout2  (  pix_clk ),    // output 148.5

    .lock (  locked     )
);


   reg         vs_in_1   ;
   reg         hs_in_1   ;
   reg         de_in_1   ;
   reg  [7:0]  r_in_1   ;
   reg  [7:0]  g_in_1   ;
   reg  [7:0]  b_in_1   ;
   reg         vs_in_2   ;
   reg         hs_in_2   ;
   reg         de_in_2   ;
   reg   [7:0] r_in_2   ;
   reg   [7:0] g_in_2   ;
   reg   [7:0] b_in_2   ;


    assign pixclk_out   =  pixclk_in    ;


    always  @(posedge pixclk_out)begin
        if(!rstn_out)begin
    	    vs_in_1       <=  1'b0        ;
            hs_in_1       <=  1'b0        ;
            de_in_1       <=  1'b0        ;
             r_in_1       <=  8'b0        ;
             g_in_1       <=  8'b0        ;
             b_in_1       <=  8'b0        ;
    	    vs_in_2       <=  1'b0        ;
            hs_in_2       <=  1'b0        ;
            de_in_2       <=  1'b0        ;
             r_in_2       <=  8'b0        ;
             g_in_2       <=  8'b0        ;
             b_in_2       <=  8'b0        ;
        end
    	    else begin
            vs_in_1       <=  vs_in        ;
            hs_in_1       <=  hs_in        ;
            de_in_1       <=  de_in        ;
             r_in_1        <=  r_in         ;
             g_in_1        <=  g_in         ;
             b_in_1        <=  b_in         ;
            vs_in_2       <=  vs_in_1        ;
            hs_in_2       <=  hs_in_1        ;
            de_in_2       <=  de_in_1        ;
             r_in_2        <=  r_in_1         ;
             g_in_2        <=  g_in_1         ;
             b_in_2        <=  b_in_1         ;
        end
    end



cfg_pll cfg_pll_inst (
  .clkout0(cfg_clk),    // output
  .lock(),          // output
  .clkin1(sys_clk)       // input
);




// ==== KEY0_MODE_BEGIN ====
// 显示模式三选一（互斥）：
//   上电        -> 矫正后（全屏）
//   KEY0 按下   -> 矫正后 <-> 左右对比；若当时在全屏原图，直接进左右对比
//   KEY3 按下   -> 矫正后 <-> 全屏原图；若当时在左右对比，直接进全屏原图
// 演示流程：上电(矫正后) -> KEY3 切"矫正前"全屏 -> KEY0 切"左右对比"。
// 两个键各产生"按下脉冲"，谁后按谁生效，不会出现按了没反应（旧版
// display_compare = compare && !original 会在原图模式下把 KEY0 吃掉）。
wire compare_req_cfg, original_req_cfg;
reg  compare_req_meta, compare_req_core, compare_req_core_d;
reg  original_req_meta, original_req_core, original_req_core_d;
image_mode_key compare_key_inst(
    .clk(cfg_clk), .rst_n(rstn_out), .key_n(rst_in),
    .show_original(compare_req_cfg)
);
image_mode_key original_key_inst(
    .clk(cfg_clk), .rst_n(rstn_out), .key_n(key3_in),
    .show_original(original_req_cfg)
);
// 翻转位每按一次变一次，跨到 core_clk 后取边沿就是"按下脉冲"
wire key0_press = compare_req_core  ^ compare_req_core_d;
wire key3_press = original_req_core ^ original_req_core_d;
reg mode_compare_core, mode_original_core;
reg fullscreen_meta,fullscreen_sync,fullscreen_seen;
always @(posedge core_clk) begin
    if (!ddr_init_done) begin
        fullscreen_meta<=0;fullscreen_sync<=0;fullscreen_seen<=0;
        compare_req_meta  <= 1'b0; compare_req_core  <= 1'b0; compare_req_core_d  <= 1'b0;
        original_req_meta <= 1'b0; original_req_core <= 1'b0; original_req_core_d <= 1'b0;
        mode_compare_core <= 1'b0; mode_original_core <= 1'b0;
    end else begin
        compare_req_meta  <= compare_req_cfg;  compare_req_core  <= compare_req_meta;  compare_req_core_d  <= compare_req_core;
        original_req_meta <= original_req_cfg; original_req_core <= original_req_meta; original_req_core_d <= original_req_core;
        fullscreen_meta<=fullscreen_toggle_cfg;fullscreen_sync<=fullscreen_meta;fullscreen_seen<=fullscreen_sync;
        if(fullscreen_sync!=fullscreen_seen)begin mode_original_core<=0;mode_compare_core<=0;end
        else if (key3_press) begin
            mode_original_core <= ~mode_original_core;   // 原图开/关
            mode_compare_core  <= 1'b0;                  // 互斥：关掉对比
        end else if (key0_press) begin
            mode_compare_core  <= ~mode_compare_core;    // 对比开/关
            mode_original_core <= 1'b0;                  // 互斥：关掉原图
        end
    end
end
wire display_compare_core  = mode_compare_core;
wire display_original_core = mode_original_core;
// ==== KEY0_MODE_END ====

// KEY2 toggles Gamma. It is independent from the KEY0 layout state.
wire gamma_bypass_cfg;
reg gamma_bypass_core_meta, gamma_bypass_core;
image_mode_key gamma_mode_key_inst(
    .clk(cfg_clk), .rst_n(rstn_out), .key_n(key2_in),
    .show_original(gamma_bypass_cfg)
);
always @(posedge core_clk) begin
    if (!ddr_init_done) begin
        gamma_bypass_core_meta <= 1'b0;
        gamma_bypass_core <= 1'b0;
    end else begin
        gamma_bypass_core_meta <= gamma_bypass_cfg;
        gamma_bypass_core <= gamma_bypass_core_meta;
    end
end

// ---------------------------------------------------------------------------
// IMU pose -> affine chain, ported from the 50K reference project (V13_26).
//   wit_pose_rx -> imu_dynamic_affine -> imu_matrix_affine
// imu_dynamic_affine owns the 0.25 deg (46 count) dead band, the 100 ms update
// period and the candidate/commit handshake; imu_matrix_affine computes the
// Q16.16 centre affine and rejects poses whose optical-axis tilt exceeds the
// model limit (Brown: r22 < 50211, about 40 deg).
// ---------------------------------------------------------------------------
wire signed [15:0] wit_roll,wit_pitch,wit_yaw;
wire signed [31:0] p_ofx_q16,p_ofy_q16;
wire wit_valid,wit_fresh,wit_received;
wit_pose_rx #(.CLK_HZ(10000000),.BAUD(115200),.STALE_MS(500)) pose_rx(
    .clk(cfg_clk),.rst_n(rstn_out),.sensor_rx(sensor_uart_rx),
    .frame_valid(wit_valid),.roll_raw(wit_roll),.pitch_raw(wit_pitch),
    .yaw_raw(wit_yaw),.received_latched(wit_received),.fresh(wit_fresh)
);
wire affine_enable_cfg;
wire pose_on_pulse,pose_off_pulse,affine_off_pulse;
wire key1_short,key1_long;
pose_key_control #(.CLK_HZ(10000000)) pose_key(
 .clk(cfg_clk),.rst_n(rstn_out),.key_n(key1_in),.short_press(key1_short),.long_press(key1_long));
reg pose_user_enabled,manual_blocked;
reg pose_control_update,fullscreen_toggle_cfg;
always @(posedge cfg_clk)begin
 if(!rstn_out)begin pose_user_enabled<=1;manual_blocked<=0;pose_control_update<=0;fullscreen_toggle_cfg<=0;end
 else begin
  pose_control_update<=0;
  if(param_apply)begin manual_blocked<=0;pose_control_update<=1;end
  if(pose_on_pulse || key1_short)begin pose_user_enabled<=1;manual_blocked<=1;pose_control_update<=1;end
  if(pose_off_pulse || key1_long || affine_off_pulse)begin
   pose_user_enabled<=0;pose_control_update<=1;fullscreen_toggle_cfg<=~fullscreen_toggle_cfg;
  end
  if(affine_off_pulse)manual_blocked<=1;
 end
end
wire signed [31:0] affine_a00_cfg,affine_a01_cfg,affine_a02_cfg;
wire signed [31:0] affine_a10_cfg,affine_a11_cfg,affine_a12_cfg;
wire affine_frame_valid;
wire pose_legal_cfg,debug_calibrated;
wire signed [15:0] debug_roll,debug_pitch,debug_yaw;
wire signed [15:0] debug_base_roll,debug_base_pitch,debug_base_yaw;
wire signed [15:0] debug_delta_roll=debug_roll-debug_base_roll;
wire signed [15:0] debug_delta_pitch=debug_pitch-debug_base_pitch;
wire signed [15:0] debug_delta_yaw=debug_yaw-debug_base_yaw;
wire pose_valid_cfg=wit_fresh;
imu_dynamic_affine #(
    .CLK_HZ(10000000),.UPDATE_PERIOD_MS(100),.DEADBAND_RAW(46)
) pose_affine (
    .clk(cfg_clk),.rst_n(rstn_out),
    .sample_valid(wit_valid),.imu_alive(wit_fresh),
    .roll_raw(wit_roll),.pitch_raw(wit_pitch),.yaw_raw(wit_yaw),
    .model(p_fisheye ? 2'd1 : 2'd2),
    .fx_q16(p_ofx_q16),.fy_q16(p_ofy_q16),
    .zero_key_n(1'b1),.zero_request(key1_short),.commit_ready(1'b1),
    .affine_enable(affine_enable_cfg),
    .a00(affine_a00_cfg),.a01(affine_a01_cfg),.a02(affine_a02_cfg),
    .a10(affine_a10_cfg),.a11(affine_a11_cfg),.a12(affine_a12_cfg),
    .affine_rebuild(affine_frame_valid),
    .calibrated(debug_calibrated),.pose_legal(pose_legal_cfg),
    .latest_roll_raw(debug_roll),.latest_pitch_raw(debug_pitch),
    .latest_yaw_raw(debug_yaw),
    .base_roll_raw(debug_base_roll),.base_pitch_raw(debug_base_pitch),
    .base_yaw_raw(debug_base_yaw)
);
// The projective homography path is not used any more: fram_buf always takes
// the affine coefficients while the correction is enabled.
wire signed [31:0] homography_h00_cfg=32'sd65536,homography_h01_cfg=32'sd0,homography_h02_cfg=32'sd0;
wire signed [31:0] homography_h10_cfg=32'sd0,homography_h11_cfg=32'sd65536,homography_h12_cfg=32'sd0;
wire signed [31:0] homography_h20_cfg=32'sd0,homography_h21_cfg=32'sd0,homography_h22_cfg=32'sd65536;
wire debug_fresh=wit_fresh;
// The four metrics live in the 125 MHz domain and update once per frame. Latch
// them here on the report toggle; two sync stages guarantee the data has been
// settled for well over one 10 MHz period before the edge arrives.
reg [15:0] perf_process_q, perf_period_q, perf_capture_q, perf_publish_q;
reg [3:0]  perf_stage_q;
reg perf_toggle_meta, perf_toggle_sync, perf_toggle_seen;
always @(posedge cfg_clk) begin
    if (!rstn_out) begin
        perf_toggle_meta<=0; perf_toggle_sync<=0; perf_toggle_seen<=0;
        perf_process_q<=0; perf_period_q<=0; perf_capture_q<=0;
        perf_publish_q<=0; perf_stage_q<=0;
    end else begin
        perf_toggle_meta<=undistort_report_toggle;
        perf_toggle_sync<=perf_toggle_meta;
        if (perf_toggle_sync!=perf_toggle_seen) begin
            perf_toggle_seen<=perf_toggle_sync;
            perf_process_q<=measured_process_10us;
            perf_period_q<=measured_result_period_10us;
            perf_capture_q<=measured_capture_wait_10us;
            perf_publish_q<=measured_publish_wait_10us;
            perf_stage_q<=undistort_report_stage;
        end
    end
end

// Swap interval measured entirely in the 10 MHz domain: independent of the
// 125 MHz divider, so it is the reference the other numbers are checked against.
reg swap_toggle_meta, swap_toggle_sync, swap_toggle_seen;
reg [9:0] swap_div;
reg [17:0] swap_10us, swap_hold;
always @(posedge cfg_clk) begin
    if (!rstn_out) begin
        swap_toggle_meta<=0; swap_toggle_sync<=0; swap_toggle_seen<=0;
        swap_div<=0; swap_10us<=0; swap_hold<=0;
    end else begin
        swap_toggle_meta<=undistort_swap_toggle;
        swap_toggle_sync<=swap_toggle_meta;
        if (swap_toggle_sync!=swap_toggle_seen) begin
            swap_toggle_seen<=swap_toggle_sync;
            swap_hold<=swap_10us;
            swap_10us<=0; swap_div<=0;
        end else if (swap_div==10'd99) begin
            swap_div<=0;
            if (swap_10us!=18'h3ffff) swap_10us<=swap_10us+1'b1;
        end else swap_div<=swap_div+1'b1;
    end
end

// Debug text (uart_pose_debug) is instantiated next to the PARAM-V1 slave and
// shares C24 through uart_tx_arbiter; it stays silent until the first "adjust ok".
reg affine_enable_core;
reg pose_valid_meta,pose_valid_core;
reg pose_legal_meta,pose_legal_core;
reg signed [31:0] affine_a00_core,affine_a01_core,affine_a02_core;
reg signed [31:0] affine_a10_core,affine_a11_core,affine_a12_core;
reg signed [31:0] homography_h00_core,homography_h01_core,homography_h02_core;
reg signed [31:0] homography_h10_core,homography_h11_core,homography_h12_core;
reg signed [31:0] homography_h20_core,homography_h21_core,homography_h22_core;
wire manual_enable_cfg;
wire signed [31:0] manual_b00_cfg,manual_b01_cfg,manual_b02_cfg;
wire signed [31:0] manual_b10_cfg,manual_b11_cfg,manual_b12_cfg;
wire combined_enable_cfg,combined_valid_cfg;
wire signed [31:0] combined_00,combined_01,combined_02,combined_10,combined_11,combined_12;
affine_compose_cfg manual_pose_composer(
 .clk(cfg_clk),.rst_n(rstn_out),.update(affine_frame_valid || pose_control_update),
 .manual_enable(manual_enable_cfg && !manual_blocked),.pose_enable(affine_enable_cfg && pose_user_enabled),
 .cx(p_ocx_q16),.cy(p_ocy_q16),
 .b00(manual_b00_cfg),.b01(manual_b01_cfg),.b02(manual_b02_cfg),
 .b10(manual_b10_cfg),.b11(manual_b11_cfg),.b12(manual_b12_cfg),
 .p00(affine_a00_cfg),.p01(affine_a01_cfg),.p02(affine_a02_cfg),
 .p10(affine_a10_cfg),.p11(affine_a11_cfg),.p12(affine_a12_cfg),
 .valid(combined_valid_cfg),.busy(),.enabled(combined_enable_cfg),.error(),
 .c00(combined_00),.c01(combined_01),.c02(combined_02),
 .c10(combined_10),.c11(combined_11),.c12(combined_12));
reg signed [31:0] cal_fx_core,cal_fy_core,cal_ifx_core,cal_ify_core;
reg signed [31:0] cal_icx_core,cal_icy_core,cal_ocx_core,cal_ocy_core;
reg signed [31:0] cal_k1_core,cal_k2_core,cal_p1_core,cal_p2_core;
reg [7:0] cal_gen_core,cal_gen_cfg;
always @(posedge cfg_clk) begin
 if(!rstn_out)cal_gen_cfg<=0;
 else if(param_apply)cal_gen_cfg<=cal_gen_cfg+1'b1;
end
wire p_fisheye; reg cal_fisheye_core; wire csm_fisheye;
wire p_lut_bank,csm_lut_bank,lut_wr_en;
wire [12:0] lut_wr_addr;wire [24:0] lut_wr_data;
reg cal_lut_bank_core;
(* async_reg="true" *) reg csm_bank_meta_cfg,csm_bank_sync_cfg;
always @(posedge cfg_clk) begin
    if(!rstn_out)begin csm_bank_meta_cfg<=0;csm_bank_sync_cfg<=0;end
    else begin csm_bank_meta_cfg<=csm_lut_bank;csm_bank_sync_cfg<=csm_bank_meta_cfg;end
end
localparam integer POSE_BUNDLE_WIDTH=875;
wire [POSE_BUNDLE_WIDTH-1:0] pose_bundle_cfg={combined_enable_cfg,
    combined_00,combined_01,combined_02,
    combined_10,combined_11,combined_12,
    homography_h00_cfg,homography_h01_cfg,homography_h02_cfg,
    homography_h10_cfg,homography_h11_cfg,homography_h12_cfg,
    homography_h20_cfg,homography_h21_cfg,homography_h22_cfg,
    p_fx_q16,p_fy_q16,p_ifx_q30,p_ify_q30,p_icx_q16,p_icy_q16,p_ocx_q16,p_ocy_q16,
    p_k1_q24,p_k2_q24,p_p1_q24,p_p2_q24,cal_gen_cfg,p_fisheye,p_lut_bank};
reg [POSE_BUNDLE_WIDTH-1:0] pose_bundle_hold_cfg,pose_bundle_pending_cfg;
reg pose_req_toggle_cfg,pose_pending_valid_cfg;
reg pose_ack_meta_cfg,pose_ack_sync_cfg;
reg pose_req_meta_core,pose_req_sync_core,pose_req_seen_core,pose_ack_toggle_core;
wire pose_cdc_idle_cfg=pose_req_toggle_cfg==pose_ack_sync_cfg;
always @(posedge cfg_clk) begin
    if(!rstn_out) begin
        pose_ack_meta_cfg<=0;pose_ack_sync_cfg<=0;
        pose_bundle_hold_cfg<=0;pose_bundle_pending_cfg<=0;
        pose_req_toggle_cfg<=0;pose_pending_valid_cfg<=0;
    end else begin
        pose_ack_meta_cfg<=pose_ack_toggle_core;
        pose_ack_sync_cfg<=pose_ack_meta_cfg;
        if(combined_valid_cfg) begin
            if(pose_cdc_idle_cfg) begin
                pose_bundle_hold_cfg<=pose_bundle_cfg;
                pose_req_toggle_cfg<=~pose_req_toggle_cfg;
                pose_pending_valid_cfg<=0;
            end else begin
                pose_bundle_pending_cfg<=pose_bundle_cfg;
                pose_pending_valid_cfg<=1;
            end
        end else if(pose_pending_valid_cfg && pose_cdc_idle_cfg) begin
            pose_bundle_hold_cfg<=pose_bundle_pending_cfg;
            pose_req_toggle_cfg<=~pose_req_toggle_cfg;
            pose_pending_valid_cfg<=0;
        end
    end
end
always @(posedge core_clk) begin
    if(!ddr_init_done) begin
        affine_enable_core<=0;
        cal_gen_core<=0;cal_fisheye_core<=0;cal_lut_bank_core<=0;
        cal_fx_core<=`V132_FX_Q16;cal_fy_core<=`V132_FY_Q16;
        cal_ifx_core<=`V132_INV_FX_Q30;cal_ify_core<=`V132_INV_FY_Q30;
        cal_icx_core<=`V132_CX_Q16;cal_icy_core<=`V132_CY_Q16;
        cal_ocx_core<=`V132_CX_Q16;cal_ocy_core<=`V132_CY_Q16;
        cal_k1_core<=`V132_K1_Q24;cal_k2_core<=`V132_K2_Q24;
        cal_p1_core<=`V132_P1_Q24;cal_p2_core<=`V132_P2_Q24;
        pose_valid_meta<=0;pose_valid_core<=0;pose_legal_meta<=0;pose_legal_core<=0;
        affine_a00_core<=65536;affine_a01_core<=0;affine_a02_core<=0;
        affine_a10_core<=0;affine_a11_core<=65536;affine_a12_core<=0;
        homography_h00_core<=65536;homography_h01_core<=0;homography_h02_core<=0;
        homography_h10_core<=0;homography_h11_core<=65536;homography_h12_core<=0;
        homography_h20_core<=0;homography_h21_core<=0;homography_h22_core<=65536;
        pose_req_meta_core<=0;pose_req_sync_core<=0;pose_req_seen_core<=0;pose_ack_toggle_core<=0;
    end else begin
        pose_valid_meta<=pose_valid_cfg;pose_valid_core<=pose_valid_meta;
        pose_legal_meta<=pose_legal_cfg;pose_legal_core<=pose_legal_meta;
        pose_req_meta_core<=pose_req_toggle_cfg;
        pose_req_sync_core<=pose_req_meta_core;
        if(pose_req_sync_core!=pose_req_seen_core) begin
            {affine_enable_core,affine_a00_core,affine_a01_core,affine_a02_core,
                affine_a10_core,affine_a11_core,affine_a12_core,
                homography_h00_core,homography_h01_core,homography_h02_core,
                homography_h10_core,homography_h11_core,homography_h12_core,
                homography_h20_core,homography_h21_core,homography_h22_core,
                cal_fx_core,cal_fy_core,cal_ifx_core,cal_ify_core,
                cal_icx_core,cal_icy_core,cal_ocx_core,cal_ocy_core,
                cal_k1_core,cal_k2_core,cal_p1_core,cal_p2_core,cal_gen_core,cal_fisheye_core,cal_lut_bank_core}<=pose_bundle_hold_cfg;
            pose_req_seen_core<=pose_req_sync_core;
            pose_ack_toggle_core<=pose_req_sync_core;
        end
    end
end

ms72xx_ctl ms72xx_ctl(
    .clk         (  cfg_clk    ), //input       clk,
    .rst_n       (  rstn_out   ), //input       rstn,
           
    .init_over_rx(  rx_init_done),                 
    .init_over   (  init_over  ), //output      init_over,
    .iic_scl     (  hd_scl    ), //output      iic_scl,
    .iic_sda     (  hd_sda    )  //inout       iic_sda
);
    // Physical LED diagnostics.
    // LED2 (C19): off = stale/missing pose; blink = waiting for KEY1 or
    // relative pose outside the supported local-affine range; steady = legal.
    // LED3 (E18): a fresh, calibrated, legal correction is active.
    //
    // The LED state is generated in cfg_clk, which is constrained to 10 MHz.
    // One toggle every 5,000,000 cycles gives a 0.5 s half-period and a 1 Hz
    // blink rate.  This avoids using core_clk, whose exact frequency depends
    // on the DDR implementation.
    localparam [22:0] POSE_LED_HALF_PERIOD = 23'd4_999_999;
    reg [22:0] pose_led_count;
    reg pose_led_blink;
    always @(posedge cfg_clk) begin
        if (!rstn_out) begin
            pose_led_count <= 23'd0;
            pose_led_blink <= 1'b0;
        end else if (pose_valid_cfg && !pose_legal_cfg) begin
            if (pose_led_count == POSE_LED_HALF_PERIOD) begin
                pose_led_count <= 23'd0;
                pose_led_blink <= ~pose_led_blink;
            end else begin
                pose_led_count <= pose_led_count + 1'b1;
            end
        end else begin
            pose_led_count <= 23'd0;
            pose_led_blink <= 1'b0;
        end
    end
    assign init_over_rx = !pose_valid_cfg ? 1'b0 :
                          pose_legal_cfg ? 1'b1 : pose_led_blink;
    assign hdmi_int_led = affine_enable_cfg;
    
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 18'd0;
    	else
    	begin
    		if(rstn_1ms == 18'd200000)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 18'd200000);


wire    [15:0]    hdmi_data_in;

wire [23:0] fps_rgb_out;
wire [7:0]  measured_fps;
wire [3:0]  measured_fps_tens;
wire [3:0]  measured_fps_ones;
wire [3:0] measured_fps_frac_tens, measured_fps_frac_ones;
wire        measured_fps_update;
wire [19:0] measured_latency_bcd;
wire measured_latency_valid, measured_latency_overflow;
wire [15:0] measured_process_10us /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
wire [15:0] measured_result_period_10us /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
wire [15:0] measured_publish_wait_10us /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
wire [15:0] measured_capture_wait_10us /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
wire [3:0]  undistort_report_stage;
wire        undistort_report_toggle;
wire        undistort_swap_toggle;
wire        undistort_publish_toggle;
// ---------------- 以太网同帧采集（UDP 6002，ARP/ping 应答，丢包补发）----------------
wire        eth_freeze_req, eth_frozen, eth_ext_own, eth_ext_valid, eth_ext_ready, eth_ext_rvalid, eth_ext_rlast;
wire [2:0] eth_frozen_gamma_curve;
wire [27:0] eth_frozen_raw_base, eth_frozen_res_base, eth_ext_addr;
wire [3:0]  eth_ext_len;
wire [255:0] eth_ext_rdata;
wire fb_frozen, fb_ready, fb_rvalid, fb_rlast;
wire [255:0] fb_rdata;
reg [1:0] snapshot_owner;
wire hdmi_owned,hdmi_request,hdmi_release,hdmi_reader_busy;
reg hdmi_freeze;
wire [27:0] hdmi_addr;
wire [3:0] hdmi_len;
wire hdmi_valid;
wire h_export_req,h_export_accept,h_export_valid,h_export_done;
wire [1:0] h_export_kind;
wire [31:0] h_export_offset;
wire [255:0] h_export_data;
wire cap_start_t,cap_abort_t,cap_release_t;
(* async_reg="true" *) reg [2:0] cap_start_s,cap_abort_s,cap_release_s;
wire cap_start=cap_start_s[2]^cap_start_s[1];
wire cap_abort=cap_abort_s[2]^cap_abort_s[1];
wire cap_release=cap_release_s[2]^cap_release_s[1];
wire fb_freeze=snapshot_owner==2 ? hdmi_freeze : snapshot_owner==1 ? eth_freeze_req : 1'b0;
wire fb_own=snapshot_owner==2 ? fb_frozen : snapshot_owner==1 ? eth_ext_own : 1'b0;
wire [27:0] fb_addr=snapshot_owner==2 ? hdmi_addr : eth_ext_addr;
wire [3:0] fb_len=snapshot_owner==2 ? hdmi_len : eth_ext_len;
wire fb_valid=snapshot_owner==2 ? hdmi_valid : snapshot_owner==1 && eth_ext_valid;
assign eth_frozen=fb_frozen && snapshot_owner==1;
assign eth_ext_ready=fb_ready && snapshot_owner==1;
assign eth_ext_rvalid=fb_rvalid && snapshot_owner==1;
assign eth_ext_rlast=fb_rlast && snapshot_owner==1;
assign eth_ext_rdata=fb_rdata;
always @(posedge core_clk) begin
    if(!ddr_init_done) begin
        snapshot_owner<=0; hdmi_freeze<=0;
        cap_start_s<=0;cap_abort_s<=0;cap_release_s<=0;
    end else begin
        cap_start_s<={cap_start_s[1:0],cap_start_t};
        cap_abort_s<={cap_abort_s[1:0],cap_abort_t};
        cap_release_s<={cap_release_s[1:0],cap_release_t};
        if(snapshot_owner==0) begin
            if(hdmi_owned) snapshot_owner<=2;
            else if(eth_freeze_req) snapshot_owner<=1;
        end else if(snapshot_owner==1) begin
            if(!eth_freeze_req && !eth_ext_own) snapshot_owner<=0;
        end else if(!hdmi_owned && !hdmi_reader_busy) snapshot_owner<=0;
        if(hdmi_request) hdmi_freeze<=1;
        if(hdmi_release) hdmi_freeze<=0;
    end
end
wire [7:0] csm_read_addr,csm_count,csm_generation;
wire [7:0] hdmi_csm_addr,eth_csm_addr;
assign csm_read_addr=snapshot_owner==1 ? eth_csm_addr : hdmi_csm_addr;
wire [63:0] csm_read_data;
wire [383:0] csm_calibration;
wire [191:0] csm_affine;
wire csm_affine_enabled,csm_complete;
wire [3:0] csm_gamma;
// SFH2 legacy profile view is derived from the same frozen native parameters.
wire [287:0] csm_words={32'd65536,csm_calibration[383:352],csm_calibration[351:320],
    csm_calibration[319:288],csm_calibration[287:256],csm_calibration[191:160],
    csm_calibration[159:128],csm_calibration[63:32],csm_calibration[31:0]};
wire hvs,hhs,hde,h_digital;
wire [15:0] h_rgb;
hdmi_full_transport #(.REPEATS(8)) hdmi_transport(
    .clk(core_clk),.rst_n(ddr_init_done),.enable(1'b0),
    .allowed(snapshot_owner==0 && !eth_freeze_req),
    .one_shot_start(cap_start),.abort(cap_abort),.release_cmd(cap_release),
    .capture_state(hdmi_freeze ? (fb_frozen && snapshot_owner==2 ? 4'd6:4'd1):4'd0),
    .frame_id(undistort_publish_count),.errors(32'd0),
    .active_words(csm_words),.inv_fx(csm_calibration[95:64]),.inv_fy(csm_calibration[127:96]),
    .active_sequence({8'd0,csm_generation}),.model(csm_fisheye ? 2'd0 : 2'd1),
    .coord_read_addr(hdmi_csm_addr),.coord_read_data(csm_read_data),
    .coord_calibration(csm_calibration),.coord_affine(csm_affine),
    .coord_affine_enabled(csm_affine_enabled),.coord_complete(csm_complete),
    .coord_sample_count(csm_count),.coord_generation(csm_generation),.coord_gamma(csm_gamma),
    .owned(hdmi_owned),.capture_request(hdmi_request),.capture_release(hdmi_release),
    .export_req(h_export_req),.export_kind(h_export_kind),.export_offset(h_export_offset),
    .export_accept(h_export_accept),.export_valid(h_export_valid),
    .export_done(h_export_done),.export_data(h_export_data),
    .pclk(pix_clk),.prst_n(rstn_out),.vs(vs_out),.hs(hs_out),.de(de_out),
    .rgb({r_out[7:3],g_out[7:2],b_out[7:3]}),
    .ovs(hvs),.ohs(hhs),.ode(hde),.orgb(h_rgb),.digital(h_digital));
hdmi_capture_reader hdmi_reader(
    .clk(core_clk),.rst_n(ddr_init_done),.req(h_export_req),.kind(h_export_kind),
    .offset(h_export_offset),.raw_base(eth_frozen_raw_base),.res_base(eth_frozen_res_base),
    .gamma_curve(eth_frozen_gamma_curve),.accept(h_export_accept),.valid(h_export_valid),
    .done(h_export_done),.data(h_export_data),.busy(hdmi_reader_busy),
    .ext_addr(hdmi_addr),.ext_len(hdmi_len),.ext_valid(hdmi_valid),
    .ext_ready(fb_ready && snapshot_owner==2),.ext_rdata(fb_rdata),
    .ext_rvalid(fb_rvalid && snapshot_owner==2),.ext_rlast(fb_rlast && snapshot_owner==2));
wire [15:0] eth_rx_frames, eth_rx_fcs_bad, eth_rx_cmd;
eth_top eth_capture (
    .clk(core_clk), .rst_n(ddr_init_done),
    .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
    .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
    .freeze_req(eth_freeze_req), .frozen(eth_frozen),
    .frozen_raw_base(eth_frozen_raw_base), .frozen_res_base(eth_frozen_res_base),.frozen_gamma_curve(eth_frozen_gamma_curve),
        .coord_read_addr(eth_csm_addr),.coord_read_data(csm_read_data),
        .coord_fisheye(csm_fisheye),.coord_calibration(csm_calibration),.coord_affine(csm_affine),
        .coord_affine_enabled(csm_affine_enabled),.coord_complete(csm_complete),
        .coord_sample_count(csm_count),.coord_generation(csm_generation),.coord_gamma(csm_gamma),
    .ext_own(eth_ext_own), .ext_addr(eth_ext_addr), .ext_len(eth_ext_len), .ext_valid(eth_ext_valid),
    .ext_ready(eth_ext_ready), .ext_rdata(eth_ext_rdata), .ext_rvalid(eth_ext_rvalid), .ext_rlast(eth_ext_rlast),
    .dbg_rx_frames(eth_rx_frames), .dbg_rx_fcs_bad(eth_rx_fcs_bad), .dbg_rx_cmd(eth_rx_cmd));

reg [4:0]test_b;
always @(posedge pixclk_in)
begin
    if(!rstn_out)
        test_b <= 5'd0;
    else if(de_in_2)
        test_b <= test_b + 1'd1;
    else
        test_b <= 5'd0;
end

assign    hdmi_data_in = {r_in_2[7:3],g_in_2[7:2],b_in_2[7:3]};


wire    vs_reg;
wire    hs_reg;
wire    rd_en ;


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//婵犵數鍎戠徊钘壝归崒鐐茬獥婵°倕鎳庨弸浣虹磼椤栨粎澧眗闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄柨鐣烽鐐茬婵犮垹瀚崰鏍垂妤ｅ啯鎯炴い鎰垫線濞ｇ晬1
    wire [3:0] undistort_stage /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    wire undistort_error /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    wire undistort_map_ready /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    wire undistort_busy /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    wire [31:0] undistort_publish_count /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    wire [31:0] undistort_last_process_cycles /* synthesis PAP_MARK_DEBUG="1", syn_keep=1 */;
    // Isolate the video reset fanout from the DDR PHY initialization register.
    // Video reset follows ddr_init_done by one core clock (8 ns).
    reg video_ddr_ready=1'b0;
    always @(posedge core_clk) begin
        video_ddr_ready <= ddr_init_done;
    end

    // PARAM-V1 runtime Brown calibration over the on-board CP2102 Type-C UART.
    // C24 carries protocol replies, status text and (after "adjust ok") debug text.
    // ==== PARAM_C24_BEGIN (tb_c24 仿真直接截取到 PARAM_C24_END 为止) ====
    wire signed [31:0] p_fx_q16,p_fy_q16,p_icx_q16,p_icy_q16;
    wire signed [31:0] p_ocx_q16,p_ocy_q16,p_ifx_q30,p_ify_q30;
    wire signed [31:0] p_k1_q24,p_k2_q24,p_p1_q24,p_p2_q24;
    wire param_tx,param_tx_busy,param_apply,param_tx_req,param_tx_gnt;
    param_uart_slave #(.CLK_HZ(10000000),.BAUD(115200)) param_slave(
        .clk(cfg_clk),.rst_n(rstn_out),.rx(pc_uart_rx),.tx(param_tx),
        .tx_busy(param_tx_busy),.tx_req(param_tx_req),.tx_gnt(param_tx_gnt),
        .cfg_apply(param_apply),.p_fisheye(p_fisheye),.p_lut_bank(p_lut_bank),.frame_lut_bank(csm_bank_sync_cfg),.lut_wr_en(lut_wr_en),.lut_wr_addr(lut_wr_addr),.lut_wr_data(lut_wr_data),
        .pose_on_pulse(pose_on_pulse),.pose_off_pulse(pose_off_pulse),.affine_off_pulse(affine_off_pulse),
        .p_ofx_q16(p_ofx_q16),.p_ofy_q16(p_ofy_q16),
        .p_affine_enable(manual_enable_cfg),
        .p_b00_q24(manual_b00_cfg),.p_b01_q24(manual_b01_cfg),.p_b02_q16(manual_b02_cfg),
        .p_b10_q24(manual_b10_cfg),.p_b11_q24(manual_b11_cfg),.p_b12_q16(manual_b12_cfg),
        .capture_start_toggle(cap_start_t),.capture_abort_toggle(cap_abort_t),.capture_release_toggle(cap_release_t),
        .p_fx_q16(p_fx_q16),.p_fy_q16(p_fy_q16),
        .p_icx_q16(p_icx_q16),.p_icy_q16(p_icy_q16),
        .p_ocx_q16(p_ocx_q16),.p_ocy_q16(p_ocy_q16),
        .p_ifx_q30(p_ifx_q30),.p_ify_q30(p_ify_q30),
        .p_k1_q24(p_k1_q24),.p_k2_q24(p_k2_q24),
        .p_p1_q24(p_p1_q24),.p_p2_q24(p_p2_q24),
        .cfg_generation(),.dbg_frames_ok(),.dbg_frames_bad());
    // 上电后 C24 静默。参数帧被接收 -> "send ok" + 0x90 回包；用新参数矫正的
    // 第一帧上屏 -> "adjust ok"；此后开始 10 Hz 调试文本。三路由仲裁器整帧/整行切换。
    // display_cal_gen 每帧最多变一次，经两级同步后在 cfg_clk 域检测变化。
    wire [7:0] display_cal_gen;
    reg [7:0] adj_gen_seen_core;
    reg       adj_toggle_core;
    always @(posedge core_clk) begin
        if (!ddr_init_done) begin adj_gen_seen_core<=8'd0; adj_toggle_core<=1'b0; end
        else if (display_cal_gen!=adj_gen_seen_core) begin
            adj_gen_seen_core<=display_cal_gen; adj_toggle_core<=~adj_toggle_core;
        end
    end
    reg adj_meta, adj_sync, adj_seen, adjusted_once;
    wire adjust_evt = adj_sync!=adj_seen;
    always @(posedge cfg_clk) begin
        if (!rstn_out) begin adj_meta<=0; adj_sync<=0; adj_seen<=0; adjusted_once<=0; end
        else begin
            adj_meta<=adj_toggle_core; adj_sync<=adj_meta; adj_seen<=adj_sync;
            if (adjust_evt) adjusted_once<=1'b1;
        end
    end
    wire msg_tx,msg_req,msg_busy,msg_gnt;
    uart_status_msg #(.CLK_HZ(10000000),.BAUD(115200)) status_msg(
        .clk(cfg_clk),.rst_n(rstn_out),.send_evt(param_apply),.adjust_evt(adjust_evt),
        .tx_req(msg_req),.tx_gnt(msg_gnt),.line_busy(msg_busy),.tx(msg_tx));
    wire dbg_tx,dbg_req,dbg_busy,dbg_gnt;
    uart_pose_debug pose_debug_inst(
        .clk(cfg_clk),.rst_n(rstn_out),.roll_raw(debug_roll),
        .pitch_raw(debug_pitch),.yaw_raw(debug_yaw),.delta_roll_raw(debug_delta_roll),
        .delta_pitch_raw(debug_delta_pitch),
        .delta_yaw_raw(debug_delta_yaw),
        .enabled(affine_enable_cfg),.fresh(debug_fresh),
        .process_10us(perf_process_q),.period_10us(perf_period_q),
        .capture_wait_10us(perf_capture_q),.publish_wait_10us(perf_publish_q),
        .stage_code(perf_stage_q),.swap_10us(swap_hold),
        .publish_count(undistort_publish_count[15:0]),
        .fps_tens(measured_fps_tens),.fps_ones(measured_fps_ones),
        .fps_frac_tens(measured_fps_frac_tens),.fps_frac_ones(measured_fps_frac_ones),
        .allow(adjusted_once),.tx_req(dbg_req),.tx_gnt(dbg_gnt),.line_busy(dbg_busy),
        .tx(dbg_tx)
    );
    uart_tx_arbiter c24_arbiter(
        .clk(cfg_clk),.rst_n(rstn_out),
        .req0(msg_req),.busy0(msg_busy),.tx0(msg_tx),
        .req1(param_tx_req),.busy1(param_tx_busy),.tx1(param_tx),
        .req2(dbg_req),.busy2(dbg_busy),.tx2(dbg_tx),
        .gnt0(msg_gnt),.gnt1(param_tx_gnt),.gnt2(dbg_gnt),
        .tx(uart_tx));

    // ==== PARAM_C24_END ====
    fram_buf #(.SIDE_BY_SIDE(1),.ADAPTIVE_GAMMA(1),.DIAG_EN(DIAG_EN)) fram_buf(
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  video_ddr_ready      ),//input                         ddr_rstn,
        .demo_mode      (  2'd0                  ),
        .enhancement_bypass( gamma_bypass_core ),.fisheye_enable(cal_fisheye_core),.lut_bank(cal_lut_bank_core),.lut_wr_clk(cfg_clk),.lut_wr_en(lut_wr_en),.lut_wr_addr(lut_wr_addr),.lut_wr_data(lut_wr_data),.coord_lut_bank(csm_lut_bank),.coord_fisheye(csm_fisheye),
        .display_compare( display_compare_core ),
        .display_original( display_original_core ),
        .affine_enable  (  affine_enable_core    ),
        // Three-axis affine pose mode, enabled together with KEY1 calibration.
        .z_affine_mode  (  affine_enable_core    ),
        .affine_a00     (  affine_a00_core       ),
        .affine_a01     (  affine_a01_core       ),
        .affine_a02     (  affine_a02_core       ),
        .affine_a10     (  affine_a10_core       ),
        .affine_a11     (  affine_a11_core       ),
        .affine_a12     (  affine_a12_core       ),
        .homography_h00 (  homography_h00_core   ),
        .homography_h01 (  homography_h01_core   ),
        .homography_h02 (  homography_h02_core   ),
        .homography_h10 (  homography_h10_core   ),
        .homography_h11 (  homography_h11_core   ),
        .homography_h12 (  homography_h12_core   ),
        .homography_h20 (  homography_h20_core   ),
        .homography_h21 (  homography_h21_core   ),
        .homography_h22 (  homography_h22_core   ),
        .cal_fx_q16     (  cal_fx_core           ),
        .cal_fy_q16     (  cal_fy_core           ),
        .cal_ifx_q30    (  cal_ifx_core          ),
        .cal_ify_q30    (  cal_ify_core          ),
        .cal_icx_q16    (  cal_icx_core          ),
        .cal_icy_q16    (  cal_icy_core          ),
        .cal_ocx_q16    (  cal_ocx_core          ),
        .cal_ocy_q16    (  cal_ocy_core          ),
        .cal_k1_q24     (  cal_k1_core           ),
        .cal_k2_q24     (  cal_k2_core           ),
        .cal_p1_q24     (  cal_p1_core           ),
        .cal_p2_q24     (  cal_p2_core           ),
        .cal_gen        (  cal_gen_core          ),
        .display_cal_gen(  display_cal_gen       ),
        .coord_read_addr(csm_read_addr),.coord_read_data(csm_read_data),
        .coord_calibration(csm_calibration),.coord_affine(csm_affine),
        .coord_affine_enabled(csm_affine_enabled),.coord_complete(csm_complete),
        .coord_sample_count(csm_count),.coord_generation(csm_generation),.coord_gamma(csm_gamma),
        //data_in                                  
        .vin_clk        (  pixclk_in         ),//input                         vin_clk,
        .wr_fsync       (  ~vs_in_2           ),//input                         wr_fsync,
        .wr_en          (  de_in_2           ),//input                         wr_en,
        .wr_data        (  hdmi_data_in      ),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  pix_clk              ),//input                         vout_clk,
        .rd_fsync       (  vs_reg               ),//input                         rd_fsync,
        .rd_en          (  rd_en                ),//input                         rd_en,
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .init_done      (  init_done            ),//output reg                    init_done,
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (                       ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ),
        .dbg_stage(undistort_stage),.dbg_error(undistort_error),
        .dbg_map_ready(undistort_map_ready),.dbg_algorithm_busy(undistort_busy),
        .dbg_publish_count(undistort_publish_count),
        .dbg_last_process_cycles(undistort_last_process_cycles),
        .dbg_process_10us(measured_process_10us),
        .dbg_result_period_10us(measured_result_period_10us),
        .dbg_publish_wait_10us(measured_publish_wait_10us),
        .dbg_capture_wait_10us(measured_capture_wait_10us),
        .dbg_swap_toggle(undistort_swap_toggle),
        .publish_toggle(undistort_publish_toggle),
        .freeze_req(fb_freeze),.frozen(fb_frozen),
        .frozen_raw_base(eth_frozen_raw_base),.frozen_res_base(eth_frozen_res_base),.frozen_gamma_curve(eth_frozen_gamma_curve),
        .ext_own(fb_own),.ext_addr(fb_addr),.ext_len(fb_len),.ext_valid(fb_valid),
        .ext_ready(fb_ready),.ext_rdata(fb_rdata),.ext_rvalid(fb_rvalid),.ext_rlast(fb_rlast),
        .dbg_report_stage(undistort_report_stage),
        .dbg_report_toggle(undistort_report_toggle),
        .latency_bcd(measured_latency_bcd),.latency_valid(measured_latency_valid),
        .latency_overflow(measured_latency_overflow)
    );

   /*  always@(posedge pix_clk) begin
        r_out<=8'hff;
        g_out<=8'hff;
        b_out<=8'h00; 
        vs_out<=vs_o;
        hs_out<=hs_o;
        de_out<=de_re;
     end*/
//assign pix_clk = pixclk_in;
always  @(posedge pix_clk)begin


        vs_out       <=  vs_reg        ;
        hs_out       <=  hs_reg        ;
        de_out       <=  de_o        ;
        r_out        <=  fps_rgb_out[23:16];
        g_out        <=  fps_rgb_out[15:8];
        b_out        <=  fps_rgb_out[7:0];

end

fps_meter_cfg #(.CLK_HZ(10000000)) fps_meter(
    .clk(cfg_clk), .rst_n(rstn_out), .frame_clk(core_clk), .frame_rst_n(ddr_init_done),
    .frame_count(undistort_publish_count), .fps_value(measured_fps),
    .fps_tens(measured_fps_tens), .fps_ones(measured_fps_ones), .fps_frac_tens(measured_fps_frac_tens), .fps_frac_ones(measured_fps_frac_ones), .fps_update(measured_fps_update)
);

fps_overlay #(.BOX_X(32),.BOX_Y(32),.DIAG_EN(DIAG_EN)) fps_overlay(
    .clk(pix_clk), .rst_n(ddr_init_done), .de(de_o), .vs(vs_reg),
    .x(act_x), .y(act_y), .rgb_in({{o_rgb565[15:11],3'd0},
                                   {o_rgb565[10:5],2'd0},
                                   {o_rgb565[4:0],3'd0}}),
    .fps_tens_async(measured_fps_tens), .fps_ones_async(measured_fps_ones),
    .fps_frac_tens_async(measured_fps_frac_tens), .fps_frac_ones_async(measured_fps_frac_ones),
    .fps_update_async(measured_fps_update),
    .latency_bcd(measured_latency_bcd),.latency_valid(measured_latency_valid),
    .latency_overflow(measured_latency_overflow),
    .diag_process_10us(measured_process_10us),
    .diag_result_period_10us(measured_result_period_10us),
    .diag_publish_wait_10us(measured_publish_wait_10us),
    .diag_stage(undistort_stage),
    .rgb_out(fps_rgb_out)
);


    rgb2tmds rgb2tmds (
        // DVI 1.0 TMDS video interface
        .tmds_clk_p         (  tmds_clk_p             ),
        .tmds_clk_n         (  tmds_clk_n             ),
        .tmds_data_p        (  tmds_data_p            ),
        .tmds_data_n        (  tmds_data_n            ),
                                                    
        //Auxiliary signals     
        //must be reset when RefClk is not within spec                     
        .rstn               (  rstn_out                   ), //-synchronous reset; 
                                                      
        // video in                                   
        .vid_pdata          (  {{h_rgb[15:11],h_rgb[15:13]},{h_rgb[10:5],h_rgb[10:9]},{h_rgb[4:0],h_rgb[4:2]}} ),
        .vid_pvde           (  hde                    ),
        .vid_phsync         (  hhs                    ),
        .vid_pvsync         (  hvs                    ),
        .pixelclk           (  pix_clk                ),
                                                      
        .serialclk          (  pix_clk_5x             )// 5x PixelClk
   ); 


/////////////////////////////////////////////////////////////////////////////////////
//婵犵數鍋涢悺銊х尵閸岀偛宸濇い鏃囧Г椤忕劑isa闂傚倷绀侀幖顐﹀疮閸愭祴鏋栨繛鎴炰亢婵?
wire                        hs         ;
wire                        vs         ;
wire                        de         ;

//MODE_1080p60
parameter H_ACT = 16'd1920;           //horizontal active time (pixels)
parameter H_FP = 16'd88;                //horizontal front porch (pixels)
parameter H_SYNC = 16'd44;               //horizontal sync time(pixels)
parameter H_BP = 16'd148;                //horizontal back porch (pixels)
parameter V_ACT = 16'd1080;            //vertical active Time (lines)
parameter V_FP  = 16'd4;                 //vertical front porch (lines)
parameter V_SYNC  = 16'd5;               //vertical sync time (lines)
parameter V_BP  = 16'd36;                //vertical back porch (lines)
parameter V_TOTAL = 16'd1125;                 //horizontal sync polarity, 1 : POSITIVE,0 : NEGATIVE;
parameter H_TOTAL = 16'd2200;                 //vertical sync polarity, 1 : POSITIVE,0 : NEGATIVE;
parameter HV_OFFSET = 12'd0;   
parameter   X_WIDTH = 4'd12;
parameter   Y_WIDTH = 4'd12; 
wire [X_WIDTH - 1'b1:0]     act_x      ;
wire [Y_WIDTH - 1'b1:0]     act_y      ;  
// 演示版使用固定 2200 x 1125 扫描时序，与已验证正常的 TCP 旧版一致。
// 按键只改变帧内容，结果发布时间不再改变 HDMI 帧周期。
sync_vg #(
    .X_BITS               (  X_WIDTH              ),
    .Y_BITS               (  Y_WIDTH              ),
    .V_TOTAL              (  V_TOTAL              ),//                        
    .V_FP                 (  V_FP                 ),//                        
    .V_BP                 (  V_BP                 ),//                        
    .V_SYNC               (  V_SYNC               ),//                        
    .V_ACT                (  V_ACT                ),//                        
    .H_TOTAL              (  H_TOTAL              ),//                        
    .H_FP                 (  H_FP                 ),//                        
    .H_BP                 (  H_BP                 ),//                        
    .H_SYNC               (  H_SYNC               ),//                        
    .H_ACT                (  H_ACT                ) //                        

) sync_vg                                         
(                                                 
    .clk                  (  pix_clk               ),//input                   clk,
    .rstn                 (  ddr_init_done                 ),//input                   rstn,
    .de_re                (                       ),
    .vs_out               (  vs_reg                   ),//output reg              vs_out,                                                                                                                                      
    .hs_out               (  hs_reg                   ),//output reg              hs_out,            
    .de_out               (  rd_en                   ),//output reg              de_out,             
    .x_act                (  act_x                ),//output reg [X_BITS-1:0] x_out,             
    .y_act                (  act_y                ) //output reg [Y_BITS:0]   y_out,             
); 


////////////////////////////////////////////////////////////////////////////////////////////

wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);

//ddr    
        ddr3_test u_ddr3_test_h (
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),          // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
  .debug_cpd_state()                     // output [3:0]
       );

//闂傚倸顭崑鍕洪妶澶婄柈闁规儼濮ら崐濠氭煥閺冨倹娅曢柍鐟扮Ч閺屽秹濡烽妷銉︽瘣婵?
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
endmodule

