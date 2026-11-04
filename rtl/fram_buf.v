`timescale 1ns/1ps
`default_nettype none

module fram_buf #(
    parameter MEM_ROW_WIDTH=15, MEM_COLUMN_WIDTH=10, MEM_BANK_WIDTH=3,
    parameter CTRL_ADDR_WIDTH=MEM_ROW_WIDTH+MEM_BANK_WIDTH+MEM_COLUMN_WIDTH,
    parameter MEM_DQ_WIDTH=32, H_NUM=1920, V_NUM=1080, PIX_WIDTH=16,
    parameter integer SIDE_BY_SIDE=0,
    parameter integer ADAPTIVE_GAMMA=0,
    parameter MAP_CX_Q16=62183231, MAP_CY_Q16=33074617,
    parameter MAP_X_SQ_FILE="source/undistort/lut/x_sq.hex",
    parameter MAP_Y_SQ_FILE="source/undistort/lut/y_sq.hex",
    parameter MAP_GAIN_FILE="source/undistort/lut/gain.hex",
    // 0 removes every diagnostic counter from the 125 MHz domain. The release
    // build sets this to 0 so placement stays identical to the timing-clean
    // baseline; the measurement build keeps it at 1.
    parameter integer DIAG_EN=1
) (
    input wire vin_clk,wr_fsync,wr_en,
    input wire [PIX_WIDTH-1:0] wr_data,
    output reg init_done=0,
    input wire ddr_clk,ddr_rstn,vout_clk,rd_fsync,rd_en,
    input wire [1:0] demo_mode,
    input wire enhancement_bypass,
    input wire fisheye_enable,
    input wire lut_bank,lut_wr_clk,lut_wr_en,
    input wire [12:0] lut_wr_addr,
    input wire [24:0] lut_wr_data,
    output wire coord_lut_bank,
    output wire coord_fisheye,
    input wire display_compare,
    input wire display_original,   // 全屏原图：恒等采样，不做畸变/姿态矫正，不加 Gamma
    input wire affine_enable,
    input wire z_affine_mode,
    input wire signed [31:0] affine_a00,affine_a01,affine_a02,
    input wire signed [31:0] affine_a10,affine_a11,affine_a12,
    input wire signed [31:0] homography_h00,homography_h01,homography_h02,
    input wire signed [31:0] homography_h10,homography_h11,homography_h12,
    input wire signed [31:0] homography_h20,homography_h21,homography_h22,
    // Runtime Brown calibration (from PARAM-V1). Latched at frame start.
    input wire signed [31:0] cal_fx_q16,cal_fy_q16,
    input wire signed [31:0] cal_ifx_q30,cal_ify_q30,
    input wire signed [31:0] cal_icx_q16,cal_icy_q16,
    input wire signed [31:0] cal_ocx_q16,cal_ocy_q16,
    input wire signed [31:0] cal_k1_q24,cal_k2_q24,cal_p1_q24,cal_p2_q24,
    // Calibration generation travels with the frame: latched with cal_* at
    // CAP_SETTLE, published with the result, shown when the display swaps.
    input wire [7:0]  cal_gen,
    output reg [7:0]  display_cal_gen,
    output wire vout_de,
    output wire [PIX_WIDTH-1:0] vout_data,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr,
    output wire [3:0] axi_awid,axi_awlen,
    output wire [2:0] axi_awsize,
    output wire [1:0] axi_awburst,
    input wire axi_awready,
    output wire axi_awvalid,
    output wire [MEM_DQ_WIDTH*8-1:0] axi_wdata,
    output wire [MEM_DQ_WIDTH-1:0] axi_wstrb,
    input wire axi_wlast,
    output wire axi_wvalid,
    input wire axi_wready,
    input wire [3:0] axi_bid,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_araddr,
    output wire [3:0] axi_arid,axi_arlen,
    output wire [2:0] axi_arsize,
    output wire [1:0] axi_arburst,
    output wire axi_arvalid,
    input wire axi_arready,
    output wire axi_rready,
    input wire [MEM_DQ_WIDTH*8-1:0] axi_rdata,
    input wire axi_rvalid,axi_rlast,
    input wire [3:0] axi_rid,
    output wire [3:0] dbg_stage,
    output wire dbg_error,dbg_map_ready,dbg_algorithm_busy,
    output reg [31:0] dbg_publish_count,
    output reg [31:0] dbg_last_process_cycles,
    output reg [15:0] dbg_process_10us,
    output reg [15:0] dbg_result_period_10us,
    output reg [15:0] dbg_publish_wait_10us,
    output reg [15:0] dbg_capture_wait_10us,
    output reg [3:0]  dbg_report_stage,
    // Flips once per display swap, so the reader can time the interval in its
    // own clock domain instead of trusting the 125 MHz divider.
    output reg        dbg_swap_toggle,
    output reg        publish_toggle,   // 每次结果发布翻转一次，供输出时序锁相（sync_vg_genlock）
    // 以太网同帧采集：freeze_req 为 1 时，本次结果发布后停在 FROZEN（不再采新帧、不再处理），
    // 屏上和 DDR 里保持同一帧的原图/矫正图。frozen=已停稳且屏上就是这帧。
    // ext_own 为 1 时，矫正通道 1 的 DDR 读口（arbiter map 口）交给以太网读取。
    input wire [7:0] coord_read_addr,
    output wire [63:0] coord_read_data,
    output wire [383:0] coord_calibration,
    output wire [191:0] coord_affine,
    output wire coord_affine_enabled,coord_complete,
    output wire [7:0] coord_sample_count,coord_generation,
    output wire [3:0] coord_gamma,
    input  wire        freeze_req,
    output wire        frozen,
    output reg  [27:0] frozen_raw_base,
    output wire [27:0] frozen_res_base,
    output reg [2:0] frozen_gamma_curve,
    input  wire        ext_own,
    input  wire [27:0] ext_addr,
    input  wire [3:0]  ext_len,
    input  wire        ext_valid,
    output wire        ext_ready,
    output wire [255:0] ext_rdata,
    output wire        ext_rvalid,ext_rlast,
    // Flips once per completed result. The reader lives in the 10 MHz domain;
    // two synchroniser stages guarantee the values above have been stable for
    // at least two of its clocks before the edge is seen.
    output reg        dbg_report_toggle,
    output wire [19:0] latency_bcd,
    output wire latency_valid,latency_overflow
);
    localparam [3:0] BOOT_START=0,BOOT_MAP=1,MAP_SETTLE=2,CAP_REQUEST=3,
        CAPTURE=4,CAP_SETTLE=5,ALGO_START=6,PROCESS=7,RESULT_SETTLE=8,WAIT_SWAP=9,FROZEN=10,FAULT=15;
    localparam [27:0] MAP_WORDS=28'h1000000,OUT0_WORDS=28'h0800000,OUT1_WORDS=28'h0c00000;
    reg [3:0] stage;
    reg [6:0] settle;
    reg request_toggle,display_valid,map_ready_flag;
    reg [1:0] work_bank,display_bank,publish_bank;
    reg publish_pending;
    reg capture_active,capture_ready,capture_bank;
    reg [31:0] capture_lines,process_cycles;
    reg [27:0] source_base_words,ready_base_words,map_write_words;
    reg [255:0] map_block;
    reg [1:0] map_slot,frame_mode,frame_mode_req;
    reg map_block_valid,map_block_last,map_last_written,algo_finished;
    reg affine_enable_frame;
    reg signed [31:0] affine_a00_frame,affine_a01_frame,affine_a02_frame;
    reg signed [31:0] affine_a10_frame,affine_a11_frame,affine_a12_frame;
    reg signed [31:0] homography_h00_frame,homography_h01_frame,homography_h02_frame;
    reg signed [31:0] homography_h10_frame,homography_h11_frame,homography_h12_frame;
    reg signed [31:0] homography_h20_frame,homography_h21_frame,homography_h22_frame;
    reg signed [31:0] cal_fx_frame,cal_fy_frame,cal_ifx_frame,cal_ify_frame;
    reg signed [31:0] cal_icx_frame,cal_icy_frame,cal_ocx_frame,cal_ocy_frame;
    reg signed [31:0] cal_k1_frame,cal_k2_frame,cal_p1_frame,cal_p2_frame;
    reg [7:0] cal_gen_frame,publish_cal_gen;
    reg queue_error;
    wire map_start=1'b0;
    wire algo_start=stage==ALGO_START;
    wire capture_de,capture_error,capture_first;
    // KEY2 toggles enhancement; all processed/frozen frames use fixed Gamma 0.8.
    localparam [2:0] FIXED_GAMMA_CURVE = 3'd2;
    wire [2:0] gamma_curve_ddr = FIXED_GAMMA_CURVE;
    reg [2:0] frame_gamma_curve;
    reg frame_gamma_enable;
    reg cap_error1,cap_error2;
    wire map_busy,map_done,map_error,map_valid,map_last;
    wire [31:0] map_u,map_v;
    wire map_accept=map_valid && !map_block_valid;
    wire algo_busy,algo_done,algo_error;
    wire [1:0] lane_busy,lane_done,lane_error,lane_result_done;
    reg [1:0] lane_finished;
    assign algo_busy=|lane_busy;
    assign algo_done=&(lane_finished|lane_done);
    assign algo_error=|lane_error;
    localparam integer TOP_ROWS=V_NUM/2;
    localparam integer BOTTOM_ROWS=V_NUM-TOP_ROWS;
    function [27:0] result_base;
        input [1:0] bank;
        begin
            case(bank)
                0: result_base=OUT0_WORDS;
                1: result_base=OUT1_WORDS;
                default: result_base=28'h1000000;
            endcase
        end
    endfunction
    reg [1:0] free_result_bank;
    reg free_result_valid;
    integer bank_select;
    always @* begin
        free_result_bank=0; free_result_valid=0;
        for(bank_select=0;bank_select<3;bank_select=bank_select+1)
            if(!free_result_valid && (!display_valid || display_bank!=bank_select) &&
                (!publish_pending || publish_bank!=bank_select)) begin
                free_result_bank=bank_select; free_result_valid=1;
            end
    end
    wire [15:0] result565;
    wire result_valid,result_ready;
    wire write_cmd_ready,write_done,write_error,write_busy,write_pop;
    wire display_error,arb_error,map_reader_error,result_error,gen_map_error;
    assign dbg_stage=stage;
    assign dbg_map_ready=map_ready_flag;
    assign dbg_algorithm_busy=algo_busy;
    assign dbg_error=map_error|gen_map_error|cap_error2|write_error|display_error|arb_error|
        algo_error|map_reader_error|result_error|queue_error;

    video_capture_gate #(.WIDTH(H_NUM),.HEIGHT(V_NUM)) capture_gate(
        .clk(vin_clk),.rst_n(ddr_rstn),.request_toggle(request_toggle),
        .vsync(~wr_fsync),.de(wr_en),.capture_de(capture_de),
        .capture_first(capture_first),.error(capture_error));
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin cap_error1<=0; cap_error2<=0; end
        else begin cap_error1<=capture_error; cap_error2<=cap_error1; end
    end
    kb_map_generator #(.WIDTH(H_NUM),.HEIGHT(V_NUM),.CX_Q16(MAP_CX_Q16),
        .CY_Q16(MAP_CY_Q16),.X_SQ_FILE(MAP_X_SQ_FILE),
        .Y_SQ_FILE(MAP_Y_SQ_FILE),.GAIN_FILE(MAP_GAIN_FILE)) map_generator(
        .clk(ddr_clk),.rst_n(ddr_rstn),.start(map_start),.busy(map_busy),.done(map_done),.error(map_error),
        .map_u(map_u),.map_v(map_v),.map_valid(map_valid),.map_ready(!map_block_valid),
        .map_last(map_last),.map_x(),.map_y());

    wire cap_req;
    wire [27:0] cap_addr;
    wire [31:0] cap_beats;
    wire [255:0] cap_data;
    reg [1:0] write_owner;
    wire cap_pop=write_owner==1 && write_pop;
    wire cap_write_done=write_owner==1 && write_done;
    reg cap_pop_d;
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) cap_pop_d<=0;
        else cap_pop_d<=cap_pop;
    end
    wr_buf #(.ADDR_WIDTH(CTRL_ADDR_WIDTH),.ADDR_OFFSET(0),.H_NUM(H_NUM),.V_NUM(V_NUM),
        .DQ_WIDTH(MEM_DQ_WIDTH),.LEN_WIDTH(32),.PIX_WIDTH(PIX_WIDTH),
        .LINE_ADDR_WIDTH(22),.FRAME_CNT_WIDTH(CTRL_ADDR_WIDTH-22)) wr_buf(
        .ddr_clk(ddr_clk),.ddr_rstn(ddr_rstn),.wr_clk(vin_clk),.wr_fsync(wr_fsync),
        .wr_en(capture_de),.wr_data(wr_data),.rd_bac(cap_pop_d && !cap_pop),
        .ddr_wreq(cap_req),.ddr_waddr(cap_addr),.ddr_wr_len(cap_beats),
        .ddr_wrdy(write_cmd_ready && capture_active && cap_req && stage!=BOOT_MAP),
        .ddr_wdone(cap_write_done),.ddr_wdata(cap_data),.ddr_wdata_req(cap_pop),.frame_wcnt(),.frame_wirq());

    wire result_cmd,result_done,result_pack_busy;
    wire result_complete_event = (stage==PROCESS) && result_done && (algo_finished || algo_done);
    wire [1:0] lane_cmd,lane_cmd_ready,lane_pop,lane_write_done,lane_pack_busy,lane_pack_error;
    wire [27:0] lane_result_addr[0:1];
    wire [31:0] lane_result_beats[0:1];
    wire [255:0] lane_payload[0:1];
    reg result_round_robin;
    wire result_select=lane_cmd[1] && (!lane_cmd[0] || result_round_robin);
    wire [27:0] result_addr;
    wire [31:0] result_beats;
    wire [255:0] result_payload;
    assign result_cmd=|lane_cmd;
    assign result_addr=lane_result_addr[result_select];
    assign result_beats=lane_result_beats[result_select];
    assign result_payload=write_owner==3 ? lane_payload[1] : lane_payload[0];
    assign result_done=&lane_result_done;
    assign result_pack_busy=|lane_pack_busy;
    assign result_error=|lane_pack_error;
    wire boot_write=stage==BOOT_MAP && map_block_valid;
    wire cap_write=capture_active && cap_req && stage!=BOOT_MAP;
    wire result_cmd_ready=write_cmd_ready && !boot_write && !cap_write;
    wire write_cmd_valid=boot_write || cap_write || result_cmd;
    wire [27:0] write_addr=boot_write?map_write_words:cap_write?
        {5'd0,capture_bank,cap_addr[21:0]}:result_addr;
    wire [31:0] write_beats=boot_write?32'd1:cap_write?cap_beats:result_beats;
    wire [255:0] write_payload=write_owner==0?map_block:write_owner==1?cap_data:result_payload;
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) write_owner<=0;
        else if (write_cmd_valid && write_cmd_ready)
            write_owner<=boot_write?0:cap_write?1:result_select?3:2;
    end
    assign lane_cmd_ready[0]=result_cmd_ready && !result_select;
    assign lane_cmd_ready[1]=result_cmd_ready && result_select;
    assign lane_pop[0]=write_pop && write_owner==2;
    assign lane_pop[1]=write_pop && write_owner==3;
    assign lane_write_done[0]=write_done && write_owner==2;
    assign lane_write_done[1]=write_done && write_owner==3;
    reg [1:0] result_finished;
    assign lane_result_done=result_finished;
    wire [1:0] result_done_pulse;
    always @(posedge ddr_clk) begin
        if(!ddr_rstn) begin result_finished<=0; lane_finished<=0; result_round_robin<=0; end
        else if(algo_start) begin result_finished<=0; lane_finished<=0; end
        else begin
            result_finished<=result_finished|result_done_pulse;
            lane_finished<=lane_finished|lane_done;
            if(result_cmd && result_cmd_ready) result_round_robin<=!result_select;
        end
    end
    native_write_stream writer(.clk(ddr_clk),.rst_n(ddr_rstn),
        .cmd_valid(write_cmd_valid),.cmd_ready(write_cmd_ready),.cmd_addr(write_addr),.cmd_beats(write_beats),
        .stream_data(write_payload),.stream_pop(write_pop),.done(write_done),.error(write_error),.busy(write_busy),
        .awaddr(axi_awaddr),.awlen(axi_awlen),.awvalid(axi_awvalid),.awready(axi_awready),
        .wdata(axi_wdata),.wstrb(axi_wstrb),.wready(axi_wready),.wlast(axi_wlast));
    assign axi_awid=0;
    assign axi_awsize=3'd5;
    assign axi_awburst=2'b01;
    assign axi_wvalid=write_busy;
    assign axi_arsize=3'd5;
    assign axi_arburst=2'b01;
    assign axi_rready=1;

    wire display_frame_start,display_cmd,display_done,display_cmd_ready,display_busy;
    wire [27:0] display_cmd_addr;
    wire [31:0] display_cmd_beats;
    wire [255:0] display_pixels;
    wire display_pixels_valid;
    reg display_pending;
    reg [27:0] display_pending_addr;
    reg [31:0] display_pending_beats;
    wire swap_safe=publish_pending && !display_busy && !display_pending;
    wire publish_ready_event = (stage==RESULT_SETTLE) && (settle==7'd63) && !publish_pending;
    wire display_take_event = display_frame_start && swap_safe;

    // 10 us event timing at the 125 MHz core clock. These values are diagnostic
    // only; they do not participate in the video datapath.
    reg [10:0] diag_tick_div;
    reg [31:0] diag_time_10us, diag_algo_start_time, diag_result_done_time,
               diag_publish_ready_time;
    function [15:0] sat16;
        input [31:0] value;
        begin sat16 = (value > 32'hffff) ? 16'hffff : value[15:0]; end
    endfunction
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            diag_tick_div<=0; diag_time_10us<=0; diag_algo_start_time<=0;
            diag_result_done_time<=0; diag_publish_ready_time<=0;
            dbg_process_10us<=0; dbg_result_period_10us<=0; dbg_publish_wait_10us<=0;
            dbg_capture_wait_10us<=0; dbg_report_stage<=0; dbg_report_toggle<=0;
        end else if (DIAG_EN) begin
            if (diag_tick_div==11'd1249) begin diag_tick_div<=0; diag_time_10us<=diag_time_10us+1'b1; end
            else diag_tick_div<=diag_tick_div+1'b1;
            if (algo_start) begin
                diag_algo_start_time<=diag_time_10us;
                // Gap between the previous result finishing and this frame
                // starting: waiting for a complete capture plus the settles.
                // P + C is therefore the whole result period R.
                if (diag_result_done_time!=0)
                    dbg_capture_wait_10us<=sat16(diag_time_10us-diag_result_done_time);
            end
            if (result_complete_event) begin
                dbg_process_10us<=sat16(diag_time_10us-diag_algo_start_time);
                if (diag_result_done_time!=0) dbg_result_period_10us<=sat16(diag_time_10us-diag_result_done_time);
                diag_result_done_time<=diag_time_10us;
                dbg_report_stage<=stage;
                dbg_report_toggle<=~dbg_report_toggle;
            end
            if (publish_ready_event) diag_publish_ready_time<=diag_time_10us;
            if (display_take_event) dbg_publish_wait_10us<=sat16(diag_time_10us-diag_publish_ready_time);
        end
    end
    // 时间戳随采集槽、结果槽传递，终点为顶层输出寄存器采样的首个 DE。
    video_latency_meter latency_meter(
        .core_clk(ddr_clk),.core_rst_n(ddr_rstn),
        .vin_clk(vin_clk),.vin_first(capture_first),
        .capture_arm(stage==CAP_REQUEST || algo_start),
        .capture_arm_bank(algo_start ? !capture_bank : capture_bank),
        .capture_bank(capture_bank),
        .process_take(stage==CAPTURE && capture_ready && free_result_valid),
        .process_source_bank(capture_bank),.process_result_bank(free_result_bank),
        .display_take(display_frame_start && swap_safe),.display_result_bank(publish_bank),
        .vout_clk(vout_clk),.vout_vs(rd_fsync),.vout_de(vout_de),
        .latency_bcd(latency_bcd),.latency_valid(latency_valid),
        .latency_overflow(latency_overflow),.latency_ticks(),.latency_update());
    wire [27:0] display_base=result_base(swap_safe ? publish_bank : display_bank);
    wire [PIX_WIDTH-1:0] raw_vout_data;
    reg show1,show2;
    always @(posedge vout_clk) begin
        if (!ddr_rstn) begin show1<=0; show2<=0; end
        else begin show1<=display_valid; show2<=show1; end
    end
    assign vout_data=show2?raw_vout_data:0;
    rd_buf #(.ADDR_WIDTH(CTRL_ADDR_WIDTH),.ADDR_OFFSET(0),.H_NUM(H_NUM),.V_NUM(V_NUM),
        .DQ_WIDTH(MEM_DQ_WIDTH),.LEN_WIDTH(32),.PIX_WIDTH(PIX_WIDTH),
        .LINE_ADDR_WIDTH(22),.FRAME_CNT_WIDTH(CTRL_ADDR_WIDTH-22)) rd_buf(
        .ddr_clk(ddr_clk),.ddr_rstn(ddr_rstn),.vout_clk(vout_clk),.rd_fsync(rd_fsync),.rd_en(rd_en),
        .vout_de(vout_de),.vout_data(raw_vout_data),.init_done(display_valid || swap_safe),
        .frame_base_words(display_base),.frame_start_core(display_frame_start),
        .ddr_rreq(display_cmd),.ddr_raddr(display_cmd_addr),.ddr_rd_len(display_cmd_beats),
        .ddr_rrdy(display_cmd_ready),.ddr_rdone(display_done),
        .ddr_rdata(display_pixels),.ddr_rdata_en(display_pixels_valid));
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            display_pending<=0; display_pending_addr<=0; display_pending_beats<=0; queue_error<=0;
        end else begin
            if (display_pending && display_cmd_ready) display_pending<=0;
            if (display_cmd) begin
                if (display_pending && !display_cmd_ready) queue_error<=1;
                display_pending<=1; display_pending_addr<=display_cmd_addr; display_pending_beats<=display_cmd_beats;
            end
        end
    end
    wire [27:0] display_araddr,source_addr,map_addr;
    wire [3:0] display_arlen,map_len,source_len;
    wire [3:0] lane_source_len[0:1];
    wire display_arvalid,display_arready,display_rvalid,display_rlast,display_rerror;
    wire [255:0] display_rdata,source_data,map_data;
    wire source_valid,source_ready,source_rvalid,source_rlast,source_rerror;
    wire map_native_valid,map_native_ready,map_rvalid,map_rlast,map_rerror;
    wire [27:0] source1_addr;
    wire source1_valid,source1_ready,source1_rvalid,source1_rlast,source1_rerror;
    wire [255:0] source1_data;
    // The former map request port is now the second source-cache lane.
    // ext_own 只在 FROZEN 且两条矫正通道都已空闲后由以太网置 1，切换时读口上没有未完成的读。
    assign map_addr=ext_own ? ext_addr : source1_addr;
    assign map_len=ext_own ? ext_len : lane_source_len[1];
    assign source_len=lane_source_len[0];
    assign map_native_valid=ext_own ? ext_valid : source1_valid;
    assign source1_ready=map_native_ready && !ext_own;
    assign ext_ready=map_native_ready && ext_own;
    assign source1_data=map_data;
    assign source1_rvalid=map_rvalid && !ext_own;
    assign source1_rlast=map_rlast;
    assign source1_rerror=map_rerror && !ext_own;
    assign ext_rdata=map_data;
    assign ext_rvalid=map_rvalid && ext_own;
    assign ext_rlast=map_rlast;
    assign frozen=(stage==FROZEN) && !publish_pending && display_valid && (display_bank==publish_bank);
    assign frozen_res_base=result_base(display_bank);
    native_read_stream display_reader(.clk(ddr_clk),.rst_n(ddr_rstn),
        .cmd_valid(display_pending),.cmd_ready(display_cmd_ready),.cmd_addr(display_pending_addr),
        .cmd_beats(display_pending_beats),.busy(display_busy),.done(display_done),.error(display_error),
        .araddr(display_araddr),.arlen(display_arlen),.arvalid(display_arvalid),.arready(display_arready),
        .rdata(display_rdata),.rvalid(display_rvalid),.rlast(display_rlast),.rerror(display_rerror),
        .stream_data(display_pixels),.stream_valid(display_pixels_valid),.stream_last(),.stream_error());
    native_read_arbiter3 #(.ROUND_ROBIN_BACKGROUND(1)) read_arbiter(.clk(ddr_clk),.rst_n(ddr_rstn),
        .display_addr(display_araddr),.display_len(display_arlen),.display_valid(display_arvalid),.display_ready(display_arready),
        .display_data(display_rdata),.display_rvalid(display_rvalid),.display_rlast(display_rlast),.display_rerror(display_rerror),
        .source_addr(source_addr),.source_len(source_len),.source_valid(source_valid),.source_ready(source_ready),
        .source_data(source_data),.source_rvalid(source_rvalid),.source_rlast(source_rlast),.source_rerror(source_rerror),
        .map_addr(map_addr),.map_len(map_len),.map_valid(map_native_valid),.map_ready(map_native_ready),
        .map_data(map_data),.map_rvalid(map_rvalid),.map_rlast(map_rlast),.map_rerror(map_rerror),
        .araddr(axi_araddr),.arlen(axi_arlen),.arid(axi_arid),.arvalid(axi_arvalid),.arready(axi_arready),
        .rdata(axi_rdata),.rid(axi_rid),.rvalid(axi_rvalid),.rlast(axi_rlast),.error(arb_error));
    wire [27:0] lane_source_addr[0:1];
    wire [255:0] lane_source_data[0:1];
    wire [1:0] lane_source_valid,lane_source_ready,lane_rvalid,lane_rlast,lane_rerror;
    wire [1:0] generated_error;
    assign gen_map_error=|generated_error;
    assign map_reader_error=0;
    assign source_addr=lane_source_addr[0];
    assign source_valid=lane_source_valid[0];
    assign lane_source_ready={source1_ready,source_ready};
    reg [255:0] source_data_q,source1_data_q;
    reg [1:0] source_valid_q,source_last_q,source_error_q;
    reg map_start_q;
    // Keep each returned beat and its flags together before cache RAM control.
    always @(posedge ddr_clk) begin
        source_data_q<=source_data; source1_data_q<=source1_data;
        source_last_q<={source1_rlast,source_rlast};
        source_error_q<={source1_rerror,source_rerror};
        if(!ddr_rstn || algo_start) source_valid_q<=0;
        else source_valid_q<={source1_rvalid,source_rvalid};
        if(!ddr_rstn) map_start_q<=0;
        else map_start_q<=algo_start;
    end
    assign lane_source_data[0]=source_data_q;
    assign lane_source_data[1]=source1_data_q;
    assign lane_rvalid=source_valid_q;
    assign lane_rlast=source_last_q;
    assign lane_rerror=source_error_q;
    assign source1_addr=lane_source_addr[1];
    assign source1_valid=lane_source_valid[1];
    wire [31:0] coord_u[0:1],coord_v[0:1];
    wire [15:0] coord_x[0:1],coord_y[0:1];
    wire [1:0] coord_take;
    assign coord_calibration={cal_p2_frame,cal_p1_frame,cal_k2_frame,cal_k1_frame,
        cal_ocy_frame,cal_ocx_frame,cal_icy_frame,cal_icx_frame,
        cal_ify_frame,cal_ifx_frame,cal_fy_frame,cal_fx_frame};
    assign coord_affine={homography_h12_frame,homography_h11_frame,homography_h10_frame,
        homography_h02_frame,homography_h01_frame,homography_h00_frame};
    assign coord_affine_enabled=affine_enable_frame;
    reg fisheye_frame,lut_bank_frame;
    assign coord_lut_bank=lut_bank_frame;
    assign coord_fisheye=fisheye_frame;
    assign coord_generation=cal_gen_frame;
    assign coord_gamma={frame_gamma_curve,frame_gamma_enable};
    coordinate_trace_100h coordinate_trace(
        .clk(ddr_clk),.rst_n(ddr_rstn),.start(map_start_q),
        .enable(frame_mode==0),
        .u0(coord_u[0]),.v0(coord_v[0]),.x0(coord_x[0]),.y0(coord_y[0]),.take0(coord_take[0]),
        .u1(coord_u[1]),.v1(coord_v[1]),.x1(coord_x[1]),.y1(coord_y[1]),.take1(coord_take[1]),
        .read_addr(coord_read_addr),.read_data(coord_read_data),
        .complete(coord_complete),.sample_count(coord_sample_count));
    genvar channel;
    generate for(channel=0;channel<2;channel=channel+1) begin : correction_lane
        localparam integer FIRST_ROW=channel==0 ? 0 : TOP_ROWS;
        localparam integer ALT_TOP_ROWS=V_NUM==1080 ? 510 : TOP_ROWS;
        localparam integer ALT_FIRST_ROW=channel==0 ? 0 : ALT_TOP_ROWS;
        localparam integer ALT_ROW_COUNT=channel==0 ? ALT_TOP_ROWS : V_NUM-ALT_TOP_ROWS;
        // The request is settled before ALGO_START; frame_mode updates on its edge.
        // Cache/writer start then, while the mapper starts one cycle later.
        wire alternate_geometry=H_NUM==1920 && V_NUM==1080 && frame_mode_req==0 && !affine_enable_frame;
        localparam integer ROW_COUNT=channel==0 ? TOP_ROWS : BOTTOM_ROWS;
        localparam [27:0] ROW_OFFSET_WORDS=FIRST_ROW*H_NUM/2;
        wire [31:0] stream_u,stream_v;
        wire stream_valid,stream_ready,stream_user,stream_last;
        wire [15:0] map_x,map_y,filtered565;
        wire [31:0] pipeline_u,pipeline_v;
        wire [15:0] pipeline_x,pipeline_y;
        wire pipeline_valid,pipeline_ready,pipeline_user,pipeline_last;
        wire result_valid,result_ready;
        wire [15:0] buffered565;
        wire buffered_valid,buffered_ready;
        // Both halves use the same immutable capture bank for this result frame.
        localparam integer PICTURE_TOP=V_NUM/4, PICTURE_BOTTOM=V_NUM*3/4;
        wire black_bar=frame_mode==2 && (map_y<PICTURE_TOP || map_y>=PICTURE_BOTTOM);
        wire [15:0] half_x=map_x<H_NUM/2 ? map_x : map_x-H_NUM/2;
        wire [15:0] original_x=frame_mode==2 ? (half_x<<1) : map_x;
        wire [15:0] original_y=frame_mode==2 ? ((map_y-PICTURE_TOP)<<1) : map_y;
        // frame_mode 1：整屏原图，每个输出像素直接取同坐标源像素（整数坐标，不插值）
        wire original_pixel=(frame_mode==2 && map_x<H_NUM/2) || frame_mode==1;
        // 原 frame_mode==3 网格叠加（每 120 像素一条绿线）：frame_mode 只会取 0/1/2，
        // 这段永远不生效，但它的两个 %120 除法链曾是 ddrphy_sysclk 的最差路径，已删除。
        brown_map_stream #(.WIDTH(H_NUM),.HEIGHT(V_NUM),.START_ROW(FIRST_ROW),.ROWS(ROW_COUNT),.ALT_START_ROW(ALT_FIRST_ROW),.ALT_ROWS(ALT_ROW_COUNT),.SIDE_BY_SIDE(SIDE_BY_SIDE)) map_pipeline(
            .clk(ddr_clk),.rst_n(ddr_rstn),.start(map_start_q),.alternate_geometry(alternate_geometry),.display_compare(frame_mode==2),.fisheye_enable(fisheye_frame),.lut_bank(lut_bank_frame),.lut_wr_clk(lut_wr_clk),.lut_wr_en(lut_wr_en),.lut_wr_addr(lut_wr_addr),.lut_wr_data(lut_wr_data),
            .FX_Q16(cal_fx_frame),.FY_Q16(cal_fy_frame),
            .INV_FX_Q30(cal_ifx_frame),.INV_FY_Q30(cal_ify_frame),
            .ICX_Q16(cal_icx_frame),.ICY_Q16(cal_icy_frame),
            .OCX_Q16(cal_ocx_frame),.OCY_Q16(cal_ocy_frame),
            .K1_Q24(cal_k1_frame),.K2_Q24(cal_k2_frame),
            .P1_Q24(cal_p1_frame),.P2_Q24(cal_p2_frame),
            .affine_enable(affine_enable_frame),
            .affine_a00(affine_a00_frame),.affine_a01(affine_a01_frame),.affine_a02(affine_a02_frame),
            .affine_a10(affine_a10_frame),.affine_a11(affine_a11_frame),.affine_a12(affine_a12_frame),
            .homography_h00(homography_h00_frame),.homography_h01(homography_h01_frame),.homography_h02(homography_h02_frame),
            .homography_h10(homography_h10_frame),.homography_h11(homography_h11_frame),.homography_h12(homography_h12_frame),
            .busy(),.done(),.error(generated_error[channel]),
            .map_u(pipeline_u),.map_v(pipeline_v),.map_valid(pipeline_valid),.map_ready(pipeline_ready),
            .map_user(pipeline_user),.map_last(pipeline_last),.map_x(pipeline_x),.map_y(pipeline_y));
        assign coord_u[channel]=pipeline_u;
        assign coord_v[channel]=pipeline_v;
        assign coord_x[channel]=pipeline_x;
        assign coord_y[channel]=pipeline_y;
        assign coord_take[channel]=pipeline_valid && pipeline_ready;
        map_stream_buffer coordinate_buffer(
            .clk(ddr_clk),.rst_n(ddr_rstn),.flush(algo_start),
            .s_data({pipeline_u,pipeline_v,pipeline_x,pipeline_y,pipeline_user,pipeline_last}),
            .s_valid(pipeline_valid),.s_ready(pipeline_ready),
            .m_data({stream_u,stream_v,map_x,map_y,stream_user,stream_last}),
            .m_valid(stream_valid),.m_ready(stream_ready));
        undistort_cache_stream #(.SRC_WIDTH(H_NUM),.SRC_HEIGHT(V_NUM),.OUTPUT_PIXELS(H_NUM*ROW_COUNT),.ALT_OUTPUT_PIXELS(H_NUM*ALT_ROW_COUNT),.SIDE_BY_SIDE(SIDE_BY_SIDE)) algorithm(
            .clk(ddr_clk),.rst_n(ddr_rstn),.start(algo_start),.alternate_geometry(alternate_geometry),.src_base_addr(source_base_words),
            .gamma_curve(frame_gamma_curve),.gamma_enable(frame_gamma_enable),.compare_mode(frame_mode==2),
            .native_addr(lane_source_addr[channel]),.native_len(lane_source_len[channel]),.native_valid(lane_source_valid[channel]),
            .native_ready(lane_source_ready[channel]),.native_data(lane_source_data[channel]),
            .native_rvalid(lane_rvalid[channel]),.native_rlast(lane_rlast[channel]),.native_rerror(lane_rerror[channel]),
            .map_u(black_bar ? 32'hffffffff : original_pixel ? {original_x,16'd0} : stream_u),
            .map_v(black_bar ? 32'hffffffff : original_pixel ? {original_y,16'd0} : stream_v),
            .map_valid(stream_valid),.map_ready(stream_ready),.map_user(stream_user),.map_last(stream_last),
            .out_rgb565(filtered565),.out_valid(result_valid),.out_ready(result_ready),.out_user(),.out_last(),
            .busy(lane_busy[channel]),.done(lane_done[channel]),.error(lane_error[channel]));
        map_stream_buffer #(.TOKEN_BITS(16)) result_buffer(
            .clk(ddr_clk),.rst_n(ddr_rstn),.flush(algo_start),
            .s_data(filtered565),
            .s_valid(result_valid),.s_ready(result_ready),
            .m_data(buffered565),.m_valid(buffered_valid),.m_ready(buffered_ready));
        result_stream_writer #(.WIDTH(H_NUM),.HEIGHT(ROW_COUNT),.ALT_HEIGHT(ALT_ROW_COUNT)) result_writer(
            .clk(ddr_clk),.rst_n(ddr_rstn),.start(algo_start),
            .alternate_geometry(alternate_geometry),
            .base_words(result_base(work_bank)+(alternate_geometry ? ALT_FIRST_ROW*H_NUM/2 : ROW_OFFSET_WORDS)),
            .pixel_data(buffered565),.pixel_valid(buffered_valid),.pixel_ready(buffered_ready),
            .cmd_valid(lane_cmd[channel]),.cmd_ready(lane_cmd_ready[channel]),
            .cmd_addr(lane_result_addr[channel]),.cmd_beats(lane_result_beats[channel]),
            .stream_data(lane_payload[channel]),.stream_pop(lane_pop[channel]),.write_done(lane_write_done[channel]),
            .busy(lane_pack_busy[channel]),.done(result_done_pulse[channel]),.error(lane_pack_error[channel]));
    end endgenerate

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            stage<=BOOT_START; settle<=0; request_toggle<=0; work_bank<=0; display_bank<=0;
            publish_pending<=0;publish_bank<=0;dbg_swap_toggle<=0;publish_toggle<=0;
            display_valid<=0; map_ready_flag<=0; init_done<=0;
            capture_active<=0; capture_ready<=0; capture_bank<=0; capture_lines<=0;
            source_base_words<=0; ready_base_words<=0; process_cycles<=0; dbg_last_process_cycles<=0;
            map_write_words<=MAP_WORDS; map_block<=0; map_slot<=0;
            map_block_valid<=0; map_block_last<=0; map_last_written<=0;
            algo_finished<=0; dbg_publish_count<=0; frame_mode<=0; frame_mode_req<=0;
            frame_gamma_curve<=3;frame_gamma_enable<=0;
            affine_enable_frame<=0;
            affine_a00_frame<=32'sd65536; affine_a01_frame<=0; affine_a02_frame<=0;
            affine_a10_frame<=0; affine_a11_frame<=32'sd65536; affine_a12_frame<=0;
            homography_h00_frame<=32'sd65536; homography_h01_frame<=0; homography_h02_frame<=0;
            homography_h10_frame<=0; homography_h11_frame<=32'sd65536; homography_h12_frame<=0;
            homography_h20_frame<=0; homography_h21_frame<=0; homography_h22_frame<=32'sd65536;
            fisheye_frame<=0;lut_bank_frame<=0;cal_fx_frame<=0;cal_fy_frame<=0;cal_ifx_frame<=0;cal_ify_frame<=0;
            cal_icx_frame<=0;cal_icy_frame<=0;cal_ocx_frame<=0;cal_ocy_frame<=0;
            cal_k1_frame<=0;cal_k2_frame<=0;cal_p1_frame<=0;cal_p2_frame<=0;
            cal_gen_frame<=0;publish_cal_gen<=0;display_cal_gen<=0; frozen_raw_base<=0; frozen_gamma_curve<=3;
        end else begin
            if(display_frame_start && swap_safe) begin
                display_bank<=publish_bank;publish_pending<=0;
                display_cal_gen<=publish_cal_gen;
                display_valid<=1;init_done<=1;
                dbg_swap_toggle<=~dbg_swap_toggle;
                dbg_publish_count<=dbg_publish_count+1'b1;
            end
            if (cap_write_done) begin
                capture_lines<=capture_lines+1'b1;
                if (capture_lines==V_NUM-1) begin
                    capture_active<=0; capture_ready<=1;
                    ready_base_words<={5'd0,capture_bank,22'd0};
                end
            end
            if (algo_done) algo_finished<=1;
            if (stage==PROCESS) process_cycles<=process_cycles+1'b1;
            case(stage)
                BOOT_START: begin stage<=CAP_REQUEST;map_ready_flag<=1;end
                BOOT_MAP: begin
                    if (map_accept) begin
                        map_block[map_slot*64 +:64]<={map_v,map_u};
                        if (map_slot==3) begin map_slot<=0; map_block_valid<=1; map_block_last<=map_last; end
                        else map_slot<=map_slot+1'b1;
                    end
                    if (write_pop && write_owner==0) begin
                        map_block_valid<=0; map_write_words<=map_write_words+8;
                        if (map_block_last) map_last_written<=1;
                    end
                    if (write_done && map_last_written) begin stage<=MAP_SETTLE; settle<=0; end
                end
                MAP_SETTLE: if (settle==63) begin stage<=CAP_REQUEST; map_ready_flag<=1; end
                    else settle<=settle+1'b1;
                CAP_REQUEST: begin
                    request_toggle<=!request_toggle; capture_lines<=0; capture_active<=1; stage<=CAPTURE;
                end
                CAPTURE: if (capture_ready && free_result_valid) begin
                    source_base_words<=ready_base_words;work_bank<=free_result_bank;
                    // 显示模式和 Gamma 在同一拍决定，保证一帧内一致
                    frame_gamma_enable<=ADAPTIVE_GAMMA && !enhancement_bypass && (!display_original || freeze_req);
                    // A capture must contain a full correction, even while the
                    // live display is showing original or split comparison.
                    frame_mode_req<=freeze_req ? 2'd0 : display_original ? 2'd1 : display_compare ? 2'd2 : 2'd0;
                    stage<=CAP_SETTLE;settle<=0;
                end
                CAP_SETTLE: if (settle==63) begin
                    frame_gamma_curve<=gamma_curve_ddr;
                    affine_enable_frame<=affine_enable;
                    affine_a00_frame<=affine_a00; affine_a01_frame<=affine_a01; affine_a02_frame<=affine_a02;
                    affine_a10_frame<=affine_a10; affine_a11_frame<=affine_a11; affine_a12_frame<=affine_a12;
                    homography_h00_frame<=affine_enable ? (z_affine_mode ? affine_a00 : homography_h00) : 32'sd65536;
                    homography_h01_frame<=affine_enable ? (z_affine_mode ? affine_a01 : homography_h01) : 32'sd0;
                    homography_h02_frame<=affine_enable ? (z_affine_mode ? affine_a02 : homography_h02) : 32'sd0;
                    homography_h10_frame<=affine_enable ? (z_affine_mode ? affine_a10 : homography_h10) : 32'sd0;
                    homography_h11_frame<=affine_enable ? (z_affine_mode ? affine_a11 : homography_h11) : 32'sd65536;
                    homography_h12_frame<=affine_enable ? (z_affine_mode ? affine_a12 : homography_h12) : 32'sd0;
                    homography_h20_frame<=affine_enable && !z_affine_mode ? homography_h20 : 32'sd0;
                    homography_h21_frame<=affine_enable && !z_affine_mode ? homography_h21 : 32'sd0;
                    homography_h22_frame<=affine_enable ? (z_affine_mode ? 32'sd65536 : homography_h22) : 32'sd65536;
                    fisheye_frame<=fisheye_enable;lut_bank_frame<=lut_bank;
                    cal_fx_frame<=cal_fx_q16;cal_fy_frame<=cal_fy_q16;
                    cal_ifx_frame<=cal_ifx_q30;cal_ify_frame<=cal_ify_q30;
                    cal_icx_frame<=cal_icx_q16;cal_icy_frame<=cal_icy_q16;
                    cal_ocx_frame<=cal_ocx_q16;cal_ocy_frame<=cal_ocy_q16;
                    cal_k1_frame<=cal_k1_q24;cal_k2_frame<=cal_k2_q24;
                    cal_p1_frame<=cal_p1_q24;cal_p2_frame<=cal_p2_q24;
                    cal_gen_frame<=cal_gen;
                    stage<=ALGO_START;
                end else settle<=settle+1'b1;
                ALGO_START: begin
                    stage<=PROCESS; algo_finished<=0; process_cycles<=0;
                    capture_ready<=0; capture_bank<=!capture_bank; capture_lines<=0; capture_active<=1;
                    request_toggle<=!request_toggle;
                    frame_mode<=frame_mode_req;
                end
                PROCESS: if (result_done && (algo_finished || algo_done)) begin
                    stage<=RESULT_SETTLE; settle<=0; dbg_last_process_cycles<=process_cycles;
                end
                RESULT_SETTLE: if (settle==63) begin
                    // Publish unconditionally. Waiting for publish_pending to
                    // clear held the whole pipeline until the display swapped,
                    // which cost up to a full output frame of LAT and framed
                    // the result rate to the swap grid. A result the display
                    // never took is stale by definition, so overwriting it is
                    // the correct trade: free_result_bank keeps the bank that
                    // is on screen out of the write pool.
                    publish_bank<=work_bank;publish_pending<=1;
                    stage<=freeze_req && frame_mode==0 && coord_complete ? FROZEN : CAPTURE;
                    frozen_gamma_curve<=frame_gamma_curve;
                    frozen_raw_base<=source_base_words;     // 这帧结果用的原图
                    publish_cal_gen<=cal_gen_frame;
                    publish_toggle<=~publish_toggle;
                end else settle<=settle+1'b1;
                WAIT_SWAP: stage<=CAPTURE;
                FROZEN: if (!freeze_req) stage<=CAPTURE;
                FAULT: begin end
                default: stage<=FAULT;
            endcase
            if (dbg_error && !write_busy) stage<=FAULT;
        end
    end
endmodule
`default_nettype wire

// Observes accepted mapper outputs only; never drives mapper ready or video data.
// 8 x 9 grid locations, two adjacent pixels each. Upper/lower lanes have
// independent narrow RAM write ports, retaining only 144 x 64 coordinate bits.
module coordinate_trace_100h(
 input wire clk,rst_n,start,enable,
 input wire [31:0] u0,v0,u1,v1,
 input wire [15:0] x0,y0,x1,y1,input wire take0,take1,
 input wire [7:0] read_addr,output wire [63:0] read_data,
 output reg complete,output wire [7:0] sample_count);
 (* ram_style="block", syn_ramstyle="block_ram" *) reg [63:0] upper[0:63];
 (* ram_style="block", syn_ramstyle="block_ram" *) reg [63:0] lower[0:79];
 reg [63:0] upper_seen;reg [79:0] lower_seen;
 reg armed;reg [63:0] upper_q,lower_q;reg read_lower;
 reg [6:0] count0,count1;
 wire [5:0] index0={y0[8:7],x0[10:8],x0[0]};
 wire [6:0] index1={y1[10:7]-4'd4,x1[10:8],x1[0]};
 wire hit0=armed&&take0&&x0[15:11]==0&&x0[7:1]==7'd32&&
           y0[15:9]==0&&y0[6:0]==7'd28;
 wire hit1=armed&&take1&&x1[15:11]==0&&x1[7:1]==7'd32&&
           y1[15:11]==0&&y1[10:7]>=4&&y1[10:7]<9&&y1[6:0]==7'd28;
 always @(posedge clk)begin
  upper_q<=upper[read_addr[5:0]];
  lower_q<=lower[read_addr-8'd64];
  read_lower<=read_addr>=64;
  if(hit0)upper[index0]<={v0,u0};
  if(hit1)lower[index1]<={v1,u1};
  if(!rst_n)begin complete<=0;upper_seen<=0;lower_seen<=0;armed<=0;count0<=0;count1<=0;end
  else if(start)begin complete<=0;upper_seen<=0;lower_seen<=0;armed<=enable;count0<=0;count1<=0;end
  else begin
   complete<=armed&&(&upper_seen)&&(&lower_seen);
   if(hit0&&!upper_seen[index0])begin upper_seen[index0]<=1;count0<=count0+1'b1;end
   if(hit1&&!lower_seen[index1])begin lower_seen[index1]<=1;count1<=count1+1'b1;end
  end
 end
 assign read_data=read_lower?lower_q:upper_q;

 assign sample_count={1'b0,count0}+{1'b0,count1};
endmodule
