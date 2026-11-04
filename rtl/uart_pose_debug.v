`timescale 1ns/1ps
`default_nettype none

// One immutable ASCII snapshot per report, UART 8N1, idle high.
// raw angle units: signed 16-bit full scale = 180 degrees.
//
// The frame-pipeline metrics are appended after the pose fields so a single
// terminal line carries everything. Values are in units of 0.01 ms:
//   P process time, R result period, C capture wait (R - P), W publish wait,
//   S frame controller stage at result completion, F displayed frames per second.
module uart_pose_debug #(
    parameter integer CLK_HZ=10000000,
    parameter integer BAUD=115200,
    parameter integer REPORT_HZ=10
)(
    input wire clk, rst_n,
    input wire signed [15:0] roll_raw,pitch_raw,yaw_raw,delta_roll_raw,delta_pitch_raw,delta_yaw_raw,
    input wire enabled,fresh,
    input wire [15:0] process_10us,
    input wire [15:0] period_10us,
    input wire [15:0] capture_wait_10us,
    input wire [15:0] publish_wait_10us,
    input wire [3:0]  stage_code,
    input wire [17:0] swap_10us,
    input wire [15:0] publish_count,
    input wire [3:0]  fps_tens,fps_ones,fps_frac_tens,fps_frac_ones,
    // allow: no report is taken until this goes high (PARAM-V1: after the
    // first "adjust ok"). tx_req/tx_gnt: a line starts only when granted and
    // is sent whole, so it never interleaves with protocol frames.
    input wire allow,
    output wire tx_req,
    input wire tx_gnt,
    output wire line_busy,
    output reg tx
);
    localparam integer REPORT_CYCLES=CLK_HZ/REPORT_HZ;
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer LINE_BYTES=149;
    localparam [3:0] IDLE=0,PRODUCT=1,BCD_INIT=2,BCD_SHIFT=3,STORE=4,
                     P_INIT=5,P_STORE=6,SEND=7;
    reg [3:0] state;
    reg [31:0] report_counter,bit_counter;
    reg signed [15:0] snapshot[0:5];
    reg [15:0] psnapshot[0:5];
    reg [3:0]  snap_stage;
    reg [3:0]  snap_fps[0:3];
    reg [7:0] line[0:LINE_BYTES-1];
    reg [2:0] axis;
    reg [2:0] metric;
    reg [7:0] field_base,pbase;
    reg [30:0] product;
    reg [15:0] binary;
    reg [19:0] bcd;
    reg [4:0] shifts;
    reg negative,perf_mode;
    reg [9:0] tx_shift;
    reg [3:0] tx_bits;
    reg tx_busy;
    reg [7:0] character;
    wire [16:0] magnitude=snapshot[axis][15] ?
        -$signed({snapshot[axis][15],snapshot[axis]}) : {1'b0,snapshot[axis]};
    wire [19:0] adjusted;
    genvar digit;
    generate for(digit=0;digit<5;digit=digit+1)begin:decimal_adjust
        assign adjusted[digit*4+:4]=bcd[digit*4+:4]>=5 ?
            bcd[digit*4+:4]+4'd3 : bcd[digit*4+:4];
    end endgenerate
    assign tx_req    = state==SEND && character==0 && !tx_busy;
    assign line_busy = state!=IDLE || tx_busy;
    wire [3:0] stage_tens = stage_code >= 4'd10 ? 4'd1 : 4'd0;
    wire [3:0] stage_ones = stage_code >= 4'd10 ? (stage_code - 4'd10) : stage_code;
    integer i;
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;report_counter<=0;bit_counter<=0;
            axis<=0;field_base<=8'd12;character<=0;product<=0;binary<=0;
            bcd<=0;shifts<=0;negative<=0;perf_mode<=0;metric<=0;pbase<=8'd76;
            tx<=1;tx_shift<=10'h3ff;tx_bits<=0;tx_busy<=0;snap_stage<=0;
            for(i=0;i<6;i=i+1)snapshot[i]<=0;
            for(i=0;i<6;i=i+1) psnapshot[i]<=0;
            for(i=0;i<4;i=i+1)snap_fps[i]<=0;
            for(i=0;i<LINE_BYTES;i=i+1)
                line[i]<= (i<74) ? 8'h30 : 8'h20;
            line[0]<=8'h52;line[1]<=8'h58;line[2]<=8'h3d;line[4]<=8'h2c;
            line[5]<=8'h45;line[6]<=8'h4e;line[7]<=8'h3d;line[9]<=8'h2c;
            line[10]<=8'h52;line[11]<=8'h3d;line[16]<=8'h2e;line[19]<=8'h2c;
            line[20]<=8'h50;line[21]<=8'h3d;line[26]<=8'h2e;line[29]<=8'h2c;
            line[30]<=8'h59;line[31]<=8'h3d;line[36]<=8'h2e;line[39]<=8'h2c;
            line[40]<=8'h64;line[41]<=8'h52;line[42]<=8'h3d;line[47]<=8'h2e;
            line[50]<=8'h2c;line[51]<=8'h64;line[52]<=8'h50;line[53]<=8'h3d;line[58]<=8'h2e;
            line[61]<=8'h2c;line[62]<=8'h64;line[63]<=8'h59;line[64]<=8'h3d;line[69]<=8'h2e;
            line[71]<=8'h30;line[72]<=8'h0d;line[73]<=8'h0a;
            // Appended frame-pipeline fields: 11 characters each.
            line[74]<=8'h50;line[75]<=8'h3d;line[79]<=8'h2e;
            line[82]<=8'h6d;line[83]<=8'h73;line[84]<=8'h20;
            line[85]<=8'h52;line[86]<=8'h3d;line[90]<=8'h2e;
            line[93]<=8'h6d;line[94]<=8'h73;line[95]<=8'h20;
            line[96]<=8'h43;line[97]<=8'h3d;line[101]<=8'h2e;
            line[104]<=8'h6d;line[105]<=8'h73;line[106]<=8'h20;
            line[107]<=8'h57;line[108]<=8'h3d;line[112]<=8'h2e;
            line[115]<=8'h6d;line[116]<=8'h73;line[117]<=8'h20;
            line[118]<=8'h53;line[119]<=8'h3d;line[122]<=8'h20;
            line[123]<=8'h46;line[124]<=8'h3d;line[127]<=8'h2e;
            line[130]<=8'h20;
            line[131]<=8'h44;line[132]<=8'h3d;line[136]<=8'h2e;
            line[139]<=8'h20;
            line[140]<=8'h4e;line[141]<=8'h3d;
            line[147]<=8'h0d;line[148]<=8'h0a;
        end else begin
            if(report_counter==REPORT_CYCLES-1)report_counter<=0;
            else report_counter<=report_counter+1'b1;
            // UART consumes one complete character before requesting another.
            if(tx_busy)begin
                if(bit_counter==BIT_CYCLES-1)begin
                    bit_counter<=0;tx_shift<={1'b1,tx_shift[9:1]};
                    if(tx_bits==9)begin tx_busy<=0;tx<=1;end
                    else begin tx_bits<=tx_bits+1'b1;tx<=tx_shift[1];end
                end else bit_counter<=bit_counter+1'b1;
            end
            case(state)
                IDLE:if(report_counter==REPORT_CYCLES-1 && allow)begin
                    snapshot[0]<=roll_raw;snapshot[1]<=pitch_raw;
                    snapshot[2]<=yaw_raw;snapshot[3]<=delta_roll_raw;
                    snapshot[4]<=delta_pitch_raw;
                    snapshot[5]<=delta_yaw_raw;
                    psnapshot[0]<=process_10us;
                    psnapshot[1]<=period_10us;
                    psnapshot[2]<=capture_wait_10us;
                    psnapshot[3]<=publish_wait_10us;
                    psnapshot[4]<=swap_10us[15:0];
                    psnapshot[5]<=publish_count;
                    snap_stage<=stage_code;
                    snap_fps[0]<=fps_tens;
                    snap_fps[1]<=fps_ones;
                    snap_fps[2]<=fps_frac_tens;
                    snap_fps[3]<=fps_frac_ones;
                    line[3]<=fresh?8'h31:8'h30;line[8]<=enabled?8'h31:8'h30;
                    axis<=0;field_base<=8'd12;perf_mode<=0;state<=PRODUCT;
                end
                PRODUCT:begin
                    negative<=snapshot[axis][15];product<=magnitude*15'd18000;
                    state<=BCD_INIT;
                end
                BCD_INIT:begin
                    binary<=product[30:15];bcd<=0;shifts<=0;state<=BCD_SHIFT;
                end
                BCD_SHIFT:begin
                    bcd<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
                    if(shifts==15)state<=perf_mode?P_STORE:STORE;
                    else shifts<=shifts+1'b1;
                end
                STORE:begin
                    // Truncate magnitude to hundredths; do not display negative zero.
                    line[field_base]<=(negative&&bcd!=0)?8'h2d:8'h2b;
                    line[field_base+8'd1]<=8'h30+{4'd0,bcd[19:16]};
                    line[field_base+8'd2]<=8'h30+{4'd0,bcd[15:12]};
                    line[field_base+8'd3]<=8'h30+{4'd0,bcd[11:8]};
                    line[field_base+8'd5]<=8'h30+{4'd0,bcd[7:4]};
                    line[field_base+8'd6]<=8'h30+{4'd0,bcd[3:0]};
                    if(axis==5)begin perf_mode<=1;metric<=0;state<=P_INIT;end
                    else begin
                        axis<=axis+1'b1;
                        if(axis==2)field_base<=8'd43;
                        else if(axis==3)field_base<=8'd54;
                        else if(axis==4)field_base<=8'd65;
                        else field_base<=field_base+8'd10;
                        state<=PRODUCT;
                    end
                end
                P_INIT:begin
                    binary<=psnapshot[metric];bcd<=0;shifts<=0;
                    case(metric)
                        2'd0:pbase<=8'd76;
                        2'd1:pbase<=8'd87;
                        2'd2:pbase<=8'd98;
                        2'd3:pbase<=8'd109;
                        3'd4:pbase<=8'd133;
                        default:pbase<=8'd142;
                    endcase
                    state<=BCD_SHIFT;
                end
                P_STORE:begin
                    line[pbase]<=8'h30+{4'd0,bcd[19:16]};
                    line[pbase+8'd1]<=8'h30+{4'd0,bcd[15:12]};
                    line[pbase+8'd2]<=8'h30+{4'd0,bcd[11:8]};
                    // The raw counter has no decimal point; D keeps the x.xx form.
                    line[pbase+(metric==3'd5 ? 8'd3 : 8'd4)]<=8'h30+{4'd0,bcd[7:4]};
                    line[pbase+(metric==3'd5 ? 8'd4 : 8'd5)]<=8'h30+{4'd0,bcd[3:0]};
                    if(metric==3'd5)begin
                        line[8'd120]<=8'h30+{2'd0,stage_tens};
                        line[8'd121]<=8'h30+{2'd0,stage_ones};
                        line[8'd125]<=8'h30+{4'd0,snap_fps[0]};
                        line[8'd126]<=8'h30+{4'd0,snap_fps[1]};
                        line[8'd128]<=8'h30+{4'd0,snap_fps[2]};
                        line[8'd129]<=8'h30+{4'd0,snap_fps[3]};
                        character<=0;state<=SEND;
                    end else begin
                        metric<=metric+1'b1;
                        state<=P_INIT;
                    end
                end
                SEND:if(!tx_busy && (character!=0 || tx_gnt))begin
                    tx_shift<={1'b1,line[character],1'b0};tx<=0;
                    bit_counter<=0;tx_bits<=0;tx_busy<=1;
                    if(character==LINE_BYTES-1)state<=IDLE;
                    else character<=character+1'b1;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
