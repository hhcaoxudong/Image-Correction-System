`timescale 1ns/1ps
`default_nettype none
// Immutable packet RAM, toggle/ack CDC. 8x8 monochrome HDMI cells, LSB first.
module hdmi_full_transport #(parameter TOTAL=4147200, parameter REPEATS=4, parameter COORDINATES=1)(
 input wire clk,rst_n,enable,allowed,input wire one_shot_start,abort,release_cmd,input wire [3:0] capture_state,
 input wire [31:0] frame_id,errors,input wire [287:0] active_words,
 input wire [31:0] inv_fx,inv_fy,input wire [15:0] active_sequence,input wire [1:0] model,
 output wire [7:0] coord_read_addr,input wire [63:0] coord_read_data,
 input wire [383:0] coord_calibration,input wire [191:0] coord_affine,
 input wire coord_affine_enabled,coord_complete,
 input wire [7:0] coord_sample_count,coord_generation,input wire [3:0] coord_gamma,
 output reg owned,capture_request,capture_release,
 output wire export_req,output reg [1:0] export_kind,output reg [31:0] export_offset,
 input wire export_accept,export_valid,export_done,input wire [255:0] export_data,
 input wire pclk,prst_n,vs,hs,de,input wire [15:0] rgb,
 output wire ovs,ohs,ode,output wire [15:0] orgb,output wire digital);
 localparam PAYLOAD=3584, CHUNKS=(TOTAL+PAYLOAD-1)/PAYLOAD;
 (* ram_style="block", syn_ramstyle="block_ram" *) reg [255:0] mem[0:127];
 reg [4:0] state;reg [7:0] wa,ra;reg [255:0] wd,rd;reg we;
 // Register the packet write command before the wide block-RAM input.
 // All packet consumers start after this extra cycle: CRC starts through
 // states 9/10/11, and pixel publication crosses a two-stage synchronizer.
 reg write_pending=0;
 reg [7:0] write_address;
 reg [255:0] write_data;
 always @(posedge clk)begin
  write_pending<=rst_n&&we;
  write_address<=wa;write_data<=wd;
  if(write_pending&&rst_n)mem[write_address]<=write_data;
  rd<=mem[ra];
 end
 reg [1023:0] coordinate_header;
 reg [255:0] coordinate_pair;
 reg [6:0] coordinate_word;
 reg [7:0] coordinate_index;
 assign coord_read_addr=coordinate_index;
 wire [15:0] coordinate_x=16'd64+{5'd0,coordinate_index[3:1],8'd0}+{15'd0,coordinate_index[0]};
 wire [15:0] coordinate_y=16'd28+{5'd0,coordinate_index[7:4],7'd0};
 wire coordinate_valid=coord_read_data[31:0]!=32'hffffffff&&coord_read_data[63:32]!=32'hffffffff;
 wire [31:0] coordinate_flags={29'd0,
     (coord_read_data[63:32]==0||coord_read_data[63:32]==32'd70647808),
     (coord_read_data[31:0]==0||coord_read_data[31:0]==32'd125698048),coordinate_valid};
 reg [511:0] header,info;reg [287:0] words;reg [31:0] ifx,ify;
 reg [15:0] seq;reg [1:0] mdl;reg [63:0] ticks,token;reg [31:0] cid,fid;
 reg [7:0] kind;reg [15:0] length,chunk_index,filled;reg [3:0] beats;
 reg [31:0] offset,icrc,pcrc,ocrc,irun,prun,orun,crc;reg [2:0] crc_valid;
 reg [12:0] pos;reg [2:0] crc_bit;reg [255:0] scan;reg publish;
 (* async_reg="true" *) reg ack_meta,ack_sync;
 reg ack,one_shot_run,video_enable;
 (* async_reg="true" *) reg idle_meta,idle_sync;
 wire pixel_idle;
 // One CRC bit per core clock avoids an eight-level XOR chain on the
 // 100 MHz control clock. Export is an offline snapshot, so this trades
 // time for timing without changing the SFH2 wire protocol.
 function [31:0] crc_step;input [31:0] c;input d;reg [31:0] t;begin
  t=c;if(t[0]^d)t=(t>>1)^32'hedb88320;else t=t>>1;crc_step=t;
 end endfunction
 wire [7:0] scan_byte=scan[pos[4:0]*8+:8];
 assign export_req=state==3&&!abort&&!release_cmd;
 always @* begin
  we=0;wa=0;wd=0;ra=pos[12:5];
  case(state)
   4:begin we=export_valid;wa=2+(filled>>5)+beats;wd=export_data;end
   6:begin we=1;wa=0;wd=header[255:0];end
   7:begin we=1;wa=1;wd=header[511:256];end
   8:if(kind==0||kind==4)begin we=1;wa=2;wd=info[255:0];end
   9:if(kind==0||kind==4)begin we=1;wa=3;wd=info[511:256];end
   25:begin we=1;wa=2+coordinate_word;wd=coordinate_pair;end
   26:begin we=1;wa=2+coordinate_word;wd=coordinate_header[coordinate_word[1:0]*256+:256];end
   14:begin we=1;wa=(64+length)>>5;wd={224'd0,~crc};end
   default:begin end
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  coordinate_header<=0;coordinate_pair<=0;coordinate_word<=0;coordinate_index<=0;
  state<=0;owned<=0;capture_request<=0;capture_release<=0;export_kind<=0;export_offset<=0;
  ticks<=0;token<=0;cid<=0;fid<=0;words<=0;ifx<=0;ify<=0;seq<=0;mdl<=0;
  kind<=0;length<=0;chunk_index<=0;filled<=0;beats<=0;offset<=0;icrc<=0;pcrc<=0;ocrc<=0;
  irun<=32'hffffffff;prun<=32'hffffffff;orun<=32'hffffffff;crc<=0;crc_valid<=0;pos<=0;crc_bit<=0;scan<=0;publish<=0;one_shot_run<=0;
  ack_meta<=0;ack_sync<=0;header<=0;info<=0;video_enable<=0;idle_meta<=0;idle_sync<=0;
 end else begin
  ticks<=ticks+1'b1;ack_meta<=ack;ack_sync<=ack_meta;idle_meta<=pixel_idle;idle_sync<=idle_meta;
  capture_request<=0;capture_release<=0;
  // A stop cannot revoke an accepted DDR transaction. Stop publication,
  // drain the reader, then wait for a pixel-frame boundary before reuse.
  if(owned&&state!=20&&state!=22&&
     (abort||release_cmd||(!enable&&!one_shot_run)))begin
   video_enable<=0;one_shot_run<=0;
   if((state==4||(state==3&&export_accept))&&!export_done)state<=22;
   else begin capture_release<=1;state<=20;end
  end else case(state)
  0:if((enable||one_shot_start)&&allowed&&capture_state==0)begin
   owned<=1;video_enable<=1;capture_request<=1;token<=ticks;cid<=cid+1'b1;
   one_shot_run<=one_shot_start&&!enable;
   words<=active_words;ifx<=inv_fx;ify<=inv_fy;seq<=active_sequence;mdl<=model;
   crc_valid<=0;icrc<=0;pcrc<=0;ocrc<=0;irun<=32'hffffffff;prun<=32'hffffffff;orun<=32'hffffffff;state<=1;
  end
  1:if(capture_state==6)begin
   if(COORDINATES&&(!coord_complete||coord_sample_count!=8'd144))state<=21;
   else begin
    // Metadata belongs to the frozen mapper frame, never the earlier request.
    words<=active_words;ifx<=inv_fx;ify<=inv_fy;seq<=active_sequence;mdl<=model;
    fid<=frame_id;kind<=0;length<=64;offset<=0;chunk_index<=0;state<=5;
    coordinate_header<=0;
    coordinate_header[31:0]<=32'h314d5343;
    coordinate_header[47:32]<=16'd144;coordinate_header[63:48]<=16'd16;
    coordinate_header[79:64]<=16'd16;
    coordinate_header[95:80]<={14'd0,coord_complete,coord_affine_enabled};
    coordinate_header[127:96]<=frame_id;coordinate_header[159:128]<={30'd0,model};
    coordinate_header[191:160]<=32'd2;
    coordinate_header[223:192]<={24'd0,coord_generation};
    coordinate_header[255:224]<=32'd72;
    coordinate_header[447:256]<=coord_affine;
    coordinate_header[479:448]<=32'd125698048;
    coordinate_header[511:480]<=32'd70647808;
    coordinate_header[895:512]<=coord_calibration;
    coordinate_header[927:896]<={16'd1080,16'd1920};
    coordinate_header[959:928]<={24'd0,coord_sample_count};
    coordinate_header[991:960]<={28'd0,coord_gamma};
   end
  end
  2:begin
   filled<=0;beats<=0;export_kind<=kind[1:0];export_offset<=offset;
   length<=TOTAL-offset<PAYLOAD?TOTAL-offset:PAYLOAD;state<=3;
  end
  3:if(export_accept)begin beats<=0;state<=4;end
  4:begin
   if(export_valid)beats<=beats+1'b1;
    if(export_done)begin
    // native_adapter returns eight 256-bit beats.  export_done is asserted
    // with the final valid beat, so the pre-increment counter is seven.
    if(beats!=7)state<=21;
    else if(filled+256>=length)state<=5;
    else begin filled<=filled+256;export_offset<=export_offset+256;beats<=0;state<=3;end
   end
  end
  5:begin
   header<=0;
   header[31:0]<=32'h32484653;header[39:32]<=2;header[47:40]<=kind;header[63:48]<=length;
   header[95:64]<=cid;header[127:96]<=fid;header[159:128]<=TOTAL;header[191:160]<=offset;
   header[207:192]<=1920;header[223:208]<=1080;header[239:224]<=seq;header[247:240]<={6'd0,mdl};header[255:248]<=COORDINATES?8'd1:8'd0;
   header[319:256]<=token;header[335:320]<=CHUNKS;header[351:336]<=chunk_index;
   header[383:352]<=icrc;header[415:384]<=ocrc;header[447:416]<={errors[28:0],crc_valid};header[479:448]<=pcrc;
   info<={ocrc,icrc,errors,TOTAL[31:0],16'd1080,16'd1920,ify,ifx,words};state<=6;
  end
  6:state<=7;
  7:state<=8;
  8:if(kind==5)begin coordinate_word<=0;coordinate_index<=0;state<=26;end else state<=9;
  23:state<=24; // synchronous mapper sample RAM latency
  24:begin
   if(!coordinate_index[0])coordinate_pair[127:0]<={coordinate_y,coordinate_x,coordinate_flags,coord_read_data};
   else coordinate_pair[255:128]<={coordinate_y,coordinate_x,coordinate_flags,coord_read_data};
   if(coordinate_index[0])state<=25;
   else begin coordinate_index<=coordinate_index+1'b1;state<=23;end
  end
  25:if(coordinate_index==143)state<=9;
     else begin coordinate_index<=coordinate_index+1'b1;coordinate_word<=coordinate_word+1'b1;state<=23;end
  26:if(coordinate_word==3)begin coordinate_word<=4;state<=23;end
     else coordinate_word<=coordinate_word+1'b1;
  9:begin pos<=0;crc_bit<=0;crc<=32'hffffffff;state<=10;end
  10:state<=11;
  11:begin scan<=rd;crc_bit<=0;state<=12;end
  12:begin
   crc<=crc_step(crc,scan_byte[crc_bit]);
   if(pos>=64&&(kind==1||kind==2||kind==3))begin
    if(kind==1)irun<=crc_step(irun,scan_byte[crc_bit]);
    else if(kind==2)prun<=crc_step(prun,scan_byte[crc_bit]);
    else orun<=crc_step(orun,scan_byte[crc_bit]);
   end
   if(crc_bit==7)begin
    crc_bit<=0;
    if(pos+1==64+length)state<=14;
    else begin pos<=pos+1'b1;if(pos[4:0]==31)state<=10;end
   end else crc_bit<=crc_bit+1'b1;
  end
  14:state<=15;
  15:begin publish<=~publish;state<=16;end
  16:if(ack_sync==publish)begin
   if(kind==0)begin kind<=1;offset<=0;chunk_index<=0;state<=2;end
   else if(kind==1||kind==2||kind==3)begin
    if(offset+length==TOTAL)begin
     if(kind==1)begin icrc<=~irun;crc_valid[0]<=1;kind<=2;offset<=0;chunk_index<=0;prun<=32'hffffffff;state<=2;end
     else if(kind==2)begin pcrc<=~prun;crc_valid[1]<=1;kind<=3;offset<=0;chunk_index<=0;orun<=32'hffffffff;state<=2;end
     else begin ocrc<=~orun;crc_valid[2]<=1;kind<=COORDINATES?5:4;offset<=0;chunk_index<=0;length<=COORDINATES?2432:64;state<=5;end
    end else begin offset<=offset+length;chunk_index<=chunk_index+1'b1;state<=2;end
   end else if(kind==5)begin kind<=4;length<=64;state<=5;end else begin
    // Keep the frozen frame and identity alive after the final packet.
    // The host can repair dropped HDMI blocks during the next cycle.
    if(kind==4&&one_shot_run)begin
     kind<=0;length<=64;offset<=0;chunk_index<=0;irun<=32'hffffffff;prun<=32'hffffffff;orun<=32'hffffffff;state<=5;
    end else begin
     kind<=0;length<=64;offset<=0;chunk_index<=0;irun<=32'hffffffff;prun<=32'hffffffff;orun<=32'hffffffff;state<=5;
    end
   end
  end
  20:if(capture_state==0&&idle_sync)begin owned<=0;state<=0;end
  22:if(export_done)begin capture_release<=1;state<=20;end
  21:begin end // fail closed; user exits mode to release
  default:state<=0;
  endcase
 end end
 // Pixel side only reads stable published RAM; packet buffers never overlap.
 (* async_reg="true" *) reg pub_meta,pub_sync,en_meta,en_sync;
 reg seen,active;reg [7:0] repeat_count;
 reg prev_vs,prev_de;reg [11:0] x,y;reg [15:0] rowbase;
 assign pixel_idle=!en_sync&&!active&&(ack==pub_sync);
 wire sof=vs&&!prev_vs;
 wire [15:0] cell_index=rowbase+(de?(prev_de?x:12'd0)>>3:0);
 wire [6:0] paddr=cell_index[14:8];
 reg [255:0] prd;reg [4:0] byte_sel;reg [2:0] bit_sel0,bit_sel1;
 reg [7:0] byte_value;reg [18:0] delay0,delay1,delay2;reg dig0,dig1,dig2;
 reg bit_value;
 always @(posedge pclk)begin
  prd<=mem[paddr];byte_sel<=cell_index[7:3];bit_sel0<=cell_index[2:0];
  byte_value<=prd[byte_sel*8+:8];bit_sel1<=bit_sel0;bit_value<=byte_value[bit_sel1];
  delay0<={vs,hs,de,rgb};delay1<=delay0;delay2<=delay1;
  dig0<=active;dig1<=dig0;dig2<=dig1;
 end
 assign {ovs,ohs,ode}=delay2[18:16];
 assign orgb=dig2?(bit_value?16'hffff:16'h0000):delay2[15:0];assign digital=dig2;
 always @(posedge pclk or negedge prst_n)begin
 if(!prst_n)begin pub_meta<=0;pub_sync<=0;en_meta<=0;en_sync<=0;seen<=0;ack<=0;active<=0;repeat_count<=0;prev_vs<=0;prev_de<=0;x<=0;y<=0;rowbase<=0;end
 else begin
  pub_meta<=publish;pub_sync<=pub_meta;en_meta<=video_enable;en_sync<=en_meta;
  prev_vs<=vs;prev_de<=de;
  if(sof)begin
   x<=0;y<=0;rowbase<=0;
   if(!en_sync)begin active<=0;repeat_count<=0;seen<=pub_sync;ack<=pub_sync;end
   else if(active)begin
    if(repeat_count==REPEATS-1)begin ack<=seen;active<=0;repeat_count<=0;end
    else repeat_count<=repeat_count+1'b1;
   end else if(en_sync&&pub_sync!=seen)begin seen<=pub_sync;active<=1;repeat_count<=0;end
  end else begin
   if(de)x<=prev_de?x+1'b1:1;else x<=0;
   if(prev_de&&!de)begin y<=y+1'b1;if(y[2:0]==7)rowbase<=rowbase+240;end
  end
 end end
endmodule
`default_nettype wire

