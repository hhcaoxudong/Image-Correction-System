`timescale 1ns/1ps
`default_nettype none
// One accepted request returns 256 bytes. Keep ownership until busy falls.
// Native DDR addresses count 32-bit words; bursts never cross a 1024-word column.
module hdmi_capture_reader(
 input wire clk,rst_n,req,
 input wire [1:0] kind,
 input wire [31:0] offset,
 input wire [27:0] raw_base,res_base,
 input wire [2:0] gamma_curve,
 output reg accept,valid,done,
 output reg [255:0] data,
 output wire busy,
 output reg [27:0] ext_addr,
 output reg [3:0] ext_len,
 output reg ext_valid,
 input wire ext_ready,
 input wire [255:0] ext_rdata,
 input wire ext_rvalid,ext_rlast
);
 localparam IDLE=0,PREP=1,ADDR=2,READ=3,LOAD=4,RAW=5,
            R_ADDR=6,R_WAIT=7,R_SAVE=8,G_WAIT=9,G_SAVE=10,
            B_WAIT=11,B_SAVE=12,EMIT=13;
 reg [3:0] state;
 reg [255:0] buffer[0:7];
 reg [3:0] received,remaining,burst_count,burst_received;
 reg [2:0] out_index,curve;
 reg [1:0] saved_kind;
 reg [3:0] pixel_index;
 reg [255:0] word_in,word_out;
 reg [10:0] gamma_addr;
 reg [7:0] gamma_value;
 reg [7:0] gamma_lut[0:1791];
 wire [10:0] column_words=11'd1024-{1'b0,ext_addr[9:0]};
 wire [7:0] column_beats=column_words[10:3];
 wire [3:0] next_burst=column_beats<remaining ? column_beats[3:0] : remaining;
 wire [15:0] pixel=word_in[pixel_index*16+:16];
 assign busy=state!=IDLE;
 initial $readmemh("source/undistort/lut/gamma_8.hex",gamma_lut);
 always @(posedge clk) gamma_value<=gamma_lut[gamma_addr];
 always @(posedge clk) begin
  if(!rst_n) begin
   state<=IDLE;accept<=0;valid<=0;done<=0;data<=0;
   ext_addr<=0;ext_len<=0;ext_valid<=0;
   received<=0;remaining<=0;burst_count<=0;burst_received<=0;
   out_index<=0;curve<=0;saved_kind<=0;pixel_index<=0;
   word_in<=0;word_out<=0;gamma_addr<=0;
  end else begin
   accept<=0;valid<=0;done<=0;
   case(state)
    IDLE:if(req)begin
     accept<=1;saved_kind<=kind;curve<=gamma_curve>6 ? 3'd6 : gamma_curve;
     ext_addr<=(kind==3 ? res_base : raw_base)+offset[29:2];
     received<=0;remaining<=8;out_index<=0;state<=PREP;
    end
    PREP:begin
     burst_count<=next_burst;ext_len<=next_burst-1'b1;
     ext_valid<=1;burst_received<=0;state<=ADDR;
    end
    ADDR:if(ext_ready)begin ext_valid<=0;state<=READ;end
    READ:if(ext_rvalid)begin
     buffer[received[2:0]]<=ext_rdata;
     received<=received+1'b1;burst_received<=burst_received+1'b1;
     if(ext_rlast && burst_received+1'b1==burst_count)begin
      if(remaining==burst_count) state<=LOAD;
      else begin
       ext_addr<=ext_addr+{21'd0,burst_count,3'd0};
       remaining<=remaining-burst_count;state<=PREP;
      end
     end
    end
    LOAD:begin
     word_in<=buffer[out_index];word_out<=0;pixel_index<=0;
     state<=saved_kind==2 ? R_ADDR : RAW;
    end
    RAW:begin data<=word_in;valid<=1;state<=EMIT;end
    R_ADDR:begin gamma_addr<={curve,pixel[15:11],pixel[15:13]};state<=R_WAIT;end
    R_WAIT:state<=R_SAVE;
    R_SAVE:begin
     word_out[pixel_index*16+11+:5]<=gamma_value[7:3];
     gamma_addr<={curve,pixel[10:5],pixel[10:9]};state<=G_WAIT;
    end
    G_WAIT:state<=G_SAVE;
    G_SAVE:begin
     word_out[pixel_index*16+5+:6]<=gamma_value[7:2];
     gamma_addr<={curve,pixel[4:0],pixel[4:2]};state<=B_WAIT;
    end
    B_WAIT:state<=B_SAVE;
    B_SAVE:begin
     word_out[pixel_index*16+:5]<=gamma_value[7:3];
     if(pixel_index==15)state<=RAW;
     else begin pixel_index<=pixel_index+1'b1;state<=R_ADDR;end
    end
    EMIT:begin
     if(out_index==7)begin state<=IDLE;end
     else begin out_index<=out_index+1'b1;state<=LOAD;end
    end
    default:state<=IDLE;
   endcase
   // The final blue update precedes RAW by one clock.
   if(state==RAW && saved_kind==2)data<=word_out;
   if(state==RAW && out_index==7)done<=1;
  end
 end
endmodule
`default_nettype wire
