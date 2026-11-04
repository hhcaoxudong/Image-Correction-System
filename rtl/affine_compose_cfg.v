`timescale 1ns/1ps
`default_nettype none
// Configuration-clock affine selection: manual takes priority over IMU.
// Manual B uses absolute coordinates; convert its offset to the Brown centre.
// Pose-only coefficients retain both diagonal elements from the calibrated matrix.
module affine_compose_cfg(
 input wire clk,rst_n,update,
 input wire manual_enable,pose_enable,
 input wire signed [31:0] cx,cy,
 input wire signed [31:0] b00,b01,b02,b10,b11,b12,
 input wire signed [31:0] p00,p01,p02,p10,p11,p12,
 output reg valid,busy,enabled,error,
 output reg signed [31:0] c00,c01,c02,c10,c11,c12
);
 reg pending;
 reg [5:0] state;
 reg [3:0] index;
 reg signed [31:0] b[0:5];
 reg signed [31:0] cxq,cyq;
 reg signed [31:0] lhs,rhs;
 reg signed [63:0] product,acc;
 reg signed [31:0] tx,ty;
 reg enq,bad;
 wire signed [63:0] reduced=acc>>>16;
 integer i;
 always @(posedge clk) begin
  if(!rst_n) begin
   pending<=0;state<=0;index<=0;valid<=0;busy<=0;enabled<=0;error<=0;
   lhs<=0;rhs<=0;product<=0;acc<=0;cxq<=0;cyq<=0;tx<=0;ty<=0;enq<=0;bad<=0;
   c00<=65536;c01<=0;c02<=0;c10<=0;c11<=65536;c12<=0;
   for(i=0;i<6;i=i+1)begin b[i]<=0;end
  end else begin
   valid<=0;
   if(update)pending<=1;
   case(state)
    0:if(pending || update)begin
     pending<=0;busy<=1;bad<=0;cxq<=cx;cyq<=cy;enq<=manual_enable || pose_enable;
     b[0]<=manual_enable ? (b00>>>8) : 65536;b[1]<=manual_enable ? (b01>>>8) : 0;
     b[2]<=manual_enable ? b02 : 0;b[3]<=manual_enable ? (b10>>>8) : 0;
     b[4]<=manual_enable ? (b11>>>8) : 65536;b[5]<=manual_enable ? b12 : 0;
     index<=0;
     if(manual_enable)state<=1;
     else begin
      c00<=pose_enable ? p00 : 65536;
      c11<=pose_enable ? p11 : 65536;
      c01<=pose_enable ? p01 : 0;c02<=pose_enable ? p02 : 0;
      c10<=pose_enable ? p10 : 0;c12<=pose_enable ? p12 : 0;
      enabled<=pose_enable;error<=0;busy<=0;valid<=1;state<=0;
     end
    end
    // A shared signed multiplier. Set operands, register product, consume.
    1:begin lhs<=b[index==0 ? 0 : 3];rhs<=cxq;state<=2;end
    2:begin product<=lhs*rhs;state<=3;end
    3:begin acc<=product;lhs<=b[index==0 ? 1 : 4];rhs<=cyq;state<=4;end
    4:begin product<=lhs*rhs;state<=5;end
    5:begin acc<=acc+product+($signed(index==0 ? b[2] : b[5])<<<16)-($signed({{32{(index==0 ? cxq[31] : cyq[31])}},(index==0 ? cxq : cyq)})<<<16);state<=6;end
    6:begin
     if(reduced>2147483647 || reduced< -64'sd2147483648)bad<=1;
     if(index==0)begin tx<=reduced[31:0];index<=1;state<=1;end
     else begin ty<=reduced[31:0];state<=13;end
    end
    13:begin
     // Reject the whole new matrix on arithmetic overflow; retain the last
     // committed matrix instead of allowing coefficient wraparound.
     if(!bad)begin
      c00<=b[0];c01<=b[1];c02<=tx;
      c10<=b[3];c11<=b[4];c12<=ty;enabled<=enq;
     end
     error<=bad;valid<=!bad && !pending && !update;busy<=0;state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
