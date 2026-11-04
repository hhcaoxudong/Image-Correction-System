`timescale 1ns/1ps
`default_nettype none
// Debounced press/release classification. Long press fires once while held;
// release after a long press never emits the short-press calibration event.
module pose_key_control #(parameter integer CLK_HZ=10000000)(
 input wire clk,rst_n,key_n,
 output reg short_press,long_press
);
 localparam integer DB=(CLK_HZ/100>0)?CLK_HZ/100:1;
 localparam integer HOLD=(CLK_HZ*2>0)?CLK_HZ*2:1;
 reg meta,sync,stable,held;
 reg [31:0] db_count,hold_count;
 always @(posedge clk)begin
  if(!rst_n)begin meta<=1;sync<=1;stable<=1;held<=0;db_count<=0;hold_count<=0;short_press<=0;long_press<=0;end
  else begin
   meta<=key_n;sync<=meta;short_press<=0;long_press<=0;
   if(sync==stable)db_count<=0;
   else if(db_count<DB-1)db_count<=db_count+1'b1;
   else begin
    stable<=sync;db_count<=0;
    if(!sync)begin hold_count<=0;held<=0;end
    else begin if(!held)short_press<=1;hold_count<=0;end
   end
   if(!stable)begin
    if(hold_count<HOLD-1)hold_count<=hold_count+1'b1;
    else if(!held)begin held<=1;long_press<=1;end
   end
  end
 end
endmodule
`default_nettype wire
