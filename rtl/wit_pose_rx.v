`timescale 1ns/1ps
`default_nettype none

// WIT 11-byte packets, UART 8N1. Keep the signed raw angle resolution.
// Good non-angle packets are consumed whole; payload 0x55 is ordinary data.
module wit_pose_rx #(
    parameter integer CLK_HZ=10000000,
    parameter integer BAUD=115200,
    parameter integer STALE_MS=500
)(
    input wire clk,rst_n,sensor_rx,
    output reg frame_valid,
    output reg signed [15:0] roll_raw,pitch_raw,yaw_raw,
    output reg received_latched,
    output wire fresh
);
    localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer GAP_CYCLES=DIV*40;
    localparam integer STALE_CYCLES=(CLK_HZ/1000)*STALE_MS;
    reg rx_meta,rx_sync,rx_busy;
    reg [31:0] bit_count;
    reg [3:0] bit_index;
    reg [7:0] shift_data,byte_data;
    reg byte_valid,framing_error;
    always @(posedge clk)begin
        if(!rst_n)begin
            rx_meta<=1;rx_sync<=1;rx_busy<=0;bit_count<=0;bit_index<=0;
            shift_data<=0;byte_data<=0;byte_valid<=0;framing_error<=0;
        end else begin
            rx_meta<=sensor_rx;rx_sync<=rx_meta;
            byte_valid<=0;framing_error<=0;
            if(!rx_busy)begin
                if(!rx_sync)begin rx_busy<=1;bit_count<=DIV/2;bit_index<=0;end
            end else if(bit_count!=0)bit_count<=bit_count-1'b1;
            else if(bit_index==0)begin
                if(rx_sync)rx_busy<=0;
                else begin bit_index<=1;bit_count<=DIV-1;end
            end else if(bit_index<=8)begin
                shift_data[bit_index-1]<=rx_sync;
                bit_index<=bit_index+1'b1;bit_count<=DIV-1;
            end else begin
                rx_busy<=0;byte_data<=shift_data;
                byte_valid<=rx_sync;framing_error<=!rx_sync;
            end
        end
    end

    reg [7:0] packet[0:10];
    reg [3:0] frame_index;
    reg [7:0] checksum;
    reg recovering;
    reg [31:0] gap_count,age;
    integer j;
    integer recovery_offset;
    reg [7:0] recovery_sum;
    integer scan;
    // Only failed candidates use suffix recovery. Never split a good packet
    // merely because its payload happens to contain another 55/type pair.
    always @*begin
        recovery_offset=0;
        for(scan=9;scan>=1;scan=scan-1)
            if(packet[scan]==8'h55 && packet[scan+1][7:4]==4'h5)
                recovery_offset=scan;
        recovery_sum=0;
        for(scan=0;scan<11;scan=scan+1)
            if(scan>=recovery_offset)recovery_sum=recovery_sum+packet[scan];
    end
    assign fresh=received_latched && age<STALE_CYCLES;
    always @(posedge clk)begin
        if(!rst_n)begin
            frame_valid<=0;roll_raw<=0;pitch_raw<=0;yaw_raw<=0;
            received_latched<=0;frame_index<=0;checksum<=0;recovering<=0;
            gap_count<=0;age<=STALE_CYCLES;
            for(j=0;j<11;j=j+1)packet[j]<=0;
        end else begin
            frame_valid<=0;
            if(age<STALE_CYCLES)age<=age+1'b1;
            if(framing_error)begin
                frame_index<=0;checksum<=0;recovering<=0;gap_count<=0;
            end else if(recovering)begin
                recovering<=0;gap_count<=0;
                if(recovery_offset!=0)begin
                    for(j=0;j<10;j=j+1)
                        if(j<11-recovery_offset)packet[j]<=packet[j+recovery_offset];
                    frame_index<=11-recovery_offset;checksum<=recovery_sum;
                end else if(packet[10]==8'h55)begin
                    packet[0]<=8'h55;frame_index<=1;checksum<=8'h55;
                end else begin frame_index<=0;checksum<=0;end
            end else if(byte_valid)begin
                gap_count<=0;
                if(frame_index==0)begin
                    if(byte_data==8'h55)begin packet[0]<=byte_data;frame_index<=1;checksum<=byte_data;end
                end else if(frame_index==1)begin
                    if(byte_data[7:4]==4'h5)begin
                        packet[1]<=byte_data;frame_index<=2;checksum<=checksum+byte_data;
                    end else begin frame_index<=0;checksum<=0;end
                end else if(frame_index<10)begin
                    packet[frame_index]<=byte_data;frame_index<=frame_index+1'b1;
                    checksum<=checksum+byte_data;
                end else begin
                    packet[10]<=byte_data;frame_index<=0;checksum<=0;
                    if(byte_data==checksum)begin
                        if(packet[1]==8'h53)begin
                            roll_raw<=$signed({packet[3],packet[2]});
                            pitch_raw<=$signed({packet[5],packet[4]});
                            yaw_raw<=$signed({packet[7],packet[6]});
                            frame_valid<=1;received_latched<=1;age<=0;
                        end
                    end else recovering<=1;
                end
            end else if(frame_index!=0)begin
                if(gap_count<GAP_CYCLES-1)gap_count<=gap_count+1'b1;
                else begin frame_index<=0;checksum<=0;gap_count<=0;end
            end else gap_count<=0;
        end
    end
endmodule
`default_nettype wire
