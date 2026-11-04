`timescale 1ns/1ps
`default_nettype none

// PC attitude commands: R/P/Y=+010.00, R/P/Y=-010.00, R/P/Y=0, OFF, ON.
// KEY1 snapshots all axes; three CORDICs feed one atomic affine coefficient set.
// CR or LF terminates a line. A rejected line never changes the transform.
module uart_rotation_command #(
    parameter integer CLK_HZ=10000000,
    parameter integer BAUD=115200,
    parameter signed [31:0] FY_Q16=32'sd54791085,
    parameter signed [31:0] FX_Q16=32'sd55478575
)(
    input wire clk,rst_n,rx,key1_n,
    output reg enabled,command_valid,received,calibrated,
    output reg signed [15:0] angle_raw,delta_raw,pitch_raw,delta_pitch_raw,
    output reg signed [31:0] pitch_cos_q16,pitch_sin_q16,
    output reg signed [15:0] yaw_raw,delta_yaw_raw,
    output reg signed [31:0] yaw_cos_q16,yaw_sin_q16,
    output reg signed [31:0] a00,a01,a02,a10,a11,a12
);
    localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
    reg rx_meta,rx_sync,rx_busy;
    reg [15:0] bit_count;
    reg [3:0] bit_index;
    reg [7:0] shift_data,byte_data;
    reg byte_valid;
    always @(posedge clk) begin
        if(!rst_n) begin
            rx_meta<=1;rx_sync<=1;rx_busy<=0;bit_count<=0;
            bit_index<=0;shift_data<=0;byte_data<=0;byte_valid<=0;
        end else begin
            rx_meta<=rx;rx_sync<=rx_meta;byte_valid<=0;
            if(!rx_busy) begin
                if(!rx_sync) begin rx_busy<=1;bit_count<=DIV/2;bit_index<=0;end
            end else if(bit_count!=0) bit_count<=bit_count-1'b1;
            else if(bit_index==0) begin
                if(rx_sync)rx_busy<=0;
                else begin bit_index<=1;bit_count<=DIV-1;end
            end else if(bit_index<=8) begin
                shift_data[bit_index-1]<=rx_sync;
                bit_index<=bit_index+1'b1;bit_count<=DIV-1;
            end else begin
                rx_busy<=0;byte_valid<=rx_sync;byte_data<=shift_data;
            end
        end
    end

    reg [7:0] line[0:8];
    reg [3:0] length;
    reg discard_line;
    reg [31:0] idle_count;
    reg signed [15:0] current_angle,base_angle,pending_angle,work_absolute;
    reg signed [15:0] current_pitch,base_pitch,pending_pitch,work_pitch;
    reg signed [15:0] current_yaw,base_yaw,pending_yaw,work_yaw;
    reg math_start,inflight,dirty,active_requested;
    reg [7:0] generation,work_generation;
    reg r_request,p_request,y_request,off_request,on_request;
    reg signed [15:0] requested_angle;
    wire math_busy,math_done;
    wire signed [31:0] cosine,sine;
    rotation_sincos math(.clk(clk),.rst_n(rst_n),.start(math_start),
        .angle_raw(pending_angle),.busy(math_busy),.done(math_done),
        .cos_q16(cosine),.sin_q16(sine));
    wire pitch_busy,pitch_done;
    wire signed [31:0] pitch_cosine,pitch_sine;
    reg roll_complete,pitch_complete,yaw_complete;
    reg signed [31:0] saved_cos,saved_sin,saved_pitch_cos,saved_pitch_sin;
    reg signed [31:0] saved_yaw_cos,saved_yaw_sin;
    wire yaw_busy,yaw_done;
    wire signed [31:0] yaw_cosine,yaw_sine;
    rotation_sincos yaw_math(.clk(clk),.rst_n(rst_n),.start(math_start),
        .angle_raw(pending_yaw),.busy(yaw_busy),.done(yaw_done),
        .cos_q16(yaw_cosine),.sin_q16(yaw_sine));
    reg coeff_start,coeff_started;
    wire coeff_busy,coeff_done,coeff_error;
    wire signed [31:0] coeff_a00,coeff_a01,coeff_a02,coeff_a10,coeff_a11,coeff_a12;
    pitch_affine_coeff #(.FY_Q16(FY_Q16),.FX_Q16(FX_Q16)) coefficient_math(
        .clk(clk),.rst_n(rst_n),.start(coeff_start),
        .roll_cos(saved_cos),.roll_sin(saved_sin),.pitch_cos(saved_pitch_cos),.pitch_sin(saved_pitch_sin),
        .yaw_cos(saved_yaw_cos),.yaw_sin(saved_yaw_sin),
        .busy(coeff_busy),.done(coeff_done),.error(coeff_error),
        .a00(coeff_a00),.a01(coeff_a01),.a02(coeff_a02),.a10(coeff_a10),.a11(coeff_a11),.a12(coeff_a12));
    rotation_sincos pitch_math(.clk(clk),.rst_n(rst_n),.start(math_start),
        .angle_raw(pending_pitch),.busy(pitch_busy),.done(pitch_done),
        .cos_q16(pitch_cosine),.sin_q16(pitch_sine));
    wire angle_axis=line[0]==8'h52 || line[0]==8'h50 || line[0]==8'h59;
    wire decimal_format=length==9 && angle_axis && line[1]==8'h3d &&
        (line[2]==8'h2b || line[2]==8'h2d) && line[6]==8'h2e &&
        line[3]>=8'h30 && line[3]<=8'h39 && line[4]>=8'h30 && line[4]<=8'h39 &&
        line[5]>=8'h30 && line[5]<=8'h39 && line[7]>=8'h30 && line[7]<=8'h39 &&
        line[8]>=8'h30 && line[8]<=8'h39;
    wire [31:0] centidegrees=(line[3]-8'h30)*32'd10000+(line[4]-8'h30)*32'd1000+
        (line[5]-8'h30)*32'd100+(line[7]-8'h30)*32'd10+(line[8]-8'h30);
    wire [31:0] rounded_raw=(centidegrees*32'd32768+32'd9000)/32'd18000;
    wire zero_format=length==3 && angle_axis && line[1]==8'h3d && line[2]==8'h30;
    wire off_format=length==3 && line[0]==8'h4f && line[1]==8'h46 && line[2]==8'h46;
    wire on_format=length==2 && line[0]==8'h4f && line[1]==8'h4e;
    reg key_meta,key_sync,key_stable,key_press;
    reg [31:0] key_count;
    always @(posedge clk) begin
        if(!rst_n) begin
            key_meta<=1;key_sync<=1;key_stable<=1;key_press<=0;key_count<=0;
        end else begin
            key_meta<=key1_n;key_sync<=key_meta;key_press<=0;
            if(key_sync==key_stable)key_count<=0;
            else if(key_count<CLK_HZ/100-1)key_count<=key_count+1'b1;
            else begin key_count<=0;key_stable<=key_sync;if(!key_sync)key_press<=1;end
        end
    end
    wire signed [15:0] relative_angle=current_angle-base_angle;
    wire signed [15:0] relative_pitch=current_pitch-base_pitch;
    wire signed [15:0] relative_yaw=current_yaw-base_yaw;
    wire signed [15:0] requested_delta=requested_angle-base_angle;
    wire signed [15:0] requested_pitch_delta=requested_angle-base_pitch;
    wire signed [15:0] requested_yaw_delta=requested_angle-base_yaw;
    integer i;
    always @(posedge clk) begin
        if(!rst_n) begin
            length<=0;discard_line<=0;idle_count<=0;
            r_request<=0;p_request<=0;y_request<=0;off_request<=0;on_request<=0;requested_angle<=0;
            for(i=0;i<9;i=i+1)line[i]<=0;
        end else begin
            r_request<=0;p_request<=0;y_request<=0;off_request<=0;on_request<=0;
            if(byte_valid) begin
                idle_count<=0;
                if(byte_data==8'h0d || byte_data==8'h0a) begin
                    length<=0;discard_line<=0;
                    if(!discard_line && length!=0) begin
                        if(off_format)off_request<=1;
                        else if(on_format)on_request<=1;
                        else if(zero_format || (decimal_format && centidegrees<=3000)) begin
                            if(zero_format)requested_angle<=0;
                            else if(line[2]==8'h2d)requested_angle<=-$signed(rounded_raw[15:0]);
                            else requested_angle<=rounded_raw[15:0];
                            if(line[0]==8'h52)r_request<=1;
                            else if(line[0]==8'h50)p_request<=1;else y_request<=1;
                        end
                    end
                end else if(!discard_line) begin
                    if(length<9) begin line[length]<=byte_data;length<=length+1'b1;end
                    else discard_line<=1;
                end
            end else if(length!=0 || discard_line) begin
                if(idle_count<CLK_HZ-1)idle_count<=idle_count+1'b1;
                else begin length<=0;discard_line<=0;idle_count<=0;end
            end else idle_count<=0;
        end
    end
    // Desired state is separate from the in-flight CORDIC/helper snapshot. A new
    // key/command invalidates old work before it can publish stale pixels.
    always @(posedge clk) begin
        if(!rst_n) begin
            enabled<=0;command_valid<=0;received<=0;calibrated<=0;
            angle_raw<=0;delta_raw<=0;current_angle<=0;base_angle<=0;
            pitch_raw<=0;delta_pitch_raw<=0;current_pitch<=0;base_pitch<=0;
            pitch_cos_q16<=65536;pitch_sin_q16<=0;
            yaw_raw<=0;delta_yaw_raw<=0;current_yaw<=0;base_yaw<=0;
            yaw_cos_q16<=65536;yaw_sin_q16<=0;
            a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
            pending_angle<=0;work_absolute<=0;math_start<=0;inflight<=0;
            pending_pitch<=0;work_pitch<=0;roll_complete<=0;pitch_complete<=0;
            pending_yaw<=0;work_yaw<=0;yaw_complete<=0;
            saved_cos<=65536;saved_sin<=0;saved_pitch_cos<=65536;saved_pitch_sin<=0;
            saved_yaw_cos<=65536;saved_yaw_sin<=0;
            coeff_start<=0;coeff_started<=0;
            dirty<=0;active_requested<=0;generation<=0;work_generation<=0;
        end else begin
            command_valid<=0;math_start<=0;coeff_start<=0;
            if(inflight)begin
                if(math_done)begin saved_cos<=cosine;saved_sin<=sine;roll_complete<=1;end
                if(pitch_done)begin saved_pitch_cos<=pitch_cosine;saved_pitch_sin<=pitch_sine;pitch_complete<=1;end
                if(yaw_done)begin saved_yaw_cos<=yaw_cosine;saved_yaw_sin<=yaw_sine;yaw_complete<=1;end
                if(roll_complete && pitch_complete && yaw_complete && !coeff_started && !coeff_busy)begin
                    coeff_start<=1;coeff_started<=1;
                end
                if(coeff_done && coeff_started)inflight<=0;
            end
            if(key_press && received) begin
                base_angle<=current_angle;calibrated<=1;active_requested<=1;
                base_pitch<=current_pitch;
                base_yaw<=current_yaw;
                enabled<=1;angle_raw<=current_angle;delta_raw<=0;
                pitch_raw<=current_pitch;delta_pitch_raw<=0;
                pitch_cos_q16<=65536;pitch_sin_q16<=0;
                yaw_raw<=current_yaw;delta_yaw_raw<=0;yaw_cos_q16<=65536;yaw_sin_q16<=0;
                a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                generation<=generation+1'b1;dirty<=0;command_valid<=1;
            end else if(r_request || p_request || y_request) begin
                if(r_request)current_angle<=requested_angle;
                else if(p_request)current_pitch<=requested_angle;else current_yaw<=requested_angle;
                received<=1;generation<=generation+1'b1;
                if(calibrated && active_requested)dirty<=1;
                else begin
                    angle_raw<=r_request ? requested_angle : current_angle;
                    pitch_raw<=p_request ? requested_angle : current_pitch;
                    yaw_raw<=y_request ? requested_angle : current_yaw;
                    delta_raw<=calibrated ? (r_request ? requested_delta : relative_angle) : 16'sd0;
                    delta_pitch_raw<=calibrated ? (p_request ? requested_pitch_delta : relative_pitch) : 16'sd0;
                    delta_yaw_raw<=calibrated ? (y_request ? requested_yaw_delta : relative_yaw) : 16'sd0;
                    enabled<=0;a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                    pitch_cos_q16<=65536;pitch_sin_q16<=0;
                    yaw_cos_q16<=65536;yaw_sin_q16<=0;
                    dirty<=0;command_valid<=1;
                end
            end else if(off_request) begin
                active_requested<=0;enabled<=0;dirty<=0;generation<=generation+1'b1;
                angle_raw<=current_angle;delta_raw<=calibrated ? relative_angle : 16'sd0;
                pitch_raw<=current_pitch;delta_pitch_raw<=calibrated ? relative_pitch : 16'sd0;
                pitch_cos_q16<=65536;pitch_sin_q16<=0;
                yaw_raw<=current_yaw;delta_yaw_raw<=calibrated ? relative_yaw : 16'sd0;
                yaw_cos_q16<=65536;yaw_sin_q16<=0;
                a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;command_valid<=1;
            end else if(on_request) begin
                if(calibrated) begin active_requested<=1;dirty<=1;generation<=generation+1'b1;end
            end else if(inflight && coeff_started && coeff_done) begin
                // Invalid helper input retires without publishing or retrying.
                // A later legal command can queue a fresh generation.
                if(!coeff_error && work_generation==generation && !dirty && calibrated && active_requested) begin
                    enabled<=1;angle_raw<=work_absolute;delta_raw<=pending_angle;
                    pitch_raw<=work_pitch;delta_pitch_raw<=pending_pitch;
                    pitch_cos_q16<=saved_pitch_cos;pitch_sin_q16<=saved_pitch_sin;
                    yaw_raw<=work_yaw;delta_yaw_raw<=pending_yaw;
                    yaw_cos_q16<=saved_yaw_cos;yaw_sin_q16<=saved_yaw_sin;
                    a00<=coeff_a00;a01<=coeff_a01;a02<=coeff_a02;
                    a10<=coeff_a10;a11<=coeff_a11;a12<=coeff_a12;
                    command_valid<=1;
                end
            end else if(dirty && !inflight && !math_busy && !pitch_busy && !yaw_busy && !math_start && !coeff_busy && !coeff_start) begin
                pending_angle<=relative_angle;work_absolute<=current_angle;
                pending_pitch<=relative_pitch;work_pitch<=current_pitch;
                pending_yaw<=relative_yaw;work_yaw<=current_yaw;
                roll_complete<=0;pitch_complete<=0;yaw_complete<=0;
                coeff_started<=0;
                work_generation<=generation;math_start<=1;inflight<=1;dirty<=0;
            end
        end
    end
endmodule
`default_nettype wire
