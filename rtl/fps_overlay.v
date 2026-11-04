`timescale 1ns / 1ps
`default_nettype none

// Frame-stable FPS and latency telemetry, using the existing video alignment.
module fps_overlay #(
    parameter integer BOX_X = 32,
    parameter integer BOX_Y = 32,
    // 0 hides the diagnostic block so the release build draws nothing extra.
    parameter integer DIAG_EN = 1
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de,
    input  wire       vs,
    input  wire [11:0] x,
    input  wire [11:0] y,
    input  wire [23:0] rgb_in,
    input  wire [3:0] fps_tens_async,
    input  wire [3:0] fps_ones_async,
    input wire [3:0] fps_frac_tens_async, fps_frac_ones_async,
    input  wire       fps_update_async,
    input  wire [19:0] latency_bcd,
    input  wire       latency_valid,
    input  wire       latency_overflow,
    input  wire [15:0] diag_process_10us,
    input  wire [15:0] diag_result_period_10us,
    input  wire [15:0] diag_publish_wait_10us,
    input  wire [3:0]  diag_stage,
    output reg  [23:0] rgb_out
);
    reg [15:0] digits_meta, digits_sync, pending_digits;
    reg [3:0] fps_tens, fps_ones, fps_frac_tens, fps_frac_ones;
    reg [2:0] update_sync;
    reg update_seen, vs_d;
    reg [19:0] latency_digits;
    reg [1:0] latency_leading_zeros;
    reg latency_show_valid, latency_show_overflow;
    reg [15:0] diag_p_meta,diag_p_sync,diag_r_meta,diag_r_sync,diag_w_meta,diag_w_sync;
    reg [3:0] diag_s_meta,diag_s_sync;
    reg [15:0] diag_p_show,diag_r_show,diag_w_show;
    reg [3:0] diag_s_show;
    reg [11:0] x_d1, x_d2, y_d1, y_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            digits_meta <= 0; digits_sync <= 0; pending_digits <= 0;
            update_sync <= 0; update_seen <= 0; vs_d <= 0;
            x_d1 <= 0; x_d2 <= 0; y_d1 <= 0; y_d2 <= 0;
            fps_tens <= 0; fps_ones <= 0; fps_frac_tens<=0; fps_frac_ones<=0;
            latency_digits <= 0;
            latency_leading_zeros <= 0;
            latency_show_valid <= 0; latency_show_overflow <= 0;
            diag_p_meta<=0;diag_p_sync<=0;diag_r_meta<=0;diag_r_sync<=0;
            diag_w_meta<=0;diag_w_sync<=0;diag_s_meta<=0;diag_s_sync<=0;
            diag_p_show<=0;diag_r_show<=0;diag_w_show<=0;diag_s_show<=0;
        end else begin
            digits_meta <= {fps_tens_async,fps_ones_async,fps_frac_tens_async,fps_frac_ones_async};
            digits_sync <= digits_meta;
            update_sync <= {update_sync[1:0],fps_update_async};
            vs_d <= vs;
            diag_p_meta<=diag_process_10us; diag_p_sync<=diag_p_meta;
            diag_r_meta<=diag_result_period_10us; diag_r_sync<=diag_r_meta;
            diag_w_meta<=diag_publish_wait_10us; diag_w_sync<=diag_w_meta;
            diag_s_meta<=diag_stage; diag_s_sync<=diag_s_meta;
            // The toggle travels one extra stage; digits are stable before capture.
            if (update_sync[2] != update_seen) begin
                pending_digits <= digits_sync;
                update_seen <= update_sync[2];
            end
            if (vs && !vs_d) begin
                fps_tens <= pending_digits[15:12];
                fps_ones <= pending_digits[11:8];
                fps_frac_tens <= pending_digits[7:4];
                fps_frac_ones <= pending_digits[3:0];
                latency_digits <= latency_bcd;
                // Keep at least the units digit and move the remaining text left.
                latency_leading_zeros <= (latency_valid && latency_bcd[19:16]==0) ?
                                        ((latency_bcd[15:12]==0) ? 2'd2 : 2'd1) : 2'd0;
                latency_show_valid <= latency_valid;
                latency_show_overflow <= latency_overflow;
                diag_p_show<=diag_p_sync; diag_r_show<=diag_r_sync;
                diag_w_show<=diag_w_sync; diag_s_show<=diag_s_sync;
            end
            // rd_buf delays DE and RGB by two pixel clocks.
            x_d1 <= x; x_d2 <= x_d1; y_d1 <= y; y_d2 <= y_d1;
        end
    end

    function [4:0] glyph_row;
        input [4:0] code;
        input [2:0] row;
        begin
            glyph_row = 5'b00000;
            case (code)
                0: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0; endcase
                1: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0; endcase
                2: case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0; endcase
                3: case(row) 2:glyph_row=5'b00100;4:glyph_row=5'b00100;default:glyph_row=0; endcase
                4: glyph_row=0;
                15: case(row) 6:glyph_row=5'b00100;default:glyph_row=0;endcase
                16: case(row) 0,1,2,3,4,5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                17: case(row) 0:glyph_row=5'b01110;1,2,4,5,6:glyph_row=5'b10001;3:glyph_row=5'b11111;default:glyph_row=0;endcase
                18: case(row) 0:glyph_row=5'b11111;1,2,3,4,5,6:glyph_row=5'b00100;default:glyph_row=0;endcase
                19: case(row) 2:glyph_row=5'b11010;3:glyph_row=5'b10101;4,5,6:glyph_row=5'b10101;default:glyph_row=0;endcase
                20: case(row) 2:glyph_row=5'b01111;3:glyph_row=5'b10000;4:glyph_row=5'b01110;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                21: case(row) 3:glyph_row=5'b11111;default:glyph_row=0;endcase
                22: case(row) 1,2,4,5:glyph_row=5'b00100;3:glyph_row=5'b11111;default:glyph_row=0;endcase
                23: case(row) 0:glyph_row=5'b01110;1,2:glyph_row=5'b10001;3:glyph_row=5'b11111;4,5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                24: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4,5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                25: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                26: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                27: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                28: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                5: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0; endcase
                6: case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0; endcase
                7: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;6:glyph_row=5'b11111;default:glyph_row=0; endcase
                8: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0; endcase
                9: case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;6:glyph_row=5'b00010;default:glyph_row=0; endcase
                10: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0; endcase
                11: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0; endcase
                12: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b01000;default:glyph_row=0; endcase
                13: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0; endcase
                default: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b01110;default:glyph_row=0; endcase
            endcase
        end
    endfunction

    reg [7:0] xr,yr;
    reg [3:0] char_index;
    reg [4:0] latency_char_index;
    reg [2:0] glyph_x,glyph_y;
    reg [4:0] char_code;
    reg [4:0] row_bits;
    reg box_on,glyph_on,latency_box_on,diag_box_on;
    reg [1:0] diag_line;
    reg [15:0] diag_value;
    function [4:0] hex_code;
        input [3:0] n;
        begin hex_code = (n<10) ? (n+5) : (n-10+23); end
    endfunction
    always @* begin
        rgb_out = rgb_in;
        xr = x_d2 - BOX_X;
        yr = y_d2 - BOX_Y;
        box_on = de && x_d2 >= BOX_X && x_d2 < BOX_X+160 && y_d2 >= BOX_Y && y_d2 < BOX_Y+16;
        latency_box_on = de && x_d2 >= BOX_X && x_d2 < BOX_X+224 && y_d2 >= BOX_Y+24 && y_d2 < BOX_Y+40;
        // The diagnostic block moved to the UART report: at 8x8 glyphs the
        // numbers changed faster than they could be read and the fields
        // overlapped. The ports stay for source compatibility; nothing is drawn.
        diag_box_on = 1'b0;
        diag_line = 0; diag_value = diag_p_show;
        if (y_d2 >= BOX_Y+72 && y_d2 < BOX_Y+88) begin diag_line=1; diag_value=diag_r_show; end
        else if (y_d2 >= BOX_Y+96 && y_d2 < BOX_Y+112) begin diag_line=2; diag_value=diag_w_show; end
        else if (y_d2 >= BOX_Y+120 && y_d2 < BOX_Y+136) begin diag_line=3; diag_value={12'd0,diag_s_show}; end
        if (diag_box_on) yr = y_d2 - (BOX_Y+48 + diag_line*24);
        if (latency_box_on) yr = y_d2 - (BOX_Y+24);
        char_index = xr[7:4];
        latency_char_index = {1'b0,char_index};
        if (char_index >= 5)
            latency_char_index = {1'b0,char_index} + {3'd0,latency_leading_zeros};
        glyph_x = xr[3:1];
        glyph_y = yr[3:1];
        if (diag_box_on) begin
            case (char_index)
                0: char_code = (diag_line==0)?1:(diag_line==1)?19:(diag_line==2)?16:2;
                1: char_code = 3;
                3: char_code = hex_code(diag_value[15:12]);
                4: char_code = hex_code(diag_value[11:8]);
                5: char_code = hex_code(diag_value[7:4]);
                6: char_code = hex_code(diag_value[3:0]);
                default: char_code=4;
            endcase
        end else if (latency_box_on) begin
            case (latency_char_index)
                0:char_code=16; 1:char_code=17; 2:char_code=18; 3:char_code=3;
                5:char_code=latency_show_valid ? {1'b0,latency_digits[19:16]}+5'd5 : 5'd21;
                6:char_code=latency_show_valid ? {1'b0,latency_digits[15:12]}+5'd5 : 5'd21;
                7:char_code=latency_show_valid ? {1'b0,latency_digits[11:8]}+5'd5 : 5'd21;
                8:char_code=15;
                9:char_code=latency_show_valid ? {1'b0,latency_digits[7:4]}+5'd5 : 5'd21;
                10:char_code=latency_show_valid ? {1'b0,latency_digits[3:0]}+5'd5 : 5'd21;
                11:char_code=(latency_show_valid && latency_show_overflow) ? 5'd22 : 5'd4;
                12:char_code=19; 13:char_code=20;
                default:char_code=4;
            endcase
        end else begin
            case(char_index)
                0:char_code=0; 1:char_code=1; 2:char_code=2; 3:char_code=3;
                5:char_code={1'b0,fps_tens}+5'd5; 6:char_code={1'b0,fps_ones}+5'd5; 7:char_code=15;
                8:char_code={1'b0,fps_frac_tens}+5'd5; 9:char_code={1'b0,fps_frac_ones}+5'd5; default:char_code=4;
            endcase
        end
        row_bits = glyph_row(char_code,glyph_y);
        glyph_on = (box_on || latency_box_on || diag_box_on) && glyph_x < 5 && row_bits[4-glyph_x];
        if (box_on || latency_box_on || diag_box_on) rgb_out = glyph_on ? 24'hFFFFFF : 24'h102040;
    end
endmodule
`default_nettype wire
