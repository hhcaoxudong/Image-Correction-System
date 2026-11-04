`default_nettype none

// Euler Rs=Rz(yaw)*Ry(pitch)*Rx(roll). Relative inverse camera map is
// C*Rcurr^T*Rbase*C^T, C=[0,-1,0;0,0,-1;1,0,0]. Q16 centre Jacobian.
// The FSM shares one 18x18 rotation multiplier, one 32x18 focal multiplier,
// and one restoring divider across the matrix and coefficient calculations.
// Illegal poses report done/error and preserve the previous affine outputs.
module imu_matrix_affine(
    input wire clk,rst_n,start,
    input wire signed [31:0] fx_q16,fy_q16,tilt_limit_q16,
    input wire signed [15:0] current_roll,current_pitch,current_yaw,
    input wire signed [15:0] base_roll,base_pitch,base_yaw,
    output reg busy,done,error,legal,
    output reg signed [31:0] rel_r22,rel_trace,
    output reg signed [31:0] a00,a01,a02,a10,a11,a12
);
    localparam [5:0] IDLE=0,TRIG_ISSUE=1,TRIG_WAIT=2,MAT_LOAD=3,
        MAT_SET=4,MAT_MUL=5,MAT_ACC=6,ROTATE_BASIS=7,CHECK_POSE=8,SQUARE_SET=9,
        SQUARE_SAVE=10,NUM0_SET=11,NUM0_MUL=12,NUM0_SAVE=13,NUM1_SET=14,
        NUM1_MUL=15,NUM1_SAVE=16,DIV_INIT=17,DIV_RUN=18,ASPECT_SET=19,
        ASPECT_SAVE=20,RESULT=21,COMMIT=22,REJECT=23,IDENTITY=24;
    reg [5:0] state,div_return;
    reg signed [15:0] angles[0:5];
    reg signed [31:0] ct[0:5],sn[0:5];
    reg [2:0] trig_index,mat_pass,coeff_index;
    reg trig_start;
    reg signed [15:0] trig_angle;
    wire trig_busy,trig_done;
    wire signed [31:0] trig_cos,trig_sin;
    rotation_sincos shared_trig(.clk(clk),.rst_n(rst_n),.start(trig_start),
        .angle_raw(trig_angle),.busy(trig_busy),.done(trig_done),
        .cos_q16(trig_cos),.sin_q16(trig_sin));

    // Rotation matrices are Q16.16 and stay inside [-1, 1]; 18 signed
    // bits retain Q16.16 precision while reducing the core-clock load.
    reg signed [17:0] ma[0:8],mb[0:8],temp[0:8],rc[0:8],rb[0:8],rs[0:8],r[0:8];
    reg [1:0] row,col,inner;
    reg signed [63:0] acc,mul_result_q;
    reg signed [31:0] mul_a,mul_b;
    wire large_mul_en=(state==ASPECT_SAVE)||
        ((state==NUM0_MUL || state==NUM0_SAVE)&&coeff_index>=4);
    wire signed [17:0] small_a=$signed(mul_a[17:0]);
    wire signed [17:0] small_b=$signed(mul_b[17:0]);
    wire signed [35:0] small_product=small_a*small_b;
    wire focal_in_a=((state==NUM0_MUL || state==NUM0_SAVE) && coeff_index>=4);
    wire signed [31:0] big_operand=large_mul_en?
        (focal_in_a?$signed(mul_a):$signed(mul_b)):$signed(mul_a);
    wire signed [17:0] narrow_operand=large_mul_en?
        (focal_in_a?small_b:small_a):small_b;
    wire signed [49:0] large_product=narrow_operand*big_operand;
    wire signed [63:0] mul_result=large_mul_en?{{14{large_product[49]}},large_product}:{{28{small_product[35]}},small_product};
    reg signed [63:0] den_square,cross_first,div_num;
    reg [63:0] div_den,divisor,div_shift,div_quot;
    reg [64:0] div_rem;
    reg [5:0] div_count;
    reg div_sign;
    reg signed [31:0] div_result;
    reg signed [31:0] work[0:5];
    wire [64:0] shifted_rem={div_rem[63:0],div_shift[63]};
    wire div_take=(shifted_rem>={1'b0,divisor});
    wire [63:0] quotient_next={div_quot[62:0],div_take};
    wire quotient_overflow=(|quotient_next[63:32]) ||
        (!div_sign && quotient_next[31]) ||
        (div_sign && quotient_next>64'h0000_0000_8000_0000);
    integer z,angle_offset;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;trig_start<=0;busy<=0;done<=0;error<=0;legal<=0;
            a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
            rel_r22<=65536;rel_trace<=196608;
        end else begin
            trig_start<=0;done<=0;
            case(state)
                IDLE: if(start) begin
                    busy<=1;
                    angles[0]<=current_roll;angles[1]<=current_pitch;angles[2]<=current_yaw;
                    angles[3]<=base_roll;angles[4]<=base_pitch;angles[5]<=base_yaw;
                    trig_index<=0;
                    if(current_roll==base_roll && current_pitch==base_pitch && current_yaw==base_yaw)
                        state<=IDENTITY;
                    else state<=TRIG_ISSUE;
                end
                TRIG_ISSUE: begin
                    trig_angle<=angles[trig_index];trig_start<=1;state<=TRIG_WAIT;
                end
                TRIG_WAIT: if(trig_done) begin
                    ct[trig_index]<=trig_cos;sn[trig_index]<=trig_sin;
                    if(trig_index==5) begin mat_pass<=0;state<=MAT_LOAD;end
                    else begin trig_index<=trig_index+1'b1;state<=TRIG_ISSUE;end
                end
                MAT_LOAD: begin
                    for(z=0;z<9;z=z+1) begin ma[z]<=0;mb[z]<=0;end
                    case(mat_pass)
                        0,2: begin
                            angle_offset=(mat_pass==2) ? 3 : 0;
                            // Ry * Rx.
                            ma[0]<=ct[angle_offset+1];ma[2]<=sn[angle_offset+1];
                            ma[4]<=65536;ma[6]<=-sn[angle_offset+1];ma[8]<=ct[angle_offset+1];
                            mb[0]<=65536;mb[4]<=ct[angle_offset];mb[5]<=-sn[angle_offset];
                            mb[7]<=sn[angle_offset];mb[8]<=ct[angle_offset];
                        end
                        1,3: begin
                            angle_offset=(mat_pass==3) ? 5 : 2;
                            // Rz * (Ry * Rx).
                            ma[0]<=ct[angle_offset];ma[1]<=-sn[angle_offset];
                            ma[3]<=sn[angle_offset];ma[4]<=ct[angle_offset];ma[8]<=65536;
                            for(z=0;z<9;z=z+1) mb[z]<=temp[z];
                        end
                        4: begin
                            ma[0]<=rc[0];ma[1]<=rc[3];ma[2]<=rc[6];
                            ma[3]<=rc[1];ma[4]<=rc[4];ma[5]<=rc[7];
                            ma[6]<=rc[2];ma[7]<=rc[5];ma[8]<=rc[8];
                            for(z=0;z<9;z=z+1) mb[z]<=rb[z];
                        end
                        default: begin end
                    endcase
                    row<=0;col<=0;inner<=0;acc<=0;state<=MAT_SET;
                end
                MAT_SET: begin
                    mul_a<=ma[row*3+inner];mul_b<=mb[inner*3+col];state<=MAT_MUL;
                end
                MAT_MUL: begin
                    // Register the shared APM product before the accumulator
                    // adder. This removes the APM-to-wide-adder path from the
                    // 100 MHz core clock critical path.
                    mul_result_q<=mul_result;state<=MAT_ACC;
                end
                MAT_ACC: begin
                    if(inner==2) begin
                        case(mat_pass)
                            0,2: temp[row*3+col]<=(acc+mul_result_q)>>>16;
                            1: rc[row*3+col]<=(acc+mul_result_q)>>>16;
                            3: rb[row*3+col]<=(acc+mul_result_q)>>>16;
                            4: rs[row*3+col]<=(acc+mul_result_q)>>>16;
                            default: begin end
                        endcase
                        inner<=0;acc<=0;
                        if(col==2) begin
                            col<=0;
                            if(row==2) begin
                                if(mat_pass==4) state<=ROTATE_BASIS;
                                else begin mat_pass<=mat_pass+1'b1;state<=MAT_LOAD;end
                            end else begin row<=row+1'b1;state<=MAT_SET;end
                        end else begin col<=col+1'b1;state<=MAT_SET;end
                    end else begin acc<=acc+mul_result_q;inner<=inner+1'b1;state<=MAT_SET;end
                end
                ROTATE_BASIS: begin
                    r[0]<=rs[4];r[1]<=rs[5];r[2]<=-rs[3];
                    r[3]<=rs[7];r[4]<=rs[8];r[5]<=-rs[6];
                    r[6]<=-rs[1];r[7]<=-rs[2];r[8]<=rs[0];state<=CHECK_POSE;
                end
                CHECK_POSE: begin
                    rel_r22<=r[8];rel_trace<=r[0]+r[4]+r[8];
                    // Do not limit in-plane rotation with the matrix trace:
                    // that incorrectly capped Roll near +/-30 degrees. Bound
                    // optical-axis tilt per radial model to its verified
                    // fixed-point error envelope.
                    if(r[8]<tilt_limit_q16) state<=REJECT;
                    else state<=SQUARE_SET;
                end
                SQUARE_SET: begin mul_a<=r[8];mul_b<=r[8];state<=SQUARE_SAVE;end
                SQUARE_SAVE: begin den_square<=mul_result;coeff_index<=0;state<=NUM0_SET;end
                NUM0_SET: begin
                    case(coeff_index)
                        0: begin mul_a<=r[0];mul_b<=r[8];end
                        1: begin mul_a<=r[1];mul_b<=r[8];end
                        2: begin mul_a<=r[3];mul_b<=r[8];end
                        3: begin mul_a<=r[4];mul_b<=r[8];end
                        4: begin mul_a<=fx_q16;mul_b<=r[2];end
                        5: begin mul_a<=fy_q16;mul_b<=r[5];end
                        default: begin mul_a<=0;mul_b<=0;end
                    endcase
                    state<=NUM0_MUL;
                end
                NUM0_MUL: begin
                    mul_result_q<=mul_result;state<=NUM0_SAVE;
                end
                NUM0_SAVE: begin
                    cross_first<=mul_result_q;
                    if(coeff_index>=4) begin
                        div_num<=mul_result_q;div_den<={32'd0,r[8]};
                        div_return<=RESULT;state<=DIV_INIT;
                    end else state<=NUM1_SET;
                end
                NUM1_SET: begin
                    mul_a<=(coeff_index<2) ? r[2] : r[5];
                    mul_b<=(coeff_index==0 || coeff_index==2) ? r[6] : r[7];
                    state<=NUM1_MUL;
                end
                NUM1_MUL: begin
                    mul_result_q<=mul_result;state<=NUM1_SAVE;
                end
                NUM1_SAVE: begin
                    div_num<=(cross_first-mul_result_q)<<<16;div_den<=den_square;
                    div_return<=(coeff_index==1 || coeff_index==2) ? ASPECT_SET : RESULT;
                    state<=DIV_INIT;
                end
                DIV_INIT: begin
                    div_shift<=div_num[63] ? -div_num : div_num;
                    div_sign<=div_num[63];div_rem<=0;div_quot<=0;
                    divisor<=div_den;div_count<=0;
                    if(div_den==0) state<=REJECT;else state<=DIV_RUN;
                end
                DIV_RUN: begin
                    div_rem<=div_take ? shifted_rem-{1'b0,divisor} : shifted_rem;
                    div_shift<={div_shift[62:0],1'b0};div_quot<=quotient_next;
                    if(div_count==63) begin
                        if(quotient_overflow) state<=REJECT;
                        else begin
                            div_result<=div_sign ? -$signed(quotient_next[31:0]) : $signed(quotient_next[31:0]);
                            state<=div_return;
                        end
                    end else div_count<=div_count+1'b1;
                end
                ASPECT_SET: begin
                    mul_a<=div_result;mul_b<=(coeff_index==1) ? fx_q16 : fy_q16;state<=ASPECT_SAVE;
                end
                ASPECT_SAVE: begin
                    div_num<=mul_result;div_den<=(coeff_index==1) ? {32'd0,fy_q16} : {32'd0,fx_q16};
                    div_return<=RESULT;state<=DIV_INIT;
                end
                RESULT: begin
                    work[coeff_index]<=div_result;
                    if(coeff_index==5) state<=COMMIT;
                    else begin coeff_index<=coeff_index+1'b1;state<=NUM0_SET;end
                end
                COMMIT: begin
                    if ((work[0][31:18]=={14{work[0][17]}}) &&
                        (work[1][31:18]=={14{work[1][17]}}) &&
                        (work[2][31:18]=={14{work[2][17]}}) &&
                        (work[3][31:18]=={14{work[3][17]}})) begin
                        a00<=work[0];a01<=work[1];a02<=work[4];
                        a10<=work[2];a11<=work[3];a12<=work[5];
                        busy<=0;done<=1;error<=0;legal<=1;state<=IDLE;
                    end else state<=REJECT;
                end
                REJECT: begin busy<=0;done<=1;error<=1;legal<=0;state<=IDLE;end
                IDENTITY: begin
                    a00<=65536;a01<=0;a02<=0;a10<=0;a11<=65536;a12<=0;
                    rel_r22<=65536;rel_trace<=196608;
                    busy<=0;done<=1;error<=0;legal<=1;state<=IDLE;
                end
                default: begin busy<=0;error<=1;legal<=0;state<=IDLE;end
            endcase
        end
    end
endmodule
`default_nettype wire
