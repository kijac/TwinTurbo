`timescale 1ns / 1ps
// ============================================================================
// rht_l2_normalize_dmax.v
//
// 원본 대비 변경점
//   D_MAX parameter가 실제 dimension을 결정한다.
//      D_MAX=64/128/256 -> BATCHES=2/4/8. dim_mode 포트 없음.
//   GAMMA_W = 20 은 D=256 에서도 충분:
//        sum <= 256 * (2^15)^2 = 2^38,  gamma <= 2^19 < 2^20 - 1
//   din_reg 제거.
//      din 을 직접 참조한다. 대신 호출자는 start 부터 done 까지 din 을
//      안정적으로 유지해야 한다 (top 에서 io_buffer bank 가 유지해 줌).
//   4) 모듈/서브모듈 이름을 바꿔 기존 파일과 동시 컴파일해도 충돌 없음
//
// Q-format
//   din   : signed Q?.FRAC_W (16b)
//   gamma : unsigned Q?.FRAC_W (20b)      <- packer / dequant 로
//   u_out : signed Q?.FRAC_W (16b), |u| <= 1.0  -> |u_int| <= 2^FRAC_W
//   비활성 상위 원소(i >= D)의 u_out 은 0
// ============================================================================

// ----------------------------------------------------------------------------
// Unsigned serial restoring divider  (원본과 동일, 이름만 변경)
// ----------------------------------------------------------------------------
module RHT_Unsigned_Divider_Serial_DMAX #(
    parameter integer DIVIDEND_W = 33,
    parameter integer DIVISOR_W  = 20
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire [DIVIDEND_W-1:0]    dividend,
    input  wire [DIVISOR_W-1:0]     divisor,
    output reg  [DIVIDEND_W-1:0]    quotient,
    output reg  [DIVISOR_W:0]       remainder,
    output reg                      valid_out,
    output reg                      done,
    output wire                     busy
);
    localparam S_IDLE = 1'b0;
    localparam S_RUN  = 1'b1;

    reg state;
    reg [DIVIDEND_W-1:0] dividend_reg;
    reg [DIVISOR_W-1:0]  divisor_reg;
    reg [7:0]            bit_idx;

    reg [DIVISOR_W:0] divisor_ext;
    reg [DIVISOR_W:0] rem_shift;
    reg [DIVISOR_W:0] rem_next;
    reg               q_bit_next;

    assign busy = (state != S_IDLE);

    always @(*) begin
        divisor_ext = {1'b0, divisor_reg};
        rem_shift   = {remainder[DIVISOR_W-1:0], dividend_reg[bit_idx]};
        if ((divisor_ext != {(DIVISOR_W+1){1'b0}}) && (rem_shift >= divisor_ext)) begin
            rem_next   = rem_shift - divisor_ext;
            q_bit_next = 1'b1;
        end else begin
            rem_next   = rem_shift;
            q_bit_next = 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; dividend_reg <= 0; divisor_reg <= 0;
            quotient <= 0;   remainder <= 0;    bit_idx <= 0;
            valid_out <= 1'b0; done <= 1'b0;
        end else begin
            valid_out <= 1'b0;
            done      <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    dividend_reg <= dividend;
                    divisor_reg  <= divisor;
                    quotient     <= {DIVIDEND_W{1'b0}};
                    remainder    <= {(DIVISOR_W+1){1'b0}};
                    bit_idx      <= DIVIDEND_W - 1;
                    if (divisor == {DIVISOR_W{1'b0}}) begin
                        quotient  <= {DIVIDEND_W{1'b1}};
                        valid_out <= 1'b1;
                        done      <= 1'b1;
                        state     <= S_IDLE;
                    end else begin
                        state <= S_RUN;
                    end
                end
                S_RUN: begin
                    remainder         <= rem_next;
                    quotient[bit_idx] <= q_bit_next;
                    if (bit_idx == 8'd0) begin
                        valid_out <= 1'b1;
                        done      <= 1'b1;
                        state     <= S_IDLE;
                    end else begin
                        bit_idx <= bit_idx - 1'b1;
                    end
                end
            endcase
        end
    end
endmodule


// ----------------------------------------------------------------------------
// D_MAX-parameter shared-32-multiplier L2 Normalize Frontend
// ----------------------------------------------------------------------------
module RHT_L2_Normalize_Frontend_DMAX #(
    parameter integer INPUT_W    = 16,
    parameter integer OUT_W      = 16,
    parameter integer D_MAX      = 256,
    parameter integer FRAC_W     = 12,
    parameter integer LANES      = 32,
    parameter integer INV_FRAC_W = 32,
    parameter integer GAMMA_W    = 20,
    parameter integer INV_W      = INV_FRAC_W + 1
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire [INPUT_W*D_MAX-1:0] din,          // start~done 동안 안정 유지 필요

    output wire [OUT_W*D_MAX-1:0]   u_out,
    output wire [GAMMA_W-1:0]       gamma,
    output wire [INV_W-1:0]         inv_gamma,
    output reg                      valid_out,
    output reg                      done,
    output wire                     busy
);
    localparam integer D_LOG_MAX = (D_MAX <= 64) ? 6 : ((D_MAX <= 128) ? 7 : 8);
    localparam [7:0] BATCHES_U = D_MAX / LANES;
    localparam integer SQUARE_W  = 2 * INPUT_W;             // 32
    localparam integer SUM_W     = SQUARE_W + D_LOG_MAX;    // 40
    localparam integer INV_SHIFT = INV_FRAC_W - FRAC_W;     // 20

    localparam integer GAMMA_SIGN_SAFE_W = GAMMA_W + 1;
    localparam integer MULT_A_W = (GAMMA_SIGN_SAFE_W > INPUT_W) ? GAMMA_SIGN_SAFE_W : INPUT_W;
    localparam integer MULT_B_BASE_W = MULT_A_W;
    localparam integer MULT_B_W = ((INV_W + 1) > MULT_B_BASE_W) ? (INV_W + 1) : MULT_B_BASE_W;
    localparam integer PROD_W   = MULT_A_W + MULT_B_W;
    localparam integer COMP_W   = (SUM_W > PROD_W) ? SUM_W : PROD_W;

    localparam S_IDLE       = 3'd0;
    localparam S_ACCUM      = 3'd1;
    localparam S_SQRT       = 3'd2;
    localparam S_RECIP_WAIT = 3'd3;
    localparam S_MUL        = 3'd4;

    reg [2:0] state;
    assign busy = (state != S_IDLE);

    reg [OUT_W*D_MAX-1:0] u_out_reg;
    reg [GAMMA_W-1:0]     gamma_reg;
    reg [INV_W-1:0]       inv_gamma_reg;

    assign u_out     = u_out_reg;
    assign gamma     = gamma_reg;
    assign inv_gamma = inv_gamma_reg;

    reg [7:0] batch_cnt;

    reg [SUM_W-1:0]   sum_acc;
    reg [SUM_W-1:0]   sqrt_value;
    reg [7:0]         sqrt_bit;
    reg [GAMMA_W-1:0] sqrt_root;
    reg [GAMMA_W-1:0] sqrt_trial;
    reg [GAMMA_W-1:0] sqrt_next_root;

    function signed [MULT_A_W-1:0] sext_input_to_a;
        input [INPUT_W-1:0] value;
        begin
            sext_input_to_a = {MULT_A_W{value[INPUT_W-1]}};
            sext_input_to_a[INPUT_W-1:0] = value;
        end
    endfunction

    function signed [MULT_B_W-1:0] sext_input_to_b;
        input [INPUT_W-1:0] value;
        begin
            sext_input_to_b = {MULT_B_W{value[INPUT_W-1]}};
            sext_input_to_b[INPUT_W-1:0] = value;
        end
    endfunction

    function signed [MULT_A_W-1:0] zext_gamma_to_a;
        input [GAMMA_W-1:0] value;
        begin
            zext_gamma_to_a = {MULT_A_W{1'b0}};
            zext_gamma_to_a[GAMMA_W-1:0] = value;
        end
    endfunction

    function signed [MULT_B_W-1:0] zext_gamma_to_b;
        input [GAMMA_W-1:0] value;
        begin
            zext_gamma_to_b = {MULT_B_W{1'b0}};
            zext_gamma_to_b[GAMMA_W-1:0] = value;
        end
    endfunction

    function signed [MULT_B_W-1:0] zext_inv_to_b;
        input [INV_W-1:0] value;
        begin
            zext_inv_to_b = {MULT_B_W{1'b0}};
            zext_inv_to_b[INV_W-1:0] = value;
        end
    endfunction

    genvar g;
    wire signed [MULT_A_W-1:0] lane_mult_a  [0:LANES-1];
    wire signed [MULT_B_W-1:0] lane_mult_b  [0:LANES-1];
    wire signed [PROD_W-1:0]   lane_product [0:LANES-1];

    generate
        for (g = 0; g < LANES; g = g + 1) begin : GEN_SHARED_MULT
            wire signed [INPUT_W-1:0] lane_x;
            assign lane_x = din[((batch_cnt*LANES + g)*INPUT_W) +: INPUT_W];

            if (g == 0) begin : GEN_LANE0
                assign lane_mult_a[g] =
                    (state == S_SQRT)  ? zext_gamma_to_a(sqrt_trial) :
                    (state == S_ACCUM) ? sext_input_to_a(lane_x)     :
                    (state == S_MUL)   ? sext_input_to_a(lane_x)     : {MULT_A_W{1'b0}};
                assign lane_mult_b[g] =
                    (state == S_SQRT)  ? zext_gamma_to_b(sqrt_trial)     :
                    (state == S_ACCUM) ? sext_input_to_b(lane_x)         :
                    (state == S_MUL)   ? zext_inv_to_b(inv_gamma_reg)    : {MULT_B_W{1'b0}};
            end else begin : GEN_LANEN
                assign lane_mult_a[g] =
                    (state == S_ACCUM) ? sext_input_to_a(lane_x) :
                    (state == S_MUL)   ? sext_input_to_a(lane_x) : {MULT_A_W{1'b0}};
                assign lane_mult_b[g] =
                    (state == S_ACCUM) ? sext_input_to_b(lane_x)      :
                    (state == S_MUL)   ? zext_inv_to_b(inv_gamma_reg) : {MULT_B_W{1'b0}};
            end
            assign lane_product[g] = lane_mult_a[g] * lane_mult_b[g];
        end
    endgenerate

    integer k;
    reg [SUM_W-1:0] batch_sum;
    always @(*) begin
        batch_sum = {SUM_W{1'b0}};
        for (k = 0; k < LANES; k = k + 1)
            batch_sum = batch_sum + {{(SUM_W-SQUARE_W){1'b0}}, lane_product[k][SQUARE_W-1:0]};
    end

    reg [COMP_W-1:0] sqrt_trial_sq_ext;
    reg [COMP_W-1:0] sqrt_value_ext;
    always @(*) begin
        sqrt_trial        = sqrt_root | ({{(GAMMA_W-1){1'b0}}, 1'b1} << sqrt_bit);
        sqrt_trial_sq_ext = {COMP_W{1'b0}};
        sqrt_trial_sq_ext[PROD_W-1:0] = lane_product[0];
        sqrt_value_ext    = {COMP_W{1'b0}};
        sqrt_value_ext[SUM_W-1:0] = sqrt_value;
        sqrt_next_root = (sqrt_trial_sq_ext <= sqrt_value_ext) ? sqrt_trial : sqrt_root;
    end

    reg                recip_start;
    reg  [GAMMA_W-1:0] recip_divisor;
    wire [INV_W-1:0]   recip_dividend = {1'b1, {INV_FRAC_W{1'b0}}};
    wire [INV_W-1:0]   recip_quotient;
    wire               recip_valid;

    RHT_Unsigned_Divider_Serial_DMAX #(
        .DIVIDEND_W(INV_W), .DIVISOR_W(GAMMA_W)
    ) u_recip_divider (
        .clk(clk), .rst_n(rst_n),
        .start(recip_start), .dividend(recip_dividend), .divisor(recip_divisor),
        .quotient(recip_quotient), .remainder(), .valid_out(recip_valid),
        .done(), .busy()
    );

    function [OUT_W-1:0] sat_to_outw;
        input signed [PROD_W-1:0] value;
        reg signed [PROD_W-1:0] max_value;
        reg signed [PROD_W-1:0] min_value;
        begin
            max_value = {{(PROD_W-OUT_W){1'b0}}, {1'b0, {(OUT_W-1){1'b1}}}};
            min_value = {{(PROD_W-OUT_W){1'b1}}, {1'b1, {(OUT_W-1){1'b0}}}};
            if      (value > max_value) sat_to_outw = {1'b0, {(OUT_W-1){1'b1}}};
            else if (value < min_value) sat_to_outw = {1'b1, {(OUT_W-1){1'b0}}};
            else                        sat_to_outw = value[OUT_W-1:0];
        end
    endfunction

    integer m;
    reg signed [PROD_W-1:0] norm_shifted;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            u_out_reg     <= {(OUT_W*D_MAX){1'b0}};
            gamma_reg     <= {GAMMA_W{1'b0}};
            inv_gamma_reg <= {INV_W{1'b0}};
            batch_cnt     <= 8'd0;
            sum_acc       <= {SUM_W{1'b0}};
            sqrt_value    <= {SUM_W{1'b0}};
            sqrt_root     <= {GAMMA_W{1'b0}};
            sqrt_bit      <= 8'd0;
            recip_start   <= 1'b0;
            recip_divisor <= {GAMMA_W{1'b0}};
            valid_out     <= 1'b0;
            done          <= 1'b0;
        end else begin
            recip_start <= 1'b0;
            valid_out   <= 1'b0;
            done        <= 1'b0;

            case (state)
                S_IDLE: if (start) begin
                    u_out_reg     <= {(OUT_W*D_MAX){1'b0}};
                    gamma_reg     <= {GAMMA_W{1'b0}};
                    inv_gamma_reg <= {INV_W{1'b0}};
                    batch_cnt     <= 8'd0;
                    sum_acc       <= {SUM_W{1'b0}};
                    state         <= S_ACCUM;
                end

                S_ACCUM: begin
                    if (batch_cnt == BATCHES_U - 8'd1) begin
                        sqrt_value <= sum_acc + batch_sum;
                        sqrt_root  <= {GAMMA_W{1'b0}};
                        sqrt_bit   <= GAMMA_W - 1;
                        state      <= S_SQRT;
                    end else begin
                        sum_acc   <= sum_acc + batch_sum;
                        batch_cnt <= batch_cnt + 1'b1;
                    end
                end

                S_SQRT: begin
                    sqrt_root <= sqrt_next_root;
                    if (sqrt_bit == 8'd0) begin
                        gamma_reg <= sqrt_next_root;
                        if (sqrt_next_root == {GAMMA_W{1'b0}}) begin
                            inv_gamma_reg <= {INV_W{1'b0}};
                            u_out_reg     <= {(OUT_W*D_MAX){1'b0}};
                            valid_out     <= 1'b1;
                            done          <= 1'b1;
                            state         <= S_IDLE;
                        end else begin
                            recip_divisor <= sqrt_next_root;
                            recip_start   <= 1'b1;
                            state         <= S_RECIP_WAIT;
                        end
                    end else begin
                        sqrt_bit <= sqrt_bit - 1'b1;
                    end
                end

                S_RECIP_WAIT: if (recip_valid) begin
                    inv_gamma_reg <= recip_quotient;
                    batch_cnt     <= 8'd0;
                    state         <= S_MUL;
                end

                S_MUL: begin
                    for (m = 0; m < LANES; m = m + 1) begin
                        norm_shifted = lane_product[m] >>> INV_SHIFT;
                        u_out_reg[((batch_cnt*LANES + m)*OUT_W) +: OUT_W] <= sat_to_outw(norm_shifted);
                    end
                    if (batch_cnt == BATCHES_U - 8'd1) begin
                        valid_out <= 1'b1;
                        done      <= 1'b1;
                        state     <= S_IDLE;
                    end else begin
                        batch_cnt <= batch_cnt + 1'b1;
                    end
                end
            endcase
        end
    end
endmodule
