`timescale 1ns / 1ps

// ============================================================================
// 1-lane fixed-point to FP16 converter
// 3-stage pipeline
//
// fixed real value = fixed_in / 2^FRAC_W
//
// Stage 1:
//   sign / abs / zero / leading-one detect
//
// Stage 2:
//   exponent calculation / mantissa alignment / guard-sticky generation
//
// Stage 3:
//   round-to-nearest-even / FP16 pack
//
// Note:
//   - Subnormal FP16 is flushed to zero for simpler hardware.
//   - Overflow is converted to FP16 Inf.
// ============================================================================

`timescale 1ns / 1ps

module fixed_to_fp16_pipe3 #(
    parameter int DATA_W = 16,
    parameter int FRAC_W = 12
)(
    input  logic clk,
    input  logic rst_n,

    input  logic valid_in,
    input  logic signed [DATA_W-1:0] fixed_in,

    output logic valid_out,
    output logic [15:0] fp16_out,

    // 1-lane 내부 디버깅용 flag
    output logic zero,
    output logic overflow,
    output logic underflow
);

    localparam int MANT_W   = 10;
    localparam int EXP_BIAS = 15;
    localparam int EXP_MAX  = 31;
    localparam int MSB_W    = (DATA_W <= 2) ? 1 : $clog2(DATA_W);

    // ------------------------------------------------------------------------
    // Leading-one detector
    // ------------------------------------------------------------------------
    function automatic [MSB_W-1:0] find_msb;
        input logic [DATA_W-1:0] value;

        integer i;
        logic found;

        begin
            find_msb = '0;
            found    = 1'b0;

            for (i = DATA_W-1; i >= 0; i = i - 1) begin
                if (!found && value[i]) begin
                    find_msb = i[MSB_W-1:0];
                    found    = 1'b1;
                end
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // Stage 1: sign / abs / zero / leading-one detect
    // ------------------------------------------------------------------------
    logic s1_valid;
    logic s1_sign;
    logic s1_zero;
    logic [DATA_W-1:0] s1_abs;
    logic [MSB_W-1:0]  s1_msb_pos;

    logic [DATA_W-1:0] abs_comb;

    always_comb begin
        if (fixed_in[DATA_W-1])
            abs_comb = ~fixed_in + DATA_W'(1);
        else
            abs_comb = fixed_in;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid   <= 1'b0;
            s1_sign    <= 1'b0;
            s1_zero    <= 1'b0;
            s1_abs     <= '0;
            s1_msb_pos <= '0;
        end
        else begin
            s1_valid <= valid_in;

            if (valid_in) begin
                s1_sign <= fixed_in[DATA_W-1];
                s1_abs  <= abs_comb;
                s1_zero <= (abs_comb == '0);

                if (abs_comb == '0)
                    s1_msb_pos <= '0;
                else
                    s1_msb_pos <= find_msb(abs_comb);
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 2 combinational:
    // exponent calculation / mantissa alignment / guard-sticky generation
    // ------------------------------------------------------------------------
    logic             s2_overflow_comb;
    logic             s2_underflow_comb;
    logic [4:0]       s2_exp_comb;
    logic [MANT_W:0]  s2_mant_trunc_comb;
    logic             s2_guard_comb;
    logic             s2_sticky_comb;

    integer exp_unbiased_i;
    integer exp_biased_i;
    integer shift_r_i;
    integer shift_l_i;
    integer k;

    logic [DATA_W+MANT_W:0] shift_tmp;

    always_comb begin
        s2_overflow_comb   = 1'b0;
        s2_underflow_comb  = 1'b0;
        s2_exp_comb        = 5'd0;
        s2_mant_trunc_comb = '0;
        s2_guard_comb      = 1'b0;
        s2_sticky_comb     = 1'b0;

        exp_unbiased_i = 0;
        exp_biased_i   = 0;
        shift_r_i      = 0;
        shift_l_i      = 0;
        shift_tmp      = '0;

        if (!s1_zero) begin
            exp_unbiased_i = s1_msb_pos;
            exp_unbiased_i = exp_unbiased_i - FRAC_W;
            exp_biased_i   = exp_unbiased_i + EXP_BIAS;

            if (exp_biased_i >= EXP_MAX) begin
                s2_overflow_comb = 1'b1;
            end
            else if (exp_biased_i <= 0) begin
                // subnormal FP16은 단순화를 위해 zero로 flush
                s2_underflow_comb = 1'b1;
            end
            else begin
                s2_exp_comb = exp_biased_i[4:0];

                // hidden bit가 bit[10]에 오도록 정렬
                if (s1_msb_pos > MANT_W) begin
                    shift_r_i = s1_msb_pos - MANT_W;

                    shift_tmp = {{(MANT_W+1){1'b0}}, s1_abs} >> shift_r_i;
                    s2_mant_trunc_comb = shift_tmp[MANT_W:0];

                    // round-to-nearest-even용 guard/sticky
                    s2_guard_comb = s1_abs[shift_r_i - 1];

                    s2_sticky_comb = 1'b0;
                    for (k = 0; k < DATA_W; k = k + 1) begin
                        if (k < (shift_r_i - 1))
                            s2_sticky_comb = s2_sticky_comb | s1_abs[k];
                    end
                end
                else begin
                    shift_l_i = MANT_W - s1_msb_pos;

                    shift_tmp = {{(MANT_W+1){1'b0}}, s1_abs} << shift_l_i;
                    s2_mant_trunc_comb = shift_tmp[MANT_W:0];

                    s2_guard_comb  = 1'b0;
                    s2_sticky_comb = 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 2 registers
    // ------------------------------------------------------------------------
    logic            s2_valid;
    logic            s2_sign;
    logic            s2_zero;
    logic            s2_overflow;
    logic            s2_underflow;
    logic [4:0]      s2_exp;
    logic [MANT_W:0] s2_mant_trunc;
    logic            s2_guard;
    logic            s2_sticky;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid      <= 1'b0;
            s2_sign       <= 1'b0;
            s2_zero       <= 1'b0;
            s2_overflow   <= 1'b0;
            s2_underflow  <= 1'b0;
            s2_exp        <= 5'd0;
            s2_mant_trunc <= '0;
            s2_guard      <= 1'b0;
            s2_sticky     <= 1'b0;
        end
        else begin
            s2_valid <= s1_valid;

            if (s1_valid) begin
                s2_sign       <= s1_sign;
                s2_zero       <= s1_zero;
                s2_overflow   <= s2_overflow_comb;
                s2_underflow  <= s2_underflow_comb;
                s2_exp        <= s2_exp_comb;
                s2_mant_trunc <= s2_mant_trunc_comb;
                s2_guard      <= s2_guard_comb;
                s2_sticky     <= s2_sticky_comb;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 3 combinational: rounding + FP16 pack
    // ------------------------------------------------------------------------
    logic round_up;
    logic [MANT_W+1:0] rounded_mant;
    logic [5:0] exp_after_round;

    logic [15:0] fp16_next;
    logic zero_next;
    logic overflow_next;
    logic underflow_next;

    always_comb begin
        fp16_next      = 16'd0;
        zero_next      = 1'b0;
        overflow_next  = 1'b0;
        underflow_next = 1'b0;

        round_up        = 1'b0;
        rounded_mant    = '0;
        exp_after_round = 6'd0;

        if (s2_zero) begin
            fp16_next = {s2_sign, 5'd0, 10'd0};
            zero_next = 1'b1;
        end
        else if (s2_underflow) begin
            fp16_next = {s2_sign, 5'd0, 10'd0};
            underflow_next = 1'b1;
        end
        else if (s2_overflow) begin
            fp16_next = {s2_sign, 5'b11111, 10'd0};
            overflow_next = 1'b1;
        end
        else begin
            // round-to-nearest-even
            round_up = s2_guard & (s2_sticky | s2_mant_trunc[0]);

            rounded_mant = {1'b0, s2_mant_trunc} + round_up;

            // mantissa rounding overflow
            if (rounded_mant[MANT_W+1]) begin
                exp_after_round = {1'b0, s2_exp} + 6'd1;

                if (exp_after_round >= 6'd31) begin
                    fp16_next = {s2_sign, 5'b11111, 10'd0};
                    overflow_next = 1'b1;
                end
                else begin
                    fp16_next = {
                        s2_sign,
                        exp_after_round[4:0],
                        rounded_mant[MANT_W:1]
                    };
                end
            end
            else begin
                fp16_next = {
                    s2_sign,
                    s2_exp,
                    rounded_mant[MANT_W-1:0]
                };
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 3 registers
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            fp16_out  <= 16'd0;
            zero      <= 1'b0;
            overflow  <= 1'b0;
            underflow <= 1'b0;
        end
        else begin
            valid_out <= s2_valid;

            if (s2_valid) begin
                fp16_out  <= fp16_next;
                zero      <= zero_next;
                overflow  <= overflow_next;
                underflow <= underflow_next;
            end
        end
    end

endmodule