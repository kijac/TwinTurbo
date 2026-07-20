`timescale 1ns / 1ps

// ============================================================================
// FWHT_FSM_DMAX
//  - Verilog-2001 compatible FWHT/RHT core
//  - D_MAX is the selected dimension. No runtime dim_mode port.
//  - Set D_MAX to 64, 128, or 256 at module instantiation.
//  - mode : 0 = forward RHT, 1 = inverse RHT
//  - NUM_PE : 32 butterfly pairs/cycle recommended
//  - din  : IN_W  * D_MAX packed vector
//  - dout : DATA_W * D_MAX packed vector
//
// INVERSE_POST_SHIFT_EN:
//  - 1: inverse final stage에서 >>> log2(D) 수행
//  - 0: inverse final stage에서 sign flip만 수행하고 /D shift는 하지 않음
//
// Cycles with NUM_PE=32:
//  D_MAX=64  -> 6 stages * 1 phase = 6 compute cycles
//  D_MAX=128 -> 7 stages * 2 phase = 14 compute cycles
//  D_MAX=256 -> 8 stages * 4 phase = 32 compute cycles
// ============================================================================
module FWHT_FSM_DMAX #(
    parameter integer DATA_W = 24,
    parameter integer IN_W   = 16,
    parameter integer D_MAX  = 128,
    parameter integer NUM_PE = 32,
    parameter integer INVERSE_POST_SHIFT_EN = 1
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear,
    input  wire                         start,
    input  wire                         mode,
    input  wire [IN_W*D_MAX-1:0]        din,
    output reg  [DATA_W*D_MAX-1:0]      dout,
    output reg                          valid_out,
    output reg                          done,
    output wire                         busy
);

    localparam FORWARD = 1'b0;
    localparam INVERSE = 1'b1;

    localparam integer NUM_STAGE =
        (D_MAX == 64)  ? 6 :
        (D_MAX == 128) ? 7 :
        (D_MAX == 256) ? 8 : 7;

    localparam integer NUM_PHASE   = (D_MAX / 2) / NUM_PE;
    localparam integer FINAL_SHIFT = NUM_STAGE;

    localparam S_IDLE    = 1'b0;
    localparam S_COMPUTE = 1'b1;

    reg state;
    reg mode_reg;

    reg signed [DATA_W-1:0] bank [0:D_MAX-1];

    integer stage_cnt;
    integer phase_cnt;

    integer i;
    integer pe;
    integer pair_id;
    integer stride;
    integer group_id;
    integer offset;
    integer idx_a;
    integer idx_b;
    integer d;

    reg signed [DATA_W-1:0] a;
    reg signed [DATA_W-1:0] b;
    reg signed [DATA_W-1:0] next_a;
    reg signed [DATA_W-1:0] next_b;
    reg final_stage;

    assign busy = (state == S_COMPUTE);

    function fixed_sign_bit;
        input integer idx;
        begin
            // 현재 버전은 고정 alternating sign mask: +, -, +, -, ...
            fixed_sign_bit = idx[0];
        end
    endfunction

    function signed [DATA_W-1:0] apply_forward_sign;
        input [DATA_W-1:0] value;
        input integer idx;
        begin
            if (fixed_sign_bit(idx))
                apply_forward_sign = $signed(-$signed(value));
            else
                apply_forward_sign = $signed(value);
        end
    endfunction

    function signed [DATA_W-1:0] apply_inverse_post;
        input [DATA_W-1:0] value;
        input integer idx;
        reg signed [DATA_W-1:0] scaled;
        begin
            if (INVERSE_POST_SHIFT_EN)
                scaled = $signed(value) >>> FINAL_SHIFT;
            else
                scaled = $signed(value);

            if (fixed_sign_bit(idx))
                apply_inverse_post = $signed(-scaled);
            else
                apply_inverse_post = scaled;
        end
    endfunction

    // dout is always the current bank image.
    always @* begin
        for (d = 0; d < D_MAX; d = d + 1) begin
            dout[d*DATA_W +: DATA_W] = bank[d];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            mode_reg  <= FORWARD;
            stage_cnt <= 0;
            phase_cnt <= 0;
            valid_out <= 1'b0;
            done      <= 1'b0;

            for (i = 0; i < D_MAX; i = i + 1) begin
                bank[i] <= {DATA_W{1'b0}};
            end
        end else if (clear) begin
            state     <= S_IDLE;
            mode_reg  <= FORWARD;
            stage_cnt <= 0;
            phase_cnt <= 0;
            valid_out <= 1'b0;
            done      <= 1'b0;

            for (i = 0; i < D_MAX; i = i + 1) begin
                bank[i] <= {DATA_W{1'b0}};
            end
        end else begin
            valid_out <= 1'b0;
            done      <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        mode_reg <= mode;

                        for (i = 0; i < D_MAX; i = i + 1) begin
                            if (mode == FORWARD)
                                bank[i] <= apply_forward_sign(
                                    {{(DATA_W-IN_W){din[i*IN_W + IN_W-1]}}, din[i*IN_W +: IN_W]},
                                    i
                                );
                            else
                                bank[i] <= $signed(
                                    {{(DATA_W-IN_W){din[i*IN_W + IN_W-1]}}, din[i*IN_W +: IN_W]}
                                );
                        end

                        stage_cnt <= 0;
                        phase_cnt <= 0;
                        state     <= S_COMPUTE;
                    end
                end

                S_COMPUTE: begin
                    stride      = (1 << stage_cnt);
                    final_stage = (stage_cnt == NUM_STAGE - 1);

                    for (pe = 0; pe < NUM_PE; pe = pe + 1) begin
                        pair_id  = (phase_cnt * NUM_PE) + pe;
                        group_id = pair_id / (1 << stage_cnt);
                        offset   = pair_id & (stride - 1);
                        idx_a    = (group_id << (stage_cnt + 1)) + offset;
                        idx_b    = idx_a + stride;

                        a = bank[idx_a];
                        b = bank[idx_b];

                        if (final_stage && (mode_reg == INVERSE)) begin
                            next_a = apply_inverse_post(a + b, idx_a);
                            next_b = apply_inverse_post(a - b, idx_b);
                        end else begin
                            next_a = a + b;
                            next_b = a - b;
                        end

                        bank[idx_a] <= next_a;
                        bank[idx_b] <= next_b;
                    end

                    if (phase_cnt == NUM_PHASE - 1) begin
                        phase_cnt <= 0;

                        if (stage_cnt == NUM_STAGE - 1) begin
                            state     <= S_IDLE;
                            valid_out <= 1'b1;
                            done      <= 1'b1;
                        end else begin
                            stage_cnt <= stage_cnt + 1;
                        end
                    end else begin
                        phase_cnt <= phase_cnt + 1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule