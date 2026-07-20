`timescale 1ns / 1ps
// ============================================================================
// quant_top_full_dmax.v
//
// Quantization datapath wrapper using D_MAX as the fixed dimension.
// No runtime dim_mode/d_sel port.
//
// Pipeline:
//   FP16 32-lane stream
//     -> fp16_to_fixed_32lane_dmax
//     -> io_buffer_32lane_dmax
//     -> RHT_L2_Normalize_Frontend_DMAX
//     -> external FWHT_CORE_TOP_DMAX quant port
//     -> tq_centroid_sorter_fsm_dmax
//     -> tq_bit_packer_rdy_dmax
//     -> AXI-stream-like packed output
//
// The FWHT core is intentionally external so that the same system top can keep
// quant/dequant FWHT cores together in FWHT_CORE_TOP_DMAX.
// ============================================================================
module quant_top_full_dmax #(
    parameter integer FP16_W     = 16,
    parameter integer FIXED_W    = 16,
    parameter integer FWHT_W     = 24,
    parameter integer GAMMA_W    = 20,
    parameter integer FRAC_W     = 12,
    parameter integer D_MAX      = 128,
    parameter integer IN_LANES   = 32,
    parameter integer SORT_LANES = 8,
    parameter integer AXI_DATA_W = 128
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          clear,

    input  wire                          fwht_mode_from_ctrl,

    input  wire                          s_valid,
    output wire                          s_ready,
    input  wire                          s_start,
    input  wire [IN_LANES*FP16_W-1:0]    s_fp16_vec,

    output wire                          m_tvalid,
    output wire [AXI_DATA_W-1:0]         m_tdata,
    output wire                          m_tlast,
    output wire [(AXI_DATA_W/8)-1:0]     m_tkeep,
    input  wire                          m_tready,

    output wire                          busy,
    output wire                          input_overrun,
    output wire                          packer_drop_flag,
    output reg                           quant_error,

    output wire                          fwht_start,
    output wire                          fwht_mode,
    output wire [FIXED_W*D_MAX-1:0]      fwht_din,
    input  wire [FWHT_W*D_MAX-1:0]       fwht_dout,
    input  wire                          fwht_valid_out,
    input  wire                          fwht_done,
    input  wire                          fwht_busy,

    input  wire signed [23:0]             boundary_0,
    input  wire signed [23:0]             boundary_1,
    input  wire signed [23:0]             boundary_2,
    input  wire signed [23:0]             boundary_3,
    input  wire signed [23:0]             boundary_4,
    input  wire signed [23:0]             boundary_5,
    input  wire signed [23:0]             boundary_6,
    input  wire signed [23:0]             boundary_7,
    input  wire signed [23:0]             boundary_8,
    input  wire signed [23:0]             boundary_9,
    input  wire signed [23:0]             boundary_10,
    input  wire signed [23:0]             boundary_11,
    input  wire signed [23:0]             boundary_12,
    input  wire signed [23:0]             boundary_13,
    input  wire signed [23:0]             boundary_14
);

    // ---------------------------------------------------------------------
    // FP16 -> fixed
    // ---------------------------------------------------------------------
    wire                          fixed_valid;
    wire                          fixed_ready;
    wire                          fixed_last;
    wire                          fixed_done;
    wire signed [IN_LANES*FIXED_W-1:0] fixed_vec;
    wire [IN_LANES-1:0]           fixed_overflow_vec;
    wire [IN_LANES-1:0]           fixed_zero_vec;
    wire [IN_LANES-1:0]           fixed_invalid_vec;

    fp16_to_fixed_32lane_dmax #(
        .DATA_W(FIXED_W),
        .FRAC_W(FRAC_W),
        .LANES(IN_LANES),
        .D_MAX(D_MAX)
    ) u_fp16_to_fixed_32lane (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(s_valid),
        .ready_out(s_ready),
        .start_in(s_start),
        .fp16_in_vec(s_fp16_vec),
        .valid_out(fixed_valid),
        .ready_in(fixed_ready),
        .last_out(fixed_last),
        .done_out(fixed_done),
        .fixed_out_vec(fixed_vec),
        .overflow_vec(fixed_overflow_vec),
        .zero_vec(fixed_zero_vec),
        .invalid_vec(fixed_invalid_vec)
    );

    // ---------------------------------------------------------------------
    // I/O buffer
    // ---------------------------------------------------------------------
    wire                         buf_valid;
    wire                         buf_ready;
    wire [D_MAX*FIXED_W-1:0]     buf_data;
    wire                         buf_error;
    wire                         buf_busy;

    io_buffer_32lane_dmax #(
        .DATA_W(FIXED_W),
        .D_MAX(D_MAX),
        .LANES(IN_LANES)
    ) u_io_buffer (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .in_valid(fixed_valid),
        .in_ready(fixed_ready),
        .in_data(fixed_vec),
        .in_overflow(fixed_overflow_vec),
        .in_invalid(fixed_invalid_vec),
        .out_valid(buf_valid),
        .out_ready(buf_ready),
        .out_data(buf_data),
        .out_error(buf_error),
        .input_overrun(input_overrun),
        .busy(buf_busy)
    );

    // ---------------------------------------------------------------------
    // L2 normalization
    // ---------------------------------------------------------------------
    wire                         l2_busy;
    wire                         l2_valid;
    wire                         l2_done;
    wire [D_MAX*FIXED_W-1:0]     l2_u_out;
    wire [GAMMA_W-1:0]           l2_gamma;
    wire [32:0]                  l2_inv_gamma_unused;

    reg                          l2_hold_valid;
    reg [D_MAX*FIXED_W-1:0]      l2_u_hold;
    reg [GAMMA_W-1:0]            gamma_hold;
    reg                          err_l2_hold;
    reg                          err_buf_active;

    reg                          rht_hold_valid;
    reg [FWHT_W*D_MAX-1:0]       rht_hold;
    reg                          err_rht_hold;

    wire can_start_l2 = buf_valid & ~l2_busy & ~l2_hold_valid;
    wire l2_start     = can_start_l2;

    assign buf_ready  = l2_done;

    RHT_L2_Normalize_Frontend_DMAX #(
        .INPUT_W(FIXED_W),
        .OUT_W(FIXED_W),
        .D_MAX(D_MAX),
        .FRAC_W(FRAC_W),
        .LANES(IN_LANES),
        .GAMMA_W(GAMMA_W)
    ) u_l2_normalize (
        .clk(clk),
        .rst_n(rst_n),
        .start(l2_start),
        .din(buf_data),
        .u_out(l2_u_out),
        .gamma(l2_gamma),
        .inv_gamma(l2_inv_gamma_unused),
        .valid_out(l2_valid),
        .done(l2_done),
        .busy(l2_busy)
    );

    // ---------------------------------------------------------------------
    // FWHT interface
    // ---------------------------------------------------------------------
    assign fwht_start = l2_hold_valid & ~fwht_busy & ~rht_hold_valid;
    assign fwht_mode  = fwht_mode_from_ctrl;
    assign fwht_din   = l2_u_hold;

    // ---------------------------------------------------------------------
    // Centroid sorter and bit packer
    // ---------------------------------------------------------------------
    wire [4*SORT_LANES-1:0] sorter_batch_code_vec;
    wire                    sorter_batch_valid;
    wire                    sorter_batch_ready;
    wire                    sorter_batch_last;
    wire                    sorter_done;
    wire                    sorter_busy;
    wire                    sorter_ready;

    wire                    packer_busy;
    wire                    packer_norm_ready;
    wire                    packer_norm_valid;
    wire                    packer_norm_ignored;

    wire sorter_start = rht_hold_valid & sorter_ready & packer_norm_ready;

    assign packer_norm_valid = sorter_start;

    // ---------------------------------------------------------------------
    // Q-format alignment before centroid sorter
    //
    // L2/FWHT datapath uses FRAC_W fractional bits.
    // Current quant setting: FRAC_W = 12.
    //
    // tq_centroid_sorter boundary LUT is Q8.15.
    // Therefore FWHT output must be aligned:
    //   Q?.12 -> Q8.15 : left shift by 3
    // ---------------------------------------------------------------------
    localparam integer SORTER_FRAC_W       = 15;
    localparam integer RHT_TO_SORTER_SHIFT = SORTER_FRAC_W - FRAC_W;

    wire [FWHT_W*D_MAX-1:0] rht_hold_for_sorter;

    genvar qfmt_i;
    generate
        if (RHT_TO_SORTER_SHIFT > 0) begin : GEN_RHT_QFORMAT_LSHIFT
            for (qfmt_i = 0; qfmt_i < D_MAX; qfmt_i = qfmt_i + 1) begin : GEN_LANE
                assign rht_hold_for_sorter[qfmt_i*FWHT_W +: FWHT_W]
                    = $signed(rht_hold[qfmt_i*FWHT_W +: FWHT_W]) <<< RHT_TO_SORTER_SHIFT;
            end
        end else if (RHT_TO_SORTER_SHIFT == 0) begin : GEN_RHT_QFORMAT_PASS
            for (qfmt_i = 0; qfmt_i < D_MAX; qfmt_i = qfmt_i + 1) begin : GEN_LANE
                assign rht_hold_for_sorter[qfmt_i*FWHT_W +: FWHT_W]
                    = rht_hold[qfmt_i*FWHT_W +: FWHT_W];
            end
        end else begin : GEN_RHT_QFORMAT_RSHIFT
            for (qfmt_i = 0; qfmt_i < D_MAX; qfmt_i = qfmt_i + 1) begin : GEN_LANE
                assign rht_hold_for_sorter[qfmt_i*FWHT_W +: FWHT_W]
                    = $signed(rht_hold[qfmt_i*FWHT_W +: FWHT_W]) >>> (FRAC_W - SORTER_FRAC_W);
            end
        end
    endgenerate

    tq_centroid_sorter_fsm_dmax #(
        .DATA_W(FWHT_W),
        .D_MAX(D_MAX),
        .NUM_LANE(SORT_LANES)
    ) u_centroid_sorter_fsm (
        .clk(clk),
        .rst_n(rst_n),
        .start(sorter_start),
        .ready(sorter_ready),

        // IMPORTANT:
        // sorter boundary is Q8.15, so use aligned RHT output here.
        .din(rht_hold_for_sorter),

        .boundary_0(boundary_0),
        .boundary_1(boundary_1),
        .boundary_2(boundary_2),
        .boundary_3(boundary_3),
        .boundary_4(boundary_4),
        .boundary_5(boundary_5),
        .boundary_6(boundary_6),
        .boundary_7(boundary_7),
        .boundary_8(boundary_8),
        .boundary_9(boundary_9),
        .boundary_10(boundary_10),
        .boundary_11(boundary_11),
        .boundary_12(boundary_12),
        .boundary_13(boundary_13),
        .boundary_14(boundary_14),
        .batch_code_vec(sorter_batch_code_vec),
        .batch_valid(sorter_batch_valid),
        .batch_ready(sorter_batch_ready),
        .batch_last(sorter_batch_last),
        .done(sorter_done),
        .busy(sorter_busy)
    );

    tq_bit_packer_rdy_dmax #(
        .DATA_W(GAMMA_W),
        .IDX_W(4),
        .NUM_LANE(SORT_LANES),
        .D_MAX(D_MAX),
        .AXI_DATA_W(AXI_DATA_W)
    ) u_bit_packer (
        .clk(clk),
        .rst_n(rst_n),
        .norm_valid(packer_norm_valid),
        .norm_ready(packer_norm_ready),
        .norm_in(gamma_hold),
        .batch_valid(sorter_batch_valid),
        .batch_ready(sorter_batch_ready),
        .batch_code_vec(sorter_batch_code_vec),
        .batch_last(sorter_batch_last),
        .m_tvalid(m_tvalid),
        .m_tdata(m_tdata),
        .m_tlast(m_tlast),
        .m_tkeep(m_tkeep),
        .m_tready(m_tready),
        .busy(packer_busy),
        .norm_ignored(packer_norm_ignored),
        .drop_flag(packer_drop_flag)
    );

    // ---------------------------------------------------------------------
    // Pipeline hold registers and error propagation
    // ---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l2_hold_valid  <= 1'b0;
            l2_u_hold      <= {(D_MAX*FIXED_W){1'b0}};
            gamma_hold     <= {GAMMA_W{1'b0}};
            err_l2_hold    <= 1'b0;
            err_buf_active <= 1'b0;

            rht_hold_valid <= 1'b0;
            rht_hold       <= {(FWHT_W*D_MAX){1'b0}};
            err_rht_hold   <= 1'b0;
            quant_error    <= 1'b0;
        end else if (clear) begin
            l2_hold_valid  <= 1'b0;
            err_l2_hold    <= 1'b0;
            err_buf_active <= 1'b0;

            rht_hold_valid <= 1'b0;
            err_rht_hold   <= 1'b0;
            quant_error    <= 1'b0;
        end else begin
            if (l2_start) begin
                err_buf_active <= buf_error;
            end

            if (l2_valid) begin
                l2_hold_valid <= 1'b1;
                l2_u_hold     <= l2_u_out;
                gamma_hold    <= l2_gamma;
                err_l2_hold   <= err_buf_active;
            end

            if (fwht_start) begin
                l2_hold_valid <= 1'b0;
            end

            if (fwht_valid_out | fwht_done) begin
                rht_hold_valid <= 1'b1;
                rht_hold       <= fwht_dout;
                err_rht_hold   <= err_l2_hold;
            end

            if (sorter_done) begin
                rht_hold_valid <= 1'b0;

                if (err_rht_hold) begin
                    quant_error <= 1'b1;
                end
            end

            if (packer_norm_ignored | packer_drop_flag) begin
                quant_error <= 1'b1;
            end
        end
    end

    assign busy = buf_busy | l2_busy | l2_hold_valid | fwht_busy |
                  rht_hold_valid | sorter_busy | packer_busy | m_tvalid;

endmodule