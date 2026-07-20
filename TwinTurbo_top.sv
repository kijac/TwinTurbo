`timescale 1ns / 1ps
// ============================================================================
// TwinTurbo_top.sv
//
// First shared-resource top for Feeder IP.
// - Quant path uses existing quant_top_full_dmax with external FWHT ports.
// - Dequant path uses dequant_top_shared with external FWHT ports.
// - FWHT_CORE_TOP_DMAX is instantiated once at this top level.
//
// Note:
//   This version shares FWHT/RHT calculator and centroid LUT.
//   Quant sorter receives boundary_0~14 from the shared LUT.
//   Dequant centroid transformer receives centroid16_0~15 from the shared LUT.
// ============================================================================

module TwinTurbo_top #(
    parameter int D_ACTIVE   = 128,
    parameter int FP16_W     = 16,
    parameter int FIXED_W    = 16,
    parameter int FWHT_W     = 24,
    parameter int GAMMA_W    = 20,
    parameter int FRAC_W     = 12,
    parameter int IN_LANES   = 32,
    parameter int SORT_LANES = 8,
    parameter int AXI_DATA_W = 128,
    parameter int IDX_W      = 4,
    parameter int CT_LANES   = 64,
    parameter int MUL_LANES  = 128,
    parameter int NUM_PE     = 32
)(
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         clear,

    // ============================================================
    // Quantizer input: Matrix Processing Unit -> Feeder IP
    // 32 lanes of FP16 per beat
    // ============================================================
    input  logic                         q_s_valid,
    output logic                         q_s_ready,
    input  logic                         q_s_start,
    input  logic [IN_LANES*FP16_W-1:0]   q_s_fp16_vec,

    // Quant FWHT mode control. For normal quant/RHT, tie this to 1'b0.
    input  logic                         q_fwht_mode_from_ctrl,

    // ============================================================
    // Quantizer output: Feeder IP -> Device Memory
    // packed compressed stream: norm/gamma + 4-bit indices
    // ============================================================
    output logic                         q_m_tvalid,
    output logic [AXI_DATA_W-1:0]        q_m_tdata,
    output logic                         q_m_tlast,
    output logic [(AXI_DATA_W/8)-1:0]    q_m_tkeep,
    input  logic                         q_m_tready,

    // ============================================================
    // DeQuantizer input: Device Memory -> Feeder IP
    // packed compressed stream: norm/gamma + 4-bit indices
    // ============================================================
    input  logic                         dq_start,
    output logic                         dq_ready,
    output logic                         dq_busy,

    input  logic                         dq_s_tvalid,
    input  logic [AXI_DATA_W-1:0]        dq_s_tdata,
    input  logic                         dq_s_tlast,
    input  logic [(AXI_DATA_W/8)-1:0]    dq_s_tkeep,
    output logic                         dq_s_tready,

    // ============================================================
    // DeQuantizer output: Feeder IP -> Matrix Processing Unit
    // ============================================================
    output logic [FP16_W*D_ACTIVE-1:0]   dq_fp16_out,
    output logic                         dq_valid_out,
    output logic                         dq_done,

    // ============================================================
    // Status
    // ============================================================
    output logic                         q_busy,
    output logic                         q_input_overrun,
    output logic                         q_packer_drop_flag,
    output logic                         q_error,

    output logic                         dq_unpack_error,
    output logic                         dq_overflow,
    output logic [2:0]                   dq_gamma_fifo_count,
    output logic                         dq_gamma_fifo_full,
    output logic                         dq_gamma_fifo_empty,

    output logic                         shared_quant_fwht_busy,
    output logic                         shared_dequant_fwht_busy
);

    // ============================================================
    // Shared FWHT/RHT calculator wires
    // ============================================================
    logic                         q_fwht_start;
    logic                         q_fwht_mode;
    logic [FIXED_W*D_ACTIVE-1:0]  q_fwht_din;
    logic [FWHT_W*D_ACTIVE-1:0]   q_fwht_dout;
    logic                         q_fwht_valid_out;
    logic                         q_fwht_done;
    logic                         q_fwht_busy;

    logic                         dq_fwht_start;
    logic                         dq_fwht_mode;
    logic [FIXED_W*D_ACTIVE-1:0]  dq_fwht_din;
    logic [FWHT_W*D_ACTIVE-1:0]   dq_fwht_dout;
    logic                         dq_fwht_valid_out;
    logic                         dq_fwht_done;
    logic                         dq_fwht_busy_int;

    assign shared_quant_fwht_busy   = q_fwht_busy;
    assign shared_dequant_fwht_busy = dq_fwht_busy_int;

    // ============================================================
    // Shared centroid LUT
    // - Quant boundary sharing can use the 24-bit boundary outputs later.
    // - Dequant centroid transformer uses centroid16 outputs now.
    // ============================================================
    logic signed [15:0] lut_c16_0;
    logic signed [15:0] lut_c16_1;
    logic signed [15:0] lut_c16_2;
    logic signed [15:0] lut_c16_3;
    logic signed [15:0] lut_c16_4;
    logic signed [15:0] lut_c16_5;
    logic signed [15:0] lut_c16_6;
    logic signed [15:0] lut_c16_7;
    logic signed [15:0] lut_c16_8;
    logic signed [15:0] lut_c16_9;
    logic signed [15:0] lut_c16_10;
    logic signed [15:0] lut_c16_11;
    logic signed [15:0] lut_c16_12;
    logic signed [15:0] lut_c16_13;
    logic signed [15:0] lut_c16_14;
    logic signed [15:0] lut_c16_15;

    logic signed [23:0] lut_b0;
    logic signed [23:0] lut_b1;
    logic signed [23:0] lut_b2;
    logic signed [23:0] lut_b3;
    logic signed [23:0] lut_b4;
    logic signed [23:0] lut_b5;
    logic signed [23:0] lut_b6;
    logic signed [23:0] lut_b7;
    logic signed [23:0] lut_b8;
    logic signed [23:0] lut_b9;
    logic signed [23:0] lut_b10;
    logic signed [23:0] lut_b11;
    logic signed [23:0] lut_b12;
    logic signed [23:0] lut_b13;
    logic signed [23:0] lut_b14;

    tq_lut_rom_imp u_shared_centroid_lut (
        .centroid_out_0(),   .centroid_out_1(),
        .centroid_out_2(),   .centroid_out_3(),
        .centroid_out_4(),   .centroid_out_5(),
        .centroid_out_6(),   .centroid_out_7(),
        .centroid_out_8(),   .centroid_out_9(),
        .centroid_out_10(),  .centroid_out_11(),
        .centroid_out_12(),  .centroid_out_13(),
        .centroid_out_14(),  .centroid_out_15(),

        .centroid16_out_0(lut_c16_0),
        .centroid16_out_1(lut_c16_1),
        .centroid16_out_2(lut_c16_2),
        .centroid16_out_3(lut_c16_3),
        .centroid16_out_4(lut_c16_4),
        .centroid16_out_5(lut_c16_5),
        .centroid16_out_6(lut_c16_6),
        .centroid16_out_7(lut_c16_7),
        .centroid16_out_8(lut_c16_8),
        .centroid16_out_9(lut_c16_9),
        .centroid16_out_10(lut_c16_10),
        .centroid16_out_11(lut_c16_11),
        .centroid16_out_12(lut_c16_12),
        .centroid16_out_13(lut_c16_13),
        .centroid16_out_14(lut_c16_14),
        .centroid16_out_15(lut_c16_15),

        .boundary_out_0(lut_b0),
        .boundary_out_1(lut_b1),
        .boundary_out_2(lut_b2),
        .boundary_out_3(lut_b3),
        .boundary_out_4(lut_b4),
        .boundary_out_5(lut_b5),
        .boundary_out_6(lut_b6),
        .boundary_out_7(lut_b7),
        .boundary_out_8(lut_b8),
        .boundary_out_9(lut_b9),
        .boundary_out_10(lut_b10),
        .boundary_out_11(lut_b11),
        .boundary_out_12(lut_b12),
        .boundary_out_13(lut_b13),
        .boundary_out_14(lut_b14)
    );

    // One top-level RHT calculator block.
    // It contains two FWHT cores internally: one for quant, one for dequant.
    FWHT_CORE_TOP_DMAX #(
        .DATA_W (FWHT_W),
        .IN_W   (FIXED_W),
        .D_MAX  (D_ACTIVE),
        .NUM_PE (NUM_PE)
    ) u_shared_rht_calculator (
        .clk                (clk),
        .rst_n              (rst_n),
        .clear              (clear),

        .quant_start        (q_fwht_start),
        .quant_mode         (q_fwht_mode),
        .quant_din          (q_fwht_din),
        .quant_dout         (q_fwht_dout),
        .quant_valid_out    (q_fwht_valid_out),
        .quant_done         (q_fwht_done),
        .quant_busy         (q_fwht_busy),

        .dequant_start      (dq_fwht_start),
        .dequant_mode       (dq_fwht_mode),
        .dequant_din        (dq_fwht_din),
        .dequant_dout       (dq_fwht_dout),
        .dequant_valid_out  (dq_fwht_valid_out),
        .dequant_done       (dq_fwht_done),
        .dequant_busy       (dq_fwht_busy_int)
    );

    // ============================================================
    // Quantizer core
    // Uses external quant-side FWHT ports from u_shared_rht_calculator.
    // Centroid sorter uses boundary values from shared top-level LUT.
    // ============================================================
    quant_top_full_dmax #(
        .FP16_W     (FP16_W),
        .FIXED_W    (FIXED_W),
        .FWHT_W     (FWHT_W),
        .GAMMA_W    (GAMMA_W),
        .FRAC_W     (FRAC_W),
        .D_MAX      (D_ACTIVE),
        .IN_LANES   (IN_LANES),
        .SORT_LANES (SORT_LANES),
        .AXI_DATA_W (AXI_DATA_W)
    ) u_quantizer (
        .clk                 (clk),
        .rst_n               (rst_n),
        .clear               (clear),

        .fwht_mode_from_ctrl (q_fwht_mode_from_ctrl),

        .s_valid             (q_s_valid),
        .s_ready             (q_s_ready),
        .s_start             (q_s_start),
        .s_fp16_vec          (q_s_fp16_vec),

        .m_tvalid            (q_m_tvalid),
        .m_tdata             (q_m_tdata),
        .m_tlast             (q_m_tlast),
        .m_tkeep             (q_m_tkeep),
        .m_tready            (q_m_tready),

        .busy                (q_busy),
        .input_overrun       (q_input_overrun),
        .packer_drop_flag    (q_packer_drop_flag),
        .quant_error         (q_error),

        .boundary_0          (lut_b0),
        .boundary_1          (lut_b1),
        .boundary_2          (lut_b2),
        .boundary_3          (lut_b3),
        .boundary_4          (lut_b4),
        .boundary_5          (lut_b5),
        .boundary_6          (lut_b6),
        .boundary_7          (lut_b7),
        .boundary_8          (lut_b8),
        .boundary_9          (lut_b9),
        .boundary_10         (lut_b10),
        .boundary_11         (lut_b11),
        .boundary_12         (lut_b12),
        .boundary_13         (lut_b13),
        .boundary_14         (lut_b14),

        .fwht_start          (q_fwht_start),
        .fwht_mode           (q_fwht_mode),
        .fwht_din            (q_fwht_din),
        .fwht_dout           (q_fwht_dout),
        .fwht_valid_out      (q_fwht_valid_out),
        .fwht_done           (q_fwht_done),
        .fwht_busy           (q_fwht_busy)
    );

    // ============================================================
    // DeQuantizer core
    // Uses external dequant-side FWHT ports from u_shared_rht_calculator.
    // Centroid transformer uses centroid16 values from shared top-level LUT.
    // ============================================================
    dequant_top_shared #(
        .D_ACTIVE   (D_ACTIVE),
        .AXI_DATA_W (AXI_DATA_W),
        .IDX_W      (IDX_W),
        .CT_LANES   (CT_LANES),
        .IN_W       (FIXED_W),
        .FWHT_W     (FWHT_W),
        .FP16_W     (FP16_W),
        .MUL_LANES  (MUL_LANES),
        .FRAC_W     (FRAC_W),
        .GAMMA_W    (GAMMA_W),
        .NUM_PE     (NUM_PE)
    ) u_dequantizer (
        .clk                  (clk),
        .rst_n                (rst_n),
        .clear                (clear),

        .start                (dq_start),
        .ready                (dq_ready),
        .busy                 (dq_busy),

        .s_tvalid             (dq_s_tvalid),
        .s_tdata              (dq_s_tdata),
        .s_tlast              (dq_s_tlast),
        .s_tkeep              (dq_s_tkeep),
        .s_tready             (dq_s_tready),

        .fp16_out             (dq_fp16_out),
        .valid_out            (dq_valid_out),
        .done                 (dq_done),

        .unpack_error         (dq_unpack_error),
        .overflow             (dq_overflow),
        .gamma_fifo_count     (dq_gamma_fifo_count),
        .gamma_fifo_full      (dq_gamma_fifo_full),
        .gamma_fifo_empty     (dq_gamma_fifo_empty),

        .centroid16_0         (lut_c16_0),
        .centroid16_1         (lut_c16_1),
        .centroid16_2         (lut_c16_2),
        .centroid16_3         (lut_c16_3),
        .centroid16_4         (lut_c16_4),
        .centroid16_5         (lut_c16_5),
        .centroid16_6         (lut_c16_6),
        .centroid16_7         (lut_c16_7),
        .centroid16_8         (lut_c16_8),
        .centroid16_9         (lut_c16_9),
        .centroid16_10        (lut_c16_10),
        .centroid16_11        (lut_c16_11),
        .centroid16_12        (lut_c16_12),
        .centroid16_13        (lut_c16_13),
        .centroid16_14        (lut_c16_14),
        .centroid16_15        (lut_c16_15),

        .fwht_start           (dq_fwht_start),
        .fwht_mode            (dq_fwht_mode),
        .fwht_din             (dq_fwht_din),
        .fwht_dout            (dq_fwht_dout),
        .fwht_valid_out       (dq_fwht_valid_out),
        .fwht_done            (dq_fwht_done),
        .fwht_busy            (dq_fwht_busy_int)
    );

endmodule
