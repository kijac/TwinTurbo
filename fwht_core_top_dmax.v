`timescale 1ns / 1ps

// ============================================================================
// FWHT_CORE_TOP_DMAX
//  - Two independent FWHT cores for quant/dequant paths
//  - D_MAX is the selected dimension. No runtime dim_mode port.
//  - mode : 0 = forward, 1 = inverse
//
// Scaling policy:
//  - quant FWHT  : 기존 동작 유지
//  - dequant FWHT: inverse post shift(/D) 비활성화
//                  /D scaling은 dequant_mul_128lane에서 수행
// ============================================================================
module FWHT_CORE_TOP_DMAX #(
    parameter integer DATA_W = 24,
    parameter integer IN_W   = 16,
    parameter integer D_MAX  = 128,
    parameter integer NUM_PE = 32
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear,

    input  wire                         quant_start,
    input  wire                         quant_mode,
    input  wire [IN_W*D_MAX-1:0]        quant_din,
    output wire [DATA_W*D_MAX-1:0]      quant_dout,
    output wire                         quant_valid_out,
    output wire                         quant_done,
    output wire                         quant_busy,

    input  wire                         dequant_start,
    input  wire                         dequant_mode,
    input  wire [IN_W*D_MAX-1:0]        dequant_din,
    output wire [DATA_W*D_MAX-1:0]      dequant_dout,
    output wire                         dequant_valid_out,
    output wire                         dequant_done,
    output wire                         dequant_busy
);

    FWHT_FSM_DMAX #(
        .DATA_W(DATA_W),
        .IN_W(IN_W),
        .D_MAX(D_MAX),
        .NUM_PE(NUM_PE),
        .INVERSE_POST_SHIFT_EN(1)
    ) u_quant_fwht (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .start(quant_start),
        .mode(quant_mode),
        .din(quant_din),
        .dout(quant_dout),
        .valid_out(quant_valid_out),
        .done(quant_done),
        .busy(quant_busy)
    );

    FWHT_FSM_DMAX #(
        .DATA_W(DATA_W),
        .IN_W(IN_W),
        .D_MAX(D_MAX),
        .NUM_PE(NUM_PE),
        .INVERSE_POST_SHIFT_EN(0)
    ) u_dequant_fwht (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .start(dequant_start),
        .mode(dequant_mode),
        .din(dequant_din),
        .dout(dequant_dout),
        .valid_out(dequant_valid_out),
        .done(dequant_done),
        .busy(dequant_busy)
    );

endmodule