`timescale 1ns / 1ps

// ============================================================
// centroid_transformer_shared.sv
//
// Shared-LUT version of centroid_transformer.
// - No internal tq_lut_rom instance.
// - 16 centroid16 values are supplied from top-level shared LUT.
// - Interface matches the current dequant_top_shared usage:
//     valid_in / ready_out / last_in / valid_out / ready_in / last_out
//
// Format:
//   idx_in_vec   : 4-bit index x LANES
//   y_hat_16_vec : 16-bit signed Q7.8 centroid x LANES
// ============================================================

module centroid_transformer_shared #(
    parameter int OUT_W = 16,
    parameter int IDX_W = 4,
    parameter int LANES = 64
)(
    input  logic clk,
    input  logic rst_n,

    input  logic valid_in,
    output logic ready_out,
    input  logic last_in,

    input  logic [LANES*IDX_W-1:0] idx_in_vec,

    output logic valid_out,
    input  logic ready_in,

    output logic last_out,
    output logic done_out,

    output logic signed [LANES*OUT_W-1:0] y_hat_16_vec,

    // Shared centroid LUT inputs from top-level tq_lut_rom_imp
    input  logic signed [15:0] centroid16_0,
    input  logic signed [15:0] centroid16_1,
    input  logic signed [15:0] centroid16_2,
    input  logic signed [15:0] centroid16_3,
    input  logic signed [15:0] centroid16_4,
    input  logic signed [15:0] centroid16_5,
    input  logic signed [15:0] centroid16_6,
    input  logic signed [15:0] centroid16_7,
    input  logic signed [15:0] centroid16_8,
    input  logic signed [15:0] centroid16_9,
    input  logic signed [15:0] centroid16_10,
    input  logic signed [15:0] centroid16_11,
    input  logic signed [15:0] centroid16_12,
    input  logic signed [15:0] centroid16_13,
    input  logic signed [15:0] centroid16_14,
    input  logic signed [15:0] centroid16_15
);

    logic signed [LANES*OUT_W-1:0] centroid_lookup_vec;
    integer i;

    function automatic logic signed [OUT_W-1:0] select_centroid16;
        input logic [IDX_W-1:0] idx;
        begin
            unique case (idx)
                4'd0:  select_centroid16 = centroid16_0;
                4'd1:  select_centroid16 = centroid16_1;
                4'd2:  select_centroid16 = centroid16_2;
                4'd3:  select_centroid16 = centroid16_3;
                4'd4:  select_centroid16 = centroid16_4;
                4'd5:  select_centroid16 = centroid16_5;
                4'd6:  select_centroid16 = centroid16_6;
                4'd7:  select_centroid16 = centroid16_7;
                4'd8:  select_centroid16 = centroid16_8;
                4'd9:  select_centroid16 = centroid16_9;
                4'd10: select_centroid16 = centroid16_10;
                4'd11: select_centroid16 = centroid16_11;
                4'd12: select_centroid16 = centroid16_12;
                4'd13: select_centroid16 = centroid16_13;
                4'd14: select_centroid16 = centroid16_14;
                4'd15: select_centroid16 = centroid16_15;
                default: select_centroid16 = '0;
            endcase
        end
    endfunction

    always_comb begin
        centroid_lookup_vec = '0;
        for (i = 0; i < LANES; i = i + 1) begin
            centroid_lookup_vec[i*OUT_W +: OUT_W]
                = select_centroid16(idx_in_vec[i*IDX_W +: IDX_W]);
        end
    end

    assign ready_out = (!valid_out) || ready_in;
    assign done_out  = valid_out && ready_in && last_out;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out    <= 1'b0;
            last_out     <= 1'b0;
            y_hat_16_vec <= '0;
        end
        else begin
            if (ready_out) begin
                if (valid_in) begin
                    y_hat_16_vec <= centroid_lookup_vec;
                    valid_out    <= 1'b1;
                    last_out     <= last_in;
                end
                else begin
                    valid_out <= 1'b0;
                    last_out  <= 1'b0;
                end
            end
        end
    end

endmodule
