`timescale 1ns / 1ps

// ============================================================================
// io_buffer_32lane_dmax
//  - 입력 : LANES(32) x DATA_W(16) / cycle, valid-ready
//  - 출력 : D_MAX x DATA_W packed vector, valid-ready
//  - D_MAX is fixed by parameter. No runtime dim_mode port.
//  - Set D_MAX to 64, 128, or 256 at module instantiation.
//
// BEATS = D_MAX / LANES
//   D_MAX=64  -> 2 beats
//   D_MAX=128 -> 4 beats
//   D_MAX=256 -> 8 beats
//
// out_ready means the downstream block has consumed the current full vector.
// ============================================================================
module io_buffer_32lane_dmax #(
    parameter integer DATA_W = 16,
    parameter integer D_MAX  = 128,
    parameter integer LANES  = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     clear,

    // ---- upstream : fp16_to_fixed_32lane ----
    input  wire                     in_valid,
    output wire                     in_ready,
    input  wire [LANES*DATA_W-1:0]  in_data,
    input  wire [LANES-1:0]         in_overflow,
    input  wire [LANES-1:0]         in_invalid,

    // ---- downstream : L2 normalizer or FWHT wrapper ----
    output wire                     out_valid,
    input  wire                     out_ready,
    output wire [D_MAX*DATA_W-1:0]  out_data,
    output wire                     out_error,

    output reg                      input_overrun,
    output wire                     busy
);
    localparam integer BEAT_W = LANES * DATA_W;
    localparam integer BEATS  = D_MAX / LANES;
    localparam integer BCNT_W = 4;

    reg [D_MAX*DATA_W-1:0] bank0, bank1;
    reg                    bank0_full, bank1_full;
    reg                    bank0_err,  bank1_err;

    reg              wr_bank, rd_bank;
    reg [BCNT_W-1:0] beat;
    reg              wr_err;

    wire cur_wr_full = wr_bank ? bank1_full : bank0_full;

    assign in_ready  = ~cur_wr_full;
    assign out_valid = rd_bank ? bank1_full : bank0_full;
    assign out_data  = rd_bank ? bank1      : bank0;
    assign out_error = rd_bank ? bank1_err  : bank0_err;
    assign busy      = bank0_full | bank1_full | (beat != {BCNT_W{1'b0}});

    wire wr_fire   = in_valid  & in_ready;
    wire rd_fire   = out_valid & out_ready;
    wire beat_err  = (|in_overflow) | (|in_invalid);
    wire last_beat = (beat == BEATS - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank0_full <= 1'b0;
            bank1_full <= 1'b0;
            bank0_err  <= 1'b0;
            bank1_err  <= 1'b0;
            wr_bank    <= 1'b0;
            rd_bank    <= 1'b0;
            beat       <= {BCNT_W{1'b0}};
            wr_err     <= 1'b0;
            input_overrun <= 1'b0;
            bank0 <= {(D_MAX*DATA_W){1'b0}};
            bank1 <= {(D_MAX*DATA_W){1'b0}};
        end else if (clear) begin
            bank0_full <= 1'b0;
            bank1_full <= 1'b0;
            bank0_err  <= 1'b0;
            bank1_err  <= 1'b0;
            wr_bank    <= 1'b0;
            rd_bank    <= 1'b0;
            beat       <= {BCNT_W{1'b0}};
            wr_err     <= 1'b0;
            input_overrun <= 1'b0;
        end else begin
            input_overrun <= in_valid & ~in_ready;

            if (wr_fire) begin
                if (wr_bank == 1'b0)
                    bank0[beat*BEAT_W +: BEAT_W] <= in_data;
                else
                    bank1[beat*BEAT_W +: BEAT_W] <= in_data;

                if (last_beat) begin
                    if (wr_bank == 1'b0) begin
                        bank0_full <= 1'b1;
                        bank0_err  <= wr_err | beat_err;
                    end else begin
                        bank1_full <= 1'b1;
                        bank1_err  <= wr_err | beat_err;
                    end
                    beat    <= {BCNT_W{1'b0}};
                    wr_err  <= 1'b0;
                    wr_bank <= ~wr_bank;
                end else begin
                    beat   <= beat + 1'b1;
                    wr_err <= wr_err | beat_err;
                end
            end

            if (rd_fire) begin
                if (rd_bank == 1'b0)
                    bank0_full <= 1'b0;
                else
                    bank1_full <= 1'b0;
                rd_bank <= ~rd_bank;
            end
        end
    end

endmodule
