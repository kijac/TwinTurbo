`timescale 1ns / 1ps
// ============================================================================
// fp16_to_fixed_32lane_dmax.v
//
// 32-lane FP16 -> fixed-point converter.
// D is fixed by parameter D_MAX. There is no runtime d_sel/dim_mode input.
//
// D_MAX=64  -> TOTAL_BLOCKS=2
// D_MAX=128 -> TOTAL_BLOCKS=4
// D_MAX=256 -> TOTAL_BLOCKS=8
//
// start_in should be asserted with valid_in on the first 32-lane beat of a new
// vector. The module still works as a simple block counter when continuous
// vectors are provided in order.
// ============================================================================
module fp16_to_fixed_32lane_dmax #(
    parameter integer DATA_W = 16,
    parameter integer FRAC_W = 12,
    parameter integer LANES  = 32,
    parameter integer D_MAX  = 128
)(
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          valid_in,
    output wire                          ready_out,
    input  wire                          start_in,

    input  wire [LANES*16-1:0]           fp16_in_vec,

    output reg                           valid_out,
    input  wire                          ready_in,

    output reg                           last_out,
    output wire                          done_out,

    output reg  signed [LANES*DATA_W-1:0] fixed_out_vec,
    output reg  [LANES-1:0]              overflow_vec,
    output reg  [LANES-1:0]              zero_vec,
    output reg  [LANES-1:0]              invalid_vec
);

    localparam integer TOTAL_BLOCKS = D_MAX / LANES;
    localparam integer BCNT_W       = 4;

    reg [BCNT_W-1:0] block_cnt;
    wire [BCNT_W-1:0] cur_block = start_in ? {BCNT_W{1'b0}} : block_cnt;
    wire [BCNT_W-1:0] last_block = TOTAL_BLOCKS - 1;

    wire signed [DATA_W-1:0] fixed_comb [0:LANES-1];
    wire overflow_comb [0:LANES-1];
    wire zero_comb     [0:LANES-1];
    wire invalid_comb  [0:LANES-1];

    genvar g;
    integer i;

    assign ready_out = (!valid_out) || ready_in;
    assign done_out  = valid_out && ready_in && last_out;

    generate
        for (g = 0; g < LANES; g = g + 1) begin : GEN_FP16_TO_FIXED
            fp16_to_fixed_dmax #(
                .DATA_W(DATA_W),
                .FRAC_W(FRAC_W)
            ) u_fp16_to_fixed (
                .fp16_in   (fp16_in_vec[g*16 +: 16]),
                .fixed_out (fixed_comb[g]),
                .overflow  (overflow_comb[g]),
                .zero      (zero_comb[g]),
                .invalid   (invalid_comb[g])
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out     <= 1'b0;
            last_out      <= 1'b0;
            block_cnt     <= {BCNT_W{1'b0}};
            fixed_out_vec <= {(LANES*DATA_W){1'b0}};
            overflow_vec  <= {LANES{1'b0}};
            zero_vec      <= {LANES{1'b0}};
            invalid_vec   <= {LANES{1'b0}};
        end
        else begin
            if (ready_out) begin
                if (valid_in) begin
                    for (i = 0; i < LANES; i = i + 1) begin
                        fixed_out_vec[i*DATA_W +: DATA_W] <= fixed_comb[i];
                        overflow_vec[i] <= overflow_comb[i];
                        zero_vec[i]     <= zero_comb[i];
                        invalid_vec[i]  <= invalid_comb[i];
                    end

                    valid_out <= 1'b1;
                    last_out  <= (cur_block == last_block);

                    if (cur_block == last_block)
                        block_cnt <= {BCNT_W{1'b0}};
                    else
                        block_cnt <= cur_block + 1'b1;
                end
                else begin
                    valid_out <= 1'b0;
                    last_out  <= 1'b0;
                end
            end
        end
    end

endmodule
