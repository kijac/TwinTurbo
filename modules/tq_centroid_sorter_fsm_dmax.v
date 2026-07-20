`timescale 1ns / 1ps
// ============================================================================
// tq_centroid_sorter_fsm_dmax.v
//
// D_MAX 길이의 FWHT 출력 vector를 8-lane 단위로 centroid sorter에 통과시킨다.
// Dimension은 D_MAX parameter로 고정된다. dim_mode/d_sel 포트 없음.
//
// Shared LUT version:
//   - 내부 LUT 없음
//   - top-level shared tq_lut_rom_imp에서 boundary_0~14를 받아 사용
//
// D_MAX=64  -> 8 batches
// D_MAX=128 -> 16 batches
// D_MAX=256 -> 32 batches
//
// din은 start부터 done까지 안정적으로 유지되어야 한다.
// ============================================================================
module tq_centroid_sorter_fsm_dmax #(
    parameter integer DATA_W   = 24,
    parameter integer D_MAX    = 128,
    parameter integer NUM_LANE = 8
)(
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        start,
    output wire                        ready,
    input  wire [DATA_W*D_MAX-1:0]     din,

    // Shared LUT boundary inputs
    input  wire signed [23:0] boundary_0,
    input  wire signed [23:0] boundary_1,
    input  wire signed [23:0] boundary_2,
    input  wire signed [23:0] boundary_3,
    input  wire signed [23:0] boundary_4,
    input  wire signed [23:0] boundary_5,
    input  wire signed [23:0] boundary_6,
    input  wire signed [23:0] boundary_7,
    input  wire signed [23:0] boundary_8,
    input  wire signed [23:0] boundary_9,
    input  wire signed [23:0] boundary_10,
    input  wire signed [23:0] boundary_11,
    input  wire signed [23:0] boundary_12,
    input  wire signed [23:0] boundary_13,
    input  wire signed [23:0] boundary_14,

    output reg  [4*NUM_LANE-1:0]       batch_code_vec,
    output reg                         batch_valid,
    input  wire                        batch_ready,
    output reg                         batch_last,

    output reg                         done,
    output wire                        busy
);
    localparam S_IDLE = 1'b0;
    localparam S_RUN  = 1'b1;

    localparam [7:0] LAST_IDX = D_MAX - NUM_LANE;

    reg       state;
    reg [DATA_W*D_MAX-1:0] hold_vec;
    reg [DATA_W*NUM_LANE-1:0] issue_sample_vec;
    reg [7:0] issue_idx;
    reg [7:0] capture_idx;
    reg       issue_valid;

    wire [DATA_W*NUM_LANE-1:0] current_sample_vec;
    wire [3:0] lane_idx_0, lane_idx_1, lane_idx_2, lane_idx_3;
    wire [3:0] lane_idx_4, lane_idx_5, lane_idx_6, lane_idx_7;
    wire       sorter_valid_nc;

    assign busy  = (state != S_IDLE);
    assign ready = ~busy & ~batch_valid;

    wire adv            = (~batch_valid) | batch_ready;
    wire batch_fire     = batch_valid & batch_ready;
    wire array_valid_in = issue_valid & adv;
    assign current_sample_vec = issue_sample_vec;

    tq_centroid_sorter_array u_sorter_array (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(array_valid_in),
        .y_in_0(current_sample_vec[0*DATA_W +: DATA_W]),
        .y_in_1(current_sample_vec[1*DATA_W +: DATA_W]),
        .y_in_2(current_sample_vec[2*DATA_W +: DATA_W]),
        .y_in_3(current_sample_vec[3*DATA_W +: DATA_W]),
        .y_in_4(current_sample_vec[4*DATA_W +: DATA_W]),
        .y_in_5(current_sample_vec[5*DATA_W +: DATA_W]),
        .y_in_6(current_sample_vec[6*DATA_W +: DATA_W]),
        .y_in_7(current_sample_vec[7*DATA_W +: DATA_W]),
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
        .idx_out_0(lane_idx_0),
        .idx_out_1(lane_idx_1),
        .idx_out_2(lane_idx_2),
        .idx_out_3(lane_idx_3),
        .idx_out_4(lane_idx_4),
        .idx_out_5(lane_idx_5),
        .idx_out_6(lane_idx_6),
        .idx_out_7(lane_idx_7),
        .valid_out(sorter_valid_nc)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) done <= 1'b0;
        else        done <= batch_fire & batch_last;
    end

    integer lane_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            hold_vec         <= {(DATA_W*D_MAX){1'b0}};
            issue_sample_vec <= {(DATA_W*NUM_LANE){1'b0}};
            batch_code_vec   <= {(4*NUM_LANE){1'b0}};
            batch_valid      <= 1'b0;
            batch_last       <= 1'b0;
            issue_idx        <= 8'd0;
            capture_idx      <= 8'd0;
            issue_valid      <= 1'b0;
        end
        else if (adv) begin
            batch_valid   <= 1'b0;
            batch_last    <= 1'b0;
            capture_idx   <= issue_idx;

            case (state)
                S_IDLE: begin
                    issue_valid <= 1'b0;
                    if (start) begin
                        hold_vec         <= din;
                        issue_sample_vec <= din[0 +: DATA_W*NUM_LANE];
                        issue_idx        <= 8'd0;
                        capture_idx      <= 8'd0;
                        issue_valid      <= 1'b1;
                        state            <= S_RUN;
                    end
                end

                S_RUN: begin
                    if (sorter_valid_nc) begin
                        batch_code_vec[0*4 +: 4] <= lane_idx_0;
                        batch_code_vec[1*4 +: 4] <= lane_idx_1;
                        batch_code_vec[2*4 +: 4] <= lane_idx_2;
                        batch_code_vec[3*4 +: 4] <= lane_idx_3;
                        batch_code_vec[4*4 +: 4] <= lane_idx_4;
                        batch_code_vec[5*4 +: 4] <= lane_idx_5;
                        batch_code_vec[6*4 +: 4] <= lane_idx_6;
                        batch_code_vec[7*4 +: 4] <= lane_idx_7;
                        batch_valid <= 1'b1;

                        if (capture_idx == LAST_IDX) begin
                            batch_last  <= 1'b1;
                            issue_valid <= 1'b0;
                            state       <= S_IDLE;
                        end
                    end

                    if (issue_valid) begin
                        if (issue_idx == LAST_IDX) issue_valid <= 1'b0;
                        else begin
                            issue_idx <= issue_idx + NUM_LANE;
                            for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1) begin
                                issue_sample_vec[lane_i*DATA_W +: DATA_W]
                                    <= hold_vec[(issue_idx + NUM_LANE + lane_i)*DATA_W +: DATA_W];
                            end
                        end
                    end
                end
            endcase
        end
    end
endmodule
