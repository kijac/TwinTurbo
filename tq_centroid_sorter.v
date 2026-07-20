`timescale 1ns / 1ps
// ============================================================
// tq_centroid_sorter.v
//
// TurboQuant 4bit Centroid Sorter (양자화 분류기)
//
// Shared LUT version:
//   - tq_centroid_sorter itself receives boundary_0~14 as input.
//   - tq_centroid_sorter_array no longer instantiates tq_lut_rom_imp.
//   - boundary values must be supplied from the top-level shared LUT.
//
// 동작:
//   RHT 출력 y'[j] (24bit Q8.15 signed)를
//   top shared LUT의 boundary 15개와 비교하여 4bit idx 출력
//
//   boundary 오름차순 정렬 → thermometer code
//   comparator 15개 병렬 → popcount = idx
//
// 레이턴시: 1사이클 (출력 레지스터)
// ============================================================

module tq_centroid_sorter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,

    // FWHT 출력 (Q8.15 24bit signed)
    input  wire signed [23:0] y_in,

    // Shared LUT에서 받은 boundary 15개
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

    output reg  [3:0]  idx_out,
    output reg         valid_out
);

    wire cmp_0  = ($signed(y_in) >= $signed(boundary_0));
    wire cmp_1  = ($signed(y_in) >= $signed(boundary_1));
    wire cmp_2  = ($signed(y_in) >= $signed(boundary_2));
    wire cmp_3  = ($signed(y_in) >= $signed(boundary_3));
    wire cmp_4  = ($signed(y_in) >= $signed(boundary_4));
    wire cmp_5  = ($signed(y_in) >= $signed(boundary_5));
    wire cmp_6  = ($signed(y_in) >= $signed(boundary_6));
    wire cmp_7  = ($signed(y_in) >= $signed(boundary_7));
    wire cmp_8  = ($signed(y_in) >= $signed(boundary_8));
    wire cmp_9  = ($signed(y_in) >= $signed(boundary_9));
    wire cmp_10 = ($signed(y_in) >= $signed(boundary_10));
    wire cmp_11 = ($signed(y_in) >= $signed(boundary_11));
    wire cmp_12 = ($signed(y_in) >= $signed(boundary_12));
    wire cmp_13 = ($signed(y_in) >= $signed(boundary_13));
    wire cmp_14 = ($signed(y_in) >= $signed(boundary_14));

    wire [3:0] idx_comb =
        cmp_0  + cmp_1  + cmp_2  + cmp_3  +
        cmp_4  + cmp_5  + cmp_6  + cmp_7  +
        cmp_8  + cmp_9  + cmp_10 + cmp_11 +
        cmp_12 + cmp_13 + cmp_14;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx_out   <= 4'd0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                idx_out <= idx_comb;
            end
        end
    end

endmodule


// ============================================================
// tq_centroid_sorter_array.v
//
// K=8개의 Sorter를 병렬로 묶은 배열
// Shared LUT version:
//   내부 tq_lut_rom_imp 제거
//   top-level shared LUT에서 boundary_0~14를 입력으로 받음
// ============================================================

module tq_centroid_sorter_array (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,

    // FWHT 출력 8개 (K=8 고정)
    input  wire signed [23:0] y_in_0,
    input  wire signed [23:0] y_in_1,
    input  wire signed [23:0] y_in_2,
    input  wire signed [23:0] y_in_3,
    input  wire signed [23:0] y_in_4,
    input  wire signed [23:0] y_in_5,
    input  wire signed [23:0] y_in_6,
    input  wire signed [23:0] y_in_7,

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

    // idx 출력 8개
    output wire [3:0] idx_out_0,
    output wire [3:0] idx_out_1,
    output wire [3:0] idx_out_2,
    output wire [3:0] idx_out_3,
    output wire [3:0] idx_out_4,
    output wire [3:0] idx_out_5,
    output wire [3:0] idx_out_6,
    output wire [3:0] idx_out_7,

    output wire        valid_out
);

    wire valid_arr [0:7];

    tq_centroid_sorter u_s0 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_0),
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
        .idx_out(idx_out_0),
        .valid_out(valid_arr[0])
    );

    tq_centroid_sorter u_s1 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_1),
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
        .idx_out(idx_out_1),
        .valid_out(valid_arr[1])
    );

    tq_centroid_sorter u_s2 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_2),
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
        .idx_out(idx_out_2),
        .valid_out(valid_arr[2])
    );

    tq_centroid_sorter u_s3 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_3),
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
        .idx_out(idx_out_3),
        .valid_out(valid_arr[3])
    );

    tq_centroid_sorter u_s4 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_4),
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
        .idx_out(idx_out_4),
        .valid_out(valid_arr[4])
    );

    tq_centroid_sorter u_s5 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_5),
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
        .idx_out(idx_out_5),
        .valid_out(valid_arr[5])
    );

    tq_centroid_sorter u_s6 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_6),
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
        .idx_out(idx_out_6),
        .valid_out(valid_arr[6])
    );

    tq_centroid_sorter u_s7 (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .y_in(y_in_7),
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
        .idx_out(idx_out_7),
        .valid_out(valid_arr[7])
    );

    assign valid_out = valid_arr[0];

endmodule


// ============================================================
// tq_centroid_sorter_fsm.v
//
// Old non-DMAX wrapper kept for compatibility.
// Shared LUT version: boundary_0~14 are external inputs.
// New top normally uses tq_centroid_sorter_fsm_dmax.
// ============================================================

module tq_centroid_sorter_fsm #(
    parameter DATA_W = 24,
    parameter D_MAX  = 64,
    parameter NUM_LANE = 8
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        start,
    input  wire [DATA_W*D_MAX-1:0]     din,

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
    output reg                         batch_last,
    output reg                         done,
    output wire                        busy
);

    localparam S_IDLE = 1'b0;
    localparam S_RUN  = 1'b1;

    reg state;
    reg [DATA_W*D_MAX-1:0] hold_vec;
    reg [DATA_W*NUM_LANE-1:0] issue_sample_vec;
    reg [7:0] issue_idx;
    reg [7:0] capture_idx;
    reg issue_valid;
    integer lane_i;

    wire [3:0] lane_idx_0;
    wire [3:0] lane_idx_1;
    wire [3:0] lane_idx_2;
    wire [3:0] lane_idx_3;
    wire [3:0] lane_idx_4;
    wire [3:0] lane_idx_5;
    wire [3:0] lane_idx_6;
    wire [3:0] lane_idx_7;
    wire sorter_valid;

    assign busy = (state != S_IDLE);

    tq_centroid_sorter_array u_sorter_array (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(issue_valid),
        .y_in_0(issue_sample_vec[0*DATA_W +: DATA_W]),
        .y_in_1(issue_sample_vec[1*DATA_W +: DATA_W]),
        .y_in_2(issue_sample_vec[2*DATA_W +: DATA_W]),
        .y_in_3(issue_sample_vec[3*DATA_W +: DATA_W]),
        .y_in_4(issue_sample_vec[4*DATA_W +: DATA_W]),
        .y_in_5(issue_sample_vec[5*DATA_W +: DATA_W]),
        .y_in_6(issue_sample_vec[6*DATA_W +: DATA_W]),
        .y_in_7(issue_sample_vec[7*DATA_W +: DATA_W]),
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
        .valid_out(sorter_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            hold_vec <= {DATA_W*D_MAX{1'b0}};
            issue_sample_vec <= {DATA_W*NUM_LANE{1'b0}};
            batch_code_vec <= {4*NUM_LANE{1'b0}};
            batch_valid <= 1'b0;
            batch_last <= 1'b0;
            issue_idx <= 8'd0;
            capture_idx <= 8'd0;
            issue_valid <= 1'b0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            batch_valid <= 1'b0;
            batch_last <= 1'b0;
            capture_idx <= issue_idx;

            case (state)
                S_IDLE: begin
                    issue_valid <= 1'b0;
                    if (start) begin
                        hold_vec <= din;
                        issue_sample_vec <= din[0 +: DATA_W*NUM_LANE];
                        issue_idx <= 8'd0;
                        capture_idx <= 8'd0;
                        issue_valid <= 1'b1;
                        state <= S_RUN;
                    end
                end

                S_RUN: begin
                    if (sorter_valid && (capture_idx == (D_MAX - NUM_LANE))) begin
                        batch_code_vec[0*4 +: 4] <= lane_idx_0;
                        batch_code_vec[1*4 +: 4] <= lane_idx_1;
                        batch_code_vec[2*4 +: 4] <= lane_idx_2;
                        batch_code_vec[3*4 +: 4] <= lane_idx_3;
                        batch_code_vec[4*4 +: 4] <= lane_idx_4;
                        batch_code_vec[5*4 +: 4] <= lane_idx_5;
                        batch_code_vec[6*4 +: 4] <= lane_idx_6;
                        batch_code_vec[7*4 +: 4] <= lane_idx_7;
                        batch_valid <= 1'b1;
                        batch_last <= 1'b1;
                        done <= 1'b1;
                        state <= S_IDLE;
                        issue_valid <= 1'b0;
                    end else if (sorter_valid) begin
                        batch_code_vec[0*4 +: 4] <= lane_idx_0;
                        batch_code_vec[1*4 +: 4] <= lane_idx_1;
                        batch_code_vec[2*4 +: 4] <= lane_idx_2;
                        batch_code_vec[3*4 +: 4] <= lane_idx_3;
                        batch_code_vec[4*4 +: 4] <= lane_idx_4;
                        batch_code_vec[5*4 +: 4] <= lane_idx_5;
                        batch_code_vec[6*4 +: 4] <= lane_idx_6;
                        batch_code_vec[7*4 +: 4] <= lane_idx_7;
                        batch_valid <= 1'b1;
                    end

                    if (issue_valid) begin
                        if (issue_idx == (D_MAX - NUM_LANE)) begin
                            issue_valid <= 1'b0;
                        end else begin
                            issue_idx <= issue_idx + NUM_LANE;
                            for (lane_i = 0; lane_i < NUM_LANE; lane_i = lane_i + 1) begin
                                issue_sample_vec[lane_i*DATA_W +: DATA_W] <= hold_vec[(issue_idx + NUM_LANE + lane_i)*DATA_W +: DATA_W];
                            end
                        end
                    end
                end
            endcase
        end
    end

endmodule
