`timescale 1ns / 1ps

module fixed_to_fp16_128lane_pipe3 #(
    parameter int DATA_W = 16,
    parameter int FRAC_W = 12,
    parameter int D_MAX  = 256,
    parameter int LANES  = 128
)(
    input  logic clk,
    input  logic rst_n,
    input  logic clear,

    input  logic start,
    output logic ready,
    output logic busy,

    input  logic [1:0] dim_mode,

    input  logic [DATA_W*D_MAX-1:0] fixed_in,

    output logic valid_out,
    output logic done,
    output logic [16*D_MAX-1:0] fp16_out
);

    localparam int LEN_W = $clog2(D_MAX + 1);

    localparam logic [1:0] S_IDLE  = 2'd0;
    localparam logic [1:0] S_FEED  = 2'd1;
    localparam logic [1:0] S_DRAIN = 2'd2;

    logic [1:0] state;

    assign ready = (state == S_IDLE);
    assign busy  = (state != S_IDLE);

    // ------------------------------------------------------------------------
    // Dimension decode
    // ------------------------------------------------------------------------
    function automatic [LEN_W-1:0] get_dim_len;
        input logic [1:0] mode;
        begin
            case (mode)
                2'b00: get_dim_len = LEN_W'(64);
                2'b01: get_dim_len = LEN_W'(128);
                2'b10: get_dim_len = LEN_W'(256);
                default: get_dim_len = LEN_W'(128);
            endcase
        end
    endfunction

    function automatic [7:0] get_active_groups;
        input logic [1:0] mode;
        begin
            case (mode)
                2'b00: get_active_groups = 8'd1;
                2'b01: get_active_groups = 8'd1;
                2'b10: get_active_groups = 8'd2;
                default: get_active_groups = 8'd1;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------------
    logic [DATA_W*D_MAX-1:0] fixed_reg;
    logic [16*D_MAX-1:0]     fp16_reg;

    assign fp16_out = fp16_reg;

    logic [LEN_W-1:0] active_len_reg;
    logic [7:0]       active_groups_reg;

    logic [7:0] feed_cnt;
    logic [7:0] out_cnt;

    // ------------------------------------------------------------------------
    // 128-lane converter input
    // ------------------------------------------------------------------------
    logic conv_valid_in;
    logic [DATA_W*LANES-1:0] conv_fixed_in;

    logic [7:0]       feed_group_idx;
    logic [LEN_W-1:0] feed_active_len;
    logic [DATA_W*D_MAX-1:0] fixed_src;

    assign conv_valid_in =
        ((state == S_IDLE) && start) ||
        (state == S_FEED);

    assign feed_group_idx =
        (state == S_IDLE) ? 8'd0 : feed_cnt;

    assign feed_active_len =
        (state == S_IDLE) ? get_dim_len(dim_mode) : active_len_reg;

    assign fixed_src =
        (state == S_IDLE) ? fixed_in : fixed_reg;

    integer c;
    integer feed_abs_idx;

    always_comb begin
        conv_fixed_in = '0;

        for (c = 0; c < LANES; c = c + 1) begin
            feed_abs_idx = feed_group_idx * LANES + c;

            if (feed_abs_idx < feed_active_len) begin
                conv_fixed_in[(c*DATA_W) +: DATA_W]
                    = fixed_src[(feed_abs_idx*DATA_W) +: DATA_W];
            end
        end
    end

    // ------------------------------------------------------------------------
    // 128 parallel 1-lane converters
    // ------------------------------------------------------------------------
    logic [LANES-1:0] lane_valid_out;
    logic [16*LANES-1:0] lane_fp16_out;

    logic [LANES-1:0] lane_zero;
    logic [LANES-1:0] lane_overflow;
    logic [LANES-1:0] lane_underflow;

    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : GEN_FIXED_TO_FP16_LANE

            logic signed [DATA_W-1:0] lane_fixed_in;

            assign lane_fixed_in = conv_fixed_in[(g*DATA_W) +: DATA_W];

            fixed_to_fp16_pipe3 #(
                .DATA_W(DATA_W),
                .FRAC_W(FRAC_W)
            ) u_fixed_to_fp16_pipe3 (
                .clk       (clk),
                .rst_n     (rst_n),

                .valid_in  (conv_valid_in),
                .fixed_in  (lane_fixed_in),

                .valid_out (lane_valid_out[g]),
                .fp16_out  (lane_fp16_out[(g*16) +: 16]),

                .zero      (lane_zero[g]),
                .overflow  (lane_overflow[g]),
                .underflow (lane_underflow[g])
            );

        end
    endgenerate

    wire conv_valid_out;
    assign conv_valid_out = lane_valid_out[0];

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    integer j;
    integer out_abs_idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            fixed_reg         <= '0;
            fp16_reg          <= '0;
            active_len_reg    <= '0;
            active_groups_reg <= 8'd1;
            feed_cnt          <= 8'd0;
            out_cnt           <= 8'd0;
            valid_out         <= 1'b0;
            done              <= 1'b0;
        end
        else begin
            valid_out <= 1'b0;
            done      <= 1'b0;

            if (clear) begin
                state             <= S_IDLE;
                fixed_reg         <= '0;
                fp16_reg          <= '0;
                active_len_reg    <= '0;
                active_groups_reg <= 8'd1;
                feed_cnt          <= 8'd0;
                out_cnt           <= 8'd0;
                valid_out         <= 1'b0;
                done              <= 1'b0;
            end
            else begin
                case (state)

                    S_IDLE: begin
                        if (start) begin
                            fixed_reg         <= fixed_in;
                            fp16_reg          <= '0;
                            active_len_reg    <= get_dim_len(dim_mode);
                            active_groups_reg <= get_active_groups(dim_mode);

                            out_cnt <= 8'd0;

                            // start cycle에 group0은 이미 conv_valid_in으로 들어감.
                            // D=64/128은 group0 하나뿐이라 바로 drain.
                            // D=256은 다음 cycle에 group1을 feed.
                            if (get_active_groups(dim_mode) == 8'd1) begin
                                feed_cnt <= 8'd0;
                                state    <= S_DRAIN;
                            end
                            else begin
                                feed_cnt <= 8'd1;
                                state    <= S_FEED;
                            end
                        end
                    end

                    S_FEED: begin
                        // 여기서는 D=256의 group1을 feed하는 cycle
                        if (feed_cnt == active_groups_reg - 1'b1) begin
                            state <= S_DRAIN;
                        end
                        else begin
                            feed_cnt <= feed_cnt + 1'b1;
                        end
                    end

                    S_DRAIN: begin
                        if (conv_valid_out) begin
                            for (j = 0; j < LANES; j = j + 1) begin
                                out_abs_idx = out_cnt * LANES + j;

                                if (out_abs_idx < active_len_reg) begin
                                    fp16_reg[(out_abs_idx*16) +: 16]
                                        <= lane_fp16_out[(j*16) +: 16];
                                end
                            end

                            if (out_cnt == active_groups_reg - 1'b1) begin
                                valid_out <= 1'b1;
                                done      <= 1'b1;
                                state     <= S_IDLE;
                            end
                            else begin
                                out_cnt <= out_cnt + 1'b1;
                            end
                        end
                    end

                    default: begin
                        state <= S_IDLE;
                    end

                endcase
            end
        end
    end

endmodule