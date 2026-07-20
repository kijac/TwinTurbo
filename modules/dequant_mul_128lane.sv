`timescale 1ns / 1ps

// ============================================================================
// 128-lane dequant multiplier
//
// Function:
//   y[i] = gamma * (x[i] / D)
//
// Instead of using a multiplier for 1/D, this module fuses the operation as:
//   product = x[i] * gamma
//   y[i]   = product >>> (IN_FRAC_W + log2(D))
//
// Supported dimension modes:
//   dim_mode = 2'b00 : D = 64
//   dim_mode = 2'b01 : D = 128
//   dim_mode = 2'b10 : D = 256
//
// Throughput with LANES=128:
//   D=64  : 1 compute cycle
//   D=128 : 1 compute cycle
//   D=256 : 2 compute cycles
//
// Fixed-point assumption:
//   x     : signed Q?.IN_FRAC_W
//           FWHT input comes from the centroid LUT, currently Q7.8.
//   gamma : unsigned GAMMA_W-bit Q?.FRAC_W
//   y     : signed OUT_DATA_W-bit Q?.FRAC_W after saturation
// ============================================================================

module dequant_mul_128lane #(
    parameter int DATA_W     = 24,
    parameter int OUT_DATA_W = 16,
    parameter int D_MAX      = 256,
    parameter int LANES      = 128,
    parameter int FRAC_W     = 12,
    parameter int IN_FRAC_W  = 8,
    parameter int GAMMA_W    = 20,
    parameter int ADDR_W     = 8
)(
    input  logic                            clk,
    input  logic                            rst_n,
    input  logic                            clear,

    input  logic                            start,
    output logic                            ready,
    input  logic [1:0]                      dim_mode,

    input  logic [DATA_W*D_MAX-1:0]         din,
    input  logic [GAMMA_W-1:0]              gamma,

    output logic [OUT_DATA_W*D_MAX-1:0]     dout,
    output logic                            valid_out,
    output logic                            done,
    output logic                            busy,
    output logic                            out_overflow
);

    // ------------------------------------------------------------------------
    // Width definitions
    // ------------------------------------------------------------------------
    localparam int LEN_W       = ADDR_W + 1;
    localparam int GAMMA_EXT_W = GAMMA_W + 1;
    localparam int PROD_W      = DATA_W + GAMMA_EXT_W;

    typedef enum logic {
        S_IDLE = 1'b0,
        S_RUN  = 1'b1
    } state_t;

    state_t state;

    assign busy  = (state != S_IDLE);
    assign ready = (state == S_IDLE);

    // ------------------------------------------------------------------------
    // Registered input vector and output vector
    // ------------------------------------------------------------------------
    logic [DATA_W*D_MAX-1:0]     din_reg;
    logic [OUT_DATA_W*D_MAX-1:0] dout_reg;
    logic [GAMMA_W-1:0]          gamma_reg;
    logic [1:0]                  dim_mode_reg;

    assign dout = dout_reg;

    logic [7:0]       group_cnt;
    logic [LEN_W-1:0] active_len_reg;
    logic [7:0]       active_groups_reg;
    logic [7:0]       dim_shift_reg;

    // ------------------------------------------------------------------------
    // Dimension decode
    // ------------------------------------------------------------------------
    function automatic logic [LEN_W-1:0] get_dim_len(input logic [1:0] mode);
        begin
            case (mode)
                2'b00:   get_dim_len = LEN_W'(64);
                2'b01:   get_dim_len = LEN_W'(128);
                2'b10:   get_dim_len = LEN_W'(256);
                default: get_dim_len = LEN_W'(128);
            endcase
        end
    endfunction

    function automatic logic [7:0] get_dim_shift(input logic [1:0] mode);
        begin
            case (mode)
                2'b00:   get_dim_shift = 8'd6; // 1/64
                2'b01:   get_dim_shift = 8'd7; // 1/128
                2'b10:   get_dim_shift = 8'd8; // 1/256
                default: get_dim_shift = 8'd7;
            endcase
        end
    endfunction

    function automatic logic [7:0] get_active_groups(input logic [1:0] mode);
        begin
            case (mode)
                2'b00:   get_active_groups = 8'd1; // D=64, one 128-lane group
                2'b01:   get_active_groups = 8'd1; // D=128, one 128-lane group
                2'b10:   get_active_groups = 8'd2; // D=256, two 128-lane groups
                default: get_active_groups = 8'd1;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // Saturation helpers
    // ------------------------------------------------------------------------
    function automatic logic [OUT_DATA_W-1:0] sat_to_out(
        input logic signed [PROD_W-1:0] value
    );
        logic signed [PROD_W-1:0] max_value;
        logic signed [PROD_W-1:0] min_value;
        begin
            max_value = {{(PROD_W-OUT_DATA_W){1'b0}}, {1'b0, {(OUT_DATA_W-1){1'b1}}}};
            min_value = {{(PROD_W-OUT_DATA_W){1'b1}}, {1'b1, {(OUT_DATA_W-1){1'b0}}}};

            if (value > max_value)
                sat_to_out = {1'b0, {(OUT_DATA_W-1){1'b1}}};
            else if (value < min_value)
                sat_to_out = {1'b1, {(OUT_DATA_W-1){1'b0}}};
            else
                sat_to_out = value[OUT_DATA_W-1:0];
        end
    endfunction

    function automatic logic is_overflow(
        input logic signed [PROD_W-1:0] value
    );
        logic signed [PROD_W-1:0] max_value;
        logic signed [PROD_W-1:0] min_value;
        begin
            max_value = {{(PROD_W-OUT_DATA_W){1'b0}}, {1'b0, {(OUT_DATA_W-1){1'b1}}}};
            min_value = {{(PROD_W-OUT_DATA_W){1'b1}}, {1'b1, {(OUT_DATA_W-1){1'b0}}}};
            is_overflow = (value > max_value) || (value < min_value);
        end
    endfunction

    // ------------------------------------------------------------------------
    // 128 parallel multiplier lanes
    // ------------------------------------------------------------------------
    logic [LEN_W-1:0] group_base;
    assign group_base = group_cnt * LANES;

    logic signed [GAMMA_EXT_W-1:0] gamma_signed;
    assign gamma_signed = {1'b0, gamma_reg};

    logic [7:0] total_shift;
    // product fractional bits = IN_FRAC_W + FRAC_W.
    // output fractional bits  = FRAC_W.
    // so the scale-conversion shift is IN_FRAC_W, then divide by D.
    assign total_shift = IN_FRAC_W + dim_shift_reg;

    logic [OUT_DATA_W*LANES-1:0] lane_sat_bus;
    logic [LANES-1:0]            lane_overflow;
    logic [LANES-1:0]            lane_active;

    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : GEN_DEQUANT_LANE
            logic [LEN_W-1:0] elem_idx;
            logic signed [DATA_W-1:0] lane_x;
            (* use_dsp = "yes" *) logic signed [PROD_W-1:0] lane_product;
            logic signed [PROD_W-1:0] lane_scaled;

            assign elem_idx = group_base + LEN_W'(g);
            assign lane_active[g] = (elem_idx < active_len_reg);

            assign lane_x       = din_reg[(elem_idx*DATA_W) +: DATA_W];
            assign lane_product = lane_x * gamma_signed;
            assign lane_scaled  = lane_product >>> total_shift;

            assign lane_sat_bus[(g*OUT_DATA_W) +: OUT_DATA_W] = sat_to_out(lane_scaled);
            assign lane_overflow[g] = lane_active[g] & is_overflow(lane_scaled);
        end
    endgenerate

    // Reduce overflow from active lanes of current 128-lane group.
    int r;
    logic batch_overflow;

    always_comb begin
        batch_overflow = 1'b0;
        for (r = 0; r < LANES; r = r + 1) begin
            batch_overflow = batch_overflow | lane_overflow[r];
        end
    end

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    int j;
    int abs_idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            din_reg           <= '0;
            dout_reg          <= '0;
            gamma_reg         <= '0;
            dim_mode_reg      <= 2'b01;
            group_cnt         <= 8'd0;
            active_len_reg    <= '0;
            active_groups_reg <= 8'd1;
            dim_shift_reg     <= 8'd7;
            valid_out         <= 1'b0;
            done              <= 1'b0;
            out_overflow      <= 1'b0;
        end
        else begin
            valid_out <= 1'b0;
            done      <= 1'b0;

            if (clear) begin
                state             <= S_IDLE;
                din_reg           <= '0;
                dout_reg          <= '0;
                gamma_reg         <= '0;
                dim_mode_reg      <= 2'b01;
                group_cnt         <= 8'd0;
                active_len_reg    <= '0;
                active_groups_reg <= 8'd1;
                dim_shift_reg     <= 8'd7;
                valid_out         <= 1'b0;
                done              <= 1'b0;
                out_overflow      <= 1'b0;
            end
            else begin
                case (state)
                    S_IDLE: begin
                        if (start) begin
                            din_reg           <= din;
                            dout_reg          <= '0;
                            gamma_reg         <= gamma;
                            dim_mode_reg      <= dim_mode;
                            active_len_reg    <= get_dim_len(dim_mode);
                            active_groups_reg <= get_active_groups(dim_mode);
                            dim_shift_reg     <= get_dim_shift(dim_mode);
                            group_cnt         <= 8'd0;
                            out_overflow      <= 1'b0;
                            state             <= S_RUN;
                        end
                    end

                    S_RUN: begin
                        // Store current 128-lane group into full packed output.
                        for (j = 0; j < LANES; j = j + 1) begin
                            abs_idx = group_cnt * LANES + j;

                            if (abs_idx < active_len_reg) begin
                                dout_reg[(abs_idx*OUT_DATA_W) +: OUT_DATA_W]
                                    <= lane_sat_bus[(j*OUT_DATA_W) +: OUT_DATA_W];
                            end
                        end

                        out_overflow <= out_overflow | batch_overflow;

                        if (group_cnt == (active_groups_reg - 8'd1)) begin
                            valid_out <= 1'b1;
                            done      <= 1'b1;
                            state     <= S_IDLE;
                        end
                        else begin
                            group_cnt <= group_cnt + 8'd1;
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
