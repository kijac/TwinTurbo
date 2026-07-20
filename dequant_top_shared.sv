`timescale 1ns / 1ps

module dequant_top_shared #(
    parameter int D_ACTIVE   = 128,

    parameter int AXI_DATA_W = 128,
    parameter int IDX_W      = 4,
    parameter int CT_LANES   = 64,

    parameter int IN_W       = 16,
    parameter int FWHT_W     = 24,
    parameter int FP16_W     = 16,

    parameter int MUL_LANES  = 128,
    parameter int FRAC_W     = 12,
    parameter int GAMMA_W    = 20,
    parameter int NUM_PE     = 32
)(
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         clear,

    // start is latched as stream enable. Use clear to stop/reset.
    input  logic                         start,
    output logic                         ready,
    output logic                         busy,

    input  logic                         s_tvalid,
    input  logic [AXI_DATA_W-1:0]        s_tdata,
    input  logic                         s_tlast,
    input  logic [(AXI_DATA_W/8)-1:0]    s_tkeep,
    output logic                         s_tready,

    output logic [FP16_W*D_ACTIVE-1:0]   fp16_out,
    output logic                         valid_out,
    output logic                         done,

    output logic                         unpack_error,
    output logic                         overflow,

    output logic [2:0]                   gamma_fifo_count,
    output logic                         gamma_fifo_full,
    output logic                         gamma_fifo_empty,

    // External shared centroid LUT inputs
    input  logic signed [15:0]           centroid16_0,
    input  logic signed [15:0]           centroid16_1,
    input  logic signed [15:0]           centroid16_2,
    input  logic signed [15:0]           centroid16_3,
    input  logic signed [15:0]           centroid16_4,
    input  logic signed [15:0]           centroid16_5,
    input  logic signed [15:0]           centroid16_6,
    input  logic signed [15:0]           centroid16_7,
    input  logic signed [15:0]           centroid16_8,
    input  logic signed [15:0]           centroid16_9,
    input  logic signed [15:0]           centroid16_10,
    input  logic signed [15:0]           centroid16_11,
    input  logic signed [15:0]           centroid16_12,
    input  logic signed [15:0]           centroid16_13,
    input  logic signed [15:0]           centroid16_14,
    input  logic signed [15:0]           centroid16_15,

    // External shared FWHT/RHT calculator interface
    output logic                         fwht_start,
    output logic                         fwht_mode,
    output logic [IN_W*D_ACTIVE-1:0]     fwht_din,

    input  logic [FWHT_W*D_ACTIVE-1:0]   fwht_dout,
    input  logic                         fwht_valid_out,
    input  logic                         fwht_done,
    input  logic                         fwht_busy
);

    localparam int D_MAX_ALL = 256;

    localparam logic [1:0] DIM_MODE =
        (D_ACTIVE == 64)  ? 2'b00 :
        (D_ACTIVE == 128) ? 2'b01 :
        (D_ACTIVE == 256) ? 2'b10 : 2'b01;

    localparam logic FWHT_FORWARD = 1'b0;
    localparam logic FWHT_INVERSE = 1'b1;

    localparam int NUM_BLOCKS = D_ACTIVE / CT_LANES;

    logic core_rst_n;
    assign core_rst_n = rst_n & ~clear;

    // start is latched, so one pulse enables streaming operation.
    logic run_enable;

    // ============================================================
    // Declarations used before busy assignment
    // ============================================================

    logic [1:0] cent_buf_valid;
    logic [1:0] cent_buf_busy;
    logic [1:0] cent_buf_ready;
    logic [1:0] cent_buf_free;

    logic fwht_out_valid;

    typedef enum logic [1:0] {
        P_IDLE     = 2'd0,
        P_MUL_RUN  = 2'd1,
        P_FP16_RUN = 2'd2
    } post_state_t;

    post_state_t post_state;

    assign cent_buf_ready = cent_buf_valid & ~cent_buf_busy;
    assign cent_buf_free  = ~(cent_buf_valid | cent_buf_busy);

    // ============================================================
    // Bit unpacker / front stage signals
    // ============================================================

    logic                         front_active;
    logic                         front_wr_sel;
    logic                         front_can_start;
    logic                         front_can_accept;

    logic                         unpack_s_tvalid;
    logic                         unpack_s_tready;

    logic                         unpack_gamma_valid;
    logic [GAMMA_W-1:0]           unpack_gamma;

    logic                         unpack_idx_valid;
    logic [IDX_W*CT_LANES-1:0]    unpack_idx_vec;
    logic                         unpack_idx_last;
    logic                         unpack_done;
    logic                         unpack_busy;
    logic                         unpack_dim_mode_mismatch;

    logic                         gamma_fifo_in_ready;
    logic                         gamma_fifo_out_valid;
    logic                         gamma_fifo_out_ready;
    logic [GAMMA_W-1:0]           gamma_fifo_out_data;

    assign front_can_start =
        run_enable &&
        !front_active &&
        !gamma_fifo_full &&
        (cent_buf_free != 2'b00);

    assign front_can_accept = front_active || front_can_start;

    assign s_tready        = front_can_accept && unpack_s_tready;
    assign unpack_s_tvalid = s_tvalid && s_tready;

    assign ready = front_can_start;

    assign busy = front_active ||
                  unpack_busy ||
                  (cent_buf_valid != 2'b00) ||
                  (cent_buf_busy  != 2'b00) ||
                  fwht_busy ||
                  fwht_out_valid ||
                  (post_state != P_IDLE);

    tq_bit_unpacker #(
        .DATA_W     (GAMMA_W),
        .IDX_W      (IDX_W),
        .LANE_OUT   (CT_LANES),
        .AXI_DATA_W (AXI_DATA_W),
        .GAMMA_W    (GAMMA_W)
    ) u_bit_unpacker (
        .clk               (clk),
        .rst_n             (core_rst_n),

        .s_tvalid          (unpack_s_tvalid),
        .s_tdata           (s_tdata),
        .s_tlast           (s_tlast),
        .s_tkeep           (s_tkeep),
        .s_tready          (unpack_s_tready),

        .dim_mode          (DIM_MODE),

        .gamma_valid       (unpack_gamma_valid),
        .gamma             (unpack_gamma),

        .idx_batch_valid   (unpack_idx_valid),
        .idx_batch_vec     (unpack_idx_vec),
        .idx_batch_last    (unpack_idx_last),

        .unpack_done       (unpack_done),
        .busy              (unpack_busy),

        .dim_mode_mismatch (unpack_dim_mode_mismatch)
    );

    assign unpack_error = unpack_dim_mode_mismatch;

    // ============================================================
    // Gamma FIFO
    // ============================================================

    tq_scalar_fifo_4x20 #(
        .DATA_W (GAMMA_W),
        .DEPTH  (4)
    ) u_gamma_fifo (
        .clk       (clk),
        .rst_n     (core_rst_n),

        .in_valid  (unpack_gamma_valid),
        .in_ready  (gamma_fifo_in_ready),
        .in_data   (unpack_gamma),

        .out_valid (gamma_fifo_out_valid),
        .out_ready (gamma_fifo_out_ready),
        .out_data  (gamma_fifo_out_data),

        .count     (gamma_fifo_count),
        .full      (gamma_fifo_full),
        .empty     (gamma_fifo_empty)
    );

    // ============================================================
    // Centroid transformer
    // ============================================================

    logic                         ct_start_in;
    logic                         ct_ready_out;
    logic                         ct_valid_out;
    logic                         ct_last_out;
    logic                         ct_done_out;
    logic signed [CT_LANES*IN_W-1:0] ct_y_hat_64;

    logic [2:0] ct_in_block_cnt;
    logic [2:0] ct_out_block_cnt;

    assign ct_start_in = unpack_idx_valid && (ct_in_block_cnt == 3'd0);

    centroid_transformer_shared #(
        .OUT_W (IN_W),
        .IDX_W (IDX_W),
        .LANES (CT_LANES)
    ) u_centroid_transformer (
        .clk          (clk),
        .rst_n        (core_rst_n),

        .valid_in     (unpack_idx_valid),
        .ready_out    (ct_ready_out),
        .last_in      (unpack_idx_last),

        .idx_in_vec   (unpack_idx_vec),

        .valid_out    (ct_valid_out),
        .ready_in     (1'b1),

        .last_out     (ct_last_out),
        .done_out     (ct_done_out),

        .y_hat_16_vec (ct_y_hat_64),

        .centroid16_0  (centroid16_0),
        .centroid16_1  (centroid16_1),
        .centroid16_2  (centroid16_2),
        .centroid16_3  (centroid16_3),
        .centroid16_4  (centroid16_4),
        .centroid16_5  (centroid16_5),
        .centroid16_6  (centroid16_6),
        .centroid16_7  (centroid16_7),
        .centroid16_8  (centroid16_8),
        .centroid16_9  (centroid16_9),
        .centroid16_10 (centroid16_10),
        .centroid16_11 (centroid16_11),
        .centroid16_12 (centroid16_12),
        .centroid16_13 (centroid16_13),
        .centroid16_14 (centroid16_14),
        .centroid16_15 (centroid16_15)
    );

    // ============================================================
    // Centroid double buffer
    // ============================================================

    logic signed [IN_W*D_ACTIVE-1:0] centroid_buf [0:1];

    logic fwht_start_fire;
    logic fwht_sel_comb;
    logic fwht_buf_sel;
    logic fwht_buf_active;

    always_comb begin
        if (cent_buf_ready[0]) begin
            fwht_sel_comb = 1'b0;
        end
        else begin
            fwht_sel_comb = 1'b1;
        end
    end

    // ============================================================
    // External shared FWHT/RHT calculator interface
    // ============================================================

    logic [IN_W*D_ACTIVE-1:0]      fwht_din_mux;

    // If FWHT done and next FWHT start happen in the same cycle,
    // the new start must see the newly selected ready buffer.
    // During normal FWHT processing, keep the input mux locked to fwht_buf_sel.
    assign fwht_din_mux =
        fwht_start_fire ? centroid_buf[fwht_sel_comb] :
        fwht_buf_active ? centroid_buf[fwht_buf_sel]  :
                          centroid_buf[fwht_sel_comb];

    assign fwht_start = fwht_start_fire;
    assign fwht_mode  = FWHT_INVERSE;
    assign fwht_din   = fwht_din_mux;

    // one-entry FWHT output holding register
    logic [FWHT_W*D_ACTIVE-1:0] fwht_out_reg;
    logic [FWHT_W*D_MAX_ALL-1:0] fwht_out_full;

    always_comb begin
        fwht_out_full = '0;
        fwht_out_full[FWHT_W*D_ACTIVE-1:0] = fwht_out_reg;
    end

    assign fwht_start_fire =
        !fwht_busy &&
        !fwht_out_valid &&
        (cent_buf_ready != 2'b00);

    // ============================================================
    // Dequant multiplier
    // ============================================================

    logic                         mul_start_fire;
    logic                         mul_ready;
    logic                         mul_busy;
    logic                         mul_valid_out;
    logic                         mul_done;
    logic                         mul_overflow;
    logic [FP16_W*D_MAX_ALL-1:0]  mul_dout_full;

    assign mul_start_fire =
        (post_state == P_IDLE) &&
        fwht_out_valid &&
        gamma_fifo_out_valid &&
        mul_ready;

    assign gamma_fifo_out_ready = mul_start_fire;

    dequant_mul_128lane #(
        .DATA_W     (FWHT_W),
        .OUT_DATA_W (FP16_W),
        .D_MAX      (D_MAX_ALL),
        .LANES      (MUL_LANES),
        .FRAC_W     (FRAC_W),
        .GAMMA_W    (GAMMA_W),
        .ADDR_W     (8)
    ) u_dequant_mul (
        .clk          (clk),
        .rst_n        (core_rst_n),
        .clear        (clear),

        .start        (mul_start_fire),
        .ready        (mul_ready),
        .dim_mode     (DIM_MODE),

        .din          (fwht_out_full),
        .gamma        (gamma_fifo_out_data),

        .dout         (mul_dout_full),
        .valid_out    (mul_valid_out),
        .done         (mul_done),
        .busy         (mul_busy),
        .out_overflow (mul_overflow)
    );

    assign overflow = mul_overflow;

    // ============================================================
    // Fixed-point to FP16
    // ============================================================

    logic                         fp16_start_fire;
    logic                         fp16_ready;
    logic                         fp16_busy;
    logic                         fp16_valid_out;
    logic                         fp16_done;
    logic [FP16_W*D_MAX_ALL-1:0]  fp16_dout_full;

    // Start fixed_to_fp16 as soon as multiplier done is visible.
    assign fp16_start_fire =
        (post_state == P_MUL_RUN) &&
        mul_done &&
        fp16_ready;

    fixed_to_fp16_128lane_pipe3 #(
        .DATA_W (FP16_W),
        .FRAC_W (FRAC_W),
        .D_MAX  (D_MAX_ALL),
        .LANES  (MUL_LANES)
    ) u_fixed_to_fp16 (
        .clk       (clk),
        .rst_n     (core_rst_n),
        .clear     (clear),

        .start     (fp16_start_fire),
        .ready     (fp16_ready),
        .busy      (fp16_busy),

        .dim_mode  (DIM_MODE),
        .fixed_in  (mul_dout_full),

        .valid_out (fp16_valid_out),
        .done      (fp16_done),
        .fp16_out  (fp16_dout_full)
    );

    assign fp16_out  = fp16_dout_full[FP16_W*D_ACTIVE-1:0];
    assign valid_out = fp16_valid_out;
    assign done      = fp16_done;

    // ============================================================
    // Main sequential control
    // ============================================================

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_enable      <= 1'b0;

            front_active    <= 1'b0;
            front_wr_sel    <= 1'b0;

            cent_buf_valid  <= 2'b00;
            cent_buf_busy   <= 2'b00;

            ct_in_block_cnt  <= 3'd0;
            ct_out_block_cnt <= 3'd0;

            centroid_buf[0] <= '0;
            centroid_buf[1] <= '0;

            fwht_buf_sel    <= 1'b0;
            fwht_buf_active <= 1'b0;

            fwht_out_reg    <= '0;
            fwht_out_valid  <= 1'b0;

            post_state       <= P_IDLE;
        end
        else begin
            if (clear) begin
                run_enable      <= 1'b0;

                front_active    <= 1'b0;
                front_wr_sel    <= 1'b0;

                cent_buf_valid  <= 2'b00;
                cent_buf_busy   <= 2'b00;

                ct_in_block_cnt  <= 3'd0;
                ct_out_block_cnt <= 3'd0;

                centroid_buf[0] <= '0;
                centroid_buf[1] <= '0;

                fwht_buf_sel    <= 1'b0;
                fwht_buf_active <= 1'b0;

                fwht_out_reg    <= '0;
                fwht_out_valid  <= 1'b0;

                post_state       <= P_IDLE;
            end
            else begin
                if (start) begin
                    run_enable <= 1'b1;
                end

                // ------------------------------------------------------------
                // Start accepting a new token.
                // Pick a truly free buffer: not valid and not busy.
                // ------------------------------------------------------------
                if (!front_active && front_can_start && s_tvalid && s_tready) begin
                    front_active <= 1'b1;

                    if (cent_buf_free[0]) begin
                        front_wr_sel <= 1'b0;
                    end
                    else begin
                        front_wr_sel <= 1'b1;
                    end

                    ct_in_block_cnt  <= 3'd0;
                    ct_out_block_cnt <= 3'd0;
                end

                // ------------------------------------------------------------
                // Count unpacked index batches
                // ------------------------------------------------------------
                if (unpack_idx_valid) begin
                    if (unpack_idx_last) begin
                        ct_in_block_cnt <= 3'd0;
                    end
                    else begin
                        ct_in_block_cnt <= ct_in_block_cnt + 3'd1;
                    end
                end

                // ------------------------------------------------------------
                // Store centroid output blocks into selected centroid buffer
                // ------------------------------------------------------------
                if (ct_valid_out) begin
                    centroid_buf[front_wr_sel]
                        [ct_out_block_cnt*CT_LANES*IN_W +: CT_LANES*IN_W]
                        <= ct_y_hat_64;

                    if (ct_last_out) begin
                        cent_buf_valid[front_wr_sel] <= 1'b1;
                        front_active                 <= 1'b0;
                        ct_out_block_cnt             <= 3'd0;
                    end
                    else begin
                        ct_out_block_cnt <= ct_out_block_cnt + 3'd1;
                    end
                end

                // ------------------------------------------------------------
                // Capture FWHT output and release the centroid buffer.
                // Important:
                // Do not free the buffer at FWHT start.
                // Release it only after FWHT done.
                // ------------------------------------------------------------
                if (fwht_done) begin
                    fwht_out_reg                 <= fwht_dout;
                    fwht_out_valid               <= 1'b1;

                    cent_buf_valid[fwht_buf_sel] <= 1'b0;
                    cent_buf_busy [fwht_buf_sel] <= 1'b0;
                    fwht_buf_active              <= 1'b0;
                end

                // ------------------------------------------------------------
                // FWHT starts when one centroid buffer is ready.
                // Mark selected buffer busy so CT/front cannot overwrite it.
                // This block intentionally comes after fwht_done block to allow
                // back-to-back FWHT done/start in the same clock.
                // ------------------------------------------------------------
                if (fwht_start_fire) begin
                    fwht_buf_sel                  <= fwht_sel_comb;
                    fwht_buf_active               <= 1'b1;
                    cent_buf_busy [fwht_sel_comb] <= 1'b1;
                end

                // ------------------------------------------------------------
                // Post stage: multiplier -> fixed_to_fp16
                // ------------------------------------------------------------
                case (post_state)

                    P_IDLE: begin
                        if (mul_start_fire) begin
                            fwht_out_valid <= 1'b0;
                            post_state     <= P_MUL_RUN;
                        end
                    end

                    P_MUL_RUN: begin
                        if (fp16_start_fire) begin
                            post_state <= P_FP16_RUN;
                        end
                    end

                    P_FP16_RUN: begin
                        if (fp16_done) begin
                            post_state <= P_IDLE;
                        end
                    end

                    default: begin
                        post_state <= P_IDLE;
                    end

                endcase
            end
        end
    end

endmodule