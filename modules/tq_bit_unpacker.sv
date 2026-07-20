`timescale 1ns / 1ps

module tq_bit_unpacker #(
    parameter int DATA_W     = 20,
    parameter int IDX_W      = 4,
    parameter int LANE_OUT   = 64,
    parameter int AXI_DATA_W = 128,
    parameter int GAMMA_W    = 20
)(
    input  logic                         clk,
    input  logic                         rst_n,

    input  logic                         s_tvalid,
    input  logic [AXI_DATA_W-1:0]        s_tdata,
    input  logic                         s_tlast,
    input  logic [(AXI_DATA_W/8)-1:0]    s_tkeep,
    output logic                         s_tready,

    input  logic [1:0]                   dim_mode,

    output logic                         gamma_valid,
    output logic [GAMMA_W-1:0]           gamma,

    output logic                         idx_batch_valid,
    output logic [IDX_W*LANE_OUT-1:0]    idx_batch_vec,
    output logic                         idx_batch_last,

    output logic                         unpack_done,
    output logic                         busy,
    output logic                         dim_mode_mismatch
);

    localparam int OUT            = IDX_W * LANE_OUT;
    localparam int PERIOD_U       = OUT / AXI_DATA_W;
    localparam int PHASE_W        = (PERIOD_U <= 1) ? 1 : $clog2(PERIOD_U);
    localparam int BUF_W          = OUT + AXI_DATA_W;
    localparam int KEEP_W         = AXI_DATA_W / 8;
    localparam int FIRST_IDX_BITS = AXI_DATA_W - DATA_W;

    localparam logic [PHASE_W-1:0] LAST_PHASE = PERIOD_U - 1;

    assign s_tready = 1'b1;

    logic beat_fire;
    logic [KEEP_W-1:0] keep_sample;

    assign beat_fire   = s_tvalid & s_tready;
    assign keep_sample = s_tkeep;

    function automatic logic [7:0] get_num_batches(input logic [1:0] mode);
        begin
            case (mode)
                2'b00:   get_num_batches = 8'd1;  // D=64
                2'b01:   get_num_batches = 8'd2;  // D=128
                2'b10:   get_num_batches = 8'd4;  // D=256
                default: get_num_batches = 8'd2;
            endcase
        end
    endfunction

    logic [BUF_W-1:0]   buf_reg;
    logic [PHASE_W-1:0] phase;
    logic               first_beat;
    logic [7:0]         batch_cnt;
    logic [1:0]         dim_mode_latched;

    logic [BUF_W-1:0] beat_ext;
    logic [BUF_W-1:0] shifted_cand [0:PERIOD_U-1];

    assign beat_ext = {{(BUF_W-AXI_DATA_W){1'b0}}, s_tdata};

    genvar gp;
    generate
        for (gp = 0; gp < PERIOD_U; gp = gp + 1) begin : GEN_SHIFT
            assign shifted_cand[gp] = beat_ext << (FIRST_IDX_BITS + gp*AXI_DATA_W);
        end
    endgenerate

    logic [BUF_W-1:0] shifted_beat;
    logic [BUF_W-1:0] buf_after_add;

    assign shifted_beat  = shifted_cand[phase];
    assign buf_after_add = buf_reg | shifted_beat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_reg            <= '0;
            phase              <= '0;
            first_beat         <= 1'b1;
            batch_cnt          <= 8'd0;
            dim_mode_latched   <= 2'b01;
            busy               <= 1'b0;
            gamma_valid        <= 1'b0;
            gamma              <= '0;
            idx_batch_valid    <= 1'b0;
            idx_batch_vec      <= '0;
            idx_batch_last     <= 1'b0;
            unpack_done        <= 1'b0;
            dim_mode_mismatch  <= 1'b0;
        end
        else begin
            gamma_valid     <= 1'b0;
            idx_batch_valid <= 1'b0;
            idx_batch_last  <= 1'b0;
            unpack_done     <= 1'b0;

            if (beat_fire) begin
                if (first_beat) begin
                    gamma_valid <= 1'b1;
                    gamma       <= {{(GAMMA_W-DATA_W){1'b0}}, s_tdata[DATA_W-1:0]};

                    buf_reg <= {
                        {(BUF_W-FIRST_IDX_BITS){1'b0}},
                        s_tdata[AXI_DATA_W-1:DATA_W]
                    };

                    phase             <= '0;
                    first_beat        <= 1'b0;
                    busy              <= 1'b1;
                    batch_cnt         <= 8'd0;
                    dim_mode_latched  <= dim_mode;
                end
                else begin
                    if (phase == LAST_PHASE) begin
                        idx_batch_valid <= 1'b1;
                        idx_batch_vec   <= buf_after_add[OUT-1:0];

                        if (keep_sample != {KEEP_W{1'b1}}) begin
                            dim_mode_mismatch <= 1'b1;
                        end

                        buf_reg <= buf_after_add >> OUT;
                        phase   <= '0;

                        if (s_tlast) begin
                            idx_batch_last <= 1'b1;
                            unpack_done    <= 1'b1;
                            busy           <= 1'b0;
                            first_beat     <= 1'b1;
                            batch_cnt      <= 8'd0;

                            if ((batch_cnt + 8'd1) != get_num_batches(dim_mode_latched)) begin
                                dim_mode_mismatch <= 1'b1;
                            end
                        end
                        else begin
                            batch_cnt <= batch_cnt + 8'd1;
                        end
                    end
                    else begin
                        buf_reg <= buf_after_add;
                        phase   <= phase + 1'b1;

                        if (s_tlast) begin
                            dim_mode_mismatch <= 1'b1;
                            busy              <= 1'b0;
                            first_beat        <= 1'b1;
                            batch_cnt         <= 8'd0;
                        end
                    end
                end
            end
        end
    end

endmodule