// ============================================================
// tq_bit_packer_rdy_dmax.v
//
// tq_bit_packer_imp2.v 의 backpressure 버전.
//
// 원본과의 차이:
//   1) + output norm_ready  (= ~busy)
//   2) + output batch_ready : beat 이 완성되는 사이클에 FIFO 여유가 없으면 0
//   3) push 조건을 room = (count<2) || m_tready 로 완화
//      -> count==2 & pop 인 사이클에도 push 가능 (case 2'b11 이 이미 처리)
//   4) in_final_path 를 room 으로 게이팅 -> flush 도 drop 되지 않음
//   => 정상 동작에서 drop 은 발생하지 않는다. drop_flag 는 안전망(sticky).
//
// D_MAX 방식에서의 동작:
//   batch 개수 = D_MAX/8 이고 PERIOD = AXI_DATA_W/BATCH_BITS = 4.
//   D_MAX=64/128/256 모두 4의 배수 batch이므로 accumulator 잔여 비트가 정확히
//   DATA_W(=20) 이고 phase 는 0 으로 돌아온다.
//   -> FINAL_KEEP = 3 byte 상수가 그대로 유효.
//
// 레코드 포맷 (LSB first):
//   [ norm(DATA_W=20b) | idx[0](4b) | ... | idx[D-1](4b) ]
//     D=64  : 276 bit  -> 128b beat x2 + 20b final beat (tkeep=3B)
//     D=128 : 532 bit  -> 128b beat x4 + 20b final beat
//     D=256 : 1044 bit -> 128b beat x8 + 20b final beat
// ============================================================
module tq_bit_packer_rdy_dmax #(
    parameter integer DATA_W     = 20,
    parameter integer IDX_W      = 4,
    parameter integer NUM_LANE   = 8,
    parameter integer D_MAX      = 128,
    parameter integer AXI_DATA_W = 128
)(
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          norm_valid,
    output wire                          norm_ready,
    input  wire [DATA_W-1:0]             norm_in,

    input  wire                          batch_valid,
    output wire                          batch_ready,
    input  wire [IDX_W*NUM_LANE-1:0]     batch_code_vec,
    input  wire                          batch_last,

    output wire                          m_tvalid,
    output wire [AXI_DATA_W-1:0]         m_tdata,
    output wire                          m_tlast,
    output wire [(AXI_DATA_W/8)-1:0]     m_tkeep,
    input  wire                          m_tready,

    output reg                           busy,
    output reg                           norm_ignored,
    output reg                           drop_flag
);
    localparam integer BATCH_BITS = IDX_W * NUM_LANE;
    localparam integer PERIOD     = AXI_DATA_W / BATCH_BITS;
    localparam integer PHASE_W    = (PERIOD <= 1) ? 1 : $clog2(PERIOD);
    localparam integer ACC_W      = AXI_DATA_W + BATCH_BITS;
    localparam integer KEEP_W     = AXI_DATA_W / 8;
    localparam integer TOTAL_BATCHES = D_MAX / NUM_LANE; // informational; batch_last controls the actual flush
    localparam [PHASE_W-1:0] LAST_PHASE = PERIOD - 1;

    localparam integer FINAL_VALID_BYTES = (DATA_W + 7) / 8;
    localparam [KEEP_W-1:0] FINAL_KEEP =
        {{(KEEP_W-FINAL_VALID_BYTES){1'b0}}, {FINAL_VALID_BYTES{1'b1}}};

    reg [ACC_W-1:0]   acc;
    reg [PHASE_W-1:0] phase;
    reg               final_flush_pending;

    reg [1:0]            count;
    reg [AXI_DATA_W-1:0] slot0_data, slot1_data;
    reg                  slot0_last, slot1_last;
    reg [KEEP_W-1:0]     slot0_keep, slot1_keep;

    wire [ACC_W-1:0] batch_ext = {{(ACC_W-BATCH_BITS){1'b0}}, batch_code_vec};
    wire [ACC_W-1:0] shifted_cand [0:PERIOD-1];

    genvar gp;
    generate
        for (gp = 0; gp < PERIOD; gp = gp + 1) begin : GEN_SHIFT
            assign shifted_cand[gp] = batch_ext << (DATA_W + gp*BATCH_BITS);
        end
    endgenerate

    wire [ACC_W-1:0] shifted_batch = shifted_cand[phase];
    wire [ACC_W-1:0] acc_after_add = acc | shifted_batch;
    wire             beat_complete = (phase == LAST_PHASE);

    wire pop  = (count != 2'd0) && m_tready;
    wire room = (count < 2'd2) || m_tready;   // 이번 사이클에 슬롯 여유가 생김

    wire accept_norm   = norm_valid && !busy;
    wire in_norm_path  = accept_norm;

    // norm 을 받는 사이클에는 batch 를 소비하지 않는다 (원본은 조용히 drop 했음)
    wire batch_can     = !in_norm_path && (!beat_complete || room);
    wire in_batch_path = batch_valid && batch_can;
    wire in_final_path = !in_norm_path && !batch_valid && final_flush_pending && room;

    assign norm_ready  = ~busy;
    assign batch_ready = batch_can;

    wire                  producer_valid = (in_batch_path && beat_complete) || in_final_path;
    wire [AXI_DATA_W-1:0] producer_data  = in_final_path ? acc[AXI_DATA_W-1:0]
                                                         : acc_after_add[AXI_DATA_W-1:0];
    wire                  producer_last  = in_final_path;
    wire [KEEP_W-1:0]     producer_keep  = in_final_path ? FINAL_KEEP : {KEEP_W{1'b1}};

    wire push = producer_valid && room;
    wire drop = producer_valid && !room;   // 정상 동작에서는 발생하지 않음

    assign m_tvalid = (count != 2'd0);
    assign m_tdata  = slot0_data;
    assign m_tlast  = slot0_last;
    assign m_tkeep  = slot0_keep;

    // ---------------- accumulator / busy ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc                 <= {ACC_W{1'b0}};
            phase               <= {PHASE_W{1'b0}};
            final_flush_pending <= 1'b0;
            busy                <= 1'b0;
            norm_ignored        <= 1'b0;
        end else begin
            norm_ignored <= 1'b0;

            if (in_norm_path) begin
                acc                 <= {{(ACC_W-DATA_W){1'b0}}, norm_in};
                phase               <= {PHASE_W{1'b0}};
                busy                <= 1'b1;
                final_flush_pending <= 1'b0;
            end
            else if (norm_valid && busy) begin
                norm_ignored <= 1'b1;      // top 이 norm_ready 를 지키면 안 뜬다
            end
            else if (in_batch_path) begin
                if (beat_complete) begin
                    acc   <= acc_after_add >> AXI_DATA_W;
                    phase <= {PHASE_W{1'b0}};
                end else begin
                    acc   <= acc_after_add;
                    phase <= phase + 1'b1;
                end
                if (batch_last) final_flush_pending <= 1'b1;
            end
            else if (in_final_path) begin
                acc                 <= {ACC_W{1'b0}};
                phase               <= {PHASE_W{1'b0}};
                final_flush_pending <= 1'b0;
                busy                <= 1'b1;   // final beat 이 pop 될 때까지 유지
            end
            else if (pop && slot0_last) begin
                busy <= 1'b0;
            end
        end
    end

    // ---------------- depth-2 output FIFO ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count      <= 2'd0;
            slot0_data <= {AXI_DATA_W{1'b0}};
            slot0_last <= 1'b0;
            slot0_keep <= {KEEP_W{1'b0}};
            slot1_data <= {AXI_DATA_W{1'b0}};
            slot1_last <= 1'b0;
            slot1_keep <= {KEEP_W{1'b0}};
            drop_flag  <= 1'b0;
        end else begin
            if (drop) drop_flag <= 1'b1;

            case ({push, pop})
                2'b01: begin
                    count <= count - 2'd1;
                    if (count == 2'd2) begin
                        slot0_data <= slot1_data;
                        slot0_last <= slot1_last;
                        slot0_keep <= slot1_keep;
                    end
                end
                2'b10: begin
                    count <= count + 2'd1;
                    if (count == 2'd0) begin
                        slot0_data <= producer_data;
                        slot0_last <= producer_last;
                        slot0_keep <= producer_keep;
                    end else begin
                        slot1_data <= producer_data;
                        slot1_last <= producer_last;
                        slot1_keep <= producer_keep;
                    end
                end
                2'b11: begin
                    if (count == 2'd1) begin
                        slot0_data <= producer_data;
                        slot0_last <= producer_last;
                        slot0_keep <= producer_keep;
                    end else if (count == 2'd2) begin
                        slot0_data <= slot1_data;
                        slot0_last <= slot1_last;
                        slot0_keep <= slot1_keep;
                        slot1_data <= producer_data;
                        slot1_last <= producer_last;
                        slot1_keep <= producer_keep;
                    end
                    // count == 1 -> 유지, count == 2 -> 유지
                end
                default: ;
            endcase
        end
    end
endmodule
