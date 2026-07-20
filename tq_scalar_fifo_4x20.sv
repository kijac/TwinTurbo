`timescale 1ns / 1ps

module tq_scalar_fifo_4x20 #(
    parameter int DATA_W = 20,
    parameter int DEPTH  = 4
)(
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  in_valid,
    output logic                  in_ready,
    input  logic [DATA_W-1:0]     in_data,

    output logic                  out_valid,
    input  logic                  out_ready,
    output logic [DATA_W-1:0]     out_data,

    output logic [2:0]            count,
    output logic                  full,
    output logic                  empty
);

    localparam int PTR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam logic [2:0] DEPTH_COUNT = DEPTH[2:0];

    logic [DATA_W-1:0] mem [0:DEPTH-1];

    logic [PTR_W-1:0] wr_ptr;
    logic [PTR_W-1:0] rd_ptr;
    logic [2:0]       count_reg;

    logic push;
    logic pop;

    assign full  = (count_reg == DEPTH_COUNT);
    assign empty = (count_reg == 3'd0);

    assign in_ready  = !full;
    assign out_valid = !empty;

    assign push = in_valid  && in_ready;
    assign pop  = out_valid && out_ready;

    assign out_data = empty ? '0 : mem[rd_ptr];
    assign count    = count_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= '0;
            rd_ptr    <= '0;
            count_reg <= 3'd0;
        end
        else begin
            case ({push, pop})

                2'b10: begin
                    mem[wr_ptr] <= in_data;
                    wr_ptr      <= wr_ptr + 1'b1;
                    count_reg   <= count_reg + 1'b1;
                end

                2'b01: begin
                    rd_ptr    <= rd_ptr + 1'b1;
                    count_reg <= count_reg - 1'b1;
                end

                2'b11: begin
                    mem[wr_ptr] <= in_data;
                    wr_ptr      <= wr_ptr + 1'b1;
                    rd_ptr      <= rd_ptr + 1'b1;
                end

                default: begin
                    // no operation
                end

            endcase
        end
    end

endmodule