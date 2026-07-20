`timescale 1ns / 1ps
// ============================================================================
// fp16_to_fixed_dmax.v
//  - Combinational FP16 -> signed fixed-point converter
//  - Pure Verilog version of fp16_to_fixed.sv
//  - No dimension control here. Dimension is handled by the 32-lane wrapper's
//    D_MAX parameter.
// ============================================================================
module fp16_to_fixed_dmax #(
    parameter integer DATA_W = 16,
    parameter integer FRAC_W = 12
)(
    input  wire [15:0] fp16_in,

    output reg signed [DATA_W-1:0] fixed_out,
    output reg                     overflow,
    output reg                     zero,
    output reg                     invalid
);

    reg        sign;
    reg [4:0]  exp;
    reg [9:0]  frac;

    reg [10:0] mantissa;
    reg signed [7:0] shift_amt;

    reg [31:0] mag;
    reg signed [31:0] signed_val;

    reg signed [31:0] max_val;
    reg signed [31:0] min_val;

    always @(*) begin
        sign = fp16_in[15];
        exp  = fp16_in[14:10];
        frac = fp16_in[9:0];

        fixed_out  = {DATA_W{1'b0}};
        overflow   = 1'b0;
        zero       = 1'b0;
        invalid    = 1'b0;

        mantissa   = 11'd0;
        shift_amt  = 8'sd0;
        mag        = 32'd0;
        signed_val = 32'sd0;

        max_val = (32'sd1 << (DATA_W-1)) - 32'sd1;
        min_val = -(32'sd1 << (DATA_W-1));

        // zero
        if (exp == 5'd0 && frac == 10'd0) begin
            fixed_out = {DATA_W{1'b0}};
            zero      = 1'b1;
        end

        // Inf or NaN
        else if (exp == 5'd31) begin
            overflow = 1'b1;
            invalid  = 1'b1;

            if (sign)
                fixed_out = {1'b1, {(DATA_W-1){1'b0}}}; // minimum negative
            else
                fixed_out = {1'b0, {(DATA_W-1){1'b1}}}; // maximum positive
        end

        else begin
            // normalized number
            if (exp != 5'd0) begin
                mantissa = {1'b1, frac};
                // fixed = mantissa * 2^(exp - 15 + FRAC_W - 10)
                shift_amt = $signed({3'b000, exp}) - 8'sd15 + FRAC_W - 8'sd10;
            end
            // subnormal number
            else begin
                mantissa = {1'b0, frac};
                // subnormal exponent = -14
                shift_amt = -8'sd14 + FRAC_W - 8'sd10;
            end

            // shift
            if (shift_amt >= 0)
                mag = {21'd0, mantissa} << shift_amt;
            else
                mag = {21'd0, mantissa} >> (-shift_amt);

            // sign apply
            if (sign)
                signed_val = -$signed({1'b0, mag[30:0]});
            else
                signed_val =  $signed({1'b0, mag[30:0]});

            // saturation
            if (signed_val > max_val) begin
                fixed_out = max_val[DATA_W-1:0];
                overflow  = 1'b1;
            end
            else if (signed_val < min_val) begin
                fixed_out = min_val[DATA_W-1:0];
                overflow  = 1'b1;
            end
            else begin
                fixed_out = signed_val[DATA_W-1:0];
            end
        end
    end

endmodule
