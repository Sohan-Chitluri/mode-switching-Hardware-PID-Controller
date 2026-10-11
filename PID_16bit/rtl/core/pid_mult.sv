//==============================================================================
// Module: pid_mult.sv
// Description: Signed 32x32 fixed-point multiplier with Q16.16 scaling.
//              Computes: (a * b) >>> FRACT_W with two's complement arithmetic.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_MULT_SV
`define PID_MULT_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_mult #(
    parameter int DATA_W  = 16,
    parameter int FRACT_W = 12
) (
    input  logic signed [DATA_W-1:0] a,
    input  logic signed [DATA_W-1:0] b,
    output logic signed [DATA_W-1:0] p
);

    // Full-precision signed product
    logic signed [2*DATA_W-1:0] prod64;

    assign prod64 = a * b;

    // Signed overflow detection on discarded upper bits [2*DATA_W-1 : DATA_W-1+FRACT_W]
    // A signed product in Q(DATA_W).(FRACT_W) format fits in DATA_W bits if and only if
    // all upper bits from the product sign bit down to the result sign bit are identical.
    logic pos_ovf;
    logic neg_ovf;

    assign pos_ovf = !prod64[2*DATA_W-1] && (prod64[2*DATA_W-1 : DATA_W-1+FRACT_W] != '0);
    assign neg_ovf =  prod64[2*DATA_W-1] && (~prod64[2*DATA_W-1 : DATA_W-1+FRACT_W] != '0);

    always_comb begin
        if (pos_ovf) begin
            p = {1'b0, {(DATA_W-1){1'b1}}}; // Saturated MAX_POS
        end else if (neg_ovf) begin
            p = {1'b1, {(DATA_W-1){1'b0}}}; // Saturated MIN_NEG
        end else begin
            p = prod64[FRACT_W +: DATA_W];  // In-range bit-exact
        end
    end

endmodule : pid_mult

`endif // PID_MULT_SV
