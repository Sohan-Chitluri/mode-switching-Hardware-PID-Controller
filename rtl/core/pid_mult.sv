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
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16
) (
    input  logic signed [DATA_W-1:0] a,
    input  logic signed [DATA_W-1:0] b,
    output logic signed [DATA_W-1:0] p
);

    // Full 64-bit signed product (Q32.32)
    logic signed [2*DATA_W-1:0] prod64;

    assign prod64 = a * b;

    // Rescale by arithmetic right-shift by FRACT_W
    // Take lower 32 bits of the shifted result (Q32.16 -> Q16.16)
    assign p = prod64[FRACT_W +: DATA_W];

endmodule : pid_mult

`endif // PID_MULT_SV
