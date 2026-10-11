//==============================================================================
// Module: pid_sat_unit.sv
// Description: Output summation stage with 48-bit guarded accumulation (16 guard bits),
//              saturation clamping to [u_min, u_max], and clamp status flag generation.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_SAT_UNIT_SV
`define PID_SAT_UNIT_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_sat_unit #(
    parameter int DATA_W  = 16,
    parameter int ACCUM_W = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Feedback Terms (Q16.16)
    input  logic signed [DATA_W-1:0] p_term,
    input  logic signed [DATA_W-1:0] i_term,
    input  logic signed [DATA_W-1:0] d_term,

    // Configurable Saturation Bounds (Q16.16)
    input  logic signed [DATA_W-1:0] u_min,
    input  logic signed [DATA_W-1:0] u_max,

    // Saturated Output & Status
    output logic signed [DATA_W-1:0] u_sat,
    output logic signed [ACCUM_W-1:0] raw_sum,
    output logic                     clamp_high,
    output logic                     clamp_low
);

    // Guarded 48-bit summation (sign-extended operands)
    logic signed [ACCUM_W-1:0] p_ext;
    logic signed [ACCUM_W-1:0] i_ext;
    logic signed [ACCUM_W-1:0] d_ext;
    logic signed [ACCUM_W-1:0] sum48;

    logic signed [ACCUM_W-1:0] u_max_ext;
    logic signed [ACCUM_W-1:0] u_min_ext;

    assign p_ext = {{ (ACCUM_W-DATA_W){p_term[DATA_W-1]} }, p_term};
    assign i_ext = {{ (ACCUM_W-DATA_W){i_term[DATA_W-1]} }, i_term};
    assign d_ext = {{ (ACCUM_W-DATA_W){d_term[DATA_W-1]} }, d_term};

    assign sum48 = p_ext + i_ext + d_ext;
    assign raw_sum = sum48;

    assign u_max_ext = {{ (ACCUM_W-DATA_W){u_max[DATA_W-1]} }, u_max};
    assign u_min_ext = {{ (ACCUM_W-DATA_W){u_min[DATA_W-1]} }, u_min};

    // Combinational Saturation Logic
    always_comb begin
        if (sum48 >= u_max_ext) begin
            u_sat      = u_max;
            clamp_high = 1'b1;
            clamp_low  = 1'b0;
        end else if (sum48 <= u_min_ext) begin
            u_sat      = u_min;
            clamp_high = 1'b0;
            clamp_low  = 1'b1;
        end else begin
            u_sat      = sum48[DATA_W-1:0];
            clamp_high = 1'b0;
            clamp_low  = 1'b0;
        end
    end

`ifdef FORMAL
    always_comb begin
        if (u_min <= u_max) begin
            assert(u_sat >= u_min);
            assert(u_sat <= u_max);
        end
    end
`endif

endmodule : pid_sat_unit

`endif // PID_SAT_UNIT_SV
