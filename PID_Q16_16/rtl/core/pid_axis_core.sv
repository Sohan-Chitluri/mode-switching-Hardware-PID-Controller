//==============================================================================
// Module: pid_axis_core.sv
// Description: Integrated single-channel PID feedback control engine.
//              Contains proportional unit, IIR filtered derivative unit,
//              anti-windup integrator, and 48-bit guarded output saturation stage.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_AXIS_CORE_SV
`define PID_AXIS_CORE_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"
`include "pid_deriv_filter.sv"
`include "pid_integ_unit.sv"
`include "pid_sat_unit.sv"

import pid_pkg::*;

module pid_axis_core #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16,
    parameter int ACCUM_W = 48
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Feedback Inputs (Q16.16)
    input  logic signed [DATA_W-1:0] setpoint,
    input  logic signed [DATA_W-1:0] measurement,

    // Channel Parameters (Q16.16)
    input  logic signed [DATA_W-1:0] kp,
    input  logic signed [DATA_W-1:0] ki_eff,
    input  logic signed [DATA_W-1:0] kd_eff,
    input  logic signed [DATA_W-1:0] alpha,

    // Configurable Saturation Bounds (Q16.16)
    input  logic signed [DATA_W-1:0] u_min,
    input  logic signed [DATA_W-1:0] u_max,
    input  logic signed [DATA_W-1:0] i_min,
    input  logic signed [DATA_W-1:0] i_max,

    // Bumpless Transfer Re-seeding Port
    input  logic                     reseed_valid,
    input  logic signed [DATA_W-1:0] reseed_val,

    // Output Signals
    output logic signed [DATA_W-1:0] u_sat,
    output logic signed [DATA_W-1:0] error_out,
    output logic signed [DATA_W-1:0] p_term_out,
    output logic signed [DATA_W-1:0] i_term_out,
    output logic signed [DATA_W-1:0] d_term_out,
    output logic signed [ACCUM_W-1:0] raw_sum_out,
    output logic                     clamp_high_out,
    output logic                     clamp_low_out
);

    // Error evaluation
    logic signed [DATA_W-1:0] error;
    assign error = setpoint - measurement;
    assign error_out = error;

    // Proportional Term
    logic signed [DATA_W-1:0] p_term;
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_kp_mult (
        .a(kp),
        .b(error),
        .p(p_term)
    );
    assign p_term_out = p_term;

    // Derivative Unit with IIR Filter
    logic signed [DATA_W-1:0] d_term;
    logic signed [DATA_W-1:0] filter_state;

    pid_deriv_filter #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_deriv (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .meas(measurement),
        .alpha(alpha),
        .kd_eff(kd_eff),
        .reseed_valid(reseed_valid),
        .d_term(d_term),
        .filter_state(filter_state)
    );
    assign d_term_out = d_term;

    // Clamp flag registers (fed back to anti-windup for NEXT sample)
    logic clamp_high_comb;
    logic clamp_low_comb;
    logic clamp_high_r;
    logic clamp_low_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            clamp_high_r <= 1'b0;
            clamp_low_r  <= 1'b0;
        end else if (sample_valid) begin
            clamp_high_r <= clamp_high_comb;
            clamp_low_r  <= clamp_low_comb;
        end
    end

    // Integrator Unit with Anti-Windup Clamping
    logic signed [DATA_W-1:0] i_term;

    pid_integ_unit #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_integ (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .error(error),
        .ki_eff(ki_eff),
        .i_min(i_min),
        .i_max(i_max),
        .clamp_high(clamp_high_r),
        .clamp_low(clamp_low_r),
        .reseed_valid(reseed_valid),
        .reseed_val(reseed_val),
        .i_term(i_term)
    );
    assign i_term_out = i_term;

    // Output Summation and Saturation
    pid_sat_unit #(
        .DATA_W(DATA_W),
        .ACCUM_W(ACCUM_W)
    ) u_pid_sat (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .p_term(p_term),
        .i_term(i_term),
        .d_term(d_term),
        .u_min(u_min),
        .u_max(u_max),
        .u_sat(u_sat),
        .raw_sum(raw_sum_out),
        .clamp_high(clamp_high_comb),
        .clamp_low(clamp_low_comb)
    );

    assign clamp_high_out = clamp_high_comb;
    assign clamp_low_out  = clamp_low_comb;

endmodule : pid_axis_core

`endif // PID_AXIS_CORE_SV
