//==============================================================================
// Module: pid_axis_core_2dof.sv
// Description: Two-Degree-of-Freedom (2-DOF) Single-Channel PID Controller.
//              Provides independent tuning of setpoint tracking response
//              and load disturbance rejection via programmable setpoint weights:
//                e_p[n] = b * r[n] - y[n]   (Proportional setpoint weight b in [0.0, 1.0])
//                e_d[n] = c * r[n] - y[n]   (Derivative setpoint weight c in [0.0, 1.0])
//
// When b = 1.0 and c = 0.0, this exactly reproduces classical 1-DOF PID
// with derivative-on-measurement.
//
// Arithmetic: Signed Fixed-Point with Symmetric Saturation & Clamping Anti-Windup.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_AXIS_CORE_2DOF_SV
`define PID_AXIS_CORE_2DOF_SV

`include "pid_pkg.sv"
`include "rtl/core/pid_mult.sv"
`include "rtl/core/pid_deriv_filter.sv"
`include "rtl/core/pid_integ_unit.sv"
`include "rtl/core/pid_sat_unit.sv"

import pid_pkg::*;

module pid_axis_core_2dof #(
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

    // 2-DOF Setpoint Weighting Coefficients (Q16.16, range [0.0, 1.0])
    // Defaults: weight_b = 1.0 (FIX_ONE), weight_c = 0.0 (FIX_ZERO)
    input  logic signed [DATA_W-1:0] weight_b,
    input  logic signed [DATA_W-1:0] weight_c,

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

    // Standard tracking error (for telemetry)
    logic signed [DATA_W-1:0] error_std;
    assign error_std = setpoint - measurement;
    assign error_out = error_std;

    // 2-DOF Proportional Setpoint Weighting: b * r[n]
    logic signed [DATA_W-1:0] b_r;
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_b_mult (
        .a(weight_b),
        .b(setpoint),
        .p(b_r)
    );

    // Proportional Error: e_p = b * r - y
    logic signed [DATA_W-1:0] error_p;
    assign error_p = b_r - measurement;

    // Proportional Term: P = Kp * e_p
    logic signed [DATA_W-1:0] p_term;
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_kp_mult (
        .a(kp),
        .b(error_p),
        .p(p_term)
    );
    assign p_term_out = p_term;

    // 2-DOF Derivative Setpoint Weighting: c * r[n]
    logic signed [DATA_W-1:0] c_r;
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_c_mult (
        .a(weight_c),
        .b(setpoint),
        .p(c_r)
    );

    // Effective derivative input signal:
    // If c == 0: meas_for_deriv = measurement (derivative purely on measurement)
    // If c > 0: meas_for_deriv = measurement - c_r (derivative on c*r - y)
    logic signed [DATA_W-1:0] meas_for_deriv;
    assign meas_for_deriv = (weight_c == FIX_ZERO) ? measurement : (measurement - c_r);

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
        .meas(meas_for_deriv),
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

    // Integral Unit with Anti-Windup Clamping & Bumpless Re-seeding
    logic signed [DATA_W-1:0] i_term;
    pid_integ_unit #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_integ (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .error(error_std), // Integral always integrates true error (r - y)
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

    // 48-bit Guarded Summation & Saturation Unit
    pid_sat_unit #(
        .DATA_W(DATA_W),
        .ACCUM_W(ACCUM_W)
    ) u_sat_stage (
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

endmodule : pid_axis_core_2dof

`endif // PID_AXIS_CORE_2DOF_SV
