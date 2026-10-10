//==============================================================================
// Module: pid_deriv_filter.sv
// Description: Measurement-based derivative unit with 1st-order IIR low-pass filter.
//              Eliminates setpoint derivative kick and attenuates high-frequency noise.
//              Supports instantaneous filter state re-seeding for bumpless transfer.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_DERIV_FILTER_SV
`define PID_DERIV_FILTER_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_deriv_filter #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Measurement and Parameters
    input  logic signed [DATA_W-1:0] meas,          // Sensor feedback y[n] (Q16.16)
    input  logic signed [DATA_W-1:0] alpha,         // Filter coefficient alpha in [0, 1) (Q16.16)
    input  logic signed [DATA_W-1:0] kd_eff,        // Effective derivative gain Kd / Ts (Q16.16)

    // Bumpless Transfer Re-seed
    input  logic                     reseed_valid,  // Assert to pre-load filter with current measurement

    // Output
    output logic signed [DATA_W-1:0] d_term,        // Filtered derivative term D[n] (Q16.16)
    output logic signed [DATA_W-1:0] filter_state   // Current filtered measurement y_f[n]
);

    // Internal State Register: Filtered measurement y_f[n-1]
    logic signed [DATA_W-1:0] yf_reg;

    // Intermediate calculations
    logic signed [DATA_W-1:0] delta_y;
    logic signed [DATA_W-1:0] delta_filt;
    logic signed [DATA_W-1:0] yf_next;
    logic signed [DATA_W-1:0] delta_yf;
    logic signed [DATA_W-1:0] d_mult_out;

    // Delta between incoming measurement and previous filter state
    assign delta_y = meas - yf_reg;

    // Multiplier for IIR filter update: alpha * (meas - yf_reg)
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_filter_mult (
        .a(alpha),
        .b(delta_y),
        .p(delta_filt)
    );

    // Compute next filter state
    always_comb begin
        if (reseed_valid) begin
            yf_next  = meas;
            delta_yf = 32'sd0; // Zero change on bumpless handover
        end else begin
            yf_next  = yf_reg + delta_filt;
            delta_yf = yf_next - yf_reg;
        end
    end

    // Derivative multiplier: kd_eff * delta_yf
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_kd_mult (
        .a(kd_eff),
        .b(delta_yf),
        .p(d_mult_out)
    );

    // D-term is negative derivative: - (Kd_eff * delta_yf)
    assign d_term = -d_mult_out;
    assign filter_state = yf_reg;

    // Sequential Filter State Update
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            yf_reg <= FIX_ZERO;
        end else if (sample_valid) begin
            yf_reg <= yf_next;
        end
    end

endmodule : pid_deriv_filter

`endif // PID_DERIV_FILTER_SV
