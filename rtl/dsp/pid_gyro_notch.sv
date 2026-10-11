//==============================================================================
// Module: pid_gyro_notch.sv
// Description: Synthesizable 3-Axis 2nd-Order Direct Form II Transposed IIR
//              Biquad Notch Filter for UAV Gyroscope Vibration Suppression.
//
// Attenuates motor blade-pass harmonics and airframe mechanical resonance
// (typically 150 Hz - 400 Hz) before angular rates enter the inner rate loop.
//
// Difference Equations (Direct Form II Transposed):
//   y[n]    = b0 * x[n] + s1[n-1]
//   s1[n]   = b1 * x[n] - a1 * y[n] + s2[n-1]
//   s2[n]   = b2 * x[n] - a2 * y[n]
//
// Arithmetic: Signed Fixed-Point with Symmetric Saturation Clamping.
//==============================================================================

`ifndef PID_GYRO_NOTCH_SV
`define PID_GYRO_NOTCH_SV

`include "pid_pkg.sv"
import pid_pkg::*;

module pid_gyro_notch #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16,
    parameter int ACCUM_W = 48
) (
    input  logic                     clk,
    input  logic                     rst_n,

    // Enable / Bypass Control
    input  logic                     filter_enable, // 1 = filter active, 0 = hardware bypass
    input  logic                     sample_valid,

    // Configurable Normalized Filter Coefficients (Signed Q(DATA_W-FRACT_W).FRACT_W)
    input  logic signed [DATA_W-1:0] coeff_b0,
    input  logic signed [DATA_W-1:0] coeff_b1,
    input  logic signed [DATA_W-1:0] coeff_b2,
    input  logic signed [DATA_W-1:0] coeff_a1,
    input  logic signed [DATA_W-1:0] coeff_a2,

    // Raw 3-Axis Gyroscope Rate Inputs (rad/s or deg/s)
    input  logic signed [DATA_W-1:0] gyro_p_raw,
    input  logic signed [DATA_W-1:0] gyro_q_raw,
    input  logic signed [DATA_W-1:0] gyro_r_raw,

    // Filtered Gyroscope Rate Outputs
    output logic                     valid_out,
    output logic signed [DATA_W-1:0] gyro_p_filtered,
    output logic signed [DATA_W-1:0] gyro_q_filtered,
    output logic signed [DATA_W-1:0] gyro_r_filtered
);

    // Saturation limits
    localparam signed [DATA_W-1:0] MAX_VAL = {1'b0, {(DATA_W-1){1'b1}}};
    localparam signed [DATA_W-1:0] MIN_VAL = {1'b1, {(DATA_W-1){1'b0}}};

    // Single-channel biquad core
    function automatic logic signed [DATA_W-1:0] sat_clamp(input logic signed [ACCUM_W-1:0] in_val);
        logic signed [ACCUM_W-1:0] shifted;
        shifted = in_val >>> FRACT_W;
        if (shifted > ACCUM_W'(MAX_VAL))
            return MAX_VAL;
        else if (shifted < ACCUM_W'(MIN_VAL))
            return MIN_VAL;
        else
            return shifted[DATA_W-1:0];
    endfunction

    // State registers for each axis: s1, s2
    logic signed [ACCUM_W-1:0] s1_p, s2_p;
    logic signed [ACCUM_W-1:0] s1_q, s2_q;
    logic signed [ACCUM_W-1:0] s1_r, s2_r;

    logic signed [DATA_W-1:0] y_p, y_q, y_r;

    // Combinational intermediate biquad evaluation
    wire signed [ACCUM_W-1:0] b0_x_p   = ACCUM_W'(gyro_p_raw) * ACCUM_W'(coeff_b0);
    wire signed [ACCUM_W-1:0] y_full_p = b0_x_p + s1_p;
    wire signed [DATA_W-1:0]  y_sat_p  = sat_clamp(y_full_p);

    wire signed [ACCUM_W-1:0] b0_x_q   = ACCUM_W'(gyro_q_raw) * ACCUM_W'(coeff_b0);
    wire signed [ACCUM_W-1:0] y_full_q = b0_x_q + s1_q;
    wire signed [DATA_W-1:0]  y_sat_q  = sat_clamp(y_full_q);

    wire signed [ACCUM_W-1:0] b0_x_r   = ACCUM_W'(gyro_r_raw) * ACCUM_W'(coeff_b0);
    wire signed [ACCUM_W-1:0] y_full_r = b0_x_r + s1_r;
    wire signed [DATA_W-1:0]  y_sat_r  = sat_clamp(y_full_r);

    // Direct Form II Transposed computations
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_p <= '0;
            s2_p <= '0;
            s1_q <= '0;
            s2_q <= '0;
            s1_r <= '0;
            s2_r <= '0;
            y_p  <= '0;
            y_q  <= '0;
            y_r  <= '0;
            valid_out <= 1'b0;
        end else if (sample_valid) begin
            if (!filter_enable) begin
                // Hardware bypass: pass through with zero latency
                y_p <= gyro_p_raw;
                y_q <= gyro_q_raw;
                y_r <= gyro_r_raw;
                // Reset state registers on bypass
                s1_p <= '0;
                s2_p <= '0;
                s1_q <= '0;
                s2_q <= '0;
                s1_r <= '0;
                s2_r <= '0;
            end else begin
                // Axis P (Roll Rate)
                y_p  <= y_sat_p;
                s1_p <= (ACCUM_W'(gyro_p_raw) * ACCUM_W'(coeff_b1)) - 
                        (ACCUM_W'(y_sat_p) * ACCUM_W'(coeff_a1)) + s2_p;
                s2_p <= (ACCUM_W'(gyro_p_raw) * ACCUM_W'(coeff_b2)) - 
                        (ACCUM_W'(y_sat_p) * ACCUM_W'(coeff_a2));

                // Axis Q (Pitch Rate)
                y_q  <= y_sat_q;
                s1_q <= (ACCUM_W'(gyro_q_raw) * ACCUM_W'(coeff_b1)) - 
                        (ACCUM_W'(y_sat_q) * ACCUM_W'(coeff_a1)) + s2_q;
                s2_q <= (ACCUM_W'(gyro_q_raw) * ACCUM_W'(coeff_b2)) - 
                        (ACCUM_W'(y_sat_q) * ACCUM_W'(coeff_a2));

                // Axis R (Yaw Rate)
                y_r  <= y_sat_r;
                s1_r <= (ACCUM_W'(gyro_r_raw) * ACCUM_W'(coeff_b1)) - 
                        (ACCUM_W'(y_sat_r) * ACCUM_W'(coeff_a1)) + s2_r;
                s2_r <= (ACCUM_W'(gyro_r_raw) * ACCUM_W'(coeff_b2)) - 
                        (ACCUM_W'(y_sat_r) * ACCUM_W'(coeff_a2));
            end
            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end


    assign gyro_p_filtered = y_p;
    assign gyro_q_filtered = y_q;
    assign gyro_r_filtered = y_r;

endmodule : pid_gyro_notch

`endif // PID_GYRO_NOTCH_SV
