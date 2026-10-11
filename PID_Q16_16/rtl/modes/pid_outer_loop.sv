//==============================================================================
// Module: pid_outer_loop.sv
// Description: Cascaded outer loop controller for Stabilize and Altitude Hold modes.
//              - In Acro mode: Bypassed; routes pilot RC stick inputs directly to inner loop.
//              - In Stabilize mode: Computes Roll/Pitch attitude angle errors and scales
//                by outer proportional gains (kp_att) to produce desired angular rate setpoints.
//              - In Altitude Hold mode: Computes vertical altitude error and scales by
//                kp_alt to produce vertical climb velocity setpoints.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_OUTER_LOOP_SV
`define PID_OUTER_LOOP_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"

import pid_pkg::*;

module pid_outer_loop #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16
) (
    input  logic [1:0]               active_mode,     // Current operational mode

    // Pilot RC Commands (Q16.16)
    input  logic signed [DATA_W-1:0] rc_sp_roll,      // Angle setpoint in Stab, Rate in Acro
    input  logic signed [DATA_W-1:0] rc_sp_pitch,
    input  logic signed [DATA_W-1:0] rc_sp_yaw,       // Always rate setpoint
    input  logic signed [DATA_W-1:0] rc_sp_thrust,    // Altitude setpoint in AltHold, Throttle in Acro/Stab

    // Estimated Attitude and Altitude Measurements (Q16.16)
    input  logic signed [DATA_W-1:0] imu_phi,         // Roll attitude angle
    input  logic signed [DATA_W-1:0] imu_theta,       // Pitch attitude angle
    input  logic signed [DATA_W-1:0] alt_z,           // Measured altitude
    input  logic                     alt_valid,       // Altitude sensor valid strobe

    // Outer Loop Proportional Gains (Q16.16)
    input  logic signed [DATA_W-1:0] kp_att_roll,     // Roll angle P-gain
    input  logic signed [DATA_W-1:0] kp_att_pitch,    // Pitch angle P-gain
    input  logic signed [DATA_W-1:0] kp_alt_pos,      // Altitude position P-gain

    // Inner Loop Generated Setpoints (Q16.16)
    output logic signed [DATA_W-1:0] inner_sp_roll,   // Target roll rate
    output logic signed [DATA_W-1:0] inner_sp_pitch,  // Target pitch rate
    output logic signed [DATA_W-1:0] inner_sp_yaw,    // Target yaw rate
    output logic signed [DATA_W-1:0] inner_sp_vz,     // Target vertical climb velocity
    output logic signed [DATA_W-1:0] base_thrust      // Feedforward throttle
);

    // Attitude Angle Errors
    logic signed [DATA_W-1:0] err_phi;
    logic signed [DATA_W-1:0] err_theta;
    logic signed [DATA_W-1:0] err_z;

    assign err_phi   = rc_sp_roll  - imu_phi;
    assign err_theta = rc_sp_pitch - imu_theta;
    assign err_z     = rc_sp_thrust- alt_z;

    // Outer Proportional Multipliers
    logic signed [DATA_W-1:0] rate_sp_roll_calc;
    logic signed [DATA_W-1:0] rate_sp_pitch_calc;
    logic signed [DATA_W-1:0] vz_sp_calc;

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_phi (
        .a(kp_att_roll),
        .b(err_phi),
        .p(rate_sp_roll_calc)
    );

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_theta (
        .a(kp_att_pitch),
        .b(err_theta),
        .p(rate_sp_pitch_calc)
    );

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_alt (
        .a(kp_alt_pos),
        .b(err_z),
        .p(vz_sp_calc)
    );

    // Multiplexing Based on Active Mode
    always_comb begin
        case (active_mode)
            MODE_ACRO: begin
                // Direct pilot rate control
                inner_sp_roll  = rc_sp_roll;
                inner_sp_pitch = rc_sp_pitch;
                inner_sp_yaw   = rc_sp_yaw;
                inner_sp_vz    = FIX_ZERO;
                base_thrust    = rc_sp_thrust;
            end

            MODE_STABILIZE: begin
                // Cascaded attitude angle -> rate
                inner_sp_roll  = rate_sp_roll_calc;
                inner_sp_pitch = rate_sp_pitch_calc;
                inner_sp_yaw   = rc_sp_yaw;
                inner_sp_vz    = FIX_ZERO;
                base_thrust    = rc_sp_thrust;
            end

            MODE_ALTHOLD: begin
                // Cascaded attitude angle -> rate + altitude position -> vz
                inner_sp_roll  = rate_sp_roll_calc;
                inner_sp_pitch = rate_sp_pitch_calc;
                inner_sp_yaw   = rc_sp_yaw;
                inner_sp_vz    = (alt_valid) ? vz_sp_calc : FIX_ZERO;
                base_thrust    = rc_sp_thrust;
            end

            MODE_FAILSAFE: begin
                // Zero rotational rates, zero thrust
                inner_sp_roll  = FIX_ZERO;
                inner_sp_pitch = FIX_ZERO;
                inner_sp_yaw   = FIX_ZERO;
                inner_sp_vz    = FIX_ZERO;
                base_thrust    = FIX_ZERO;
            end

            default: begin
                inner_sp_roll  = FIX_ZERO;
                inner_sp_pitch = FIX_ZERO;
                inner_sp_yaw   = FIX_ZERO;
                inner_sp_vz    = FIX_ZERO;
                base_thrust    = FIX_ZERO;
            end
        endcase
    end

endmodule : pid_outer_loop

`endif // PID_OUTER_LOOP_SV
