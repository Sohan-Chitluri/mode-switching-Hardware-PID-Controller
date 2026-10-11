//==============================================================================
// Module: pid_axis_engine_parallel.sv
// Description: Fully parallel 4-axis PID engine (Roll, Pitch, Yaw, Altitude).
//              Instantiates 4 concurrent pid_axis_core instances for single-cycle
//              throughput and zero inter-axis phase latency.
// Architecture: Fully Parallel Datapath (22 total multipliers in full top)
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_AXIS_ENGINE_PARALLEL_SV
`define PID_AXIS_ENGINE_PARALLEL_SV

`include "pid_pkg.sv"
`include "pid_axis_core.sv"

import pid_pkg::*;

module pid_axis_engine_parallel #(
    parameter int DATA_W  = 16,
    parameter int FRACT_W = 12,
    parameter int ACCUM_W = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Reseed inputs (bumpless transfer)
    input  logic                     reseed_valid,
    input  logic signed [DATA_W-1:0] reseed_i_roll,
    input  logic signed [DATA_W-1:0] reseed_i_pitch,
    input  logic signed [DATA_W-1:0] reseed_i_yaw,
    input  logic signed [DATA_W-1:0] reseed_i_thrust,

    // Setpoints (Q4.12)
    input  logic signed [DATA_W-1:0] inner_sp_roll,
    input  logic signed [DATA_W-1:0] inner_sp_pitch,
    input  logic signed [DATA_W-1:0] inner_sp_yaw,
    input  logic signed [DATA_W-1:0] inner_sp_vz,

    // Measurements (Q4.12)
    input  logic signed [DATA_W-1:0] imu_p,
    input  logic signed [DATA_W-1:0] imu_q,
    input  logic signed [DATA_W-1:0] imu_r,
    input  logic signed [DATA_W-1:0] alt_vz,

    // Channel Parameters: Roll
    input  logic signed [DATA_W-1:0] roll_kp,
    input  logic signed [DATA_W-1:0] roll_ki_eff,
    input  logic signed [DATA_W-1:0] roll_kd_eff,
    input  logic signed [DATA_W-1:0] roll_alpha,
    input  logic signed [DATA_W-1:0] roll_u_min,
    input  logic signed [DATA_W-1:0] roll_u_max,
    input  logic signed [DATA_W-1:0] roll_i_min,
    input  logic signed [DATA_W-1:0] roll_i_max,

    // Channel Parameters: Pitch
    input  logic signed [DATA_W-1:0] pitch_kp,
    input  logic signed [DATA_W-1:0] pitch_ki_eff,
    input  logic signed [DATA_W-1:0] pitch_kd_eff,
    input  logic signed [DATA_W-1:0] pitch_alpha,
    input  logic signed [DATA_W-1:0] pitch_u_min,
    input  logic signed [DATA_W-1:0] pitch_u_max,
    input  logic signed [DATA_W-1:0] pitch_i_min,
    input  logic signed [DATA_W-1:0] pitch_i_max,

    // Channel Parameters: Yaw
    input  logic signed [DATA_W-1:0] yaw_kp,
    input  logic signed [DATA_W-1:0] yaw_ki_eff,
    input  logic signed [DATA_W-1:0] yaw_kd_eff,
    input  logic signed [DATA_W-1:0] yaw_alpha,
    input  logic signed [DATA_W-1:0] yaw_u_min,
    input  logic signed [DATA_W-1:0] yaw_u_max,
    input  logic signed [DATA_W-1:0] yaw_i_min,
    input  logic signed [DATA_W-1:0] yaw_i_max,

    // Channel Parameters: Thrust
    input  logic signed [DATA_W-1:0] thrust_kp,
    input  logic signed [DATA_W-1:0] thrust_ki_eff,
    input  logic signed [DATA_W-1:0] thrust_kd_eff,
    input  logic signed [DATA_W-1:0] thrust_alpha,
    input  logic signed [DATA_W-1:0] thrust_u_min,
    input  logic signed [DATA_W-1:0] thrust_u_max,
    input  logic signed [DATA_W-1:0] thrust_i_min,
    input  logic signed [DATA_W-1:0] thrust_i_max,

    // Output Commands
    output logic                     engine_valid,
    output logic signed [DATA_W-1:0] core_u_roll,
    output logic signed [DATA_W-1:0] core_u_pitch,
    output logic signed [DATA_W-1:0] core_u_yaw,
    output logic signed [DATA_W-1:0] core_u_thrust,

    // Anti-windup clamp status flags
    output logic                     ch_r, cl_r,
    output logic                     ch_p, cl_p,
    output logic                     ch_y, cl_y,
    output logic                     ch_t, cl_t
);

    assign engine_valid = sample_valid;

    // Roll Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_roll (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_roll), .measurement(imu_p),
        .kp(roll_kp), .ki_eff(roll_ki_eff), .kd_eff(roll_kd_eff), .alpha(roll_alpha),
        .u_min(roll_u_min), .u_max(roll_u_max), .i_min(roll_i_min), .i_max(roll_i_max),
        .reseed_valid(reseed_valid), .reseed_val(reseed_i_roll),
        .u_sat(core_u_roll), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_r), .clamp_low_out(cl_r)
    );

    // Pitch Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_pitch (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_pitch), .measurement(imu_q),
        .kp(pitch_kp), .ki_eff(pitch_ki_eff), .kd_eff(pitch_kd_eff), .alpha(pitch_alpha),
        .u_min(pitch_u_min), .u_max(pitch_u_max), .i_min(pitch_i_min), .i_max(pitch_i_max),
        .reseed_valid(reseed_valid), .reseed_val(reseed_i_pitch),
        .u_sat(core_u_pitch), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_p), .clamp_low_out(cl_p)
    );

    // Yaw Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_yaw (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_yaw), .measurement(imu_r),
        .kp(yaw_kp), .ki_eff(yaw_ki_eff), .kd_eff(yaw_kd_eff), .alpha(yaw_alpha),
        .u_min(yaw_u_min), .u_max(yaw_u_max), .i_min(yaw_i_min), .i_max(yaw_i_max),
        .reseed_valid(reseed_valid), .reseed_val(reseed_i_yaw),
        .u_sat(core_u_yaw), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_y), .clamp_low_out(cl_y)
    );

    // Altitude Thrust Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_thrust (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_vz), .measurement(alt_vz),
        .kp(thrust_kp), .ki_eff(thrust_ki_eff), .kd_eff(thrust_kd_eff), .alpha(thrust_alpha),
        .u_min(thrust_u_min), .u_max(thrust_u_max), .i_min(thrust_i_min), .i_max(thrust_i_max),
        .reseed_valid(reseed_valid), .reseed_val(reseed_i_thrust),
        .u_sat(core_u_thrust), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_t), .clamp_low_out(cl_t)
    );

endmodule : pid_axis_engine_parallel

`endif // PID_AXIS_ENGINE_PARALLEL_SV
