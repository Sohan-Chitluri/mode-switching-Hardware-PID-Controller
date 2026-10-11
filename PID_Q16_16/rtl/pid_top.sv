//==============================================================================
// Module: pid_top.sv
// Description: Top-level integration of Mode-Switching Hardware PID Flight Controller.
//              Fully parallel datapath with dedicated cores for Roll, Pitch, Yaw,
//              and Altitude thrust, integrated with cascaded attitude/altitude loops,
//              autonomous mode FSM, gain banking, bumpless handover, and watchdog safety.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_TOP_SV
`define PID_TOP_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"
`include "pid_deriv_filter.sv"
`include "pid_integ_unit.sv"
`include "pid_sat_unit.sv"
`include "pid_axis_core.sv"
`include "pid_outer_loop.sv"
`include "pid_gain_bank.sv"
`include "pid_mode_mgr.sv"
`include "pid_tilt_comp.sv"
`include "pid_watchdog.sv"
`include "pid_output_stage.sv"

import pid_pkg::*;

module pid_top #(
    parameter int DATA_W      = 32,
    parameter int FRACT_W     = 16,
    parameter int ACCUM_W     = 48,
    parameter int TIMEOUT_CYC = 50000
) (
    input  logic                     clk,
    input  logic                     rst_n,

    // Sensor Sample Interface (Synchronous Strobe)
    input  logic                     sample_valid,
    output logic                     sample_ready,
    input  logic signed [DATA_W-1:0] imu_p,          // Roll angular rate (rad/s, Q16.16)
    input  logic signed [DATA_W-1:0] imu_q,          // Pitch angular rate
    input  logic signed [DATA_W-1:0] imu_r,          // Yaw angular rate
    input  logic signed [DATA_W-1:0] imu_phi,        // Roll attitude angle (rad, Q16.16)
    input  logic signed [DATA_W-1:0] imu_theta,      // Pitch attitude angle
    input  logic signed [DATA_W-1:0] alt_z,          // Vertical altitude (meters, Q16.16)
    input  logic signed [DATA_W-1:0] alt_vz,         // Vertical velocity (m/s, Q16.16)
    input  logic                     alt_valid,       // Altitude sensor valid flag

    // Pilot Command & Mode Interface
    input  logic [1:0]               mode_cmd,        // 00=Acro, 01=Stabilize, 10=AltHold, 11=Failsafe
    input  logic                     rearm_cmd,       // Emergency fault reset / re-arm
    input  logic                     failsafe_ext_n,  // Active-low external emergency kill
    input  logic                     cfg_failsafe_hold_last = 1'b0, // 0: safe output, 1: hold last on non-critical
    input  logic signed [DATA_W-1:0] rc_sp_roll,     // Setpoint (Rate or Angle depending on mode)
    input  logic signed [DATA_W-1:0] rc_sp_pitch,
    input  logic signed [DATA_W-1:0] rc_sp_yaw,
    input  logic signed [DATA_W-1:0] rc_sp_thrust,

    // Memory-Mapped Parameter Configuration Interface
    input  logic                     cfg_we,
    input  logic [7:0]               cfg_addr,
    input  logic [DATA_W-1:0]        cfg_wdata,
    output logic [DATA_W-1:0]        cfg_rdata,

    // Actuator Command Outputs
    output logic                     output_valid,
    output logic signed [DATA_W-1:0] u_roll,         // Actuator roll command (Q16.16)
    output logic signed [DATA_W-1:0] u_pitch,        // Actuator pitch command
    output logic signed [DATA_W-1:0] u_yaw,          // Actuator yaw command
    output logic signed [DATA_W-1:0] u_thrust,       // Actuator thrust command

    // Status and Telemetry
    output logic [1:0]               active_mode,     // Current operational mode
    output logic [3:0]               fault_code,      // Diagnostic fault register
    output logic                     watchdog_alarm,  // Watchdog timeout indicator
    output logic                     saturated_flag   // Asserted if any output is clamped
);

    assign sample_ready = 1'b1;

    // -------------------------------------------------------------------------
    // 1. Watchdog Timer & Input Sanity
    // -------------------------------------------------------------------------
    logic [31:0] wd_count;

    pid_watchdog #(
        .TIMEOUT_CYC_W(32),
        .DEF_TIMEOUT(TIMEOUT_CYC)
    ) u_watchdog (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .cfg_timeout_cycles(32'(TIMEOUT_CYC)),
        .watchdog_alarm(watchdog_alarm),
        .current_count(wd_count)
    );

    // -------------------------------------------------------------------------
    // 2. Mode Management FSM & Emergency Kill Synchronizer
    // -------------------------------------------------------------------------
    // 2-stage flip-flop synchronizer for external asynchronous failsafe kill pin
    logic fs_sync_q1, fs_sync_q2;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fs_sync_q1 <= 1'b1; // Active-low inactive default during reset
            fs_sync_q2 <= 1'b1;
        end else begin
            fs_sync_q1 <= failsafe_ext_n;
            fs_sync_q2 <= fs_sync_q1;
        end
    end
    wire failsafe_ext_sync_n = fs_sync_q2;

    logic [1:0] mode_active_w;
    logic [1:0] mode_next_w;
    logic [1:0] mode_prev_w;
    logic       trans_strobe_w;
    logic [3:0] fault_code_w;

    pid_mode_mgr u_mode_mgr (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .mode_cmd(mode_cmd),
        .rearm_cmd(rearm_cmd),
        .watchdog_alarm(watchdog_alarm),
        .failsafe_ext_n(failsafe_ext_sync_n),
        .alt_valid(alt_valid),
        .active_mode(mode_active_w),
        .next_mode(mode_next_w),
        .prev_mode(mode_prev_w),
        .transition_strobe(trans_strobe_w),
        .fault_code(fault_code_w)
    );

    assign active_mode = mode_active_w;
    assign fault_code  = fault_code_w;

    // -------------------------------------------------------------------------
    // 3. Multi-Mode Gain Bank
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] kp_att_roll, kp_att_pitch, kp_alt_pos, hover_thrust, failsafe_thrust;
    logic signed [DATA_W-1:0] roll_kp, roll_ki_eff, roll_kd_eff, roll_alpha, roll_u_min, roll_u_max, roll_i_min, roll_i_max;
    logic signed [DATA_W-1:0] pitch_kp, pitch_ki_eff, pitch_kd_eff, pitch_alpha, pitch_u_min, pitch_u_max, pitch_i_min, pitch_i_max;
    logic signed [DATA_W-1:0] yaw_kp, yaw_ki_eff, yaw_kd_eff, yaw_alpha, yaw_u_min, yaw_u_max, yaw_i_min, yaw_i_max;
    logic signed [DATA_W-1:0] thrust_kp, thrust_ki_eff, thrust_kd_eff, thrust_alpha, thrust_u_min, thrust_u_max, thrust_i_min, thrust_i_max;

    pid_gain_bank #(.DATA_W(DATA_W)) u_gain_bank (
        .clk(clk),
        .rst_n(rst_n),
        .active_mode(mode_next_w),
        .cfg_we(cfg_we),
        .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata),
        .cfg_rdata(cfg_rdata),
        .kp_att_roll(kp_att_roll),
        .kp_att_pitch(kp_att_pitch),
        .kp_alt_pos(kp_alt_pos),
        .hover_thrust(hover_thrust),
        .failsafe_thrust(failsafe_thrust),
        .roll_kp(roll_kp), .roll_ki_eff(roll_ki_eff), .roll_kd_eff(roll_kd_eff), .roll_alpha(roll_alpha),
        .roll_u_min(roll_u_min), .roll_u_max(roll_u_max), .roll_i_min(roll_i_min), .roll_i_max(roll_i_max),
        .pitch_kp(pitch_kp), .pitch_ki_eff(pitch_ki_eff), .pitch_kd_eff(pitch_kd_eff), .pitch_alpha(pitch_alpha),
        .pitch_u_min(pitch_u_min), .pitch_u_max(pitch_u_max), .pitch_i_min(pitch_i_min), .pitch_i_max(pitch_i_max),
        .yaw_kp(yaw_kp), .yaw_ki_eff(yaw_ki_eff), .yaw_kd_eff(yaw_kd_eff), .yaw_alpha(yaw_alpha),
        .yaw_u_min(yaw_u_min), .yaw_u_max(yaw_u_max), .yaw_i_min(yaw_i_min), .yaw_i_max(yaw_i_max),
        .thrust_kp(thrust_kp), .thrust_ki_eff(thrust_ki_eff), .thrust_kd_eff(thrust_kd_eff), .thrust_alpha(thrust_alpha),
        .thrust_u_min(thrust_u_min), .thrust_u_max(thrust_u_max), .thrust_i_min(thrust_i_min), .thrust_i_max(thrust_i_max)
    );

    // -------------------------------------------------------------------------
    // 4. Outer Attitude and Altitude Loop
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] inner_sp_roll, inner_sp_pitch, inner_sp_yaw, inner_sp_vz, base_thrust;

    pid_outer_loop #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_outer_loop (
        .active_mode(mode_next_w),
        .rc_sp_roll(rc_sp_roll),
        .rc_sp_pitch(rc_sp_pitch),
        .rc_sp_yaw(rc_sp_yaw),
        .rc_sp_thrust(rc_sp_thrust),
        .imu_phi(imu_phi),
        .imu_theta(imu_theta),
        .alt_z(alt_z),
        .alt_valid(alt_valid),
        .kp_att_roll(kp_att_roll),
        .kp_att_pitch(kp_att_pitch),
        .kp_alt_pos(kp_alt_pos),
        .inner_sp_roll(inner_sp_roll),
        .inner_sp_pitch(inner_sp_pitch),
        .inner_sp_yaw(inner_sp_yaw),
        .inner_sp_vz(inner_sp_vz),
        .base_thrust(base_thrust)
    );

    // -------------------------------------------------------------------------
    // 5. Bumpless Mode Transfer Back-Calculation Logic
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] u_roll_prev, u_pitch_prev, u_yaw_prev, u_thrust_prev;
    logic signed [DATA_W-1:0] new_p_roll, new_p_pitch, new_p_yaw, new_p_thrust;
    logic signed [DATA_W-1:0] reseed_i_roll, reseed_i_pitch, reseed_i_yaw;
    logic signed [DATA_W-1:0] raw_reseed_i_thrust, reseed_i_thrust;

    // Instantaneous P_new calculation for back-calculation
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_pnew_r (
        .a(roll_kp), .b(inner_sp_roll - imu_p), .p(new_p_roll)
    );
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_pnew_p (
        .a(pitch_kp), .b(inner_sp_pitch - imu_q), .p(new_p_pitch)
    );
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_pnew_y (
        .a(yaw_kp), .b(inner_sp_yaw - imu_r), .p(new_p_yaw)
    );
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_pnew_t (
        .a(thrust_kp), .b(inner_sp_vz - alt_vz), .p(new_p_thrust)
    );

    assign reseed_i_roll  = u_roll_prev  - new_p_roll;
    assign reseed_i_pitch = u_pitch_prev - new_p_pitch;
    assign reseed_i_yaw   = u_yaw_prev   - new_p_yaw;

    // Altitude Thrust bumpless back-calculation:
    // raw_alt_thrust = hover_thrust + core_u_thrust
    // To achieve zero discontinuity: raw_alt_thrust == u_thrust_prev
    // => core_u_thrust = u_thrust_prev - hover_thrust
    // Since core_u_thrust = new_p_thrust + reseed_i_thrust (with D=0 at handover),
    // raw_reseed_i_thrust = (u_thrust_prev - hover_thrust) - new_p_thrust.
    assign raw_reseed_i_thrust = (u_thrust_prev - hover_thrust) - new_p_thrust;

    always_comb begin
        if (raw_reseed_i_thrust > thrust_i_max)
            reseed_i_thrust = thrust_i_max;
        else if (raw_reseed_i_thrust < thrust_i_min)
            reseed_i_thrust = thrust_i_min;
        else
            reseed_i_thrust = raw_reseed_i_thrust;
    end

    // -------------------------------------------------------------------------
    // 6. Fully Parallel Inner Rate Datapath (Roll, Pitch, Yaw, Altitude)
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] core_u_roll, core_u_pitch, core_u_yaw, core_u_thrust;
    logic ch_r, cl_r, ch_p, cl_p, ch_y, cl_y, ch_t, cl_t;

    // Roll Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_roll (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_roll), .measurement(imu_p),
        .kp(roll_kp), .ki_eff(roll_ki_eff), .kd_eff(roll_kd_eff), .alpha(roll_alpha),
        .u_min(roll_u_min), .u_max(roll_u_max), .i_min(roll_i_min), .i_max(roll_i_max),
        .reseed_valid(trans_strobe_w), .reseed_val(reseed_i_roll),
        .u_sat(core_u_roll), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_r), .clamp_low_out(cl_r)
    );

    // Pitch Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_pitch (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_pitch), .measurement(imu_q),
        .kp(pitch_kp), .ki_eff(pitch_ki_eff), .kd_eff(pitch_kd_eff), .alpha(pitch_alpha),
        .u_min(pitch_u_min), .u_max(pitch_u_max), .i_min(pitch_i_min), .i_max(pitch_i_max),
        .reseed_valid(trans_strobe_w), .reseed_val(reseed_i_pitch),
        .u_sat(core_u_pitch), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_p), .clamp_low_out(cl_p)
    );

    // Yaw Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_yaw (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_yaw), .measurement(imu_r),
        .kp(yaw_kp), .ki_eff(yaw_ki_eff), .kd_eff(yaw_kd_eff), .alpha(yaw_alpha),
        .u_min(yaw_u_min), .u_max(yaw_u_max), .i_min(yaw_i_min), .i_max(yaw_i_max),
        .reseed_valid(trans_strobe_w), .reseed_val(reseed_i_yaw),
        .u_sat(core_u_yaw), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_y), .clamp_low_out(cl_y)
    );

    // Altitude Thrust Core
    pid_axis_core #(.DATA_W(DATA_W), .FRACT_W(FRACT_W), .ACCUM_W(ACCUM_W)) u_core_thrust (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .setpoint(inner_sp_vz), .measurement(alt_vz),
        .kp(thrust_kp), .ki_eff(thrust_ki_eff), .kd_eff(thrust_kd_eff), .alpha(thrust_alpha),
        .u_min(thrust_u_min), .u_max(thrust_u_max), .i_min(thrust_i_min), .i_max(thrust_i_max),
        .reseed_valid(trans_strobe_w && (mode_next_w == MODE_ALTHOLD)), .reseed_val(reseed_i_thrust),
        .u_sat(core_u_thrust), .error_out(), .p_term_out(), .i_term_out(), .d_term_out(), .raw_sum_out(),
        .clamp_high_out(ch_t), .clamp_low_out(cl_t)
    );

    assign saturated_flag = ch_r | cl_r | ch_p | cl_p | ch_y | cl_y | ch_t | cl_t;

    // -------------------------------------------------------------------------
    // 7. Thrust Tilt Compensation
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] raw_alt_thrust;
    logic signed [DATA_W-1:0] tilt_comp_thrust;
    logic signed [DATA_W-1:0] final_pre_sat_thrust;

    assign raw_alt_thrust = hover_thrust + core_u_thrust;

    pid_tilt_comp #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_tilt_comp (
        .base_thrust(raw_alt_thrust),
        .phi(imu_phi),
        .theta(imu_theta),
        .comp_thrust(tilt_comp_thrust)
    );

    assign final_pre_sat_thrust = (mode_next_w == MODE_ALTHOLD && alt_valid) ?
                                  tilt_comp_thrust : base_thrust;

    logic signed [DATA_W-1:0] final_thrust_sat;
    always_comb begin
        if (final_pre_sat_thrust > FIX_ONE)
            final_thrust_sat = FIX_ONE;
        else if (final_pre_sat_thrust < FIX_ZERO)
            final_thrust_sat = FIX_ZERO;
        else
            final_thrust_sat = final_pre_sat_thrust;
    end

    // -------------------------------------------------------------------------
    // 8. Output Stage & Failsafe Override
    // -------------------------------------------------------------------------
    pid_output_stage #(.DATA_W(DATA_W)) u_out_stage (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .active_mode(mode_next_w),
        .fault_code(fault_code_w),
        .watchdog_alarm(watchdog_alarm),
        .failsafe_ext_n(failsafe_ext_sync_n),
        .cfg_failsafe_hold_last(cfg_failsafe_hold_last),
        .in_u_roll(core_u_roll),
        .in_u_pitch(core_u_pitch),
        .in_u_yaw(core_u_yaw),
        .in_u_thrust(final_thrust_sat),
        .cfg_failsafe_thrust(failsafe_thrust),
        .output_valid(output_valid),
        .out_u_roll(u_roll),
        .out_u_pitch(u_pitch),
        .out_u_yaw(u_yaw),
        .out_u_thrust(u_thrust)
    );

    // Track previous outputs for bumpless transfer back-calculation
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            u_roll_prev   <= FIX_ZERO;
            u_pitch_prev  <= FIX_ZERO;
            u_yaw_prev    <= FIX_ZERO;
            u_thrust_prev <= FIX_ZERO;
        end else if (mode_next_w == MODE_FAILSAFE) begin
            u_roll_prev   <= FIX_ZERO;
            u_pitch_prev  <= FIX_ZERO;
            u_yaw_prev    <= FIX_ZERO;
            u_thrust_prev <= FIX_ZERO;
        end else if (sample_valid) begin
            u_roll_prev   <= core_u_roll;
            u_pitch_prev  <= core_u_pitch;
            u_yaw_prev    <= core_u_yaw;
            u_thrust_prev <= final_thrust_sat;
        end
    end

endmodule : pid_top

`endif // PID_TOP_SV
