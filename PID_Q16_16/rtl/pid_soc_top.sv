//==============================================================================
// Module: pid_soc_top.sv
// Description: Synthesizable SoC Top-Level Integration Wrapper for Hardware PID Controller.
//              Integrates:
//              1. Synthesizable 4-Axis Core Engine (pid_top.sv)
//              2. Quad-X Motor Mixer (pid_mixer.sv)
//              3. Memory-Mapped Synchronous Bus Interface (MMIO) for software host
//                 controlling setpoints, sensors, modes, status, and motor readbacks.
//
// Register Map:
//   Base Address Space:
//   - 0x000..0x01F: SoC Control, Status, Sensor Inputs, Setpoints, Readbacks
//     0x00: REG_CTRL        [RW] Bit 0: sample_valid (self-clearing strobe)
//                                Bit 1: rearm_cmd    (self-clearing strobe)
//                                Bit 2: alt_valid    (level)
//                                Bit 3: cfg_failsafe_hold_last (level)
//     0x01: REG_MODE_CMD    [RW] Bits [1:0]: mode_cmd (0=Acro, 1=Stab, 2=AltHold, 3=Failsafe)
//     0x02: REG_STATUS      [RO] Bits [1:0]: active_mode
//                                Bits [5:2]: fault_code
//                                Bit 6: watchdog_alarm
//                                Bit 7: controller_sat_flag
//                                Bit 8: mixer_sat_flag
//                                Bit 9: controller_valid
//                                Bit 10: motor_valid
//     0x03: REG_SP_ROLL     [RW] Q16.16 Roll setpoint
//     0x04: REG_SP_PITCH    [RW] Q16.16 Pitch setpoint
//     0x05: REG_SP_YAW      [RW] Q16.16 Yaw setpoint
//     0x06: REG_SP_THRUST   [RW] Q16.16 Thrust setpoint
//     0x07: REG_IMU_P       [RW] Q16.16 Body rate p (rad/s)
//     0x08: REG_IMU_Q       [RW] Q16.16 Body rate q (rad/s)
//     0x09: REG_IMU_R       [RW] Q16.16 Body rate r (rad/s)
//     0x0A: REG_IMU_PHI     [RW] Q16.16 Euler roll angle phi (rad)
//     0x0B: REG_IMU_THETA   [RW] Q16.16 Euler pitch angle theta (rad)
//     0x0C: REG_ALT_Z       [RW] Q16.16 Altitude z (m)
//     0x0D: REG_ALT_VZ      [RW] Q16.16 Vertical climb rate vz (m/s)
//     0x0E: REG_OUT_U_ROLL  [RO] Q16.16 Controller roll command readback
//     0x0F: REG_OUT_U_PITCH [RO] Q16.16 Controller pitch command readback
//     0x10: REG_OUT_U_YAW   [RO] Q16.16 Controller yaw command readback
//     0x11: REG_OUT_U_THRUST[RO] Q16.16 Controller thrust command readback
//     0x12: REG_MOTOR_1     [RO] Q16.16 Motor 1 Rear-Left [0.0, 1.0]
//     0x13: REG_MOTOR_2     [RO] Q16.16 Motor 2 Rear-Right [0.0, 1.0]
//     0x14: REG_MOTOR_3     [RO] Q16.16 Motor 3 Front-Right [0.0, 1.0]
//     0x15: REG_MOTOR_4     [RO] Q16.16 Motor 4 Front-Left [0.0, 1.0]
//
//   - 0x100..0x19F: Gain Bank Configuration Pass-Through (pid_gain_bank)
//     Forwarded directly to internal configuration bus (addr = bus_addr[7:0]).
//
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_SOC_TOP_SV
`define PID_SOC_TOP_SV

`include "pid_pkg.sv"
`include "pid_top.sv"
`include "pid_mixer.sv"

import pid_pkg::*;

module pid_soc_top #(
    parameter int DATA_W      = 32,
    parameter int FRACT_W     = 16,
    parameter int ACCUM_W     = 48,
    parameter int TIMEOUT_CYC = 200000
) (
    // Clock and Reset
    input  logic                     clk,
    input  logic                     rst_n,

    // Hardware Emergency Kill Pin (Active-Low)
    input  logic                     failsafe_ext_n,

    // Memory-Mapped Synchronous Bus Interface
    input  logic                     bus_we,
    input  logic [8:0]               bus_addr,
    input  logic signed [DATA_W-1:0] bus_wdata,
    output logic signed [DATA_W-1:0] bus_rdata,
    output logic                     bus_rvalid,

    // Dedicated Physical Actuator Outputs
    output logic                     motor_valid,
    output logic signed [DATA_W-1:0] motor_1,
    output logic signed [DATA_W-1:0] motor_2,
    output logic signed [DATA_W-1:0] motor_3,
    output logic signed [DATA_W-1:0] motor_4,

    // Direct Controller Tapped Outputs
    output logic                     sample_ready,
    output logic                     controller_valid,
    output logic signed [DATA_W-1:0] u_roll,
    output logic signed [DATA_W-1:0] u_pitch,
    output logic signed [DATA_W-1:0] u_yaw,
    output logic signed [DATA_W-1:0] u_thrust,

    // System Telemetry Taps
    output logic [1:0]               active_mode,
    output logic [3:0]               fault_code,
    output logic                     watchdog_alarm,
    output logic                     controller_sat_flag,
    output logic                     mixer_sat_flag
);

    // -------------------------------------------------------------------------
    // 1. Memory-Mapped Registers (Addresses 0x00..0x1F)
    // -------------------------------------------------------------------------
    logic                     reg_sample_valid;
    logic                     reg_rearm_cmd;
    logic                     reg_alt_valid;
    logic                     reg_cfg_failsafe_hold_last;
    logic [1:0]               reg_mode_cmd;

    logic signed [DATA_W-1:0] reg_sp_roll;
    logic signed [DATA_W-1:0] reg_sp_pitch;
    logic signed [DATA_W-1:0] reg_sp_yaw;
    logic signed [DATA_W-1:0] reg_sp_thrust;

    logic signed [DATA_W-1:0] reg_imu_p;
    logic signed [DATA_W-1:0] reg_imu_q;
    logic signed [DATA_W-1:0] reg_imu_r;
    logic signed [DATA_W-1:0] reg_imu_phi;
    logic signed [DATA_W-1:0] reg_imu_theta;
    logic signed [DATA_W-1:0] reg_alt_z;
    logic signed [DATA_W-1:0] reg_alt_vz;

    // Gain bank bus signals for pid_top pass-through
    logic                     top_cfg_we;
    logic [7:0]               top_cfg_addr;
    logic signed [DATA_W-1:0] top_cfg_wdata;
    logic signed [DATA_W-1:0] top_cfg_rdata;

    assign top_cfg_we    = bus_we && bus_addr[8]; // Forwarded when bus_addr in 0x100..0x1FF
    assign top_cfg_addr  = bus_addr[7:0];
    assign top_cfg_wdata = bus_wdata;

    // MMIO Register Write Decoding
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            reg_sample_valid           <= 1'b0;
            reg_rearm_cmd              <= 1'b0;
            reg_alt_valid              <= 1'b0;
            reg_cfg_failsafe_hold_last <= 1'b0;
            reg_mode_cmd               <= MODE_ACRO;
            reg_sp_roll                <= FIX_ZERO;
            reg_sp_pitch               <= FIX_ZERO;
            reg_sp_yaw                 <= FIX_ZERO;
            reg_sp_thrust              <= FIX_ZERO;
            reg_imu_p                  <= FIX_ZERO;
            reg_imu_q                  <= FIX_ZERO;
            reg_imu_r                  <= FIX_ZERO;
            reg_imu_phi                <= FIX_ZERO;
            reg_imu_theta              <= FIX_ZERO;
            reg_alt_z                  <= FIX_ZERO;
            reg_alt_vz                 <= FIX_ZERO;
        end else begin
            // Single-cycle self-clearing strobes
            reg_sample_valid <= 1'b0;
            reg_rearm_cmd    <= 1'b0;

            if (bus_we && (bus_addr[8:5] == 4'b0000)) begin
                case (bus_addr[4:0])
                    5'h00: begin // REG_CTRL
                        reg_sample_valid           <= bus_wdata[0];
                        reg_rearm_cmd              <= bus_wdata[1];
                        reg_alt_valid              <= bus_wdata[2];
                        reg_cfg_failsafe_hold_last <= bus_wdata[3];
                    end
                    5'h01: reg_mode_cmd   <= bus_wdata[1:0];
                    5'h03: reg_sp_roll    <= bus_wdata;
                    5'h04: reg_sp_pitch   <= bus_wdata;
                    5'h05: reg_sp_yaw     <= bus_wdata;
                    5'h06: reg_sp_thrust  <= bus_wdata;
                    5'h07: reg_imu_p      <= bus_wdata;
                    5'h08: reg_imu_q      <= bus_wdata;
                    5'h09: reg_imu_r      <= bus_wdata;
                    5'h0A: reg_imu_phi    <= bus_wdata;
                    5'h0B: reg_imu_theta  <= bus_wdata;
                    5'h0C: reg_alt_z      <= bus_wdata;
                    5'h0D: reg_alt_vz     <= bus_wdata;
                    default: ; // Read-only or unmapped registers ignore writes
                endcase
            end
        end
    end

    // MMIO Register Read Multiplexing
    always_comb begin
        if (bus_addr[8]) begin
            // 0x100..0x1FF: Forwarded Gain Bank registers
            bus_rdata = top_cfg_rdata;
        end else if (bus_addr[7:5] == 3'b000) begin
            case (bus_addr[4:0])
                5'h00: bus_rdata = {28'd0, reg_cfg_failsafe_hold_last, reg_alt_valid, reg_rearm_cmd, reg_sample_valid};
                5'h01: bus_rdata = {30'd0, reg_mode_cmd};
                5'h02: bus_rdata = {21'd0, motor_valid, controller_valid, mixer_sat_flag, controller_sat_flag,
                                    watchdog_alarm, fault_code, active_mode};
                5'h03: bus_rdata = reg_sp_roll;
                5'h04: bus_rdata = reg_sp_pitch;
                5'h05: bus_rdata = reg_sp_yaw;
                5'h06: bus_rdata = reg_sp_thrust;
                5'h07: bus_rdata = reg_imu_p;
                5'h08: bus_rdata = reg_imu_q;
                5'h09: bus_rdata = reg_imu_r;
                5'h0A: bus_rdata = reg_imu_phi;
                5'h0B: bus_rdata = reg_imu_theta;
                5'h0C: bus_rdata = reg_alt_z;
                5'h0D: bus_rdata = reg_alt_vz;
                5'h0E: bus_rdata = u_roll;
                5'h0F: bus_rdata = u_pitch;
                5'h10: bus_rdata = u_yaw;
                5'h11: bus_rdata = u_thrust;
                5'h12: bus_rdata = motor_1;
                5'h13: bus_rdata = motor_2;
                5'h14: bus_rdata = motor_3;
                5'h15: bus_rdata = motor_4;
                default: bus_rdata = 32'd0;
            endcase
        end else begin
            bus_rdata = 32'd0;
        end
    end

    // Single-cycle synchronous bus read response valid
    always_ff @(posedge clk) begin
        if (!rst_n)
            bus_rvalid <= 1'b0;
        else
            bus_rvalid <= !bus_we;
    end

    // -------------------------------------------------------------------------
    // 2. Hardware PID Top Instance
    // -------------------------------------------------------------------------
    pid_top #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W),
        .TIMEOUT_CYC(TIMEOUT_CYC)
    ) u_pid_top (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(reg_sample_valid),
        .sample_ready(sample_ready),
        .imu_p(reg_imu_p),
        .imu_q(reg_imu_q),
        .imu_r(reg_imu_r),
        .imu_phi(reg_imu_phi),
        .imu_theta(reg_imu_theta),
        .alt_z(reg_alt_z),
        .alt_vz(reg_alt_vz),
        .alt_valid(reg_alt_valid),
        .mode_cmd(reg_mode_cmd),
        .rearm_cmd(reg_rearm_cmd),
        .failsafe_ext_n(failsafe_ext_n),
        .cfg_failsafe_hold_last(reg_cfg_failsafe_hold_last),
        .rc_sp_roll(reg_sp_roll),
        .rc_sp_pitch(reg_sp_pitch),
        .rc_sp_yaw(reg_sp_yaw),
        .rc_sp_thrust(reg_sp_thrust),
        .cfg_we(top_cfg_we),
        .cfg_addr(top_cfg_addr),
        .cfg_wdata(top_cfg_wdata),
        .cfg_rdata(top_cfg_rdata),
        .output_valid(controller_valid),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .saturated_flag(controller_sat_flag)
    );

    // -------------------------------------------------------------------------
    // 3. Quad-X Motor Mixer Instance
    // -------------------------------------------------------------------------
    pid_mixer #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W)
    ) u_pid_mixer (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(controller_valid),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .motor_valid(motor_valid),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4),
        .saturated_flag(mixer_sat_flag)
    );

endmodule : pid_soc_top

`endif // PID_SOC_TOP_SV
