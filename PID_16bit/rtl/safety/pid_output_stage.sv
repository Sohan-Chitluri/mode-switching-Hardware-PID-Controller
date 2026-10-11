//==============================================================================
// Module: pid_output_stage.sv
// Description: Actuator output stage and failsafe override multiplexer.
//              - In normal operation: Registers and outputs compensated commands.
//              - In Failsafe mode: Overrides all channels instantly (within 1 cycle)
//                with zero rotational torque and configurable safe thrust.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_OUTPUT_STAGE_SV
`define PID_OUTPUT_STAGE_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_output_stage #(
    parameter int DATA_W = 16
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    input  logic [1:0]               active_mode,
    input  logic [3:0]               fault_code,             // Diagnostic fault status
    input  logic                     watchdog_alarm,         // Watchdog timeout indicator
    input  logic                     failsafe_ext_n,         // Active-low emergency kill switch
    input  logic                     cfg_failsafe_hold_last, // 0: safe output, 1: hold last on non-critical

    input  logic signed [DATA_W-1:0] in_u_roll,
    input  logic signed [DATA_W-1:0] in_u_pitch,
    input  logic signed [DATA_W-1:0] in_u_yaw,
    input  logic signed [DATA_W-1:0] in_u_thrust,
    input  logic signed [DATA_W-1:0] cfg_failsafe_thrust,

    output logic                     output_valid,
    output logic signed [DATA_W-1:0] out_u_roll,
    output logic signed [DATA_W-1:0] out_u_pitch,
    output logic signed [DATA_W-1:0] out_u_yaw,
    output logic signed [DATA_W-1:0] out_u_thrust
);

    logic signed [DATA_W-1:0] reg_roll;
    logic signed [DATA_W-1:0] reg_pitch;
    logic signed [DATA_W-1:0] reg_yaw;
    logic signed [DATA_W-1:0] reg_thrust;
    logic                     valid_r;

    // Registers to latch last valid actuator commands for hold-last failsafe
    logic signed [DATA_W-1:0] last_valid_roll;
    logic signed [DATA_W-1:0] last_valid_pitch;
    logic signed [DATA_W-1:0] last_valid_yaw;
    logic signed [DATA_W-1:0] last_valid_thrust;
    logic                     has_valid_sample;

    // Critical faults requiring immediate deterministic safe output override:
    // 1. External Emergency Kill (!failsafe_ext_n or FAULT_EXT_FAILSAFE)
    // 2. Watchdog Stale Sensor Timeout (watchdog_alarm or FAULT_WATCHDOG_STALE)
    // Under critical faults, hold-last is strictly forbidden for flight safety.
    wire is_critical_fault = (!failsafe_ext_n) ||
                             watchdog_alarm ||
                             ((fault_code & (FAULT_EXT_FAILSAFE | FAULT_WATCHDOG_STALE)) != 4'd0);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            reg_roll          <= FIX_ZERO;
            reg_pitch         <= FIX_ZERO;
            reg_yaw           <= FIX_ZERO;
            reg_thrust        <= FIX_ZERO;
            valid_r           <= 1'b0;
            last_valid_roll   <= FIX_ZERO;
            last_valid_pitch  <= FIX_ZERO;
            last_valid_yaw    <= FIX_ZERO;
            last_valid_thrust <= FIX_ZERO;
            has_valid_sample  <= 1'b0;
        end else if (active_mode == MODE_FAILSAFE) begin
            // Failsafe Mode Priority Hierarchy:
            // Priority 1: Critical fault (Kill / Watchdog) -> Immediate safe disarm (hold-last forbidden)
            // Priority 2: Non-critical failsafe with hold-last enabled & initialized -> Hold last valid
            // Priority 3: Non-critical failsafe with hold-last disabled or uninitialized -> Configured safe output
            if (is_critical_fault || !cfg_failsafe_hold_last || !has_valid_sample) begin
                reg_roll      <= FIX_ZERO;
                reg_pitch     <= FIX_ZERO;
                reg_yaw       <= FIX_ZERO;
                reg_thrust    <= cfg_failsafe_thrust;
                valid_r       <= 1'b1;
            end else begin
                reg_roll      <= last_valid_roll;
                reg_pitch     <= last_valid_pitch;
                reg_yaw       <= last_valid_yaw;
                reg_thrust    <= last_valid_thrust;
                valid_r       <= 1'b1;
            end
        end else if (sample_valid) begin
            // Normal operational flight sample
            valid_r           <= 1'b1;
            reg_roll          <= in_u_roll;
            reg_pitch         <= in_u_pitch;
            reg_yaw           <= in_u_yaw;
            reg_thrust        <= in_u_thrust;

            // Track last valid sample
            last_valid_roll   <= in_u_roll;
            last_valid_pitch  <= in_u_pitch;
            last_valid_yaw    <= in_u_yaw;
            last_valid_thrust <= in_u_thrust;
            has_valid_sample  <= 1'b1;
        end else begin
            // Inter-sample idle cycles: retain command outputs, deassert sample strobe
            valid_r           <= 1'b0;
        end
    end

    assign output_valid = valid_r;
    assign out_u_roll   = reg_roll;
    assign out_u_pitch  = reg_pitch;
    assign out_u_yaw    = reg_yaw;
    assign out_u_thrust = reg_thrust;

`ifdef FORMAL
    logic [1:0] past_mode;
    logic past_rst;
    logic signed [DATA_W-1:0] past_fs_thrust;
    logic past_crit_fault;
    logic past_hold_last;
    logic past_has_valid;
    logic signed [DATA_W-1:0] past_last_r, past_last_p, past_last_y, past_last_t;

    always_ff @(posedge clk) begin
        past_mode        <= active_mode;
        past_rst         <= rst_n;
        past_fs_thrust   <= cfg_failsafe_thrust;
        past_crit_fault  <= is_critical_fault;
        past_hold_last   <= cfg_failsafe_hold_last;
        past_has_valid   <= has_valid_sample;
        past_last_r      <= last_valid_roll;
        past_last_p      <= last_valid_pitch;
        past_last_y      <= last_valid_yaw;
        past_last_t      <= last_valid_thrust;

        if (past_rst && rst_n) begin
            // Critical fault invariant: safe outputs are unconditionally enforced
            if (past_crit_fault) begin
                assert(out_u_roll   == FIX_ZERO);
                assert(out_u_pitch  == FIX_ZERO);
                assert(out_u_yaw    == FIX_ZERO);
                assert(out_u_thrust == past_fs_thrust);
            end else if (past_mode == MODE_FAILSAFE) begin
                if (past_hold_last && past_has_valid) begin
                    assert(out_u_roll   == past_last_r);
                    assert(out_u_pitch  == past_last_p);
                    assert(out_u_yaw    == past_last_y);
                    assert(out_u_thrust == past_last_t);
                end else begin
                    assert(out_u_roll   == FIX_ZERO);
                    assert(out_u_pitch  == FIX_ZERO);
                    assert(out_u_yaw    == FIX_ZERO);
                    assert(out_u_thrust == past_fs_thrust);
                end
            end
        end
    end
`endif

endmodule : pid_output_stage

`endif // PID_OUTPUT_STAGE_SV
