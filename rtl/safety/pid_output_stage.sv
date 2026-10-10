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
    parameter int DATA_W = 32
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    input  logic [1:0]               active_mode,
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

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            reg_roll     <= FIX_ZERO;
            reg_pitch    <= FIX_ZERO;
            reg_yaw      <= FIX_ZERO;
            reg_thrust   <= FIX_ZERO;
            valid_r      <= 1'b0;
        end else if (active_mode == MODE_FAILSAFE) begin
            // Immediate deterministic failsafe override (even when sensor samples stop)
            reg_roll     <= FIX_ZERO;
            reg_pitch    <= FIX_ZERO;
            reg_yaw      <= FIX_ZERO;
            reg_thrust   <= cfg_failsafe_thrust;
            valid_r      <= 1'b1;
        end else if (sample_valid) begin
            valid_r      <= 1'b1;
            reg_roll     <= in_u_roll;
            reg_pitch    <= in_u_pitch;
            reg_yaw      <= in_u_yaw;
            reg_thrust   <= in_u_thrust;
        end else begin
            valid_r      <= 1'b0;
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
    always_ff @(posedge clk) begin
        past_mode      <= active_mode;
        past_rst       <= rst_n;
        past_fs_thrust <= cfg_failsafe_thrust;
        if (past_rst && rst_n) begin
            if (past_mode == MODE_FAILSAFE) begin
                assert(out_u_roll == FIX_ZERO);
                assert(out_u_pitch == FIX_ZERO);
                assert(out_u_yaw == FIX_ZERO);
                assert(out_u_thrust == past_fs_thrust);
            end
        end
    end
`endif

endmodule : pid_output_stage

`endif // PID_OUTPUT_STAGE_SV
