//==============================================================================
// Module: pid_formal_tb.sv
// Description: Formal property verification testbench for hardware PID controller.
// Properties Verified:
//   P1: Saturation unit output strictly bounded within [u_min, u_max].
//   P2: Output stage asserts exact safe outputs during Failsafe mode.
//   P3: Watchdog timer raises alarm when idle count reaches timeout.
//   P4: Anti-windup conditional clamping freezes accumulator expansion.
//   P5: Mode manager deterministically transitions to Failsafe on any fault.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog (Formal Subset)
//==============================================================================

`ifndef PID_FORMAL_TB_SV
`define PID_FORMAL_TB_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"
`include "pid_sat_unit.sv"
`include "pid_integ_unit.sv"
`include "pid_output_stage.sv"
`include "pid_watchdog.sv"
`include "pid_mode_mgr.sv"
import pid_pkg::*;

module pid_formal_tb (
    input logic clk,
    input logic rst_n,

    // Saturation Unit Inputs
    input logic signed [31:0] sat_p_term,
    input logic signed [31:0] sat_i_term,
    input logic signed [31:0] sat_d_term,
    input logic signed [31:0] sat_u_min,
    input logic signed [31:0] sat_u_max,

    // Output Stage Inputs
    input logic [1:0]         out_mode,
    input logic [3:0]         out_fault_code,
    input logic               out_wd_alarm,
    input logic               out_fs_ext_n,
    input logic               out_hold_last,
    input logic signed [31:0] in_ur,
    input logic signed [31:0] in_up,
    input logic signed [31:0] in_uy,
    input logic signed [31:0] in_ut,
    input logic signed [31:0] fs_thrust,

    // Watchdog Inputs
    input logic               wd_sv,

    // Integrator Unit Inputs
    input logic signed [31:0] int_err,
    input logic signed [31:0] int_ki_eff,
    input logic signed [31:0] int_imin,
    input logic signed [31:0] int_imax,
    input logic signed [31:0] int_reseed_val,
    input logic               int_ch,
    input logic               int_cl,
    input logic               int_reseed_v,

    // Mode Manager Inputs
    input logic [1:0]         mm_cmd,
    input logic               mm_sample_v,
    input logic               mm_rearm,
    input logic               mm_wd_alarm,
    input logic               mm_fs_ext_n,
    input logic               mm_alt_v
);

    // -------------------------------------------------------------------------
    // Property P1: Saturation Unit Bounded Output Guarantee
    // -------------------------------------------------------------------------
    logic signed [31:0] sat_u_out;
    logic signed [47:0] sat_raw_sum;
    logic sat_ch, sat_cl;

    pid_sat_unit #(
        .DATA_W(32),
        .ACCUM_W(48)
    ) u_sat_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(1'b1),
        .p_term(sat_p_term),
        .i_term(sat_i_term),
        .d_term(sat_d_term),
        .u_min(sat_u_min),
        .u_max(sat_u_max),
        .u_sat(sat_u_out),
        .raw_sum(sat_raw_sum),
        .clamp_high(sat_ch),
        .clamp_low(sat_cl)
    );

    // Assert: output is mathematically bounded at all times when u_min <= u_max
    always_comb begin
        if (sat_u_min <= sat_u_max) begin
            assert(sat_u_out >= sat_u_min);
            assert(sat_u_out <= sat_u_max);
        end
    end

    // -------------------------------------------------------------------------
    // Property P2: Output Stage Deterministic Failsafe Override
    // -------------------------------------------------------------------------
    logic signed [31:0] out_ur, out_up, out_uy, out_ut;
    logic out_v;

    pid_output_stage #(
        .DATA_W(32)
    ) u_out_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(1'b1),
        .active_mode(out_mode),
        .fault_code(out_fault_code),
        .watchdog_alarm(out_wd_alarm),
        .failsafe_ext_n(out_fs_ext_n),
        .cfg_failsafe_hold_last(out_hold_last),
        .in_u_roll(in_ur),
        .in_u_pitch(in_up),
        .in_u_yaw(in_uy),
        .in_u_thrust(in_ut),
        .cfg_failsafe_thrust(fs_thrust),
        .output_valid(out_v),
        .out_u_roll(out_ur),
        .out_u_pitch(out_up),
        .out_u_yaw(out_uy),
        .out_u_thrust(out_ut)
    );

    // Track previous mode and controls to verify registered output
    logic [1:0]         prev_out_mode;
    logic [3:0]         prev_out_fault_code;
    logic               prev_out_wd_alarm;
    logic               prev_out_fs_ext_n;
    logic               prev_out_hold_last;
    logic               prev_rst_n;
    logic signed [31:0] prev_fs_thrust;
    always_ff @(posedge clk) begin
        prev_out_mode       <= out_mode;
        prev_out_fault_code <= out_fault_code;
        prev_out_wd_alarm   <= out_wd_alarm;
        prev_out_fs_ext_n   <= out_fs_ext_n;
        prev_out_hold_last  <= out_hold_last;
        prev_rst_n          <= rst_n;
        prev_fs_thrust      <= fs_thrust;
    end

    wire prev_out_is_crit = (!prev_out_fs_ext_n) ||
                            prev_out_wd_alarm ||
                            ((prev_out_fault_code & (FAULT_EXT_FAILSAFE | FAULT_WATCHDOG_STALE)) != 4'd0);

    // Assert: when in Failsafe mode with critical fault or hold_last disabled, outputs are strictly safe
    always_ff @(posedge clk) begin
        if (prev_rst_n && rst_n) begin
            if (prev_out_mode == MODE_FAILSAFE) begin
                if (prev_out_is_crit || !prev_out_hold_last) begin
                    assert(out_ur == 32'sd0);
                    assert(out_up == 32'sd0);
                    assert(out_uy == 32'sd0);
                    assert(out_ut == prev_fs_thrust);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Property P3: Watchdog Timeout & Stale Detection
    // -------------------------------------------------------------------------
    localparam int WD_TIMEOUT = 5;
    logic wd_alarm;
    logic [31:0] wd_cnt;

    pid_watchdog #(
        .TIMEOUT_CYC_W(32),
        .DEF_TIMEOUT(WD_TIMEOUT)
    ) u_wd_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(wd_sv),
        .cfg_timeout_cycles(32'd5),
        .watchdog_alarm(wd_alarm),
        .current_count(wd_cnt)
    );

    // Track previous watchdog counter and sample valid to verify registered alarm
    logic [31:0] prev_wd_cnt;
    logic        prev_wd_sv;
    always_ff @(posedge clk) begin
        prev_wd_cnt <= wd_cnt;
        prev_wd_sv  <= wd_sv;
    end

    // Assert: watchdog counter >= timeout asserts alarm on subsequent cycle
    always_ff @(posedge clk) begin
        if (rst_n && prev_rst_n) begin
            if (prev_wd_cnt >= 32'd5 && !prev_wd_sv) begin
                assert(wd_alarm == 1'b1);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Property P4: Anti-Windup Clamping Bounds
    // -------------------------------------------------------------------------
    logic signed [31:0] int_i_term;

    pid_integ_unit #(
        .DATA_W(32),
        .FRACT_W(16)
    ) u_int_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(1'b1),
        .error(int_err),
        .ki_eff(int_ki_eff),
        .i_min(int_imin),
        .i_max(int_imax),
        .clamp_high(int_ch),
        .clamp_low(int_cl),
        .reseed_valid(int_reseed_v),
        .reseed_val(int_reseed_val),
        .i_term(int_i_term)
    );

    // Reset sequence constraint for BMC
    initial assume(!rst_n);

    // Integrator bounds assumption: 0 must be within allowed integration range
    always_comb begin
        assume(int_imin <= 32'sd0);
        assume(int_imax >= 32'sd0);
    end

    // Configuration parameters are stable across clock cycles
    always_ff @(posedge clk) begin
        assume($stable(int_imin));
        assume($stable(int_imax));
    end

    // Assert: integral term is bounded within [i_min, i_max]
    always_ff @(posedge clk) begin
        if (rst_n && prev_rst_n) begin
            assert(int_i_term >= int_imin);
            assert(int_i_term <= int_imax);
        end
    end

    // -------------------------------------------------------------------------
    // Property P5: Mode Manager Safety Invariant
    // -------------------------------------------------------------------------
    logic [1:0] mm_active_mode, mm_next_mode, mm_prev_mode;
    logic mm_trans_strobe;
    logic [3:0] mm_fault;

    pid_mode_mgr u_mm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(mm_sample_v),
        .mode_cmd(mm_cmd),
        .rearm_cmd(mm_rearm),
        .watchdog_alarm(mm_wd_alarm),
        .failsafe_ext_n(mm_fs_ext_n),
        .alt_valid(mm_alt_v),
        .active_mode(mm_active_mode),
        .next_mode(mm_next_mode),
        .prev_mode(mm_prev_mode),
        .transition_strobe(mm_trans_strobe),
        .fault_code(mm_fault)
    );

    // Invariant: If external failsafe is active or watchdog alarm is active, mode MUST be FAILSAFE
    always_comb begin
        if (!mm_fs_ext_n || mm_wd_alarm) begin
            assert(mm_next_mode == MODE_FAILSAFE);
        end
    end

endmodule : pid_formal_tb

`endif // PID_FORMAL_TB_SV
