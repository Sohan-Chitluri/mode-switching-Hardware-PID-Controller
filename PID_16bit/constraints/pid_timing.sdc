################################################################################
# Timing constraints for pid_top
# Clock: 50 MHz (20 ns period)
# Intended for Cadence Genus synthesis and Innovus implementation.
#
# IMPORTANT: I/O delay values below are engineering assumptions because no
# board/external-device timing specification was supplied. Replace these with
# values derived from the actual IMU/receiver/FPGA/MCU/interface timing budget
# before sign-off. This SDC alone does not establish sign-off corners.
################################################################################

# =============================================================================
# 1. Clock
# =============================================================================
create_clock -name clk -period 20.000 -waveform {0.000 10.000} [get_ports clk]
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold  0.100 [get_clocks clk]
set_clock_transition 0.150 [get_clocks clk]

# =============================================================================
# 2. Synchronous inputs
# Assumptions: maximum external input delay = 4 ns; minimum = 0 ns.
# A zero minimum is conservative for hold when a guaranteed positive minimum
# arrival time from the external device is not known.
# rst_n is included on the assumption that it is implemented as a synchronous
# reset in this RTL. If the RTL reset implementation changes, revisit this.
# =============================================================================
set sync_inputs [get_ports {
    rst_n
    sample_valid
    imu_p[*]
    imu_q[*]
    imu_r[*]
    imu_phi[*]
    imu_theta[*]
    alt_z[*]
    alt_vz[*]
    alt_valid
    mode_cmd[*]
    rearm_cmd
    rc_sp_roll[*]
    rc_sp_pitch[*]
    rc_sp_yaw[*]
    rc_sp_thrust[*]
    cfg_we
    cfg_addr[*]
    cfg_wdata[*]
    cfg_failsafe_hold_last
}]

set_input_delay -clock clk -max 4.000 $sync_inputs
set_input_delay -clock clk -min 0.000 $sync_inputs
set_input_transition 0.100 $sync_inputs

# =============================================================================
# 3. Synchronous outputs
# Assumptions: maximum output delay = 4 ns; external receiver hold requirement
# = 0.5 ns, with no additional board/clock-skew correction. Under this SDC
# convention, the minimum output delay for that assumed hold requirement is
# negative. Replace -0.500 with a value derived from the real receiving device
# and board timing if that information is available.
# =============================================================================
set sync_outputs [get_ports {
    sample_ready
    cfg_rdata[*]
    output_valid
    u_roll[*]
    u_pitch[*]
    u_yaw[*]
    u_thrust[*]
    active_mode[*]
    fault_code[*]
    watchdog_alarm
    saturated_flag
}]

set_output_delay -clock clk -max  4.000 $sync_outputs
set_output_delay -clock clk -min -0.500 $sync_outputs

# Generic estimated load: 0.025 pF = 25 fF (Liberty capacitance unit is pF).
# Replace with extracted/interfaced loads for implementation sign-off.
set_load -pin_load 0.025 [all_outputs]

# =============================================================================
# 4. Asynchronous emergency failsafe input
# The RTL must contain a two-flop synchronizer for failsafe_ext_n. This exception
# cuts only paths launched at the asynchronous input port; do NOT false-path the
# synchronizer's stage-1 Q to stage-2 D path.
# =============================================================================
set_false_path -from [get_ports failsafe_ext_n]
set_false_path -to [get_ports saturated_flag]
set_input_transition 0.100 [get_ports failsafe_ext_n]

# Clock input transition is specified through set_clock_transition above.
