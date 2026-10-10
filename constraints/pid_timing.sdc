################################################################################
# Synopsys Design Constraints (SDC) for Hardware PID Flight Controller (pid_top)
# Target Clock: 50 MHz (20.0 ns period)
# Portable constraint specification for Cadence Genus (Synthesis) & Innovus (P&R)
################################################################################

# ==============================================================================
# 1. Primary Clock Definition
# ==============================================================================
create_clock -name clk -period 20.000 -waveform {0.000 10.000} [get_ports clk]

# Clock Uncertainty (Jitter + Skew Margin: 200 ps setup, 100 ps hold)
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold  0.100 [get_clocks clk]

# Clock Transition / Slew
set_clock_transition 0.150 [get_clocks clk]

# ==============================================================================
# 2. Synchronous Input Delays (20% of clock period = 4.0 ns)
# ==============================================================================
set sync_inputs [get_ports { \
    sample_valid \
    imu_p[*] \
    imu_q[*] \
    imu_r[*] \
    imu_phi[*] \
    imu_theta[*] \
    alt_z[*] \
    alt_vz[*] \
    alt_valid \
    mode_cmd[*] \
    rearm_cmd \
    rc_sp_roll[*] \
    rc_sp_pitch[*] \
    rc_sp_yaw[*] \
    rc_sp_thrust[*] \
    cfg_we \
    cfg_addr[*] \
    cfg_wdata[*] \
}]

set_input_delay -clock clk -max 4.000 $sync_inputs
set_input_delay -clock clk -min 1.000 $sync_inputs

# ==============================================================================
# 3. Synchronous Output Delays (20% of clock period = 4.0 ns)
# ==============================================================================
set sync_outputs [get_ports { \
    sample_ready \
    cfg_rdata[*] \
    output_valid \
    u_roll[*] \
    u_pitch[*] \
    u_yaw[*] \
    u_thrust[*] \
    active_mode[*] \
    fault_code[*] \
    watchdog_alarm \
    saturated_flag \
}]

set_output_delay -clock clk -max 4.000 $sync_outputs
set_output_delay -clock clk -min 0.500 $sync_outputs

# ==============================================================================
# 4. Asynchronous / Quasi-Static Paths (False Paths)
# ==============================================================================
# Asynchronous active-low power-on reset
set_false_path -from [get_ports rst_n]

# Asynchronous physical emergency kill switch input
set_false_path -from [get_ports failsafe_ext_n]

# ==============================================================================
# 5. Driving Cell & Output Load Rules (Generic Technology Assumptions)
# ==============================================================================
# Default output capacitive load: 25 fF per output pin
set_load -pin_load 0.025 [all_outputs]

# Default input transition slew: 100 ps
set_input_transition 0.100 [all_inputs]
