# scripts/prelim_synth_shared.tcl
# Yosys structural synthesis script for Time-Shared Architecture (pid_soc_top_shared)

read_verilog -sv rtl/pkg/pid_pkg.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_mult.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_deriv_filter.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_integ_unit.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_sat_unit.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_outer_loop.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_gain_bank.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_mode_mgr.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_tilt_comp.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_watchdog.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_output_stage.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_mixer.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety -Irtl/shared rtl/shared/pid_axis_engine_shared.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety -Irtl/shared rtl/shared/pid_top_shared.sv
read_verilog -sv -Irtl -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety -Irtl/shared rtl/shared/pid_soc_top_shared.sv

# Hierarchy and sanity check
hierarchy -check -top pid_soc_top_shared
check -assert

# Full structural synthesis of Time-Shared SoC Top
synth -top pid_soc_top_shared -noabc
check -assert
stat
