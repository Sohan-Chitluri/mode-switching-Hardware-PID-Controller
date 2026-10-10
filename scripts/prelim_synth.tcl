# scripts/prelim_synth.tcl
# Yosys preliminary structural synthesis and elaboration sanity check script for pid_top

# Read SystemVerilog RTL files
read_verilog -sv rtl/pkg/pid_pkg.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_mult.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_deriv_filter.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_integ_unit.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_sat_unit.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/core/pid_axis_core.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_outer_loop.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_gain_bank.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/modes/pid_mode_mgr.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_tilt_comp.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_watchdog.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_output_stage.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/pid_top.sv
read_verilog -sv -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/safety/pid_mixer.sv
read_verilog -sv -Irtl -Irtl/pkg -Irtl/core -Irtl/modes -Irtl/safety rtl/pid_soc_top.sv

# Check structural hierarchy of pid_soc_top (integrates pid_top and pid_mixer)
hierarchy -check -top pid_soc_top
check -assert

# Full structural synthesis of SoC Top (synthesizes pid_top, pid_mixer, MMIO bus)
synth -top pid_soc_top -noabc
check -assert
stat
