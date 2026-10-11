################################################################################
# Cadence Genus Synthesis Script: pid_top
# Target Design: Mode-Switching Hardware PID Flight Controller
# Usage: genus -f genus_synth.tcl -log genus_synth.log
################################################################################

# ==============================================================================
# 1. Environment & Library Configuration (Customize for target PDK)
# ==============================================================================
set lib_paths [list "." "./lib"]
if {[info exists env(PDK_LIB_DIR)]} { lappend lib_paths $env(PDK_LIB_DIR) }
set_db init_lib_search_path $lib_paths

# Set Target and Link Libraries (Replace with target technology standard cells)
# Example: slow_vdd1v0_basicCells.lib
if {[info exists env(TARGET_LIB)]} {
    set_db target_library $env(TARGET_LIB)
}
if {[info exists env(LINK_LIB)]} {
    set_db link_library $env(LINK_LIB)
}

set_db information_level 7

# ==============================================================================
# 2. Read SystemVerilog RTL Sources (Strict Dependency Order)
# ==============================================================================
set RTL_DIR "../rtl"

set_db init_hdl_search_path [list \
    "$RTL_DIR/pkg" \
    "$RTL_DIR/core" \
    "$RTL_DIR/modes" \
    "$RTL_DIR/safety" \
    "$RTL_DIR" \
]

read_hdl -language sv [list \
    "$RTL_DIR/pkg/pid_pkg.sv" \
    "$RTL_DIR/core/pid_mult.sv" \
    "$RTL_DIR/core/pid_deriv_filter.sv" \
    "$RTL_DIR/core/pid_integ_unit.sv" \
    "$RTL_DIR/core/pid_sat_unit.sv" \
    "$RTL_DIR/core/pid_axis_core.sv" \
    "$RTL_DIR/modes/pid_outer_loop.sv" \
    "$RTL_DIR/modes/pid_gain_bank.sv" \
    "$RTL_DIR/modes/pid_mode_mgr.sv" \
    "$RTL_DIR/safety/pid_tilt_comp.sv" \
    "$RTL_DIR/safety/pid_watchdog.sv" \
    "$RTL_DIR/safety/pid_output_stage.sv" \
    "$RTL_DIR/pid_top.sv" \
]

# ==============================================================================
# 3. Design Elaboration & Unresolved Reference Check
# ==============================================================================
elaborate pid_top

check_design -unresolved
check_design -all

# ==============================================================================
# 4. Apply Timing & Environmental Constraints
# ==============================================================================
read_sdc "../constraints/pid_timing.sdc"

# Check constraints consistency
check_timing_intent

# ==============================================================================
# 5. Synthesis & Optimization (Generic -> Mapping -> Optimization)
# ==============================================================================
syn_generic
syn_map
syn_opt

# ==============================================================================
# 6. Export Gate-Level Netlist and Constraints for Innovus
# ==============================================================================
# Write netlist and SDC immediately after synthesis optimization so critical
# handoff artifacts are guaranteed to exist even if an optional report command fails.
file mkdir netlist
write_hdl -mapped > netlist/pid_top_synth.v
write_sdc         > netlist/pid_top_synth.sdc

# ==============================================================================
# 7. Reporting & Sign-Off Generation
# ==============================================================================
file mkdir reports

report_timing -max_paths 20 > reports/pid_top_timing_setup.rpt
# Note: Hold timing closure is performed post-CTS in Innovus, not in Genus.
report_area -detail         > reports/pid_top_area.rpt
report_power                > reports/pid_top_power.rpt
report_qor                  > reports/pid_top_qor.rpt
report_gates                > reports/pid_top_gates.rpt

puts "=========================================================================="
puts "  Cadence Genus Synthesis for pid_top Completed Successfully!            "
puts "  Check reports/ for timing, area, and power metrics.                    "
puts "=========================================================================="
exit
