################################################################################
# Cadence Genus Synthesis Script: pid_top
# Target Design: Mode-Switching Hardware PID Flight Controller
# Usage: cd <PID> && genus -f scripts/genus_synth.tcl -log reports/genus/genus_synth.log
################################################################################

# ==============================================================================
# 1. Path and Liberty Library Configuration
# ==============================================================================
# Resolve paths relative to this script, so the flow does not depend on the
# directory from which Genus is launched.
set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]

# Target 180 nm typical-corner standard-cell Liberty library.
set LIB_FILE "/home/install/FOUNDRY/digital/180nm/dig/lib/typical.lib"

if {![file exists $LIB_FILE]} {
    error "Target Liberty library not found: $LIB_FILE"
}
if {![file readable $LIB_FILE]} {
    error "Target Liberty library is not readable: $LIB_FILE"
}

# Genus uses the 'library' attribute to select the technology library for
# mapping. Do not use Synopsys-style target_library/link_library attributes.
set_db init_lib_search_path [list [file dirname $LIB_FILE]]
set_db library $LIB_FILE
set_db information_level 7

# Project inputs and output directories.
set RTL_DIR         [file join $PROJECT_ROOT "rtl"]
set CONSTRAINT_FILE [file join $PROJECT_ROOT "constraints" "pid_timing.sdc"]
set REPORT_DIR      [file join $PROJECT_ROOT "reports" "genus"]
set NETLIST_DIR     [file join $PROJECT_ROOT "netlist"]

if {![file isdirectory $RTL_DIR]} {
    error "RTL directory not found: $RTL_DIR"
}
if {![file exists $CONSTRAINT_FILE]} {
    error "SDC constraints file not found: $CONSTRAINT_FILE"
}

# ==============================================================================
# 2. Read SystemVerilog RTL Sources (Strict Dependency Order)
# ==============================================================================
set_db init_hdl_search_path [list \
    [file join $RTL_DIR "pkg"] \
    [file join $RTL_DIR "core"] \
    [file join $RTL_DIR "modes"] \
    [file join $RTL_DIR "safety"] \
    $RTL_DIR \
]

set RTL_FILES [list \
    [file join $RTL_DIR "pkg" "pid_pkg.sv"] \
    [file join $RTL_DIR "core" "pid_mult.sv"] \
    [file join $RTL_DIR "core" "pid_deriv_filter.sv"] \
    [file join $RTL_DIR "core" "pid_integ_unit.sv"] \
    [file join $RTL_DIR "core" "pid_sat_unit.sv"] \
    [file join $RTL_DIR "core" "pid_axis_core.sv"] \
    [file join $RTL_DIR "modes" "pid_outer_loop.sv"] \
    [file join $RTL_DIR "modes" "pid_gain_bank.sv"] \
    [file join $RTL_DIR "modes" "pid_mode_mgr.sv"] \
    [file join $RTL_DIR "safety" "pid_tilt_comp.sv"] \
    [file join $RTL_DIR "safety" "pid_watchdog.sv"] \
    [file join $RTL_DIR "safety" "pid_output_stage.sv"] \
    [file join $RTL_DIR "pid_top.sv"] \
]

foreach rtl_file $RTL_FILES {
    if {![file exists $rtl_file]} {
        error "RTL source file not found: $rtl_file"
    }
}

read_hdl -language sv $RTL_FILES

# ==============================================================================
# 3. Design Elaboration & Unresolved Reference Check
# ==============================================================================
elaborate pid_top

check_design -unresolved
check_design -all

# ==============================================================================
# 4. Apply Timing & Environmental Constraints
# ==============================================================================
read_sdc $CONSTRAINT_FILE
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
file mkdir $NETLIST_DIR
write_hdl -mapped > [file join $NETLIST_DIR "pid_top_synth.v"]
write_sdc         > [file join $NETLIST_DIR "pid_top_synth.sdc"]

# ==============================================================================
# 7. Reporting & Sign-Off Artifacts
# ==============================================================================
file mkdir $REPORT_DIR

# Genus 21.14's report_timing command help in the observed run supports
# -max_paths, but does not expose -check_type hold / -early / -late. Generate
# the synthesis-stage (setup-oriented) timing report here. Do NOT label a second
# default report as a hold report. Actual hold analysis should be done in Innovus
# with the appropriate fast/min timing corner in the active MMMC views.
report_timing -max_paths 20 > [file join $REPORT_DIR "pid_top_timing_setup.rpt"]
report_area -detail         > [file join $REPORT_DIR "pid_top_area.rpt"]
report_power                > [file join $REPORT_DIR "pid_top_power.rpt"]
report_qor                  > [file join $REPORT_DIR "pid_top_qor.rpt"]
report_gates                > [file join $REPORT_DIR "pid_top_gates.rpt"]

puts "=========================================================================="
puts "  Cadence Genus synthesis flow for pid_top finished.                      "
puts "  Library: $LIB_FILE"
puts "  Reports: $REPORT_DIR"
puts "  Netlist: [file join $NETLIST_DIR "pid_top_synth.v"]"
puts "  Hold timing report: NOT generated by this Genus script; run Innovus MMMC hold analysis."
puts "  Review warnings, timing, area, power, and QoR before declaring sign-off. "
puts "=========================================================================="
exit
