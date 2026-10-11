################################################################################
# Cadence Genus Synthesis Script
# Target Design: Mode-Switching Hardware PID Flight Controller
# Architecture : Fully Parallel (pid_top) or Time-Shared (pid_top_shared)
# Usage:
#   genus -f scripts/genus_synth.tcl -log reports/genus/genus_synth.log
#   (or setenv DESIGN_NAME pid_top_shared && genus -f scripts/genus_synth.tcl)
################################################################################

# ==============================================================================
# 1. Path and Design Configuration
# ==============================================================================
set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]

if {![info exists DESIGN_NAME]} {
    set DESIGN_NAME [expr {[info exists env(DESIGN_NAME)] ? $env(DESIGN_NAME) : "pid_top"}]
}

# Target standard-cell Liberty library (configurable via env, with 180nm fallback)
set LIB_FILE [expr {[info exists env(PDK_TYP_LIB)] ? $env(PDK_TYP_LIB) : \
                   ([info exists env(PDK_LIB_FILE)] ? $env(PDK_LIB_FILE) : \
                   "/home/install/FOUNDRY/digital/180nm/dig/lib/typical.lib")}]

if {![file exists $LIB_FILE]} {
    error "Target Liberty library not found: $LIB_FILE\nPlease export PDK_TYP_LIB=/path/to/typical.lib before running."
}
if {![file readable $LIB_FILE]} {
    error "Target Liberty library is not readable: $LIB_FILE"
}

set_db init_lib_search_path [list [file dirname $LIB_FILE]]
set_db library $LIB_FILE
set_db information_level 7

# Project inputs and output directories
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
    [file join $RTL_DIR "parallel"] \
    [file join $RTL_DIR "shared"] \
    $RTL_DIR \
]

# Common sub-modules across both architectures
set RTL_FILES [list \
    [file join $RTL_DIR "pkg" "pid_pkg.sv"] \
    [file join $RTL_DIR "core" "pid_mult.sv"] \
    [file join $RTL_DIR "core" "pid_deriv_filter.sv"] \
    [file join $RTL_DIR "core" "pid_integ_unit.sv"] \
    [file join $RTL_DIR "core" "pid_sat_unit.sv"] \
    [file join $RTL_DIR "modes" "pid_outer_loop.sv"] \
    [file join $RTL_DIR "modes" "pid_gain_bank.sv"] \
    [file join $RTL_DIR "modes" "pid_mode_mgr.sv"] \
    [file join $RTL_DIR "safety" "pid_tilt_comp.sv"] \
    [file join $RTL_DIR "safety" "pid_watchdog.sv"] \
    [file join $RTL_DIR "safety" "pid_output_stage.sv"] \
]

# Architecture-specific module selection
if {[string match "*shared*" $DESIGN_NAME]} {
    puts "=== \[GENUS\] Selecting Time-Shared Architecture Sources for $DESIGN_NAME ==="
    lappend RTL_FILES [file join $RTL_DIR "shared" "pid_axis_engine_shared.sv"]
    lappend RTL_FILES [file join $RTL_DIR "shared" "pid_top_shared.sv"]
    if {[string match "*soc*" $DESIGN_NAME]} {
        lappend RTL_FILES [file join $RTL_DIR "safety" "pid_mixer.sv"]
        lappend RTL_FILES [file join $RTL_DIR "shared" "pid_soc_top_shared.sv"]
    }
} else {
    puts "=== \[GENUS\] Selecting Fully Parallel Architecture Sources for $DESIGN_NAME ==="
    lappend RTL_FILES [file join $RTL_DIR "core" "pid_axis_core.sv"]
    lappend RTL_FILES [file join $RTL_DIR "parallel" "pid_axis_engine_parallel.sv"]
    lappend RTL_FILES [file join $RTL_DIR "pid_top.sv"]
    if {[string match "*soc*" $DESIGN_NAME]} {
        lappend RTL_FILES [file join $RTL_DIR "safety" "pid_mixer.sv"]
        lappend RTL_FILES [file join $RTL_DIR "pid_soc_top.sv"]
    }
}

foreach rtl_file $RTL_FILES {
    if {![file exists $rtl_file]} {
        error "RTL source file not found: $rtl_file"
    }
}

read_hdl -language sv $RTL_FILES

# ==============================================================================
# 3. Design Elaboration & Unresolved Reference Check
# ==============================================================================
puts "=== \[GENUS\] Elaborating top module: $DESIGN_NAME ==="
elaborate $DESIGN_NAME

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
file mkdir $NETLIST_DIR
set OUT_NETLIST [file join $NETLIST_DIR "${DESIGN_NAME}_synth.v"]
set OUT_SDC     [file join $NETLIST_DIR "${DESIGN_NAME}_synth.sdc"]

write_hdl -mapped > $OUT_NETLIST
write_sdc         > $OUT_SDC

# ==============================================================================
# 7. Reporting & Sign-Off Artifacts
# ==============================================================================
file mkdir $REPORT_DIR

report_timing -max_paths 20 > [file join $REPORT_DIR "${DESIGN_NAME}_timing_setup.rpt"]
report_area -detail         > [file join $REPORT_DIR "${DESIGN_NAME}_area.rpt"]
report_power                > [file join $REPORT_DIR "${DESIGN_NAME}_power.rpt"]
report_qor                  > [file join $REPORT_DIR "${DESIGN_NAME}_qor.rpt"]
report_gates                > [file join $REPORT_DIR "${DESIGN_NAME}_gates.rpt"]

# Write sequential handoff marker file
set timestamp [expr {[catch {clock format [clock seconds]} ts] ? [clock seconds] : $ts}]
set marker_file [open [file join $NETLIST_DIR "${DESIGN_NAME}_synth.done"] "w"]
puts $marker_file "GENUS_SYNTHESIS_SUCCESS: $DESIGN_NAME"
puts $marker_file "Timestamp: $timestamp"
puts $marker_file "Netlist: $OUT_NETLIST"
puts $marker_file "SDC: $OUT_SDC"
close $marker_file

puts "=========================================================================="
puts "  Cadence Genus synthesis for '$DESIGN_NAME' finished successfully.       "
puts "  Library: $LIB_FILE"
puts "  Netlist: $OUT_NETLIST"
puts "  SDC:     $OUT_SDC"
puts "  Reports: $REPORT_DIR"
puts "  Handoff marker written: [file join $NETLIST_DIR "${DESIGN_NAME}_synth.done"]"
puts "=========================================================================="
exit
