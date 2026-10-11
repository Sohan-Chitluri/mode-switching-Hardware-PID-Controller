################################################################################
# Innovus Timing Report Script
# Generates sign-off setup and hold timing reports for active design
# Run only AFTER the design, SDC, libraries, and physical stages are loaded.
################################################################################

set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]
set REPORT_DIR  [file join $PROJECT_ROOT "reports" "innovus"]
file mkdir $REPORT_DIR

if {![info exists DESIGN_NAME]} {
    set DESIGN_NAME [expr {[info exists env(DESIGN_NAME)] ? $env(DESIGN_NAME) : "pid_top"}]
}

# Post-Route Hold Analysis / Report (active fast/min timing corner)
setAnalysisMode -checkType hold
report_timing -max_paths 20 > [file join $REPORT_DIR "${DESIGN_NAME}_timing_hold.rpt"]

# Post-Route Setup Analysis / Report (active slow/max timing corner)
setAnalysisMode -checkType setup
report_timing -max_paths 20 > [file join $REPORT_DIR "${DESIGN_NAME}_timing_setup.rpt"]

puts "\[INNOVUS\] Hold report written : [file join $REPORT_DIR "${DESIGN_NAME}_timing_hold.rpt"]"
puts "\[INNOVUS\] Setup report written: [file join $REPORT_DIR "${DESIGN_NAME}_timing_setup.rpt"]"
