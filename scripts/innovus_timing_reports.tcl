################################################################################
# Innovus timing report snippet
# Run only AFTER the synthesized netlist, SDC, libraries, MMMC analysis views,
# and physical design setup are loaded in Innovus.
# The active hold analysis view must use the foundry's appropriate fast/min
# timing library corner. Do not use this snippet as a replacement for MMMC setup.
################################################################################

set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]
set REPORT_DIR  [file join $PROJECT_ROOT "reports" "innovus"]
file mkdir $REPORT_DIR

# Hold analysis/report (usually after CTS for meaningful clock-tree hold timing)
setAnalysisMode -checkType hold
report_timing -max_paths 20 > [file join $REPORT_DIR "pid_top_timing_hold.rpt"]

# Setup analysis/report
setAnalysisMode -checkType setup
report_timing -max_paths 20 > [file join $REPORT_DIR "pid_top_timing_setup.rpt"]
