################################################################################
# Multi-Mode Multi-Corner (MMMC) Configuration for Innovus P&R: pid_top
# Target Technology: TSMC 180nm (Artisan Sage standard cells)
################################################################################

set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]
set NETLIST_DIR [file join $PROJECT_ROOT "netlist"]

# Standard cell Liberty files (configurable via env, with 180nm fallbacks)
set SLOW_LIB [expr {[info exists env(PDK_SLOW_LIB)] ? $env(PDK_SLOW_LIB) : "/home/install/FOUNDRY/digital/180nm/dig/lib/slow.lib"}]
set FAST_LIB [expr {[info exists env(PDK_FAST_LIB)] ? $env(PDK_FAST_LIB) : "/home/install/FOUNDRY/digital/180nm/dig/lib/fast.lib"}]
set TYP_LIB  [expr {[info exists env(PDK_TYP_LIB)]  ? $env(PDK_TYP_LIB)  : "/home/install/FOUNDRY/digital/180nm/dig/lib/typical.lib"}]

# SDC constraint file
set SDC_FILE [expr {[info exists env(PDK_SDC_FILE)] ? $env(PDK_SDC_FILE) : [file join $NETLIST_DIR "pid_top_synth.sdc"]}]

# Capacitance table
set CAP_TABLE [expr {[info exists env(PDK_CAP_TABLE)] ? $env(PDK_CAP_TABLE) : "/home/install/FOUNDRY/digital/180nm/dig/captable/t018s6mlv.capTbl"}]

# Define Library Sets
create_library_set -name slow_libs -timing [list $SLOW_LIB]
create_library_set -name fast_libs -timing [list $FAST_LIB]
create_library_set -name typ_libs  -timing [list $TYP_LIB]

# Define RC Corner
if {[file exists $CAP_TABLE]} {
    create_rc_corner -name rc_typ \
        -cap_table $CAP_TABLE \
        -preRoute_res 1.0 -postRoute_res 1.0 \
        -preRoute_cap 1.0 -postRoute_cap 1.0 -postRoute_xcap 1.0
} else {
    create_rc_corner -name rc_typ \
        -preRoute_res 1.0 -postRoute_res 1.0 \
        -preRoute_cap 1.0 -postRoute_cap 1.0 -postRoute_xcap 1.0
}

# Define Delay Corners
create_delay_corner -name delay_slow -library_set slow_libs -rc_corner rc_typ
create_delay_corner -name delay_fast -library_set fast_libs -rc_corner rc_typ

# Define Constraint Mode
create_constraint_mode -name cons_func -sdc_files [list $SDC_FILE]

# Define Analysis Views (Slow for Setup, Fast for Hold)
create_analysis_view -name view_setup -constraint_mode cons_func -delay_corner delay_slow
create_analysis_view -name view_hold  -constraint_mode cons_func -delay_corner delay_fast

# Activate Analysis Views
set_analysis_view -setup [list view_setup] -hold [list view_hold]
