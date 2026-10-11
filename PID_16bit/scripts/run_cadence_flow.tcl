################################################################################
# Master Sequential Cadence Flow TCL Script
# Target Design: Mode-Switching Hardware PID Flight Controller
# Architecture : Fully Parallel (pid_top) or Time-Shared (pid_top_shared)
#
# Usage:
#   tclsh scripts/run_cadence_flow.tcl [parallel|shared] [full|fast]
################################################################################

set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]

# Parse command line arguments
set ARCH "parallel"
set PNR_MODE "full"

if {[info exists argv] && [llength $argv] >= 1} {
    set ARCH [lindex $argv 0]
}
if {[info exists argv] && [llength $argv] >= 2} {
    set PNR_MODE [lindex $argv 1]
}

switch -exact -- $ARCH {
    "shared" -
    "pid_top_shared" {
        set DESIGN_NAME "pid_top_shared"
        set ARCH_DESC "Time-Shared Datapath (10 multipliers, 4-cycle sequenced)"
    }
    "soc_shared" -
    "pid_soc_top_shared" {
        set DESIGN_NAME "pid_soc_top_shared"
        set ARCH_DESC "Time-Shared SoC Top with Mixer"
    }
    "soc" -
    "pid_soc_top" {
        set DESIGN_NAME "pid_soc_top"
        set ARCH_DESC "Fully Parallel SoC Top with Mixer"
    }
    default {
        set DESIGN_NAME "pid_top"
        set ARCH_DESC "Fully Parallel Datapath (22 multipliers, 1-cycle latency)"
    }
}

if {$PNR_MODE eq "fast"} {
    set INNOVUS_SCRIPT [file join $SCRIPT_DIR "innovus_pnr_fast.tcl"]
} else {
    set INNOVUS_SCRIPT [file join $SCRIPT_DIR "innovus_pnr.tcl"]
}

puts "=========================================================================="
puts "  Cadence Master Sequential Flow Runner (TCL)                             "
puts "  Design Target : $DESIGN_NAME                                            "
puts "  Architecture  : $ARCH_DESC                                              "
puts "  P&R Mode      : $PNR_MODE ($INNOVUS_SCRIPT)                             "
puts "=========================================================================="

# Locate executables
set GENUS_CMD [expr {[info exists env(GENUS_BIN)] ? $env(GENUS_BIN) : "genus"}]
set INNOVUS_CMD [expr {[info exists env(INNOVUS_BIN)] ? $env(INNOVUS_BIN) : "innovus"}]

file mkdir [file join $PROJECT_ROOT "reports" "genus"]
file mkdir [file join $PROJECT_ROOT "reports" "innovus"]
file mkdir [file join $PROJECT_ROOT "netlist"]

# ------------------------------------------------------------------------------
# STEP 1: Cadence Genus Synthesis
# ------------------------------------------------------------------------------
puts "\n>>> [STEP 1/2] Executing Cadence Genus Synthesis sequentially for $DESIGN_NAME..."
set GENUS_SCRIPT [file join $SCRIPT_DIR "genus_synth.tcl"]
set GENUS_LOG    [file join $PROJECT_ROOT "reports" "genus" "genus_${DESIGN_NAME}.log"]

set synth_cmd [list $GENUS_CMD -batch -no_gui -execute "set DESIGN_NAME \"$DESIGN_NAME\"; source \"$GENUS_SCRIPT\"" -log $GENUS_LOG]
puts "    Command: [join $synth_cmd { }]"

set synth_failed [catch {exec {*}$synth_cmd} synth_output]
if {$synth_failed} {
    # Check if the output netlist and SDC were generated despite warnings
    set synth_netlist [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_synth.v"]
    set synth_sdc     [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_synth.sdc"]
    if {![file exists $synth_netlist] || ![file exists $synth_sdc]} {
        puts "[-] ERROR: Genus synthesis failed!"
        puts "    Review log: $GENUS_LOG"
        error "Genus synthesis failed to produce $synth_netlist."
    }
}

puts "[+] STEP 1 COMPLETE: Genus synthesis produced netlist and constraints."
puts "    Netlist: [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_synth.v"]"
puts "    SDC:     [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_synth.sdc"]"

# ------------------------------------------------------------------------------
# STEP 2: Cadence Innovus Place & Route
# ------------------------------------------------------------------------------
puts "\n>>> [STEP 2/2] Executing Cadence Innovus P&R sequentially for $DESIGN_NAME..."
set INNOVUS_LOG [file join $PROJECT_ROOT "reports" "innovus" "innovus_${DESIGN_NAME}.log"]

set pnr_cmd [list $INNOVUS_CMD -batch -no_gui -execute "set DESIGN_NAME \"$DESIGN_NAME\"; source \"$INNOVUS_SCRIPT\"" -log $INNOVUS_LOG]
puts "    Command: [join $pnr_cmd { }]"

set pnr_failed [catch {exec {*}$pnr_cmd} pnr_output]
if {$pnr_failed} {
    set pnr_netlist [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_pnr.v"]
    if {![file exists $pnr_netlist]} {
        puts "[-] ERROR: Innovus P&R failed!"
        puts "    Review log: $INNOVUS_LOG"
        error "Innovus implementation failed."
    }
}

puts "[+] STEP 2 COMPLETE: Innovus P&R completed successfully."
puts "\n=========================================================================="
puts "  Cadence Sequential Flow Completed Successfully!                         "
puts "  Design Target : $DESIGN_NAME                                            "
puts "  GDSII Layout  : [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}.gds"]"
puts "  DEF Layout    : [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}.def"]"
puts "  Post-Route Net: [file join $PROJECT_ROOT "netlist" "${DESIGN_NAME}_pnr.v"]"
puts "  Setup Timing  : [file join $PROJECT_ROOT "reports" "innovus" "${DESIGN_NAME}_timing_setup.rpt"]"
puts "  Hold Timing   : [file join $PROJECT_ROOT "reports" "innovus" "${DESIGN_NAME}_timing_hold.rpt"]"
puts "=========================================================================="
