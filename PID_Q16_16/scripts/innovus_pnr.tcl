################################################################################
# Cadence Innovus Place & Route Script: pid_top
# Target Design: Mode-Switching Hardware PID Flight Controller
# Usage: cd <PID> && innovus -files scripts/innovus_pnr.tcl -log reports/innovus/innovus.log
################################################################################

# ==============================================================================
# 1. Environment & Design Import
# ==============================================================================
set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]

if {[info exists env(PATH)] && [string first "/home/install/GENUS211/bin" $env(PATH)] == -1} {
    set env(PATH) "/home/install/GENUS211/bin:$env(PATH)"
}

set DESIGN_NAME "pid_top"
set NETLIST_DIR [file join $PROJECT_ROOT "netlist"]
set NETLIST     [file join $NETLIST_DIR "pid_top_synth.v"]
set SDC         [file join $NETLIST_DIR "pid_top_synth.sdc"]
set MMMC_FILE   [file join $SCRIPT_DIR "pid_mmmc.tcl"]
set REPORT_DIR  [file join $PROJECT_ROOT "reports" "innovus"]

if {![file exists $NETLIST]} {
    error "Synthesized netlist not found: $NETLIST"
}
if {![file exists $SDC]} {
    error "Synthesized SDC constraints file not found: $SDC"
}

set init_verilog $NETLIST
set init_top_cell $DESIGN_NAME
set init_pwr_net "VDD"
set init_gnd_net "VSS"

# Target PDK LEF files (tech lef + standard cell lef)
set init_lef_file [list]
if {[info exists env(PDK_TECH_LEF)]} { lappend init_lef_file $env(PDK_TECH_LEF) }
if {[info exists env(PDK_CELL_LEF)]} { lappend init_lef_file $env(PDK_CELL_LEF) }
if {[llength $init_lef_file] == 0} {
    set DEFAULT_LEF "/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"
    if {[file exists $DEFAULT_LEF]} {
        lappend init_lef_file $DEFAULT_LEF
    } else {
        error "No target LEF file found: $DEFAULT_LEF"
    }
}

# MMMC timing definition
if {[info exists env(PDK_MMMC_FILE)]} {
    set init_mmmc_file $env(PDK_MMMC_FILE)
} else {
    set init_mmmc_file $MMMC_FILE
}

# Enable multi-threaded execution (up to 8 CPUs allowed by license)
setMultiCpuUsage -localCpu 8

# Specify process node for extraction calibration
setDesignMode -process 180

init_design

# Timing exception for asynchronous/diagnostic saturation telemetry flag
catch { set_false_path -to [get_ports saturated_flag] }

# Enable OCV and CPPR mode (required for AAE timing and post-route optimization)
setAnalysisMode -analysisType onChipVariation -cppr both

# ==============================================================================
# 2. Floorplanning, I/O Pin Placement & Power Mesh
# ==============================================================================
# Standard cell site name (default 'tsm3site' for 180nm)
# Target 48% core utilization — lowered from 0.58 to compensate for larger synthesized netlist
# (78,305 cells caused post-CTS density to blow up to 90%+ at 0.58; targeting ~84-86% post-CTS at 0.48)
set CORE_SITE    [expr {[info exists env(PDK_CORE_SITE)] ? $env(PDK_CORE_SITE) : "tsm3site"}]
set UTILIZATION  [expr {[info exists env(PDK_UTILIZATION)] ? $env(PDK_UTILIZATION) : "0.48"}]
floorPlan -site $CORE_SITE -r 1.0 $UTILIZATION 30 30 30 30

# Spread all 571 I/O pins evenly across 4 die boundaries on Metal3 (H) and Metal4 (V)
set all_pins [dbGet top.terms.name]
set n_pins   [llength $all_pins]
set q_pins   [expr {$n_pins / 4}]
set p_top    [lrange $all_pins 0 [expr {$q_pins - 1}]]
set p_bot    [lrange $all_pins $q_pins [expr {2 * $q_pins - 1}]]
set p_left   [lrange $all_pins [expr {2 * $q_pins}] [expr {3 * $q_pins - 1}]]
set p_right  [lrange $all_pins [expr {3 * $q_pins}] end]

editPin -pin $p_top   -side Top    -spreadType EDGE -layer Metal3 -snap TRACK
editPin -pin $p_bot   -side Bottom -spreadType EDGE -layer Metal3 -snap TRACK
editPin -pin $p_left  -side Left   -spreadType EDGE -layer Metal4 -snap TRACK
editPin -pin $p_right -side Right  -spreadType EDGE -layer Metal4 -snap TRACK

# Global Net Connections
globalNetConnect VDD -type pgpin -pin VDD -inst *
globalNetConnect VSS -type pgpin -pin VSS -inst *
globalNetConnect VDD -type tiehi
globalNetConnect VSS -type tielo

# Power Rings & Followpins
set PWR_RING_TOP_BOT    [expr {[info exists env(PDK_PWR_TOP_BOT_LAYER)] ? $env(PDK_PWR_TOP_BOT_LAYER) : "Metal6"}]
set PWR_RING_LEFT_RIGHT [expr {[info exists env(PDK_PWR_LEFT_RIGHT_LAYER)] ? $env(PDK_PWR_LEFT_RIGHT_LAYER) : "Metal5"}]
set SROUTE_MIN_LAYER    [expr {[info exists env(PDK_SROUTE_MIN_LAYER)] ? $env(PDK_SROUTE_MIN_LAYER) : "Metal1"}]
set SROUTE_MAX_LAYER    [expr {[info exists env(PDK_SROUTE_MAX_LAYER)] ? $env(PDK_SROUTE_MAX_LAYER) : "Metal6"}]

addRing -type core_rings -nets {VDD VSS} -width 4.0 -spacing 2.0 \
    -layer [list top $PWR_RING_TOP_BOT bottom $PWR_RING_TOP_BOT left $PWR_RING_LEFT_RIGHT right $PWR_RING_LEFT_RIGHT]
sroute -connect { corePin } -layerChangeRange [list $SROUTE_MIN_LAYER $SROUTE_MAX_LAYER]

# ==============================================================================
# 3. Standard Cell Placement & Pre-CTS Optimization
# ==============================================================================
setPlaceMode -timingDriven true \
             -congEffort medium \
             -place_global_uniform_density true \
             -place_global_max_density 0.58
place_opt_design

# ==============================================================================
# 4. Clock Tree Synthesis (CTS)
# ==============================================================================
create_ccopt_clock_tree_spec
set_ccopt_property target_max_trans 1.0ns
ccopt_design

# Post-CTS Optimization
optDesign -postCTS

# ==============================================================================
# 5. Detailed Routing (NanoRoute)
# ==============================================================================
setNanoRouteMode -routeWithTimingDriven true -routeWithSiDriven false \
    -envNumberProcessor 8 \
    -drouteUseMultiCutViaEffort high \
    -drouteEndIteration 15
routeDesign

# ==============================================================================
# 6. Post-Route Optimization & Hold Fixing
# ==============================================================================
setDelayCalMode -siAware false
optDesign -postRoute
optDesign -postRoute -hold

# Physical & Connectivity Verification
verifyGeometry
verifyConnectivity -type all

# Post-Route Reports
file mkdir $REPORT_DIR
report_timing > [file join $REPORT_DIR "timing_post_route.rpt"]
report_power  > [file join $REPORT_DIR "power_post_route.rpt"]
report_area   > [file join $REPORT_DIR "area_post_route.rpt"]

# Execute timing reports snippet to populate reports/innovus with sign-off hold & setup reports
source [file join $SCRIPT_DIR "innovus_timing_reports.tcl"]

# Export Netlist, DEF, and GDS
file mkdir $NETLIST_DIR

set MAP_FILE [expr {[info exists env(PDK_MAP_FILE)] ? $env(PDK_MAP_FILE) : \
                    ([file exists [file join $PROJECT_ROOT "streamOut.map"]] ? [file join $PROJECT_ROOT "streamOut.map"] : "/home/vlsi/streamOut.map")}]
if {[info exists env(PDK_GDS_FILES)] && [file exists $env(PDK_GDS_FILES)]} {
    streamOut [file join $NETLIST_DIR "pid_top.gds"] -merge $env(PDK_GDS_FILES)
} elseif {[file exists $MAP_FILE]} {
    streamOut [file join $NETLIST_DIR "pid_top.gds"] -mapFile $MAP_FILE
} else {
    streamOut [file join $NETLIST_DIR "pid_top.gds"]
}

defOut -floorplan -netlist -routing [file join $NETLIST_DIR "pid_top.def"]
saveNetlist [file join $NETLIST_DIR "pid_top_pnr.v"]
saveDesign [file join $NETLIST_DIR "pid_top.enc"]

puts "=========================================================================="
puts "  Cadence Innovus P&R for pid_top Completed Successfully!                "
puts "  Reports: $REPORT_DIR"
puts "  GDSII:   [file join $NETLIST_DIR "pid_top.gds"]"
puts "  DEF:     [file join $NETLIST_DIR "pid_top.def"]"
puts "  Netlist: [file join $NETLIST_DIR "pid_top_pnr.v"]"
puts "  ENC DB:  [file join $NETLIST_DIR "pid_top.enc"]"
puts "=========================================================================="
exit
