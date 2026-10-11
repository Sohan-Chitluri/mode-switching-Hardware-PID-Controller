################################################################################
# Cadence Innovus Place & Route Script
# Target Design: Mode-Switching Hardware PID Flight Controller
# Architecture : Fully Parallel (pid_top) or Time-Shared (pid_top_shared)
# Usage:
#   innovus -files scripts/innovus_pnr.tcl -log reports/innovus/innovus.log
#   (or setenv DESIGN_NAME pid_top_shared && innovus -files scripts/innovus_pnr.tcl)
################################################################################

# ==============================================================================
# 1. Environment & Design Import
# ==============================================================================
set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]

if {![info exists DESIGN_NAME]} {
    set DESIGN_NAME [expr {[info exists env(DESIGN_NAME)] ? $env(DESIGN_NAME) : "pid_top"}]
}

if {[info exists env(PATH)] && [string first "/home/install/GENUS211/bin" $env(PATH)] == -1} {
    set env(PATH) "/home/install/GENUS211/bin:$env(PATH)"
}

set NETLIST_DIR [file join $PROJECT_ROOT "netlist"]
set NETLIST     [file join $NETLIST_DIR "${DESIGN_NAME}_synth.v"]
set SDC         [file join $NETLIST_DIR "${DESIGN_NAME}_synth.sdc"]
set MMMC_FILE   [file join $SCRIPT_DIR "pid_mmmc.tcl"]
set REPORT_DIR  [file join $PROJECT_ROOT "reports" "innovus"]

# Sequential check: Verify synthesis netlist exists before proceeding
if {![file exists $NETLIST]} {
    error "Synthesized netlist not found: $NETLIST\nPlease execute Genus synthesis (scripts/genus_synth.tcl) before running Innovus."
}
if {![file exists $SDC]} {
    error "Synthesized SDC constraints file not found: $SDC\nPlease execute Genus synthesis before running Innovus."
}

set init_verilog $NETLIST
set init_top_cell $DESIGN_NAME
set init_pwr_net "VDD"
set init_gnd_net "VSS"

# Target PDK LEF files (tech lef + standard cell lef)
set init_lef_file [list]
if {[info exists env(PDK_TECH_LEF)]} { lappend init_lef_file $env(PDK_TECH_LEF) }
if {[info exists env(PDK_CELL_LEF)]} { lappend init_lef_file $env(PDK_CELL_LEF) }
if {[info exists env(PDK_LEF_FILE)]} { lappend init_lef_file $env(PDK_LEF_FILE) }

if {[llength $init_lef_file] == 0} {
    set DEFAULT_LEF "/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"
    if {[file exists $DEFAULT_LEF]} {
        lappend init_lef_file $DEFAULT_LEF
    } else {
        error "No target LEF file found: $DEFAULT_LEF\nPlease export PDK_LEF_FILE=/path/to/lef before running."
    }
}

# MMMC timing definition
if {[info exists env(PDK_MMMC_FILE)]} {
    set init_mmmc_file $env(PDK_MMMC_FILE)
} else {
    set init_mmmc_file $MMMC_FILE
}

# Enable multi-threaded execution (up to 8 CPUs)
setMultiCpuUsage -localCpu 8

# Specify process node for extraction calibration
setDesignMode -process 180

puts "=== \[INNOVUS\] Initializing design: $DESIGN_NAME ==="
init_design

# Timing exception for asynchronous/diagnostic saturation telemetry flag
catch { set_false_path -to [get_ports saturated_flag] }

# Enable OCV and CPPR mode (required for AAE timing and post-route optimization)
setAnalysisMode -analysisType onChipVariation -cppr both

# ==============================================================================
# 2. Floorplanning, I/O Pin Placement & Power Mesh
# ==============================================================================
set CORE_SITE    [expr {[info exists env(PDK_CORE_SITE)] ? $env(PDK_CORE_SITE) : "tsm3site"}]
set UTILIZATION  [expr {[info exists env(PDK_UTILIZATION)] ? $env(PDK_UTILIZATION) : "0.70"}]
set CORE_MARGIN  [expr {[info exists env(PDK_CORE_MARGIN)] ? $env(PDK_CORE_MARGIN) : "20"}]

puts "=== \[INNOVUS\] Creating floorplan with ${UTILIZATION} utilization ==="
floorPlan -site $CORE_SITE -r 1.0 $UTILIZATION $CORE_MARGIN $CORE_MARGIN $CORE_MARGIN $CORE_MARGIN

# Spread all I/O pins evenly across 4 die boundaries
set all_pins [dbGet top.terms.name]
set n_pins   [llength $all_pins]
if {$n_pins > 0} {
    set q_pins   [expr {$n_pins / 4}]
    set p_top    [lrange $all_pins 0 [expr {$q_pins - 1}]]
    set p_bot    [lrange $all_pins $q_pins [expr {2 * $q_pins - 1}]]
    set p_left   [lrange $all_pins [expr {2 * $q_pins}] [expr {3 * $q_pins - 1}]]
    set p_right  [lrange $all_pins [expr {3 * $q_pins}] end]

    set PIN_LAYER_H [expr {[info exists env(PDK_PIN_LAYER_H)] ? $env(PDK_PIN_LAYER_H) : "Metal3"}]
    set PIN_LAYER_V [expr {[info exists env(PDK_PIN_LAYER_V)] ? $env(PDK_PIN_LAYER_V) : "Metal4"}]

    catch {
        editPin -pin $p_top   -side Top    -spreadType SIDE -layer $PIN_LAYER_H -snap TRACK
        editPin -pin $p_bot   -side Bottom -spreadType SIDE -layer $PIN_LAYER_H -snap TRACK
        editPin -pin $p_left  -side Left   -spreadType SIDE -layer $PIN_LAYER_V -snap TRACK
        editPin -pin $p_right -side Right  -spreadType SIDE -layer $PIN_LAYER_V -snap TRACK
    }
}

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

catch {
    addRing -type core_rings -nets {VDD VSS} -width 4.0 -spacing 2.0 \
        -layer [list top $PWR_RING_TOP_BOT bottom $PWR_RING_TOP_BOT left $PWR_RING_LEFT_RIGHT right $PWR_RING_LEFT_RIGHT]
    sroute -connect { corePin } -layerChangeRange [list $SROUTE_MIN_LAYER $SROUTE_MAX_LAYER]
}

# ==============================================================================
# 3. Standard Cell Placement & Pre-CTS Optimization
# ==============================================================================
puts "=== \[INNOVUS\] Standard cell placement and optimization with uniform spreading ==="
setPlaceMode -timingDriven true \
             -congEffort medium \
             -place_global_uniform_density true
place_opt_design

# Save placed checkpoint matching test_place_70 layout
saveDesign [file join $NETLIST_DIR "${DESIGN_NAME}_placed_70.enc"]

# ==============================================================================
# 4. Clock Tree Synthesis (CTS)
# ==============================================================================
puts "=== \[INNOVUS\] Synthesizing clock tree ==="
create_ccopt_clock_tree_spec
set_ccopt_property target_max_trans 1.0ns
ccopt_design

# Post-CTS Optimization
optDesign -postCTS

# ==============================================================================
# 5. Detailed Routing (NanoRoute)
# ==============================================================================
puts "=== \[INNOVUS\] Running NanoRoute detailed routing ==="
setNanoRouteMode -routeWithTimingDriven true -routeWithSiDriven false \
    -envNumberProcessor 8 \
    -drouteUseMultiCutViaEffort high \
    -drouteEndIteration 15
routeDesign

# ==============================================================================
# 6. Post-Route Optimization & Hold Fixing
# ==============================================================================
puts "=== \[INNOVUS\] Post-route timing and hold optimization ==="
setDelayCalMode -siAware false
optDesign -postRoute
optDesign -postRoute -hold

# Add standard cell filler cells to solidly fill all whitespace
puts "=== \[INNOVUS\] Inserting standard cell filler cells ==="
addFiller -cell {FILL64 FILL32 FILL16 FILL8 FILL4 FILL2 FILL1} -prefix FILL

# Physical & Connectivity Verification
puts "=== \[INNOVUS\] Physical verification ==="
verifyGeometry
verifyConnectivity -type all

# Post-Route Reports
file mkdir $REPORT_DIR
report_timing > [file join $REPORT_DIR "${DESIGN_NAME}_timing_post_route.rpt"]
report_power  > [file join $REPORT_DIR "${DESIGN_NAME}_power_post_route.rpt"]
report_area   > [file join $REPORT_DIR "${DESIGN_NAME}_area_post_route.rpt"]

# Execute timing reports snippet to populate sign-off hold & setup reports
source [file join $SCRIPT_DIR "innovus_timing_reports.tcl"]

# ==============================================================================
# 7. Export Layout and Gate Netlist
# ==============================================================================
file mkdir $NETLIST_DIR

set MAP_FILE [expr {[info exists env(PDK_MAP_FILE)] ? $env(PDK_MAP_FILE) : \
                    ([file exists [file join $PROJECT_ROOT "streamOut.map"]] ? [file join $PROJECT_ROOT "streamOut.map"] : "/home/vlsi/streamOut.map")}]
if {[info exists env(PDK_GDS_FILES)] && [file exists $env(PDK_GDS_FILES)]} {
    streamOut [file join $NETLIST_DIR "${DESIGN_NAME}.gds"] -merge $env(PDK_GDS_FILES)
} elseif {[file exists $MAP_FILE]} {
    streamOut [file join $NETLIST_DIR "${DESIGN_NAME}.gds"] -mapFile $MAP_FILE
} else {
    streamOut [file join $NETLIST_DIR "${DESIGN_NAME}.gds"]
}

defOut -floorplan -netlist -routing [file join $NETLIST_DIR "${DESIGN_NAME}.def"]
saveNetlist [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.v"]
saveDesign [file join $NETLIST_DIR "${DESIGN_NAME}_routed_70.enc"]
saveDesign [file join $NETLIST_DIR "${DESIGN_NAME}.enc"]

# Write sequential handoff completion marker
set timestamp [expr {[catch {clock format [clock seconds]} ts] ? [clock seconds] : $ts}]
set marker_file [open [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.done"] "w"]
puts $marker_file "INNOVUS_PNR_SUCCESS: $DESIGN_NAME"
puts $marker_file "Timestamp: $timestamp"
puts $marker_file "GDS: [file join $NETLIST_DIR "${DESIGN_NAME}.gds"]"
puts $marker_file "DEF: [file join $NETLIST_DIR "${DESIGN_NAME}.def"]"
puts $marker_file "Netlist: [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.v"]"
close $marker_file

puts "=========================================================================="
puts "  Cadence Innovus P&R for '$DESIGN_NAME' Completed Successfully!          "
puts "  Reports: $REPORT_DIR"
puts "  GDSII:   [file join $NETLIST_DIR "${DESIGN_NAME}.gds"]"
puts "  DEF:     [file join $NETLIST_DIR "${DESIGN_NAME}.def"]"
puts "  Netlist: [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.v"]"
puts "  Marker:  [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.done"]"
puts "=========================================================================="
exit
