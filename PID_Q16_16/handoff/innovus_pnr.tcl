################################################################################
# Cadence Innovus Place & Route Script: pid_top
# Target Design: Mode-Switching Hardware PID Flight Controller
# Usage: innovus -files innovus_pnr.tcl -log innovus.log
################################################################################

# ==============================================================================
# 1. Environment & Design Import
# ==============================================================================
set DESIGN_NAME "pid_top"
set NETLIST     "netlist/pid_top_synth.v"
set SDC         "netlist/pid_top_synth.sdc"

set init_verilog $NETLIST
set init_top_cell $DESIGN_NAME
set init_pwr_net "VDD"
set init_gnd_net "VSS"

# Target PDK LEF files (tech lef + standard cell lef)
set init_lef_file [list]
if {[info exists env(PDK_TECH_LEF)]} { lappend init_lef_file $env(PDK_TECH_LEF) }
if {[info exists env(PDK_CELL_LEF)]} { lappend init_lef_file $env(PDK_CELL_LEF) }

init_design

# ==============================================================================
# 2. Floorplanning & Power Mesh
# ==============================================================================
# Target standard cell site (configurable via PDK_CORE_SITE, default 'core')
set CORE_SITE    [expr {[info exists env(PDK_CORE_SITE)] ? $env(PDK_CORE_SITE) : "core"}]
set UTILIZATION  [expr {[info exists env(PDK_UTILIZATION)] ? $env(PDK_UTILIZATION) : "0.70"}]
floorPlan -site $CORE_SITE -r 1.0 $UTILIZATION 15 15 15 15

# Global Net Connections
globalNetConnect VDD -type pgpin -pin VDD -inst *
globalNetConnect VSS -type pgpin -pin VSS -inst *
globalNetConnect VDD -type tiehi
globalNetConnect VSS -type tielo

# Power Rings & Stripes (configurable metal layers via env)
set PWR_RING_TOP_BOT    [expr {[info exists env(PDK_PWR_TOP_BOT_LAYER)] ? $env(PDK_PWR_TOP_BOT_LAYER) : "M5"}]
set PWR_RING_LEFT_RIGHT [expr {[info exists env(PDK_PWR_LEFT_RIGHT_LAYER)] ? $env(PDK_PWR_LEFT_RIGHT_LAYER) : "M4"}]
set SROUTE_MIN_LAYER    [expr {[info exists env(PDK_SROUTE_MIN_LAYER)] ? $env(PDK_SROUTE_MIN_LAYER) : "M1"}]
set SROUTE_MAX_LAYER    [expr {[info exists env(PDK_SROUTE_MAX_LAYER)] ? $env(PDK_SROUTE_MAX_LAYER) : "M5"}]

addRing -type core_rings -nets {VDD VSS} -width 2.0 -spacing 1.0 -layer [list top $PWR_RING_TOP_BOT bottom $PWR_RING_TOP_BOT left $PWR_RING_LEFT_RIGHT right $PWR_RING_LEFT_RIGHT]
sroute -connect { blockPin padPin padRing corePin floatingStripe } -layerChangeRange [list $SROUTE_MIN_LAYER $SROUTE_MAX_LAYER]

# ==============================================================================
# 3. Standard Cell Placement
# ==============================================================================
setPlaceMode -timingDriven true -congEffort high
place_opt_design

# ==============================================================================
# 4. Clock Tree Synthesis (CTS)
# ==============================================================================
create_ccopt_clock_tree_spec
ccopt_design

# Post-CTS Optimization
optDesign -postCTS

# ==============================================================================
# 5. Detailed Routing (NanoRoute)
# ==============================================================================
setNanoRouteMode -routeWithTimingDriven true -routeWithSiDriven true
routeDesign

# ==============================================================================
# 6. Post-Route Optimization & DRC / LVS Sign-Off
# ==============================================================================
optDesign -postRoute
optDesign -postRoute -hold

# Verification
verifyGeometry
verifyConnectivity -type all

# Reports
file mkdir pnr_reports
report_timing > pnr_reports/timing_post_route.rpt
report_power  > pnr_reports/power_post_route.rpt
report_area   > pnr_reports/area_post_route.rpt

# Export Layout
if {[info exists env(PDK_GDS_FILES)] && [file exists $env(PDK_GDS_FILES)]} {
    streamOut "netlist/pid_top.gds" -merge $env(PDK_GDS_FILES)
} elseif {[info exists env(PDK_GDS_DIR)] && [file isdirectory $env(PDK_GDS_DIR)]} {
    set gds_list [glob -nocomplain -directory $env(PDK_GDS_DIR) *.gds]
    if {[llength $gds_list] > 0} {
        streamOut "netlist/pid_top.gds" -merge $gds_list
    } else {
        streamOut "netlist/pid_top.gds"
    }
} else {
    streamOut "netlist/pid_top.gds"
}
defOut -floorplan -netlist -routing "netlist/pid_top.def"
saveNetlist "netlist/pid_top_pnr.v"

puts "=========================================================================="
puts "  Cadence Innovus P&R for pid_top Completed Successfully!                "
puts "  GDSII, DEF, and final netlist saved to netlist/                        "
puts "=========================================================================="
exit
