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
# Replace with target technology LEF files:
set init_lef_file [list \
    "$env(PDK_TECH_LEF)" \
    "$env(PDK_CELL_LEF)" \
]

init_design

# ==============================================================================
# 2. Floorplanning & Power Mesh
# ==============================================================================
# Initialize floorplan with 70% utilization, 15 um core margin
floorPlan -site core -r 1.0 0.70 15 15 15 15

# Global Net Connections
globalNetConnect VDD -type pgpin -pin VDD -inst *
globalNetConnect VSS -type pgpin -pin VSS -inst *
globalNetConnect VDD -type tiehi
globalNetConnect VSS -type tielo

# Power Rings & Stripes
addRing -type core_rings -nets {VDD VSS} -width 2.0 -spacing 1.0 -layer {top M5 bottom M5 left M4 right M4}
sroute -connect { blockPin padPin padRing corePin floatingStripe } -layerChangeRange { M1 M5 }

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
streamOut "netlist/pid_top.gds" -merge "$env(PDK_GDS_DIR)"
defOut -floorplan -netlist -routing "netlist/pid_top.def"
saveNetlist "netlist/pid_top_pnr.v"

puts "=========================================================================="
puts "  Cadence Innovus P&R for pid_top Completed Successfully!                "
puts "  GDSII, DEF, and final netlist saved to netlist/                        "
puts "=========================================================================="
exit
