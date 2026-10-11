set SCRIPT_DIR [file normalize [file dirname [info script]]]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]
set NETLIST_DIR [file join $PROJECT_ROOT "netlist"]
set DESIGN_NAME "pid_top_shared"

set init_verilog [file join $NETLIST_DIR "${DESIGN_NAME}_synth.v"]
set init_top_cell $DESIGN_NAME
set init_pwr_net "VDD"
set init_gnd_net "VSS"
set init_lef_file [list "/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"]
set init_mmmc_file [file join $SCRIPT_DIR "pid_mmmc.tcl"]

setMultiCpuUsage -localCpu 8
setDesignMode -process 180
init_design

# Floorplan with 0.70 utilization
floorPlan -site tsm3site -r 1.0 0.70 20 20 20 20

# Spread pins across 4 sides
set all_pins [dbGet top.terms.name]
set n_pins   [llength $all_pins]
set q_pins   [expr {$n_pins / 4}]
set p_top    [lrange $all_pins 0 [expr {$q_pins - 1}]]
set p_bot    [lrange $all_pins $q_pins [expr {2 * $q_pins - 1}]]
set p_left   [lrange $all_pins [expr {2 * $q_pins}] [expr {3 * $q_pins - 1}]]
set p_right  [lrange $all_pins [expr {3 * $q_pins}] end]

editPin -pin $p_top   -side Top    -spreadType SIDE -layer Metal3 -snap TRACK
editPin -pin $p_bot   -side Bottom -spreadType SIDE -layer Metal3 -snap TRACK
editPin -pin $p_left  -side Left   -spreadType SIDE -layer Metal4 -snap TRACK
editPin -pin $p_right -side Right  -spreadType SIDE -layer Metal4 -snap TRACK

# Power Ring & Sroute
globalNetConnect VDD -type pgpin -pin VDD -inst *
globalNetConnect VSS -type pgpin -pin VSS -inst *
globalNetConnect VDD -type tiehi
globalNetConnect VSS -type tielo
addRing -type core_rings -nets {VDD VSS} -width 4.0 -spacing 2.0 -layer {top Metal6 bottom Metal6 left Metal5 right Metal5}
sroute -connect { corePin } -layerChangeRange { Metal1 Metal6 }

# Placement with uniform density
setPlaceMode -timingDriven true -congEffort medium -place_global_uniform_density true
place_design

# Add filler cells to fill any remaining whitespace solidly
addFiller -cell {FILL64 FILL32 FILL16 FILL8 FILL4 FILL2 FILL1} -prefix FILL

saveDesign [file join $NETLIST_DIR "test_place_70.enc"]
puts "=== TEST 70% PLACEMENT SUCCEEDED ==="
exit
