################################################################################
# Quick Encounter/Innovus .enc database generator for 32-bit pid_top
################################################################################
set SCRIPT_PATH [file normalize [info script]]
set SCRIPT_DIR  [file dirname $SCRIPT_PATH]
set PROJECT_ROOT [file normalize [file join $SCRIPT_DIR ".."]]
set NETLIST_DIR [file join $PROJECT_ROOT "netlist"]
set DESIGN_NAME "pid_top"

set init_verilog [file join $NETLIST_DIR "${DESIGN_NAME}_pnr.v"]
set init_top_cell $DESIGN_NAME
set init_pwr_net "VDD"
set init_gnd_net "VSS"
set init_lef_file [list "/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"]
set init_mmmc_file [file join $SCRIPT_DIR "pid_mmmc.tcl"]

setDesignMode -process 180
init_design

defIn [file join $NETLIST_DIR "${DESIGN_NAME}.def"]
saveDesign [file join $NETLIST_DIR "${DESIGN_NAME}.enc"]
puts "Successfully saved design database: [file join $NETLIST_DIR "${DESIGN_NAME}.enc"]"
exit
