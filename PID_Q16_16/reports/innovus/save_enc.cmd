#######################################################
#                                                     
#  Innovus Command Logging File                     
#  Created on Sun Oct 11 11:55:15 2026                
#                                                     
#######################################################

#@(#)CDS: Innovus v21.15-s110_1 (64bit) 09/23/2022 13:08 (Linux 3.10.0-693.el7.x86_64)
#@(#)CDS: NanoRoute 21.15-s110_1 NR220912-2004/21_15-UB (database version 18.20.592) {superthreading v2.17}
#@(#)CDS: AAE 21.15-s039 (64bit) 09/23/2022 (Linux 3.10.0-693.el7.x86_64)
#@(#)CDS: CTE 21.15-s038_1 () Sep 20 2022 11:42:13 ( )
#@(#)CDS: SYNTECH 21.15-s012_1 () Sep  5 2022 10:25:51 ( )
#@(#)CDS: CPE v21.15-s076
#@(#)CDS: IQuantus/TQuantus 21.1.1-s867 (64bit) Sun Jun 26 22:12:54 PDT 2022 (Linux 3.10.0-693.el7.x86_64)

set_global _enable_mmmc_by_default_flow      $CTE::mmmc_default
suppressMessage ENCEXT-2799
getVersion
getVersion
getVersion
set init_verilog /home/vlsi/Desktop/FORGE/PID_3/PID/netlist/pid_top_pnr.v
set init_top_cell pid_top
set init_pwr_net VDD
set init_gnd_net VSS
set init_lef_file /home/install/FOUNDRY/digital/180nm/dig/lef/all.lef
set init_mmmc_file /home/vlsi/Desktop/FORGE/PID_3/PID/scripts/pid_mmmc.tcl
setDesignMode -process 180
init_design
defIn /home/vlsi/Desktop/FORGE/PID_3/PID/netlist/pid_top.def
saveDesign /home/vlsi/Desktop/FORGE/PID_3/PID/netlist/pid_top.enc
