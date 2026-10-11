if {![namespace exists ::IMEX]} { namespace eval ::IMEX {} }
set ::IMEX::dataVar [file dirname [file normalize [info script]]]
set ::IMEX::libVar ${::IMEX::dataVar}/libs

create_library_set -name fast_libs\
   -timing\
    [list ${::IMEX::libVar}/mmmc/fast.lib]
create_library_set -name slow_libs\
   -timing\
    [list ${::IMEX::libVar}/mmmc/slow.lib]
create_library_set -name typ_libs\
   -timing\
    [list ${::IMEX::libVar}/mmmc/typical.lib]
create_rc_corner -name rc_typ\
   -cap_table ${::IMEX::libVar}/mmmc/t018s6mlv.capTbl\
   -preRoute_res 1\
   -postRoute_res 1\
   -preRoute_cap 1\
   -postRoute_cap 1\
   -postRoute_xcap 1\
   -preRoute_clkres 0\
   -preRoute_clkcap 0
create_delay_corner -name delay_fast\
   -library_set fast_libs\
   -rc_corner rc_typ
create_delay_corner -name delay_slow\
   -library_set slow_libs\
   -rc_corner rc_typ
create_constraint_mode -name cons_func\
   -sdc_files\
    [list /dev/null]
create_analysis_view -name view_hold -constraint_mode cons_func -delay_corner delay_fast
create_analysis_view -name view_setup -constraint_mode cons_func -delay_corner delay_slow
set_analysis_view -setup [list view_setup] -hold [list view_hold]
