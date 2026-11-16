# Behavioural simulation of foc_top with xsim (self-checking regression).
#
#   vivado -mode batch -source scripts/run_sim.tcl                (tb_foc_top)
#   vivado -mode batch -source scripts/run_sim.tcl -tclargs tb_foc_top_single

set repo [file normalize [file join [file dirname [info script]] ..]]
set tb   [expr {[llength $argv] > 0 ? [lindex $argv 0] : "tb_foc_top"}]
set sim  [file join $repo build sim $tb]
file mkdir $sim
cd $sim
# sincos_lut loads sin_table.mem relative to the simulator's working directory.
file copy -force $repo/rtl/sin_table.mem $sim

set log ""
append log [exec xvlog -sv {*}[lsort [glob $repo/rtl/*.sv]] $repo/tb/$tb.sv]
append log [exec xelab $tb -debug typical -s ${tb}_snap]
append log [exec xsim ${tb}_snap -runall]
puts $log
if {![regexp {PASS:[^\n]*} $log pass]} { error "SIM FAILED: $tb (no PASS line, see $sim)" }
puts "SIM RESULT: $pass"
