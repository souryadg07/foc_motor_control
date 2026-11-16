# Out-of-context synthesis + implementation of the IP (foc_motor_control), with timing, resource
# and power reports written to reports/.
#
#   vivado -mode batch -source scripts/build.tcl            (from the repo root)
#   vivado -mode batch -source scripts/build.tcl -tclargs <part>
#
# The IP top is wrapped in impl/foc_top_timing_top.sv (boundary registers only),
# so every IP path is timed reg-to-reg. The *_hier reports break out the IP
# itself (instance "dut") from the harness registers.

set repo [file normalize [file join [file dirname [info script]] ..]]
set part [expr {[llength $argv] > 0 ? [lindex $argv 0] : "xczu7ev-ffvc1156-2-e"}]
set out  [file join $repo build impl]
set rpt  [file join $repo reports]
file mkdir $out $rpt

read_verilog -sv [lsort [glob $repo/rtl/*.sv]]
read_verilog     $repo/rtl/foc_motor_control.v
read_verilog -sv $repo/impl/foc_top_timing_top.sv
read_mem $repo/rtl/sin_table.mem
read_xdc $repo/impl/foc_top_timing_top.xdc

synth_design -top foc_top_timing_top -part $part -mode out_of_context
write_checkpoint -force $out/post_synth.dcp
report_utilization -hierarchical -file $rpt/utilization_synth_hier.rpt

opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force $out/post_route.dcp

report_timing_summary -max_paths 10 -report_unconstrained -file $rpt/timing_summary.rpt
report_utilization                       -file $rpt/utilization.rpt
report_utilization -hierarchical         -file $rpt/utilization_hier.rpt
report_power -hierarchical_depth 4       -file $rpt/power.rpt
report_drc                               -file $rpt/drc.rpt
report_methodology                       -file $rpt/methodology.rpt

set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
puts "BUILD DONE: part=$part WNS=$wns ns WHS=$whs ns (reports in $rpt)"
