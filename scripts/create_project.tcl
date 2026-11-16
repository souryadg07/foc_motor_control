# Create a Vivado project (.xpr) for the FOC IP, for GUI work: RTL schematic,
# simulation, synthesis / implementation runs and block designs.
#
#   vivado -mode batch -source scripts/create_project.tcl              (from the repo root)
#   vivado -mode batch -source scripts/create_project.tcl -tclargs <part>
#
# The project is generated into build/foc_motor_control/ (git-ignored); rerun
# this script to recreate it. Sources are referenced in place, not copied.
#
#   sources_1 : rtl/*.sv, rtl/foc_motor_control.v (IP top), rtl/sin_table.mem,
#               impl/foc_top_timing_top.sv (synthesis top, as in build.tcl)
#   constrs_1 : impl/foc_top_timing_top.xdc (default part only; it pins a ZU7EV BUFGCE)
#   sim_1     : tb/tb_foc_top.sv (default top), tb/tb_foc_top_single.sv
#   IP repo   : this repo, so "FOC Motor Control" appears in the IP catalog for block designs
#   foc_ip_bd : block design with the packaged IP and its interfaces made external

set repo  [file normalize [file join [file dirname [info script]] ..]]
set dpart "xczu7ev-ffvc1156-2-e"
set part  [expr {[llength $argv] > 0 ? [lindex $argv 0] : $dpart}]
set name  foc_motor_control
set dir   [file join $repo build $name]

create_project -force $name $dir -part $part
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

add_files -norecurse [lsort [glob $repo/rtl/*.sv]]
add_files -norecurse $repo/rtl/foc_motor_control.v $repo/rtl/sin_table.mem
add_files -norecurse $repo/impl/foc_top_timing_top.sv
set_property top foc_top_timing_top [get_filesets sources_1]
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value {-mode out_of_context} -objects [get_runs synth_1]

if {$part eq $dpart} {
    add_files -fileset constrs_1 -norecurse $repo/impl/foc_top_timing_top.xdc
} else {
    puts "NOTE: $repo/impl/foc_top_timing_top.xdc not added (it targets $dpart). Add a clock constraint for $part."
}

add_files -fileset sim_1 -norecurse $repo/tb/tb_foc_top.sv $repo/tb/tb_foc_top_single.sv
set_property top tb_foc_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {-all} -objects [get_filesets sim_1]

set_property ip_repo_paths $repo [current_project]
update_ip_catalog

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# Block design "foc_ip_bd": the packaged IP as one block with its AXI-Stream
# interfaces and control pins made external, for viewing it as it appears in
# IP Integrator. It is not the synthesis top.
create_bd_design foc_ip_bd
create_bd_cell -type ip -vlnv user.org:user:foc_motor_control:1.0 foc_motor_control_0
make_bd_intf_pins_external [get_bd_intf_pins foc_motor_control_0/*]
foreach p {aclk aresetn id_ref iq_ref kp_d ki_d kp_q ki_q u_limit} {
    make_bd_pins_external [get_bd_pins foc_motor_control_0/$p]
}
regenerate_bd_layout
validate_bd_design
save_bd_design
update_compile_order -fileset sources_1

puts "PROJECT READY: $dir/$name.xpr (part $part)"
