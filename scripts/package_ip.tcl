# Package foc_top as a Vivado IP. The repo root is the IP root: component.xml
# and xgui/ are written there and reference rtl/ by relative path, so the
# cloned repo can be added directly as an IP repository.
#
#   vivado -mode batch -source scripts/package_ip.tcl       (from the repo root)

set repo    [file normalize [file join [file dirname [info script]] ..]]
set part    xczu7ev-ffvc1156-2-e
set vendor  user.org
set library user
set name    foc_motor_control
set version 1.0

# Temporary packaging project (build/ is git-ignored).
set proj_dir [file join $repo build ip_pkg_proj]
create_project -force ip_pkg_proj $proj_dir -part $part
add_files -norecurse [lsort [glob $repo/rtl/*.sv]]
add_files -norecurse $repo/rtl/foc_motor_control.v
add_files -norecurse $repo/rtl/sin_table.mem
# Verilog-2001 top: the IP packager's HDL parser does not accept SystemVerilog
# "wire logic" ports, so foc_motor_control.v wraps foc_top.
set_property top foc_motor_control [current_fileset]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $repo -vendor $vendor -library $library \
    -taxonomy /UserIP -import_files false -set_current true
set core [ipx::current_core]

set_property name               $name    $core
set_property version            $version $core
set_property core_revision      1        $core
set_property display_name       "FOC Motor Control" $core
set_property description        "Field-oriented control current loop: Clarke, Park, dual PI (d/q), inverse Park and SVPWM, with AXI4-Stream in (ia, ib, ic, theta) and out (3 duty cycles). One sample per clock, 19-clock latency." $core
set_property vendor_display_name "FOC Motor Control" $core
set_property supported_families {zynquplus Production kintexuplus Production virtexuplus Production kintexu Production virtexu Production zynq Production artix7 Production kintex7 Production virtex7 Production} $core

# Clock / reset / AXI4-Stream association.
ipx::associate_bus_interfaces -busif s_axis -clock aclk $core
ipx::associate_bus_interfaces -busif m_axis -clock aclk $core
ipx::associate_bus_interfaces -clock aclk -reset aresetn $core
set rst_if [ipx::get_bus_interfaces aresetn -of_objects $core]
if {[llength [ipx::get_bus_parameters POLARITY -of_objects $rst_if]] == 0} {
    ipx::add_bus_parameter POLARITY $rst_if
}
set_property value ACTIVE_LOW [ipx::get_bus_parameters POLARITY -of_objects $rst_if]

# Only the output FIFO depth is user-configurable; the data widths are fixed
# by the tdata packing and the sine table (see foc_top.sv).
set fd [ipx::get_user_parameters FIFO_DEPTH -of_objects $core]
set_property display_name "Output FIFO depth" $fd
set_property value_validation_type list $fd
set_property value_validation_list {8 16 32 64} $fd

ipx::create_xgui_files $core
ipx::update_checksums  $core
ipx::check_integrity   $core
ipx::save_core         $core
close_project
puts "PACKAGE DONE: $vendor:$library:$name:$version -> $repo/component.xml"
