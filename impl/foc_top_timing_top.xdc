# Out-of-context characterization of the IP (foc_motor_control) through the registered harness
# impl/foc_top_timing_top.sv. Target: 100 MHz on xczu7ev-ffvc1156-2-e (ZCU106).
# Not board-level I/O signoff: every IP path is timed register-to-register
# against the harness boundary registers; only the harness exterior is excluded.
create_clock -name aclk -period 10.000 [get_ports aclk]
set_clock_uncertainty -setup 0.100 [get_clocks aclk]
set_clock_uncertainty -hold  0.050 [get_clocks aclk]

# Global clock buffer in the centre clock region (X2Y2) of the ZU7EV, so the
# OOC run estimates a realistic clock insertion delay.
set_property HD.CLK_SRC BUFGCE_X1Y48 [get_ports aclk]

set_false_path -from [get_ports {aresetn s_axis_tvalid s_axis_tdata[*] m_axis_tready id_ref[*] iq_ref[*] kp_d[*] ki_d[*] kp_q[*] ki_q[*] u_limit[*]}]
set_false_path -to   [get_ports {s_axis_tready m_axis_tvalid m_axis_tdata[*]}]
