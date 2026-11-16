`timescale 1ns/1ps
`default_nettype none

// IP top level for the Vivado IP packager: a plain Verilog-2001 wrapper
// around foc_top (SystemVerilog). No logic, just ports and wiring.
// See foc_top.sv for the tdata packing, latency, flow control and reset.
//
//   s_axis_tdata [15:0] ia, [31:16] ib, [47:32] ic (signed Q1.15),
//                [61:48] theta (2^14 = one electrical turn), [63:62] unused
//   m_axis_tdata [15:0] duty_a, [31:16] duty_b, [47:32] duty_c (Q0.16)

module foc_motor_control #(
    parameter FIFO_DEPTH = 32               // output FIFO, power of two
) (
    input  wire               aclk,
    input  wire               aresetn,
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,
    input  wire [63:0]        s_axis_tdata,
    output wire               m_axis_tvalid,
    input  wire               m_axis_tready,
    output wire [47:0]        m_axis_tdata,
    input  wire signed [16:0] id_ref,
    input  wire signed [16:0] iq_ref,
    input  wire [17:0]        kp_d,
    input  wire [17:0]        ki_d,
    input  wire [17:0]        kp_q,
    input  wire [17:0]        ki_q,
    input  wire [15:0]        u_limit
);
    foc_top #(.FIFO_DEPTH(FIFO_DEPTH)) u_foc_top (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready), .s_axis_tdata(s_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready), .m_axis_tdata(m_axis_tdata),
        .id_ref(id_ref), .iq_ref(iq_ref),
        .kp_d(kp_d), .ki_d(ki_d), .kp_q(kp_q), .ki_q(ki_q), .u_limit(u_limit)
    );
endmodule

`default_nettype wire
