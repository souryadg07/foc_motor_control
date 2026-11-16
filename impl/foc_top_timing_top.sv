`timescale 1ns/1ps

// Implementation-only registered harness for the IP top foc_motor_control
// (wraps foc_top). Boundary registers stand in for the neighbouring
// AXI-Stream master/slave and control logic, so every core path -- including
// the AXI handshake logic -- is timed reg-to-reg. Not a board-level top.
module foc_top_timing_top (
    input  logic                aclk, aresetn,
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [63:0]         s_axis_tdata,
    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [47:0]         m_axis_tdata,
    input  logic signed [16:0]  id_ref, iq_ref,
    input  logic [17:0]         kp_d, ki_d, kp_q, ki_q,
    input  logic [15:0]         u_limit
);
    (* DONT_TOUCH = "yes" *) logic               aresetn_q, s_tvalid_q, m_tready_q;
    (* DONT_TOUCH = "yes" *) logic [63:0]        s_tdata_q;
    (* DONT_TOUCH = "yes" *) logic signed [16:0] id_ref_q, iq_ref_q;
    (* DONT_TOUCH = "yes" *) logic [17:0]        kp_d_q, ki_d_q, kp_q_q, ki_q_q;
    (* DONT_TOUCH = "yes" *) logic [15:0]        u_limit_q;
    logic        core_s_tready, core_m_tvalid;
    logic [47:0] core_m_tdata;
    (* DONT_TOUCH = "yes" *) logic        s_tready_sink, m_tvalid_sink;
    (* DONT_TOUCH = "yes" *) logic [47:0] m_tdata_sink;

    always_ff @(posedge aclk) begin
        aresetn_q  <= aresetn;
        s_tvalid_q <= s_axis_tvalid;
        s_tdata_q  <= s_axis_tdata;
        m_tready_q <= m_axis_tready;
        id_ref_q   <= id_ref;
        iq_ref_q   <= iq_ref;
        kp_d_q     <= kp_d;
        ki_d_q     <= ki_d;
        kp_q_q     <= kp_q;
        ki_q_q     <= ki_q;
        u_limit_q  <= u_limit;
        s_tready_sink <= core_s_tready;
        m_tvalid_sink <= core_m_tvalid;
        m_tdata_sink  <= core_m_tdata;
    end

    foc_motor_control dut (
        .aclk(aclk), .aresetn(aresetn_q),
        .s_axis_tvalid(s_tvalid_q), .s_axis_tready(core_s_tready), .s_axis_tdata(s_tdata_q),
        .m_axis_tvalid(core_m_tvalid), .m_axis_tready(m_tready_q), .m_axis_tdata(core_m_tdata),
        .id_ref(id_ref_q), .iq_ref(iq_ref_q),
        .kp_d(kp_d_q), .ki_d(ki_d_q), .kp_q(kp_q_q), .ki_q(ki_q_q), .u_limit(u_limit_q)
    );
    assign s_axis_tready = s_tready_sink;
    assign m_axis_tvalid = m_tvalid_sink;
    assign m_axis_tdata  = m_tdata_sink;
endmodule
