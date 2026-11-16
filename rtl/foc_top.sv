`timescale 1ns/1ps
`default_nettype none

// FOC current-loop datapath with AXI-Stream interfaces:
//
//   s_axis (ia, ib, ic, theta)
//     -> [register slice] -> Clarke -> Park -> PI (d,q) -> inverse Park -> SVPWM
//     -> [output FIFO, registered] -> m_axis (duty_a, duty_b, duty_c)
//
// s_axis_tdata (64 bits):  [15:0] ia  [31:16] ib  [47:32] ic  (signed)
//                          [61:48] theta (unsigned, 2^14 = one turn)  [63:62] 0
// m_axis_tdata (48 bits):  [15:0] duty_a  [31:16] duty_b  [47:32] duty_c
//                          (unsigned Q0.16, duty = code / 65536)
// Control inputs (quasi-static set-points, see pi_controller / svpwm):
//   id_ref, iq_ref, kp_*, ki_*, u_limit. They are sampled when the matching
//   sample reaches the PI stage.
//
// Both AXI-Stream interfaces are registered: s_axis_tready, m_axis_tvalid and
// m_axis_tdata come straight from flip-flops.
//
// Flow control: the math pipeline has a fixed latency and never stalls. A
// credit counter tracks every sample that has entered the pipeline but has
// not yet left on m_axis, and only lets a new sample in when the output FIFO
// is guaranteed to have room for it. So no data is ever lost under
// backpressure, and with m_axis_tready held high the core accepts one sample
// per clock.
//
// Latency (no backpressure): a sample accepted on s_axis at clock edge E
// appears on m_axis right after edge E+19 (register slice 1, Clarke..PI 9,
// inverse Park 4, SVPWM 5, output register 1; see the testbenches).
//
// Reset: aresetn, active low, synchronous; registered internally. Samples in
// flight are discarded and the PI integrators are cleared.

module foc_top #(
    parameter int DATA_WIDTH  = 16,
    parameter int FRAC_BITS   = 15,
    parameter int PHASE_WIDTH = 14,
    parameter int GAIN_WIDTH  = 18,
    parameter int GAIN_FRAC   = 14,
    parameter int DUTY_WIDTH  = 16,
    parameter int FIFO_DEPTH  = 32
) (
    input  wire logic                         aclk,
    input  wire logic                         aresetn,
    // input stream: phase currents and electrical angle
    input  wire logic                         s_axis_tvalid,
    output      logic                         s_axis_tready,
    input  wire logic [63:0]                  s_axis_tdata,
    // output stream: duty cycles
    output      logic                         m_axis_tvalid,
    input  wire logic                         m_axis_tready,
    output      logic [47:0]                  m_axis_tdata,
    // control
    input  wire logic signed [DATA_WIDTH:0]   id_ref,
    input  wire logic signed [DATA_WIDTH:0]   iq_ref,
    input  wire logic [GAIN_WIDTH-1:0]        kp_d,
    input  wire logic [GAIN_WIDTH-1:0]        ki_d,
    input  wire logic [GAIN_WIDTH-1:0]        kp_q,
    input  wire logic [GAIN_WIDTH-1:0]        ki_q,
    input  wire logic [DATA_WIDTH-1:0]        u_limit
);
    localparam int THETA_DELAY = 9;   // chain input -> inverse Park input
    localparam int CREDITS     = FIFO_DEPTH + 1;
    localparam int CW          = $clog2(CREDITS + 1);

    // ---------------- reset ----------------
    logic rst;
    always_ff @(posedge aclk) rst <= !aresetn;

    // ---------------- input register slice ----------------
    logic        in_valid_sk, accept, credit_ok;
    logic [63:0] in_data;

    axis_skid_buffer #(.WIDTH(64)) u_in_slice (
        .clk(aclk), .rst(rst),
        .s_valid(s_axis_tvalid), .s_ready(s_axis_tready), .s_data(s_axis_tdata),
        .m_valid(in_valid_sk), .m_ready(credit_ok), .m_data(in_data)
    );

    // ---------------- credit counter ----------------
    // occ = samples accepted into the pipeline and not yet taken on m_axis.
    logic [CW-1:0] occ;
    logic          out_fire;

    assign credit_ok = (occ < CW'(CREDITS));
    assign accept    = in_valid_sk && credit_ok;
    assign out_fire  = m_axis_tvalid && m_axis_tready;

    always_ff @(posedge aclk) begin
        if (rst)
            occ <= '0;
        else
            occ <= occ + CW'(accept) - CW'(out_fire);
    end

    // ---------------- unpack ----------------
    logic signed [DATA_WIDTH-1:0] ia, ib, ic;
    logic [PHASE_WIDTH-1:0]       theta;
    assign ia    = in_data[15:0];
    assign ib    = in_data[31:16];
    assign ic    = in_data[47:32];
    assign theta = in_data[48 +: PHASE_WIDTH];

    // ---------------- Clarke -> Park -> PI ----------------
    logic                       pi_valid;
    logic signed [DATA_WIDTH:0] v_d, v_q;

    clarke_park_pi #(
        .DATA_WIDTH(DATA_WIDTH), .FRAC_BITS(FRAC_BITS), .PHASE_WIDTH(PHASE_WIDTH),
        .GAIN_WIDTH(GAIN_WIDTH), .GAIN_FRAC(GAIN_FRAC)
    ) u_cpp (
        .clk(aclk), .rst(rst), .in_valid(accept),
        .ia(ia), .ib(ib), .ic(ic), .theta(theta),
        .id_ref(id_ref), .iq_ref(iq_ref),
        .kp_d(kp_d), .ki_d(ki_d), .kp_q(kp_q), .ki_q(ki_q), .u_limit(u_limit),
        .out_valid(pi_valid), .v_d(v_d), .v_q(v_q)
    );

    // ---------------- theta for inverse Park ----------------
    // The pipeline never stalls, so delaying theta by the fixed number of
    // clocks between the chain input and the inverse Park input keeps it with
    // its own sample.
    logic [PHASE_WIDTH-1:0] theta_dl [THETA_DELAY];
    always_ff @(posedge aclk) begin
        theta_dl[0] <= theta;
        for (int i = 1; i < THETA_DELAY; i++)
            theta_dl[i] <= theta_dl[i-1];
    end

    // ---------------- inverse Park ----------------
    logic                       ipark_valid;
    logic signed [DATA_WIDTH:0] v_alpha, v_beta;

    inv_park_transform #(.DATA_WIDTH(DATA_WIDTH), .PHASE_WIDTH(PHASE_WIDTH)) u_ipark (
        .clk(aclk), .rst(rst), .in_valid(pi_valid),
        .theta(theta_dl[THETA_DELAY-1]), .v_d(v_d), .v_q(v_q),
        .out_valid(ipark_valid), .v_alpha(v_alpha), .v_beta(v_beta)
    );

    // ---------------- SVPWM ----------------
    logic                  svpwm_valid;
    logic [DUTY_WIDTH-1:0] duty_a, duty_b, duty_c;

    svpwm #(.DATA_WIDTH(DATA_WIDTH), .DUTY_WIDTH(DUTY_WIDTH)) u_svpwm (
        .clk(aclk), .rst(rst), .in_valid(ipark_valid),
        .v_alpha(v_alpha), .v_beta(v_beta),
        .out_valid(svpwm_valid), .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c)
    );

    // ---------------- output FIFO (registered m_axis) ----------------
    axis_fifo #(.WIDTH(48), .DEPTH(FIFO_DEPTH)) u_out_fifo (
        .clk(aclk), .rst(rst),
        .wr_en(svpwm_valid), .wr_data({duty_c, duty_b, duty_a}),
        .m_valid(m_axis_tvalid), .m_ready(m_axis_tready), .m_data(m_axis_tdata)
    );

    // synthesis translate_off
    initial begin
        assert (DATA_WIDTH == 16 && PHASE_WIDTH == 14 && DUTY_WIDTH == 16)
            else $fatal(1, "foc_top tdata packing assumes DATA_WIDTH=16, PHASE_WIDTH=14, DUTY_WIDTH=16");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
