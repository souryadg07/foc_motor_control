`timescale 1ns/1ps
`default_nettype none

// FOC current-path front end: abc currents -> Clarke -> Park -> PI (d and q)
// -> voltage commands v_d, v_q.
//
//   ia/ib/ic --> clarke_transform --> i_alpha/i_beta --+
//   theta ----> (1-clock align) ---------------------> park_transform --> i_d/i_q
//   id_ref/iq_ref, gains, limit -----------------------> pi_controller x2 --> v_d/v_q
//
// Formats: ia/ib/ic signed DATA_WIDTH bits; everything after Clarke is signed
// DATA_WIDTH+1 bits with FRAC_BITS fractional bits. Gains: see pi_controller.
//
// Latency: inputs stable before edge N produce v_d/v_q right after edge N+8
// (Clarke result after N, Park after N+4, PI after N+8). One sample per clock.
// The PI references, gains and limit are sampled when the matching i_d/i_q
// sample reaches the PI stage (4 clocks after the abc sample); they are
// quasi-static set-points in a motor drive, so this is the usual behaviour.
// Synchronous active-high reset clears every stage, including the integrators.

module clarke_park_pi #(
    parameter int DATA_WIDTH  = 16,
    parameter int FRAC_BITS   = 15,
    parameter int PHASE_WIDTH = 14,
    parameter int GAIN_WIDTH  = 18,
    parameter int GAIN_FRAC   = 14
) (
    input  wire logic                         clk,
    input  wire logic                         rst,
    input  wire logic                         in_valid,
    input  wire logic signed [DATA_WIDTH-1:0] ia,
    input  wire logic signed [DATA_WIDTH-1:0] ib,
    input  wire logic signed [DATA_WIDTH-1:0] ic,
    input  wire logic [PHASE_WIDTH-1:0]       theta,
    input  wire logic signed [DATA_WIDTH:0]   id_ref,
    input  wire logic signed [DATA_WIDTH:0]   iq_ref,
    input  wire logic [GAIN_WIDTH-1:0]        kp_d,
    input  wire logic [GAIN_WIDTH-1:0]        ki_d,
    input  wire logic [GAIN_WIDTH-1:0]        kp_q,
    input  wire logic [GAIN_WIDTH-1:0]        ki_q,
    input  wire logic [DATA_WIDTH-1:0]        u_limit,
    output      logic                         out_valid,
    output      logic signed [DATA_WIDTH:0]   v_d,
    output      logic signed [DATA_WIDTH:0]   v_q
);

    // Clarke: 1 register stage.
    logic                       clarke_valid;
    logic signed [DATA_WIDTH:0] i_alpha, i_beta;

    clarke_transform #(.DATA_WIDTH(DATA_WIDTH), .FRAC_BITS(FRAC_BITS)) u_clarke (
        .clk(clk), .rst(rst), .in_valid(in_valid),
        .ia(ia), .ib(ib), .ic(ic),
        .out_valid(clarke_valid), .i_alpha(i_alpha), .i_beta(i_beta)
    );

    // theta must reach Park together with the Clarke result of its sample.
    logic [PHASE_WIDTH-1:0] theta_p;
    always_ff @(posedge clk) theta_p <= theta;

    // Park: latency 3.
    logic                       park_valid;
    logic signed [DATA_WIDTH:0] i_d, i_q;

    park_transform #(
        .DATA_WIDTH(DATA_WIDTH), .FRAC_BITS(FRAC_BITS), .PHASE_WIDTH(PHASE_WIDTH)
    ) u_park (
        .clk(clk), .rst(rst), .in_valid(clarke_valid), .theta(theta_p),
        .i_alpha(i_alpha), .i_beta(i_beta),
        .out_valid(park_valid), .i_d(i_d), .i_q(i_q)
    );

    // PI, one per axis: latency 3. Both channels run in lockstep, so the
    // d channel's out_valid serves for both.
    logic pi_q_valid_unused;

    pi_controller #(
        .DATA_WIDTH(DATA_WIDTH), .GAIN_WIDTH(GAIN_WIDTH), .GAIN_FRAC(GAIN_FRAC)
    ) u_pi_d (
        .clk(clk), .rst(rst), .in_valid(park_valid),
        .setpoint(id_ref), .meas(i_d), .kp(kp_d), .ki(ki_d), .u_limit(u_limit),
        .out_valid(out_valid), .u(v_d)
    );

    pi_controller #(
        .DATA_WIDTH(DATA_WIDTH), .GAIN_WIDTH(GAIN_WIDTH), .GAIN_FRAC(GAIN_FRAC)
    ) u_pi_q (
        .clk(clk), .rst(rst), .in_valid(park_valid),
        .setpoint(iq_ref), .meas(i_q), .kp(kp_q), .ki(ki_q), .u_limit(u_limit),
        .out_valid(pi_q_valid_unused), .u(v_q)
    );

endmodule

`default_nettype wire
