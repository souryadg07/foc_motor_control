`timescale 1ns/1ps
`default_nettype none

// Inverse Park transform (rotating dq frame -> stationary alpha/beta):
//   v_alpha = v_d*cos(theta) - v_q*sin(theta)
//   v_beta  = v_d*sin(theta) + v_q*cos(theta)
//
// Same structure, formats and timing as park_transform: signed DATA_WIDTH+1
// bit inputs/outputs, trig from its own sincos_lut (Q15, <= 0.5 LSB), round
// half to even, saturation to the output range.
//
// Timing: one sample per clock, no backpressure. Inputs stable before edge M
// (with in_valid, and theta for the same sample) produce outputs right after
// edge M+3:
//   M   : capture v_d/v_q; sincos_lut reads theta
//   M+1 : trig register; second coordinate register
//   M+2 : four products
//   M+3 : sums, round, saturate
// Invalid cycles hold outputs. Synchronous active-high reset clears the valid
// pipeline and outputs. Needs sin_table.mem (see sincos_lut).

module inv_park_transform #(
    parameter int DATA_WIDTH  = 16,
    parameter int PHASE_WIDTH = 14
) (
    input  wire logic                         clk,
    input  wire logic                         rst,
    input  wire logic                         in_valid,
    input  wire logic [PHASE_WIDTH-1:0]       theta,
    input  wire logic signed [DATA_WIDTH:0]   v_d,
    input  wire logic signed [DATA_WIDTH:0]   v_q,
    output      logic                         out_valid,
    output      logic signed [DATA_WIDTH:0]   v_alpha,
    output      logic signed [DATA_WIDTH:0]   v_beta
);
    localparam int TRIG_WIDTH = 16;
    localparam int TRIG_FRAC  = 15;
    localparam int PROD_WIDTH = (DATA_WIDTH + 1) + TRIG_WIDTH;
    localparam int SUM_WIDTH  = PROD_WIDTH + 1;
    localparam int OUT_WIDTH  = DATA_WIDTH + 1;

    localparam logic signed [SUM_WIDTH-1:0] OUT_MAX = (1 << DATA_WIDTH) - 1;
    localparam logic signed [SUM_WIDTH-1:0] OUT_MIN = -(1 << DATA_WIDTH);

    // Trig: sincos_lut (1 clock, unregistered) + trig register = 2 clocks.
    logic signed [TRIG_WIDTH-1:0] sin_lut, cos_lut, sin_val, cos_val;

    sincos_lut #(.PHASE_WIDTH(PHASE_WIDTH), .OUT_WIDTH(TRIG_WIDTH)) u_sincos_lut (
        .clk(clk), .theta(theta), .sin_out(sin_lut), .cos_out(cos_lut)
    );

    always_ff @(posedge clk) begin
        sin_val <= sin_lut;
        cos_val <= cos_lut;
    end

    // floor(value / 2^TRIG_FRAC), +1 when above half or exactly half with an
    // odd result (half to even), then saturate. Shift and +1 are separate
    // statements so the shift stays arithmetic.
    function automatic logic signed [OUT_WIDTH-1:0] round_sat(
        input logic signed [SUM_WIDTH-1:0] value
    );
        logic signed [SUM_WIDTH-1:0] rounded;
        begin
            rounded = value >>> TRIG_FRAC;
            if (value[TRIG_FRAC-1] && ((|value[TRIG_FRAC-2:0]) || rounded[0]))
                rounded = rounded + 1'b1;
            if (rounded > OUT_MAX)
                round_sat = OUT_MAX[OUT_WIDTH-1:0];
            else if (rounded < OUT_MIN)
                round_sat = OUT_MIN[OUT_WIDTH-1:0];
            else
                round_sat = rounded[OUT_WIDTH-1:0];
        end
    endfunction

    logic signed [DATA_WIDTH:0]   vd_q, vq_q, vd_q2, vq_q2;
    logic signed [PROD_WIDTH-1:0] p_dc, p_qs, p_ds, p_qc;
    logic [2:0]                   valid_pipe;

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_pipe <= '0;
            out_valid  <= 1'b0;
            vd_q       <= '0;
            vq_q       <= '0;
            v_alpha    <= '0;
            v_beta     <= '0;
        end else begin
            valid_pipe <= {valid_pipe[1:0], in_valid};
            if (in_valid) begin
                vd_q <= v_d;
                vq_q <= v_q;
            end
            if (valid_pipe[2]) begin
                v_alpha <= round_sat(p_dc - p_qs);
                v_beta  <= round_sat(p_ds + p_qc);
            end
            out_valid <= valid_pipe[2];
        end
    end

    // Data-only stages (no reset): second coordinate register, products.
    always_ff @(posedge clk) begin
        vd_q2 <= vd_q;
        vq_q2 <= vq_q;
        if (valid_pipe[1]) begin
            p_dc <= vd_q2 * cos_val;
            p_qs <= vq_q2 * sin_val;
            p_ds <= vd_q2 * sin_val;
            p_qc <= vq_q2 * cos_val;
        end
    end

endmodule

`default_nettype wire
