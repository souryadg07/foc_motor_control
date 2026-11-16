`timescale 1ns/1ps

// Park transform (rotation into the synchronous dq frame):
//   i_d =  i_alpha*cos(theta) + i_beta*sin(theta);
//   i_q = -i_alpha*sin(theta) + i_beta*cos(theta).
// i_alpha/i_beta are signed fixed point with DATA_WIDTH+1 bits and FRAC_BITS
// fractional bits (matches clarke_transform outputs); i_d/i_q have the same
// format. theta is an unsigned PHASE_WIDTH-bit turn: 0..2^PHASE_WIDTH-1 maps
// to 0..2*pi, so PHASE_WIDTH=14 gives 0.022 degrees per LSB.
// Sine/cosine come from sincos_lut (quarter-wave block-RAM table, Q15,
// <= 0.5 LSB error, latency 1, unregistered outputs) followed by one trig
// register here, giving trig latency 2 -- the same timing the dds_sincos IP
// had, so the rest of this pipeline is unchanged. sincos_lut needs
// sin_table.mem (rtl/park/sincos_lut, from gen_sin_table.py) in the
// simulation/synthesis file set; it is generated for PHASE_WIDTH = 14.
// One sample may be accepted per clock; no backpressure. Inputs stable before
// edge N produce registered outputs at edge N+3 (trig 2 stages overlapping
// the input/coordinate stages, product stage 1, sum/round stage 1). Invalid
// cycles hold outputs. Synchronous active-high reset clears the valid
// pipeline and outputs; the trig path has no reset and needs none, since its
// values are only used when the valid pipeline says so. Results saturate to
// the output range instead of wrapping.
module park_transform #(
    parameter int DATA_WIDTH  = 16,
    parameter int FRAC_BITS   = 15,
    parameter int PHASE_WIDTH = 14
) (
    input  logic                         clk,
    input  logic                         rst,
    input  logic                         in_valid,
    input  logic [PHASE_WIDTH-1:0]       theta,
    input  logic signed [DATA_WIDTH:0]   i_alpha,
    input  logic signed [DATA_WIDTH:0]   i_beta,
    output logic                         out_valid,
    output logic signed [DATA_WIDTH:0]   i_d,
    output logic signed [DATA_WIDTH:0]   i_q
);
    localparam int TRIG_WIDTH  = 16;
    localparam int TRIG_FRAC   = 15;
    localparam int PROD_WIDTH  = (DATA_WIDTH + 1) + TRIG_WIDTH;
    localparam int SUM_WIDTH   = PROD_WIDTH + 1;
    localparam int ROUND_WIDTH = SUM_WIDTH - TRIG_FRAC;
    localparam int OUT_WIDTH   = DATA_WIDTH + 1;

    localparam logic signed [SUM_WIDTH-1:0] OUT_MAX = (1 << DATA_WIDTH) - 1;
    localparam logic signed [SUM_WIDTH-1:0] OUT_MIN = -(1 << DATA_WIDTH);

    logic signed [TRIG_WIDTH-1:0] sin_lut, cos_lut;   // after 1 clock, unregistered
    logic signed [TRIG_WIDTH-1:0] sin_val, cos_val;   // after 2 clocks, registered

    sincos_lut #(
        .PHASE_WIDTH (PHASE_WIDTH),
        .OUT_WIDTH   (TRIG_WIDTH)
    ) u_sincos_lut (
        .clk     (clk),
        .theta   (theta),
        .sin_out (sin_lut),
        .cos_out (cos_lut)
    );

    // Trig register: brings trig latency to 2 (as dds_sincos had) and gives
    // the multipliers registered inputs instead of the block-RAM output plus
    // sincos_lut's sign logic in the same cycle.
    always_ff @(posedge clk) begin
        sin_val <= sin_lut;
        cos_val <= cos_lut;
    end

    // Arithmetic shift gives floor(value / 2**TRIG_FRAC), even for negatives.
    // Increment when above halfway, or halfway with an odd retained result.
    // Then saturate to the two's-complement output range.
    function automatic logic signed [OUT_WIDTH-1:0] round_sat(
        input logic signed [SUM_WIDTH-1:0] value
    );
        logic signed [SUM_WIDTH-1:0] rounded;
        begin
            rounded = value >>> TRIG_FRAC;
            if (value[TRIG_FRAC-1] &&
                ((|value[TRIG_FRAC-2:0]) || rounded[0]))
                rounded = rounded + 1'b1;
            if (rounded > OUT_MAX)
                round_sat = OUT_MAX[OUT_WIDTH-1:0];
            else if (rounded < OUT_MIN)
                round_sat = OUT_MIN[OUT_WIDTH-1:0];
            else
                round_sat = rounded[OUT_WIDTH-1:0];
        end
    endfunction

    logic signed [DATA_WIDTH:0]   alpha_q, beta_q;
    logic signed [DATA_WIDTH:0]   alpha_q2, beta_q2;
    logic signed [PROD_WIDTH-1:0] p_ac, p_bs, p_as, p_bc;
    logic [3:0]                   valid_pipe;

    always_ff @(posedge clk) begin
        if (rst) begin
            out_valid  <= 1'b0;
            valid_pipe <= '0;
            alpha_q    <= '0;
            beta_q     <= '0;
            alpha_q2   <= '0;
            beta_q2    <= '0;
            p_ac       <= '0;
            p_bs       <= '0;
            p_as       <= '0;
            p_bc       <= '0;
            i_d        <= '0;
            i_q        <= '0;
        end else begin
            valid_pipe <= {valid_pipe[2:0], in_valid};
            if (in_valid) begin
                alpha_q <= i_alpha;
                beta_q  <= i_beta;
            end
            // Unconditional second stage: alpha_q2 holds the current sample's
            // coordinates through the edge where the products are taken.
            alpha_q2 <= alpha_q;
            beta_q2  <= beta_q;
            // sin_val/cos_val at this edge are trig(theta) for sample T
            // (2 clocks after its phase was sampled).
            if (valid_pipe[1]) begin
                p_ac <= alpha_q2 * cos_val;
                p_bs <= beta_q2  * sin_val;
                p_as <= alpha_q2 * sin_val;
                p_bc <= beta_q2  * cos_val;
            end
            if (valid_pipe[2]) begin
                i_d <= round_sat(p_ac + p_bs);
                i_q <= round_sat(p_bc - p_as);
            end
            out_valid <= valid_pipe[2];
        end
    end

    // Configuration checks are for simulation, not datapath hardware.
    // synthesis translate_off
    initial begin
        assert (DATA_WIDTH >= 2 && DATA_WIDTH <= 24)
            else $fatal(1, "DATA_WIDTH must be in [2,24]");
        assert (FRAC_BITS >= 0 && FRAC_BITS < DATA_WIDTH)
            else $fatal(1, "FRAC_BITS must be in [0,DATA_WIDTH-1]");
        assert (PHASE_WIDTH == 14)
            else $fatal(1, "PHASE_WIDTH must be 14 to match sin_table.mem (regenerate it with gen_sin_table.py for other widths)");
    end
    // synthesis translate_on
endmodule
