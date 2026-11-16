`timescale 1ns/1ps
`default_nettype none

// One channel of the FOC current-loop PI regulator (instantiate once for d,
// once for q).
//
//   e[n] = setpoint - meas
//   I[n] = clamp( I[n-1] + Ki*e[n],  +/- u_limit )      (anti-windup clamp)
//   u[n] = clamp( round(Kp*e[n] + I[n]),  +/- u_limit )
//
// Number formats
//   setpoint, meas, u : signed DATA_WIDTH+1 bits, the same fixed-point format
//                       as park_transform's i_d/i_q (FRAC_BITS fractional).
//   kp, ki            : unsigned GAIN_WIDTH bits, value = code / 2^GAIN_FRAC.
//                       ki is the per-sample gain (Ki * Ts).
//   u_limit           : unsigned DATA_WIDTH bits, output magnitude limit.
//
// Precision: Kp*e and Ki*e are kept at full precision; the integrator holds
// GAIN_FRAC extra fractional bits so small Ki increments are not lost. Only
// the final sum is rounded (round half to even, as park_transform does).
//
// Anti-windup: the integrator is clamped to the same +/- u_limit as the
// output, so after a long saturation it can recover as soon as the error
// changes sign instead of first unwinding a huge accumulated value.
//
// Timing: one sample per clock, no backpressure. Inputs stable before edge N
// (with in_valid) produce u at edge N+3:
//   N   : e = setpoint - meas; gains and limit captured with the sample
//   N+1 : Kp*e, Ki*e                         (DSP multipliers)
//   N+2 : I = clamp(I + Ki*e)                (only on valid samples)
//   N+3 : u = clamp(round(Kp*e + I))         (only on valid samples)
// Gains and limit travel with their sample, so they may change at any time.
// Invalid cycles hold u. Synchronous active-high reset clears the integrator,
// the valid pipeline and u.

module pi_controller #(
    parameter int DATA_WIDTH = 16,
    parameter int GAIN_WIDTH = 18,
    parameter int GAIN_FRAC  = 14
) (
    input  wire logic                         clk,
    input  wire logic                         rst,
    input  wire logic                         in_valid,
    input  wire logic signed [DATA_WIDTH:0]   setpoint,
    input  wire logic signed [DATA_WIDTH:0]   meas,
    input  wire logic [GAIN_WIDTH-1:0]        kp,
    input  wire logic [GAIN_WIDTH-1:0]        ki,
    input  wire logic [DATA_WIDTH-1:0]        u_limit,
    output      logic                         out_valid,
    output      logic signed [DATA_WIDTH:0]   u
);

    localparam int XW = DATA_WIDTH + 1;          // setpoint/meas/u width
    localparam int EW = XW + 1;                  // error width (no overflow)
    localparam int PW = EW + GAIN_WIDTH + 1;     // e * gain (gain made signed)
    localparam int AW = PW + 1;                  // integrator / I + Ki*e
    localparam int SW = AW + 1;                  // Kp*e + I

    // ------------------------------------------------------------------
    // Stage 1 (edge N): error, and capture gains/limit with the sample
    // ------------------------------------------------------------------
    logic signed [EW-1:0]   e_q;
    logic [GAIN_WIDTH-1:0]  kp_q, ki_q;
    logic [DATA_WIDTH-1:0]  lim_q1;
    logic [2:0]             valid_pipe;

    always_ff @(posedge clk) begin
        e_q    <= EW'(setpoint) - EW'(meas);
        kp_q   <= kp;
        ki_q   <= ki;
        lim_q1 <= u_limit;
    end

    // ------------------------------------------------------------------
    // Stage 2 (edge N+1): P and I-increment products
    // ------------------------------------------------------------------
    logic signed [PW-1:0]   p_term, i_inc;
    logic [DATA_WIDTH-1:0]  lim_q2;

    always_ff @(posedge clk) begin
        p_term <= e_q * $signed({1'b0, kp_q});
        i_inc  <= e_q * $signed({1'b0, ki_q});
        lim_q2 <= lim_q1;
    end

    // ------------------------------------------------------------------
    // Stage 3 (edge N+2): integrator with anti-windup clamp
    // ------------------------------------------------------------------
    // Limit expressed in the integrator's units (GAIN_FRAC extra bits).
    logic signed [AW-1:0] i_lim, i_sum, integ;
    logic signed [PW-1:0] p_q3;
    logic [DATA_WIDTH-1:0] lim_q3;

    assign i_lim = $signed({{(AW-DATA_WIDTH){1'b0}}, lim_q2}) <<< GAIN_FRAC;
    assign i_sum = integ + i_inc;

    always_ff @(posedge clk) begin
        if (rst)
            integ <= '0;
        else if (valid_pipe[1]) begin
            if (i_sum > i_lim)
                integ <= i_lim;
            else if (i_sum < -i_lim)
                integ <= -i_lim;
            else
                integ <= i_sum;
        end
    end

    always_ff @(posedge clk) begin
        p_q3   <= p_term;
        lim_q3 <= lim_q2;
    end

    // ------------------------------------------------------------------
    // Stage 4 (edge N+3): sum, round, saturate
    // ------------------------------------------------------------------
    // Arithmetic shift gives floor(sum / 2^GAIN_FRAC); add 1 when the dropped
    // part is above half, or exactly half with an odd result (half to even).
    // The shift is kept in its own signed assignment and the +1 is made
    // explicitly signed: one unsigned operand would make the whole expression
    // unsigned and turn >>> into a logical shift (negative sums -> huge).
    logic signed [SW-1:0] u_sum, u_floor, u_round;
    logic signed [SW-1:0] u_lim;
    logic                 round_up;

    assign u_sum    = p_q3 + integ;
    assign u_floor  = u_sum >>> GAIN_FRAC;
    assign round_up = u_sum[GAIN_FRAC-1] &&
                      ((|u_sum[GAIN_FRAC-2:0]) || u_sum[GAIN_FRAC]);
    assign u_round  = u_floor + $signed({{(SW-1){1'b0}}, round_up});
    assign u_lim    = $signed({{(SW-DATA_WIDTH){1'b0}}, lim_q3});

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_pipe <= '0;
            out_valid  <= 1'b0;
            u          <= '0;
        end else begin
            valid_pipe <= {valid_pipe[1:0], in_valid};
            out_valid  <= valid_pipe[2];
            if (valid_pipe[2]) begin
                if (u_round > u_lim)
                    u <= u_lim[XW-1:0];
                else if (u_round < -u_lim)
                    u <= XW'(-u_lim);
                else
                    u <= u_round[XW-1:0];
            end
        end
    end

    // synthesis translate_off
    initial begin
        assert (DATA_WIDTH >= 2)
            else $fatal(1, "DATA_WIDTH must be at least 2 (got %0d)", DATA_WIDTH);
        assert (GAIN_FRAC >= 2 && GAIN_FRAC < GAIN_WIDTH)
            else $fatal(1, "GAIN_FRAC must be in [2, GAIN_WIDTH-1] (got %0d)", GAIN_FRAC);
        assert (AW > DATA_WIDTH + GAIN_FRAC + 1)
            else $fatal(1, "integrator too narrow for u_limit << GAIN_FRAC");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
