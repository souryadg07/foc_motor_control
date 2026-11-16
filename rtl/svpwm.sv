`timescale 1ns/1ps
`default_nettype none

// Space-vector PWM duty-cycle calculation (min-max zero-sequence injection).
//
//   1. Inverse Clarke:  va = v_alpha
//                       vb = -v_alpha/2 + (sqrt(3)/2)*v_beta
//                       vc = -v_alpha/2 - (sqrt(3)/2)*v_beta
//   2. Centre:          off = (max(va,vb,vc) + min(va,vb,vc)) / 2
//   3. Duty:            d_x = 0.5 + (v_x - off) / Vdc,  clamped to [0, 1)
//
// This is mathematically identical to centred symmetric SVPWM (equal zero-
// vector time at both ends), without sector tables. Linear range:
// |v_alpha + j*v_beta| <= Vdc/sqrt(3); beyond it the duties clip.
//
// Formats
//   v_alpha, v_beta : signed DATA_WIDTH+1 bits, 2^(DATA_WIDTH-1) = Vdc
//                     (32768 = Vdc for DATA_WIDTH = 16; linear limit 18919)
//   duty_a/b/c      : unsigned DUTY_WIDTH bits, duty = code / 2^DUTY_WIDTH
//                     (the PWM generator scales this to its counter period)
// All intermediate values are exact; the only rounding is the final one
// (round half to even). sqrt(3)/2 is the Q15 constant 28378 (error 1.4e-5).
//
// Timing: one sample per clock, no backpressure. Inputs stable before edge M
// produce duties right after edge M+4:
//   M   : capture v_alpha, v_beta
//   M+1 : (sqrt(3)/2)*v_beta                  (DSP)
//   M+2 : va, vb, vc  (scaled by 2^15, exact)
//   M+3 : max and min of the three
//   M+4 : centre, scale, round, clamp
// Invalid cycles hold outputs. Synchronous active-high reset clears the valid
// pipeline and outputs (duties reset to 50 %).

module svpwm #(
    parameter int DATA_WIDTH = 16,
    parameter int DUTY_WIDTH = 16
) (
    input  wire logic                         clk,
    input  wire logic                         rst,
    input  wire logic                         in_valid,
    input  wire logic signed [DATA_WIDTH:0]   v_alpha,
    input  wire logic signed [DATA_WIDTH:0]   v_beta,
    output      logic                         out_valid,
    output      logic [DUTY_WIDTH-1:0]        duty_a,
    output      logic [DUTY_WIDTH-1:0]        duty_b,
    output      logic [DUTY_WIDTH-1:0]        duty_c
);
    localparam int FR    = DATA_WIDTH - 1;            // input fraction bits (Vdc units)
    localparam int KF    = 15;                        // fraction bits of sqrt(3)/2
    localparam int XW    = DATA_WIDTH + 1 + KF + 2;   // va/vb/vc scaled by 2^KF
    localparam int DW    = XW + 2;                    // 2*v - (max+min)
    localparam int SHIFT = FR + KF + 1 - DUTY_WIDTH;  // D -> duty LSBs

    localparam logic signed [KF:0]     K_SQRT3_2 = 16'sd28378;   // round(sqrt(3)/2 * 2^15)
    localparam logic signed [DW-1:0]   HALF      = DW'(1) <<< (DUTY_WIDTH-1);
    localparam logic signed [DW-1:0]   DUTY_MAX  = (DW'(1) <<< DUTY_WIDTH) - 1;

    logic [3:0] valid_pipe;

    // ---- M: capture ----
    logic signed [DATA_WIDTH:0] va_q, vb_q;
    always_ff @(posedge clk) begin
        if (in_valid) begin
            va_q <= v_alpha;
            vb_q <= v_beta;
        end
    end

    // ---- M+1: (sqrt(3)/2) * v_beta ----
    logic signed [DATA_WIDTH+KF+1:0] kb;
    logic signed [DATA_WIDTH:0]      va_q2;
    always_ff @(posedge clk) begin
        kb    <= vb_q * K_SQRT3_2;
        va_q2 <= va_q;
    end

    // ---- M+2: phase voltages, scaled by 2^KF (exact) ----
    logic signed [XW-1:0] xa, xb, xc, va_half;
    assign va_half = XW'(va_q2) <<< (KF - 1);
    always_ff @(posedge clk) begin
        xa <= XW'(va_q2) <<< KF;
        xb <= XW'(kb) - va_half;
        xc <= -XW'(kb) - va_half;
    end

    // ---- M+3: max and min ----
    logic signed [XW-1:0] mx, mn, xa2, xb2, xc2;
    always_ff @(posedge clk) begin
        mx  <= (xa >= xb) ? ((xa >= xc) ? xa : xc) : ((xb >= xc) ? xb : xc);
        mn  <= (xa <= xb) ? ((xa <= xc) ? xa : xc) : ((xb <= xc) ? xb : xc);
        xa2 <= xa;
        xb2 <= xb;
        xc2 <= xc;
    end

    // ---- M+4: duty = 0.5 + (2x - (max+min)) / 2^(SHIFT+DUTY_WIDTH) ----
    // Shift kept in its own signed assignment so it stays arithmetic.
    function automatic logic [DUTY_WIDTH-1:0] to_duty(input logic signed [DW-1:0] d);
        logic signed [DW-1:0] fl, r;
        begin
            fl = d >>> SHIFT;
            r  = fl + $signed({{(DW-1){1'b0}},
                               (d[SHIFT-1] && ((|d[SHIFT-2:0]) || fl[0]))});
            r  = r + HALF;
            if (r < 0)
                to_duty = '0;
            else if (r > DUTY_MAX)
                to_duty = DUTY_MAX[DUTY_WIDTH-1:0];
            else
                to_duty = r[DUTY_WIDTH-1:0];
        end
    endfunction

    logic signed [DW-1:0] sum_mm, da, db, dc;
    assign sum_mm = DW'(mx) + DW'(mn);
    assign da = (DW'(xa2) <<< 1) - sum_mm;
    assign db = (DW'(xb2) <<< 1) - sum_mm;
    assign dc = (DW'(xc2) <<< 1) - sum_mm;

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_pipe <= '0;
            out_valid  <= 1'b0;
            duty_a     <= DUTY_WIDTH'(1) << (DUTY_WIDTH-1);
            duty_b     <= DUTY_WIDTH'(1) << (DUTY_WIDTH-1);
            duty_c     <= DUTY_WIDTH'(1) << (DUTY_WIDTH-1);
        end else begin
            valid_pipe <= {valid_pipe[2:0], in_valid};
            out_valid  <= valid_pipe[3];
            if (valid_pipe[3]) begin
                duty_a <= to_duty(da);
                duty_b <= to_duty(db);
                duty_c <= to_duty(dc);
            end
        end
    end

    // synthesis translate_off
    initial begin
        assert (SHIFT >= 2)
            else $fatal(1, "DUTY_WIDTH too large for DATA_WIDTH (SHIFT=%0d)", SHIFT);
    end
    // synthesis translate_on

endmodule

`default_nettype wire
