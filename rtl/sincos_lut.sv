`timescale 1ns/1ps
`default_nettype none

// Quarter-wave sine/cosine lookup, replacement for dds_sincos (same values
// within 1 LSB, but 1 clock less latency -- see "Latency" below).
//
// One ROM of QUARTER_DEPTH x OUT_WIDTH entries holds sin(theta) magnitudes
// (unsigned Q15: 0..32767) for the first quadrant [0, pi/2). Both sin and cos
// are reconstructed from it:
//   - sin: quadrant q = theta[PHASE_WIDTH-1 -: 2], index = theta[IDX_WIDTH-1:0]
//   - cos: cos(t) = sin(t + pi/2). Adding a quarter turn only increments the
//          quadrant (q_cos = q + 1); the index is unchanged.
// Quadrant rules (applied to q for sin, to q_cos for cos -- identical rules):
//   mirror index when quadrant[0] == 1, negate when quadrant[1] == 1
// Mirroring is exact: idx -> N - idx (N = QUARTER_DEPTH). At idx == 0 in a
// mirrored quadrant that address is N, one past the table: the angle is
// exactly pi/2 or 3*pi/2 and the magnitude is the peak, supplied directly.
//
// ROM contents come from gen_sin_table.py (plain table:
// rom[k] = round(32767 * sin(k * (pi/2) / QUARTER_DEPTH)), rom[0] == 0).
// Every output is therefore round(32767 * sin/cos(2*pi*theta / 2^PHASE_WIDTH)):
// error <= 0.5 LSB, rounding only.
//
// Latency: 1 clock. theta sampled at edge N -> sin_out/cos_out valid after
// edge N (in time for edge N+1).
//   stage 1: block-RAM read (magnitudes) + sign and peak flags (registered)
//   then:    peak substitution + sign application, combinational
// NOTE: this is NOT latency-compatible with dds_sincos (Latency 2):
// park_transform's valid pipeline must be shortened by one stage if this
// module replaces the IP there. Outputs are not registered.
// One new theta per clock, no stall, no reset (none of these registers need
// a known value; outputs are valid from the first clock edge onwards).
// Resources: one dual-port block RAM (sin on port A, cos on port B).

module sincos_lut #(
    parameter int    PHASE_WIDTH = 14,              // SPEC: 2^14 phases = 0..2*pi
    parameter int    OUT_WIDTH   = 16,              // SPEC: Q15 signed output
    parameter string MEM_FILE    = "sin_table.mem"  // from gen_sin_table.py
) (
    input  wire logic                        clk,
    input  wire logic [PHASE_WIDTH-1:0]      theta,  // 0..2^PHASE_WIDTH-1 maps to 0..2*pi
    output      logic signed [OUT_WIDTH-1:0] sin_out,
    output      logic signed [OUT_WIDTH-1:0] cos_out
);

    // ------------------------------------------------------------------
    // 1. Geometry constants
    // ------------------------------------------------------------------
    localparam int QUARTER_DEPTH = 1 << (PHASE_WIDTH - 2);
    localparam int IDX_WIDTH     = PHASE_WIDTH - 2;

    // ------------------------------------------------------------------
    // 2. ROM declaration
    // ------------------------------------------------------------------
    // Unsigned magnitudes, one per first-quadrant phase step.
    (* rom_style = "block" *) logic [OUT_WIDTH-1:0] rom [QUARTER_DEPTH];

    initial begin
        $readmemh(MEM_FILE, rom);
        // synthesis translate_off
        check_config();
        // synthesis translate_on
    end

    // ------------------------------------------------------------------
    // 3. Quadrant decomposition
    // ------------------------------------------------------------------
    logic [1:0]           q, q_cos;
    logic [IDX_WIDTH-1:0] idx;

    assign {q, idx} = theta;
    assign q_cos    = q + 2'd1;

    // ------------------------------------------------------------------
    // 4. Index mirroring
    // ------------------------------------------------------------------
    logic [IDX_WIDTH-1:0] neg_idx, sin_addr, cos_addr;
    logic                 idx_zero, sin_peak, cos_peak;

    assign neg_idx  = -idx;
    assign idx_zero = (idx == '0);

    assign sin_addr = q[0] ? neg_idx : idx;
    assign cos_addr = q[0] ? idx     : neg_idx;

    assign sin_peak = q[0]     & idx_zero;   // theta = pi/2 or 3*pi/2
    assign cos_peak = q_cos[0] & idx_zero;   // theta = 0 or pi

    // ------------------------------------------------------------------
    // 5. ROM reads (stage 1, both ports of one memory)
    // ------------------------------------------------------------------
    logic [OUT_WIDTH-1:0] sin_mag, cos_mag;
    logic                 sin_neg, cos_neg;
    logic                 sin_peak_q, cos_peak_q;

    always_ff @(posedge clk) begin
        sin_mag <= rom[sin_addr];
        cos_mag <= rom[cos_addr];
    end

    always_ff @(posedge clk) begin
        sin_neg    <= q[1];
        cos_neg    <= q_cos[1];     // == q[1] ^ q[0]: original quadrant 1 or 2
        sin_peak_q <= sin_peak;
        cos_peak_q <= cos_peak;
    end

    // ------------------------------------------------------------------
    // 6. Sign application (combinational, after the stage-1 registers)
    // ------------------------------------------------------------------
    logic [OUT_WIDTH-1:0] sin_full, cos_full;

    assign sin_full = sin_mag | {1'b0, {(OUT_WIDTH-1){sin_peak_q}}};
    assign cos_full = cos_mag | {1'b0, {(OUT_WIDTH-1){cos_peak_q}}};

    assign sin_out = sin_neg ? -$signed(sin_full) : $signed(sin_full);
    assign cos_out = cos_neg ? -$signed(cos_full) : $signed(cos_full);

    // ------------------------------------------------------------------
    // 7. Simulation-only configuration checks
    // ------------------------------------------------------------------
    // synthesis translate_off
    task automatic check_config();
        assert (PHASE_WIDTH >= 3)
            else $fatal(1, "PHASE_WIDTH must be at least 3 (got %0d)", PHASE_WIDTH);
        assert (OUT_WIDTH >= 2)
            else $fatal(1, "OUT_WIDTH must be at least 2 (got %0d)", OUT_WIDTH);

        foreach (rom[k]) begin
            // Missing file, short file, or bad hex leaves X entries.
            assert (!$isunknown(rom[k]))
                else $fatal(1, "%s: rom[%0d] is X -- file missing, short, or malformed",
                            MEM_FILE, k);
            // MSB set would read as negative after $signed() in section 6.
            assert (rom[k][OUT_WIDTH-1] == 1'b0)
                else $fatal(1, "%s: rom[%0d] = %h exceeds the max magnitude %0d",
                            MEM_FILE, k, rom[k], (1 << (OUT_WIDTH-1)) - 1);
            // A first-quadrant sine never decreases; catches wrong or
            // reversed tables and tables generated for another depth.
            if (k > 0)
                assert (rom[k] >= rom[k-1])
                    else $fatal(1, "%s: not monotonic at rom[%0d] (%h < %h)",
                                MEM_FILE, k, rom[k], rom[k-1]);
        end

        // Section 6 builds the peak by OR-ing into rom[0]; that is only
        // correct if rom[0] is 0 (plain table, not an offset one).
        assert (rom[0] == '0)
            else $fatal(1, "%s: rom[0] = %h, must be 0 (generate with OFFSET = 0.0)",
                        MEM_FILE, rom[0]);
    endtask
    // synthesis translate_on

endmodule

`default_nettype wire
