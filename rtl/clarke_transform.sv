`timescale 1ns/1ps

// Three-input, amplitude-invariant Clarke transform:
//   alpha = (2*ia - ib - ic)/3; beta = (ib - ic)/sqrt(3).
// All ports are signed fixed point, with FRAC_BITS fractional bits.
// Supported: DATA_WIDTH 2..24, FRAC_BITS 0..DATA_WIDTH-1.
// One sample may be accepted per clock; no backpressure.
// Inputs stable before edge N produce registered outputs at edge N (one stage).
// Invalid cycles hold outputs. Synchronous active-high reset clears outputs
// and flushes pending samples. Inputs must already be clock-synchronous.
// Coefficient approximation plus rounding is within 1 output LSB of ideal.
module clarke_transform #(
    parameter int DATA_WIDTH = 16,
    parameter int FRAC_BITS  = 15
) (
    input  logic                         clk,
    input  logic                         rst,
    input  logic                         in_valid,
    input  logic signed [DATA_WIDTH-1:0]  ia,
    input  logic signed [DATA_WIDTH-1:0]  ib,
    input  logic signed [DATA_WIDTH-1:0]  ic,
    output logic                         out_valid,
    output logic signed [DATA_WIDTH:0]    i_alpha,
    output logic signed [DATA_WIDTH:0]    i_beta
);
    localparam int SUM_WIDTH   = DATA_WIDTH + 2;
    localparam int COEFF_FRAC  = DATA_WIDTH + 2;
    localparam int COEFF_WIDTH = COEFF_FRAC + 1;
    localparam int PROD_WIDTH  = SUM_WIDTH + COEFF_WIDTH;

    // Rounded Q30 reference constants, reduced to the required precision.
    // Integer-only elaboration; no real-valued arithmetic in the hardware.
    localparam logic signed [COEFF_WIDTH-1:0] INV_THREE =
        (64'sd357913941 + (64'sd1 << (29-COEFF_FRAC))) >>> (30-COEFF_FRAC);
    localparam logic signed [COEFF_WIDTH-1:0] INV_SQRT_THREE =
        (64'sd619925131 + (64'sd1 << (29-COEFF_FRAC))) >>> (30-COEFF_FRAC);

    logic signed [SUM_WIDTH-1:0] ia_ext, ib_ext, ic_ext;

    // Extend BEFORE arithmetic, including the left shift.
    assign ia_ext = {{2{ia[DATA_WIDTH-1]}}, ia};
    assign ib_ext = {{2{ib[DATA_WIDTH-1]}}, ib};
    assign ic_ext = {{2{ic[DATA_WIDTH-1]}}, ic};

    // Arithmetic shift gives floor(value / 2**COEFF_FRAC), even for negatives.
    // Increment when above halfway, or halfway with an odd retained result.
    function automatic logic signed [DATA_WIDTH:0] round_even(
        input logic signed [PROD_WIDTH-1:0] value
    );
        logic signed [PROD_WIDTH-1:0] rounded;
        begin
            rounded = value >>> COEFF_FRAC;
            if (value[COEFF_FRAC-1] &&
                ((|value[COEFF_FRAC-2:0]) || rounded[0]))
                rounded = rounded + 1'b1;
            round_even = rounded[DATA_WIDTH:0];
        end
    endfunction

    logic signed [SUM_WIDTH-1:0] alpha_sum_ext, beta_diff_ext;
    logic signed [PROD_WIDTH-1:0] alpha_product_ext, beta_product_ext;
    always_comb begin
        alpha_sum_ext = ((ia_ext <<< 1) - ib_ext - ic_ext);
        beta_diff_ext = (ib_ext - ic_ext);
        alpha_product_ext = alpha_sum_ext * INV_THREE;
        beta_product_ext  = beta_diff_ext * INV_SQRT_THREE;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            out_valid     <= 1'b0;
            i_alpha       <= '0;
            i_beta        <= '0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                i_alpha <= round_even(alpha_product_ext);
                i_beta  <= round_even(beta_product_ext);
            end
        end
    end

    // Configuration checks are for simulation, not datapath hardware.
    // synthesis translate_off
    initial begin
        assert (DATA_WIDTH >= 2 && DATA_WIDTH <= 24)
            else $fatal(1, "DATA_WIDTH must be in [2,24]");
        assert (FRAC_BITS >= 0 && FRAC_BITS < DATA_WIDTH)
            else $fatal(1, "FRAC_BITS must be in [0,DATA_WIDTH-1]");
    end
    // synthesis translate_on
endmodule
