`timescale 1ns/1ps
`default_nettype none

// Synchronous first-word-fall-through FIFO with a registered AXI-Stream
// output (m_valid/m_data are registers).
//   - storage: DEPTH entries (distributed RAM) + the output register
//   - when the FIFO is empty and the output register is free, a write goes
//     straight into the output register (1 clock from wr_en to m_valid)
//   - the writer must not overflow it: total capacity is DEPTH + 1 beats.
//     (foc_top guarantees this with a credit counter; simulation asserts it.)
// Synchronous active-high reset empties the FIFO.

module axis_fifo #(
    parameter int WIDTH = 48,
    parameter int DEPTH = 32          // power of two
) (
    input  wire logic             clk,
    input  wire logic             rst,
    input  wire logic             wr_en,
    input  wire logic [WIDTH-1:0] wr_data,
    output      logic             m_valid,
    input  wire logic             m_ready,
    output      logic [WIDTH-1:0] m_data
);
    localparam int AW = $clog2(DEPTH);

    logic [WIDTH-1:0] mem [DEPTH];
    logic [AW-1:0]    wr_ptr, rd_ptr;
    logic [AW:0]      count;

    logic out_free, mem_empty, bypass, mem_rd, mem_wr;
    assign out_free  = !m_valid || m_ready;
    assign mem_empty = (count == '0);
    assign bypass    = out_free && mem_empty && wr_en;   // straight to output
    assign mem_rd    = out_free && !mem_empty;           // refill output
    assign mem_wr    = wr_en && !bypass;

    always_ff @(posedge clk) begin
        if (mem_wr)
            mem[wr_ptr] <= wr_data;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr  <= '0;
            rd_ptr  <= '0;
            count   <= '0;
            m_valid <= 1'b0;
        end else begin
            if (mem_wr) wr_ptr <= wr_ptr + 1'b1;
            if (mem_rd) rd_ptr <= rd_ptr + 1'b1;
            count <= count + mem_wr - mem_rd;
            if (out_free)
                m_valid <= mem_rd || bypass;
        end
    end

    always_ff @(posedge clk) begin
        if (mem_rd)
            m_data <= mem[rd_ptr];
        else if (bypass)
            m_data <= wr_data;
    end

    // synthesis translate_off
    initial assert (DEPTH >= 2 && (DEPTH & (DEPTH-1)) == 0)
        else $fatal(1, "axis_fifo DEPTH must be a power of two >= 2 (got %0d)", DEPTH);
    always @(posedge clk)
        if (!rst && mem_wr && !mem_rd && count == DEPTH)
            $fatal(1, "axis_fifo OVERFLOW: write while full");
    // synthesis translate_on

endmodule

`default_nettype wire
