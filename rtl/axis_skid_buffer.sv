`timescale 1ns/1ps
`default_nettype none

// AXI-Stream register slice (2-entry skid buffer).
//   - s_ready is a register (no combinational path from m_ready to s_ready)
//   - m_valid/m_data are registers
//   - full throughput: one transfer per clock when m_ready stays high
// When the output stalls, one extra beat is caught in the skid register and
// s_ready drops on the next clock. Synchronous active-high reset; s_ready is
// low during reset.

module axis_skid_buffer #(
    parameter int WIDTH = 64
) (
    input  wire logic             clk,
    input  wire logic             rst,
    input  wire logic             s_valid,
    output      logic             s_ready,
    input  wire logic [WIDTH-1:0] s_data,
    output      logic             m_valid,
    input  wire logic             m_ready,
    output      logic [WIDTH-1:0] m_data
);
    logic             skid_valid;
    logic [WIDTH-1:0] skid_data;

    logic s_fire, out_free;
    logic m_valid_n, skid_valid_n;
    logic load_m_from_s, load_m_from_skid, load_skid;

    assign s_fire   = s_valid && s_ready;
    assign out_free = !m_valid || m_ready;

    always_comb begin
        m_valid_n        = m_valid;
        skid_valid_n     = skid_valid;
        load_m_from_s    = 1'b0;
        load_m_from_skid = 1'b0;
        load_skid        = 1'b0;
        if (out_free) begin
            if (skid_valid) begin
                // Drain the skid first (s_ready is low while it is full).
                load_m_from_skid = 1'b1;
                m_valid_n        = 1'b1;
                skid_valid_n     = 1'b0;
            end else begin
                load_m_from_s = s_fire;
                m_valid_n     = s_fire;
            end
        end else if (s_fire) begin
            // Output stalled: park the incoming beat in the skid register.
            load_skid    = 1'b1;
            skid_valid_n = 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            m_valid    <= 1'b0;
            skid_valid <= 1'b0;
            s_ready    <= 1'b0;
        end else begin
            m_valid    <= m_valid_n;
            skid_valid <= skid_valid_n;
            s_ready    <= !skid_valid_n;
        end
    end

    always_ff @(posedge clk) begin
        if (load_m_from_skid)
            m_data <= skid_data;
        else if (load_m_from_s)
            m_data <= s_data;
        if (load_skid)
            skid_data <= s_data;
    end

endmodule

`default_nettype wire
