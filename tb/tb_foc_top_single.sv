`timescale 1ns/1ps

// Single-sample walkthrough of foc_top, for waveform viewing.
//
// Exactly ONE AXI-Stream beat is sent; the sink is always ready. Each stage's
// valid pulses once, so the sample can be followed through the whole loop:
//
//   s_axis : ia, ib, ic = 20000, -10000, -10000, theta = 2048 (45 deg)
//   Clarke : alpha = 20000, beta = 0
//   Park   : d = 14142, q = -14142
//   PI     : Kp = 0.25, Ki = 0.05, id_ref = 0, iq_ref = 10000
//            v_d = 0.30 * (0 - 14142)       = -4242.4  -> -4242
//            v_q = 0.30 * (10000 + 14142)   =  7242.3  ->  7242
//   InvPark: v_alpha = (v_d - v_q) * cos45  = -8120.25 -> -8120
//            v_beta  = (v_d + v_q) * cos45  =  2121.3  ->  2121
//   SVPWM  : va = -0.2478 Vdc, vb = 0.1800, vc = 0.0679, offset = -0.0339
//            duty_a = 0.2861 -> 18751, duty_b = 0.7139 -> 46785,
//            duty_c = 0.6018 -> 39437   (duty_a + duty_b = 65536: centred)
//   m_axis : tdata = {39437, 46785, 18751}
//
// Each stage's output is asserted against these values.
module tb_foc_top_single;
  logic aclk=0;
  always #5 aclk=~aclk;

  logic aresetn=0;
  logic s_axis_tvalid=0;  wire s_axis_tready;  logic [63:0] s_axis_tdata=0;
  wire  m_axis_tvalid;    logic m_axis_tready=1; wire [47:0] m_axis_tdata;
  logic signed [16:0] id_ref=17'sd0, iq_ref=17'sd10000;
  logic [17:0] kp_d=18'd4096, ki_d=18'd819, kp_q=18'd4096, ki_q=18'd819;  // 0.25, 0.05
  logic [15:0] u_limit=16'd18000;

  foc_top dut (.*);

  // Decoded views of the two streams (for the waveform).
  wire signed [15:0] in_ia    = s_axis_tdata[15:0];
  wire signed [15:0] in_ib    = s_axis_tdata[31:16];
  wire signed [15:0] in_ic    = s_axis_tdata[47:32];
  wire        [13:0] in_theta = s_axis_tdata[61:48];
  wire        [15:0] out_duty_a = m_axis_tdata[15:0];
  wire        [15:0] out_duty_b = m_axis_tdata[31:16];
  wire        [15:0] out_duty_c = m_axis_tdata[47:32];

  // Clock-edge counter: edge k is at 10k-5 ns.
  integer edge_no=0;
  always @(posedge aclk) edge_no <= edge_no + 1;

  integer e_hs=-1, e_acc=-1;

  task automatic expect_eq(input string what, input longint got, input longint exp);
    if (got !== exp) $fatal(1, "SINGLE SAMPLE MISMATCH %s: got %0d expected %0d", what, got, exp);
  endtask

  always @(posedge aclk) begin
    if (s_axis_tvalid && s_axis_tready) e_hs = edge_no + 1;
    if (dut.accept) e_acc = edge_no + 1;
    #1;
    if (e_hs==edge_no)
      $display("STAGE s_axis    edge=%0d t=%0t  handshake (E)", edge_no, $time);
    if (e_acc==edge_no)
      $display("STAGE accept    edge=%0d (E+%0d) t=%0t  chain samples it (N)", edge_no, edge_no-e_hs, $time);
    if (dut.u_cpp.clarke_valid) begin
      $display("STAGE Clarke    edge=%0d (N+%0d) t=%0t  alpha=%0d beta=%0d", edge_no, edge_no-e_acc, $time,
               dut.u_cpp.i_alpha, dut.u_cpp.i_beta);
      expect_eq("alpha", dut.u_cpp.i_alpha, 20000); expect_eq("beta", dut.u_cpp.i_beta, 0);
    end
    if (dut.u_cpp.park_valid) begin
      $display("STAGE Park      edge=%0d (N+%0d) t=%0t  d=%0d q=%0d", edge_no, edge_no-e_acc, $time,
               dut.u_cpp.i_d, dut.u_cpp.i_q);
      expect_eq("d", dut.u_cpp.i_d, 14142); expect_eq("q", dut.u_cpp.i_q, -14142);
    end
    if (dut.pi_valid) begin
      $display("STAGE PI        edge=%0d (N+%0d) t=%0t  v_d=%0d v_q=%0d", edge_no, edge_no-e_acc, $time,
               dut.v_d, dut.v_q);
      expect_eq("v_d", dut.v_d, -4242); expect_eq("v_q", dut.v_q, 7242);
    end
    if (dut.ipark_valid) begin
      $display("STAGE InvPark   edge=%0d (N+%0d) t=%0t  v_alpha=%0d v_beta=%0d", edge_no, edge_no-e_acc, $time,
               dut.v_alpha, dut.v_beta);
      expect_eq("v_alpha", dut.v_alpha, -8120); expect_eq("v_beta", dut.v_beta, 2121);
    end
    if (dut.svpwm_valid) begin
      $display("STAGE SVPWM     edge=%0d (N+%0d) t=%0t  duty_a=%0d duty_b=%0d duty_c=%0d", edge_no, edge_no-e_acc,
               $time, dut.duty_a, dut.duty_b, dut.duty_c);
      expect_eq("duty_a", dut.duty_a, 18751); expect_eq("duty_b", dut.duty_b, 46785);
      expect_eq("duty_c", dut.duty_c, 39437);
    end
    if (m_axis_tvalid && m_axis_tready)
      $display("STAGE m_axis    edge=%0d (E+%0d) t=%0t  tvalid with duties %0d/%0d/%0d (handshake at next edge)",
               edge_no, edge_no-e_hs, $time, out_duty_a, out_duty_b, out_duty_c);
  end

  integer n_out=0;
  always @(posedge aclk) if (m_axis_tvalid && m_axis_tready) n_out = n_out + 1;

  initial begin
    repeat (4) @(negedge aclk);
    aresetn=1;
    repeat (3) @(negedge aclk);

    // The single beat: valid until its handshake.
    s_axis_tdata = {2'b00, 14'd2048, 16'hD8F0, 16'hD8F0, 16'h4E20};   // ic, ib = -10000; ia = 20000
    s_axis_tvalid = 1;
    @(posedge aclk); while (!s_axis_tready) @(posedge aclk);
    @(negedge aclk);
    s_axis_tvalid = 0; s_axis_tdata = '0;

    repeat (26) @(negedge aclk);
    if (n_out != 1) $fatal(1, "expected exactly one m_axis beat, got %0d", n_out);
    expect_eq("m_axis_tdata", m_axis_tdata, {16'd39437, 16'd46785, 16'd18751});
    $display("PASS: one beat abc=(20000,-10000,-10000) theta=45deg -> duties %0d/%0d/%0d",
             out_duty_a, out_duty_b, out_duty_c);
    $finish;
  end
endmodule
