`timescale 1ns/1ps

// Regression testbench: foc_top
//   s_axis -> slice -> Clarke -> Park -> PI -> inverse Park -> SVPWM -> FIFO -> m_axis
//
// Every stage is checked on every sample at its own boundary:
//   Clarke, Park   : real-valued reference, tolerances as tb_clarke_park
//   PI             : bit-exact integer model fed with the DUT's Park outputs
//   inverse Park   : bit-exact model (same quarter-wave table as sincos_lut)
//                    fed with the DUT's PI outputs and the TB's OWN copy of
//                    each sample's theta (so a theta misalignment is caught);
//                    plus a real-valued check (|err| <= 4 LSB)
//   SVPWM          : bit-exact model fed with the DUT's inverse Park outputs;
//                    plus a real-valued check (|err| <= 3 LSB of Q0.16)
//   AXI            : every s_axis beat reaches the chain in order; every
//                    SVPWM result leaves on m_axis in order, none lost or
//                    duplicated; m_axis holds tvalid/tdata while stalled;
//                    the credit counter never exceeds the FIFO capacity.
// Latency is checked exactly at every stage boundary (relative to the edge
// the chain accepts a sample), plus end-to-end s_axis -> m_axis.
// Throughput: 16384 back-to-back samples must take exactly 16384 clocks.
module foc_top_checker (output bit done=0);
  localparam integer W=16, F=15, P=14, GW=18, GF=14, DUTYW=16, FIFO_DEPTH=32;
  localparam longint signed OUT_MAX = (64'sd1 << W) - 1;
  localparam longint signed OUT_MIN = -(64'sd1 << W);
  localparam integer TOL_AB=2, TOL_PHYS=32, TOL_STRESS=64;
  localparam integer TOL_IPARK_REAL=4, TOL_SVPWM_REAL=3;
  localparam real    TWO_PI = 6.283185307179586;
  localparam real    PI_R   = 3.141592653589793;

  bit clk=0;
  always #5 clk=~clk;

  logic aresetn=0;
  logic s_tvalid=0;  wire s_tready;  logic [63:0] s_tdata=0;
  wire  m_tvalid;    logic m_tready=0; wire [47:0] m_tdata;
  logic signed [W:0] id_ref=0, iq_ref=0;
  logic [GW-1:0] kp_d=0, ki_d=0, kp_q=0, ki_q=0;
  logic [W-1:0] u_limit=0;

  foc_top #(.FIFO_DEPTH(FIFO_DEPTH)) dut (
    .aclk(clk), .aresetn(aresetn),
    .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready), .s_axis_tdata(s_tdata),
    .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready), .m_axis_tdata(m_tdata),
    .id_ref(id_ref), .iq_ref(iq_ref), .kp_d(kp_d), .ki_d(ki_d),
    .kp_q(kp_q), .ki_q(ki_q), .u_limit(u_limit)
  );

  // ================= reference models =================
  function automatic longint signed math_rne(input real x);
    real mag, fraction; longint signed q;
    begin
      mag=(x<0)?-x:x; q=$rtoi($floor(mag)); fraction=mag-q;
      if ((fraction>0.5) || ((fraction==0.5) && q[0])) q=q+1;
      math_rne=(x<0)?-q:q;
    end
  endfunction

  function automatic longint signed clamp(input longint signed v);
    clamp = (v > OUT_MAX) ? OUT_MAX : ((v < OUT_MIN) ? OUT_MIN : v);
  endfunction

  function automatic longint signed rne_shift(input longint signed s, input integer f);
    longint signed r, rem, half;
    begin
      r=s>>>f; rem=s-(r<<<f); half=64'sd1<<<(f-1);
      if ((rem>half) || ((rem==half) && r[0])) r=r+1;
      rne_shift=r;
    end
  endfunction

  task automatic pi_step(input longint signed sp, input longint signed meas,
                         input longint signed kp, input longint signed ki,
                         input longint signed lim, inout longint signed integ,
                         output longint signed u, output bit sat);
    longint signed e, s, r, ilim;
    begin
      e=sp-meas; ilim=lim<<<GF; s=integ+ki*e;
      integ=(s>ilim)?ilim:((s<-ilim)?-ilim:s);
      r=rne_shift(kp*e+integ,GF);
      sat=(r>lim)||(r<-lim);
      u=(r>lim)?lim:((r<-lim)?-lim:r);
    end
  endtask

  // Same quarter-wave table and quadrant logic as sincos_lut.
  longint signed rom[4096];
  initial for (int k=0;k<4096;k++)
    rom[k]=$rtoi(32767.0*$sin(($itor(k)*(PI_R/2.0))/4096.0)+0.5);

  task automatic trig(input integer th, output longint signed s, output longint signed c);
    integer q, qc, idx, nidx, sa, ca;
    longint signed sm, cm;
    begin
      q=th>>12; idx=th&4095; qc=(q+1)&3; nidx=(4096-idx)&4095;
      sa=(q&1)?nidx:idx; ca=(q&1)?idx:nidx;
      sm=((q&1) && idx==0) ? 32767 : rom[sa];
      cm=((qc&1) && idx==0) ? 32767 : rom[ca];
      s=(q>>1)?-sm:sm; c=(qc>>1)?-cm:cm;
    end
  endtask

  task automatic ipark_model(input longint signed vd, input longint signed vq, input integer th,
                             output longint signed va, output longint signed vb);
    longint signed s, c;
    begin
      trig(th,s,c);
      va=clamp(rne_shift(vd*c - vq*s, 15));
      vb=clamp(rne_shift(vd*s + vq*c, 15));
    end
  endtask

  function automatic longint signed duty_clamp(input longint signed r);
    duty_clamp=(r<0)?0:((r>65535)?65535:r);
  endfunction

  task automatic svpwm_model(input longint signed va, input longint signed vb,
                             output longint signed da, output longint signed db,
                             output longint signed dc);
    longint signed xa, xb, xc, kb, mx, mn;
    begin
      kb=vb*28378; xa=va<<<15; xb=kb-(va<<<14); xc=-kb-(va<<<14);
      mx=(xa>=xb)?((xa>=xc)?xa:xc):((xb>=xc)?xb:xc);
      mn=(xa<=xb)?((xa<=xc)?xa:xc):((xb<=xc)?xb:xc);
      da=duty_clamp(rne_shift(2*xa-(mx+mn),15)+32768);
      db=duty_clamp(rne_shift(2*xb-(mx+mn),15)+32768);
      dc=duty_clamp(rne_shift(2*xc-(mx+mn),15)+32768);
    end
  endtask

  // ================= scoreboard state =================
  logic [63:0]   sent_q[$];
  longint signed qab_a[$], qab_b[$]; integer qab_tol[$];
  longint signed qd[$], qq[$];       integer qdq_tol[$];
  integer        th_q[$];
  longint signed qvd[$], qvq[$], qva[$], qvb[$], qda[$], qdb[$], qdc[$];
  logic [47:0]   axi_q[$];

  logic [17:0] vpipe=0;
  longint signed held_a=0, held_b=0, held_d=0, held_q=0, held_vd=0, held_vq=0,
                 held_va=0, held_vb=0, held_da=32768, held_db=32768, held_dc=32768;
  longint signed integ_d=0, integ_q=0;
  integer cycles=0, checked_ab=0, checked_dq=0, checked_pi=0, checked_ipark=0,
          checked_svpwm=0, received=0, sent=0, holds=0, resets=0;
  integer pi_sat=0, ipark_sat=0, duty_clip=0, max_err_ab=0, max_err_phys=0,
          max_err_stress=0, s_backpressure=0, m_stall=0, max_occ=0;
  real    max_ipark_real=0.0, max_svpwm_real=0.0;
  bit stress_mode=0, m_stalled=0; logic [47:0] m_stalled_data;
  integer hs_cycle_first=-1, hs_cycle_last=-1, probe_hs=-1, probe_out=-1, probing=0;

  longint signed a,b,c, ea,eb, ed,eq, da,db, ud,uq, va,vb, xa_,xb_,xc_, sd_, s_,c_;
  real ar, br, dr, qr, th, rv, ra_, rb_, rc_, off_, rmx, rmn;
  integer err, k, j, tq;
  bit satd, satq;

  task automatic real_svpwm(input longint signed va_i, input longint signed vb_i,
                            output real ra, output real rb, output real rc);
    real x, y, mxr, mnr, o;
    begin
      x=$itor(va_i)/32768.0; y=$itor(vb_i)/32768.0;
      ra=x; rb=-x/2.0+0.8660254037844386*y; rc=-x/2.0-0.8660254037844386*y;
      mxr=(ra>rb)?((ra>rc)?ra:rc):((rb>rc)?rb:rc);
      mnr=(ra<rb)?((ra<rc)?ra:rc):((rb<rc)?rb:rc);
      o=(mxr+mnr)/2.0;
      ra=(0.5+ra-o)*65536.0; rb=(0.5+rb-o)*65536.0; rc=(0.5+rc-o)*65536.0;
      ra=(ra<0)?0:((ra>65535)?65535:ra);
      rb=(rb<0)?0:((rb>65535)?65535:rb);
      rc=(rc<0)?0:((rc>65535)?65535:rc);
    end
  endtask

  function automatic real rabs(input real x); rabs=(x<0)?-x:x; endfunction

  // ================= per-edge checking =================
  // foc_top registers its reset internally, so its state is X until the
  // first edge with the internal reset high; checking starts there.
  bit armed=0;
  always @(posedge clk) begin
    if (dut.rst === 1'b1) armed = 1;
    if (armed) begin
    cycles=cycles+1;
    // ---------- pre-edge: what the DUT samples at this edge ----------
    if (dut.rst) begin
      sent_q.delete(); qab_a.delete(); qab_b.delete(); qab_tol.delete();
      qd.delete(); qq.delete(); qdq_tol.delete(); th_q.delete();
      qvd.delete(); qvq.delete(); qva.delete(); qvb.delete();
      qda.delete(); qdb.delete(); qdc.delete(); axi_q.delete();
      held_a=0; held_b=0; held_d=0; held_q=0; held_vd=0; held_vq=0; held_va=0; held_vb=0;
      held_da=32768; held_db=32768; held_dc=32768;
      integ_d=0; integ_q=0; m_stalled=0;
      resets=resets+1;
    end else begin
      // s_axis handshake
      if (s_tvalid && s_tready) begin
        sent_q.push_back(s_tdata); sent=sent+1;
        if (hs_cycle_first<0) hs_cycle_first=cycles;
        hs_cycle_last=cycles;
        if (probing==1) begin probe_hs=cycles; probing=2; end
      end
      if (s_tvalid && !s_tready) s_backpressure=s_backpressure+1;
      // chain accepts a sample
      if (dut.accept) begin
        if (sent_q.size()==0) $fatal(1,"AXI-IN accept with no sent beat, cycle=%0d",cycles);
        if (sent_q.pop_front() !== dut.in_data)
          $fatal(1,"AXI-IN ORDER/DATA mismatch at chain input, cycle=%0d",cycles);
        a=$signed(dut.ia); b=$signed(dut.ib); c=$signed(dut.ic);
        ar=(2.0*a-b-c)/3.0; br=(b-c)/1.7320508075688772;
        qab_a.push_back(math_rne(ar)); qab_b.push_back(math_rne(br)); qab_tol.push_back(TOL_AB);
        th=TWO_PI*$itor(dut.theta)/16384.0;
        dr=ar*$cos(th)+br*$sin(th); qr=br*$cos(th)-ar*$sin(th);
        qd.push_back(clamp(math_rne(dr))); qq.push_back(clamp(math_rne(qr)));
        qdq_tol.push_back(stress_mode?TOL_STRESS:TOL_PHYS);
        th_q.push_back(dut.theta);
      end
      // PI samples Park output
      if (dut.u_cpp.park_valid) begin
        pi_step(id_ref, dut.u_cpp.i_d, kp_d, ki_d, u_limit, integ_d, ud, satd);
        pi_step(iq_ref, dut.u_cpp.i_q, kp_q, ki_q, u_limit, integ_q, uq, satq);
        qvd.push_back(ud); qvq.push_back(uq); pi_sat=pi_sat+satd+satq;
      end
      // inverse Park samples PI output (with the TB's own copy of theta)
      if (dut.pi_valid) begin
        if (th_q.size()==0) $fatal(1,"IPARK capture with no theta, cycle=%0d",cycles);
        tq=th_q.pop_front();
        ipark_model($signed(dut.v_d), $signed(dut.v_q), tq, va, vb);
        qva.push_back(va); qvb.push_back(vb);
        if (va==OUT_MAX || va==OUT_MIN || vb==OUT_MAX || vb==OUT_MIN) ipark_sat=ipark_sat+1;
        // real-valued sanity of the same rotation
        th=TWO_PI*$itor(tq)/16384.0;
        rv=$itor($signed(dut.v_d))*$cos(th)-$itor($signed(dut.v_q))*$sin(th);
        rv=(rv>OUT_MAX)?OUT_MAX:((rv<OUT_MIN)?OUT_MIN:rv);
        if (rabs(rv-va)>max_ipark_real) max_ipark_real=rabs(rv-va);
        rv=$itor($signed(dut.v_d))*$sin(th)+$itor($signed(dut.v_q))*$cos(th);
        rv=(rv>OUT_MAX)?OUT_MAX:((rv<OUT_MIN)?OUT_MIN:rv);
        if (rabs(rv-vb)>max_ipark_real) max_ipark_real=rabs(rv-vb);
      end
      // SVPWM samples inverse Park output
      if (dut.ipark_valid) begin
        svpwm_model($signed(dut.v_alpha), $signed(dut.v_beta), xa_, xb_, xc_);
        qda.push_back(xa_); qdb.push_back(xb_); qdc.push_back(xc_);
        real_svpwm($signed(dut.v_alpha), $signed(dut.v_beta), ra_, rb_, rc_);
        if (rabs(ra_-xa_)>max_svpwm_real) max_svpwm_real=rabs(ra_-xa_);
        if (rabs(rb_-xb_)>max_svpwm_real) max_svpwm_real=rabs(rb_-xb_);
        if (rabs(rc_-xc_)>max_svpwm_real) max_svpwm_real=rabs(rc_-xc_);
        if (xa_==0||xa_==65535||xb_==0||xb_==65535||xc_==0||xc_==65535) duty_clip=duty_clip+1;
      end
      // m_axis handshake
      if (m_tvalid && m_tready) begin
        if (axi_q.size()==0) $fatal(1,"AXI-OUT unexpected beat, cycle=%0d",cycles);
        if (axi_q.pop_front() !== m_tdata)
          $fatal(1,"AXI-OUT ORDER/DATA mismatch, cycle=%0d",cycles);
        received=received+1;
      end
      m_stalled = m_tvalid && !m_tready;
      m_stalled_data = m_tdata;
      if (m_stalled) m_stall=m_stall+1;
    end
    vpipe <= dut.rst ? 18'b0 : {vpipe[16:0], dut.accept};
    if (dut.occ > FIFO_DEPTH+1) $fatal(1,"CREDIT overflow occ=%0d",dut.occ);
    if (dut.occ > max_occ) max_occ=dut.occ;

    #1;
    // ---------- post-edge: stage outputs ----------
    if (m_stalled && !dut.rst && (m_tvalid!==1'b1 || m_tdata!==m_stalled_data))
      $fatal(1,"AXI-OUT protocol: tvalid/tdata changed while stalled, cycle=%0d",cycles);
    if (probing==2 && m_tvalid) begin probe_out=cycles; probing=3; end

    // Clarke
    if (dut.u_cpp.clarke_valid !== vpipe[0]) $fatal(1,"CLARKE LATENCY cycle=%0d",cycles);
    if (dut.u_cpp.clarke_valid) begin
      ea=qab_a.pop_front(); eb=qab_b.pop_front(); err=qab_tol.pop_front();
      da=($signed(dut.u_cpp.i_alpha)>ea)?($signed(dut.u_cpp.i_alpha)-ea):(ea-$signed(dut.u_cpp.i_alpha));
      db=($signed(dut.u_cpp.i_beta)>eb)?($signed(dut.u_cpp.i_beta)-eb):(eb-$signed(dut.u_cpp.i_beta));
      if (da>err || db>err) $fatal(1,"CLARKE MISMATCH cycle=%0d exp=(%0d,%0d) got=(%0d,%0d)",
                                   cycles,ea,eb,$signed(dut.u_cpp.i_alpha),$signed(dut.u_cpp.i_beta));
      if (da>max_err_ab) max_err_ab=da;
      if (db>max_err_ab) max_err_ab=db;
      held_a=$signed(dut.u_cpp.i_alpha); held_b=$signed(dut.u_cpp.i_beta);
      checked_ab=checked_ab+1;
    end else begin
      holds=holds+1;
      if ($signed(dut.u_cpp.i_alpha)!==held_a || $signed(dut.u_cpp.i_beta)!==held_b)
        $fatal(1,"CLARKE HOLD cycle=%0d",cycles);
    end
    // Park
    if (dut.u_cpp.park_valid !== vpipe[4]) $fatal(1,"PARK LATENCY cycle=%0d",cycles);
    if (dut.u_cpp.park_valid) begin
      ed=qd.pop_front(); eq=qq.pop_front(); err=qdq_tol.pop_front();
      da=($signed(dut.u_cpp.i_d)>ed)?($signed(dut.u_cpp.i_d)-ed):(ed-$signed(dut.u_cpp.i_d));
      db=($signed(dut.u_cpp.i_q)>eq)?($signed(dut.u_cpp.i_q)-eq):(eq-$signed(dut.u_cpp.i_q));
      if (da>err || db>err) $fatal(1,"PARK MISMATCH cycle=%0d exp=(%0d,%0d) got=(%0d,%0d)",
                                   cycles,ed,eq,$signed(dut.u_cpp.i_d),$signed(dut.u_cpp.i_q));
      if (err==TOL_STRESS) begin
        if (da>max_err_stress) max_err_stress=da;
        if (db>max_err_stress) max_err_stress=db;
      end else begin
        if (da>max_err_phys) max_err_phys=da;
        if (db>max_err_phys) max_err_phys=db;
      end
      held_d=$signed(dut.u_cpp.i_d); held_q=$signed(dut.u_cpp.i_q);
      checked_dq=checked_dq+1;
    end else if ($signed(dut.u_cpp.i_d)!==held_d || $signed(dut.u_cpp.i_q)!==held_q)
      $fatal(1,"PARK HOLD cycle=%0d",cycles);
    // PI
    if (dut.pi_valid !== vpipe[8]) $fatal(1,"PI LATENCY cycle=%0d",cycles);
    if (dut.pi_valid) begin
      ud=qvd.pop_front(); uq=qvq.pop_front();
      if ($signed(dut.v_d)!==ud || $signed(dut.v_q)!==uq)
        $fatal(1,"PI MISMATCH cycle=%0d exp=(%0d,%0d) got=(%0d,%0d)",cycles,ud,uq,$signed(dut.v_d),$signed(dut.v_q));
      held_vd=ud; held_vq=uq; checked_pi=checked_pi+1;
    end else if ($signed(dut.v_d)!==held_vd || $signed(dut.v_q)!==held_vq)
      $fatal(1,"PI HOLD cycle=%0d",cycles);
    // inverse Park
    if (dut.ipark_valid !== vpipe[12]) $fatal(1,"IPARK LATENCY cycle=%0d",cycles);
    if (dut.ipark_valid) begin
      va=qva.pop_front(); vb=qvb.pop_front();
      if ($signed(dut.v_alpha)!==va || $signed(dut.v_beta)!==vb)
        $fatal(1,"IPARK MISMATCH cycle=%0d exp=(%0d,%0d) got=(%0d,%0d)",cycles,va,vb,$signed(dut.v_alpha),$signed(dut.v_beta));
      held_va=va; held_vb=vb; checked_ipark=checked_ipark+1;
    end else if ($signed(dut.v_alpha)!==held_va || $signed(dut.v_beta)!==held_vb)
      $fatal(1,"IPARK HOLD cycle=%0d",cycles);
    // SVPWM
    if (dut.svpwm_valid !== vpipe[17]) $fatal(1,"SVPWM LATENCY cycle=%0d",cycles);
    if (dut.svpwm_valid) begin
      xa_=qda.pop_front(); xb_=qdb.pop_front(); xc_=qdc.pop_front();
      if (dut.duty_a!==xa_ || dut.duty_b!==xb_ || dut.duty_c!==xc_)
        $fatal(1,"SVPWM MISMATCH cycle=%0d exp=(%0d,%0d,%0d) got=(%0d,%0d,%0d)",
               cycles,xa_,xb_,xc_,dut.duty_a,dut.duty_b,dut.duty_c);
      held_da=xa_; held_db=xb_; held_dc=xc_;
      axi_q.push_back({dut.duty_c, dut.duty_b, dut.duty_a});
      checked_svpwm=checked_svpwm+1;
    end else if (dut.duty_a!==held_da || dut.duty_b!==held_db || dut.duty_c!==held_dc)
      $fatal(1,"SVPWM HOLD cycle=%0d",cycles);
    end
  end

  // ================= stimulus =================
  integer ready_mode=0;   // 0: always ready, 1: 70 %, 2: 30 %, 3: never
  logic [31:0] seed=32'h6a09e667;
  function automatic logic [31:0] rand32();
    seed=seed^(seed<<13); seed=seed^(seed>>17); seed=seed^(seed<<5);
    return seed;
  endfunction

  always @(negedge clk)
    case (ready_mode)
      0: m_tready <= 1'b1;
      1: m_tready <= (rand32()%10) < 7;
      2: m_tready <= (rand32()%10) < 3;
      default: m_tready <= 1'b0;
    endcase

  function automatic logic [63:0] beat(input longint signed x, input longint signed y,
                                       input longint signed z, input integer t);
    beat={2'b00, t[13:0], z[15:0], y[15:0], x[15:0]};
  endfunction

  // Drive one beat; return after its handshake (pre-edge s_tready high).
  task automatic axis_send(input logic [63:0] d, input integer gap);
    repeat (gap) begin @(negedge clk); s_tvalid=0; end
    @(negedge clk); s_tvalid=1; s_tdata=d;
    @(posedge clk);
    while (!s_tready) @(posedge clk);
  endtask

  task automatic axis_idle(input integer n);
    repeat (n) begin @(negedge clk); s_tvalid=0; end
  endtask

  task automatic set_gains_random();
    kp_d=rand32()[15:0]; kp_q=rand32()[15:0];          // up to 4.0
    ki_d=rand32()[12:0]; ki_q=rand32()[12:0];          // up to 0.5
    id_ref=$signed(rand32()[W:0]) >>> 1; iq_ref=$signed(rand32()[W:0]) >>> 1;
  endtask

  task automatic drain(input integer max_cycles);
    integer n; n=0;
    ready_mode=0; axis_idle(1);
    while ((received < sent) && n < max_cycles) begin @(negedge clk); n=n+1; end
    if (received != sent) $fatal(1,"DRAIN timeout: sent=%0d received=%0d",sent,received);
  endtask

  longint signed sv_a[3], sv_b[3], sv_c[3];
  integer probe_latency, tp_cycles;

  initial begin : main
    sv_a[0]=20000;  sv_b[0]=-10000; sv_c[0]=-10000;
    sv_a[1]=23170;  sv_b[1]=23170;  sv_c[1]=-16384;
    sv_a[2]=-16384; sv_b[2]=28377;  sv_c[2]=-11993;

    kp_d=18'd4096; ki_d=18'd819; kp_q=18'd4096; ki_q=18'd819;   // 0.25, 0.05
    u_limit=16'd18000; id_ref=0; iq_ref=17'sd8000;
    ready_mode=0;
    repeat (5) @(negedge clk);
    aresetn=1;
    repeat (5) @(negedge clk);

    // ---- 1. Latency probe: one sample into an idle core.
    probing=1;
    axis_send(beat(20000,-10000,-10000,2048),0);
    axis_idle(30);
    probe_latency=probe_out-probe_hs;

    // ---- 2. Exhaustive phase sweep, back-to-back, always ready (throughput).
    hs_cycle_first=-1;
    for (k=0;k<16384;k=k+1)
      axis_send(beat(sv_a[k%3],sv_b[k%3],sv_c[k%3],k),0);
    @(negedge clk);   // let the checker record the last handshake first
    tp_cycles=hs_cycle_last-hs_cycle_first+1;
    if (tp_cycles != 16384)
      $fatal(1,"THROUGHPUT: 16384 back-to-back beats took %0d clocks",tp_cycles);
    drain(200);

    // ---- 3. Random data, random gaps, 70 % ready.
    ready_mode=1;
    for (k=0;k<10000;k=k+1) begin
      if ((k%500)==0) begin set_gains_random(); u_limit=rand32()[15:0]; end
      axis_send(beat($signed(rand32()[15:0]),$signed(rand32()[15:0]),
                     $signed(rand32()[15:0]),rand32()[13:0]), rand32()%3);
    end

    // ---- 4. Heavy backpressure (30 % ready), back-to-back source.
    ready_mode=2;
    for (k=0;k<5000;k=k+1) begin
      if ((k%500)==0) set_gains_random();
      axis_send(beat($signed(rand32()[14:0]),$signed(rand32()[14:0]),
                     $signed(rand32()[14:0]),rand32()[13:0]), 0);
    end
    drain(20000);

    // ---- 5. Full stall: sink never ready for 300 clocks, source keeps sending.
    fork
      begin
        for (j=0;j<100;j=j+1)
          axis_send(beat($signed(rand32()[14:0]),$signed(rand32()[14:0]),
                         $signed(rand32()[14:0]),rand32()[13:0]), 0);
      end
      begin
        ready_mode=3;
        repeat (300) @(negedge clk);
        ready_mode=1;
      end
    join
    drain(2000);

    // ---- 6. Reset in the middle of traffic (data in every stage + FIFO).
    ready_mode=2;
    for (k=0;k<40;k=k+1)
      axis_send(beat($signed(rand32()[14:0]),$signed(rand32()[14:0]),
                     $signed(rand32()[14:0]),rand32()[13:0]), 0);
    @(negedge clk); s_tvalid=0; aresetn=0;
    repeat (4) @(negedge clk);
    aresetn=1;
    sent=0; received=0;
    repeat (3) @(negedge clk);

    // ---- 7. Traffic after reset, full-scale stress values, 70 % ready.
    ready_mode=1; stress_mode=1;
    for (k=0;k<2000;k=k+1) begin
      if ((k%250)==0) set_gains_random();
      axis_send(beat($signed(rand32()[15:0]),$signed(rand32()[15:0]),
                     $signed(rand32()[15:0]),rand32()[13:0]), rand32()%2);
    end
    stress_mode=0;
    drain(2000);
    axis_idle(30);

    if (sent_q.size()!=0 || qab_a.size()!=0 || qd.size()!=0 || th_q.size()!=0 ||
        qvd.size()!=0 || qva.size()!=0 || qda.size()!=0 || axi_q.size()!=0)
      $fatal(1,"COVERAGE leftover expectations");
    if (max_ipark_real > TOL_IPARK_REAL)
      $fatal(1,"IPARK real-valued error %.3f > %0d",max_ipark_real,TOL_IPARK_REAL);
    if (max_svpwm_real > TOL_SVPWM_REAL)
      $fatal(1,"SVPWM real-valued error %.3f > %0d",max_svpwm_real,TOL_SVPWM_REAL);
    if (checked_svpwm<33000 || resets<3 || s_backpressure<100 || m_stall<1000 ||
        pi_sat<100 || ipark_sat<10 || duty_clip<100 || max_occ!=FIFO_DEPTH+1)
      $fatal(1,"COVERAGE svpwm=%0d resets=%0d s_bp=%0d m_stall=%0d pi_sat=%0d ipark_sat=%0d clip=%0d max_occ=%0d",
             checked_svpwm,resets,s_backpressure,m_stall,pi_sat,ipark_sat,duty_clip,max_occ);

    $display("CHECKED clarke=%0d park=%0d pi=%0d ipark=%0d svpwm=%0d (every stage, every sample)",
             checked_ab,checked_dq,checked_pi,checked_ipark,checked_svpwm);
    $display("ERRORS clarke_max=%0d park_max=%0d/%0d(stress) pi=bit-exact ipark=bit-exact (real err %.2f LSB) svpwm=bit-exact (real err %.2f LSB)",
             max_err_ab,max_err_phys,max_err_stress,max_ipark_real,max_svpwm_real);
    $display("LATENCY stage boundaries from chain accept: clarke N, park N+4, pi N+8, ipark N+12, svpwm N+17; s_axis handshake -> m_axis tvalid = %0d clocks",
             probe_latency);
    $display("THROUGHPUT 16384 back-to-back beats in %0d clocks (1 sample/clock)",tp_cycles);
    $display("AXI s_backpressure_cycles=%0d m_stall_cycles=%0d max_credit_occupancy=%0d/%0d reset_edges=%0d",
             s_backpressure,m_stall,max_occ,FIFO_DEPTH+1,resets);
    $display("SATURATION pi=%0d ipark=%0d duty_clips=%0d",pi_sat,ipark_sat,duty_clip);
    done=1;
  end
endmodule

module tb_foc_top;
  wire done;
  foc_top_checker chk(done);
  initial begin
    wait (done);
    $display("PASS: foc_top (Clarke+Park+PI+invPark+SVPWM, AXI-Stream) regression");
    $finish;
  end
  initial begin
    #20000000;
    $fatal(1,"TIMEOUT: foc_top testbench did not complete");
  end
endmodule
