#include "VVCmdq.h"
#include "tests/verilator_sim/sysc_tb.h"

struct VCmdq_tb : Sysc_tb {
  sc_in<bool> io_in_ready;
  sc_out<bool> io_in_valid;
  sc_out<bool> io_in_bits_0_valid;
  sc_out<bool> io_in_bits_0_bits_m;
  sc_out<bool> io_in_bits_0_bits_vd_valid;
  sc_out<bool> io_in_bits_0_bits_ve_valid;
  sc_out<bool> io_in_bits_0_bits_vf_valid;
  sc_out<bool> io_in_bits_0_bits_vg_valid;
  sc_out<bool> io_in_bits_0_bits_vs_valid;
  sc_out<bool> io_in_bits_0_bits_vt_valid;
  sc_out<bool> io_in_bits_0_bits_vu_valid;
  sc_out<bool> io_in_bits_0_bits_vx_valid;
  sc_out<bool> io_in_bits_0_bits_vy_valid;
  sc_out<bool> io_in_bits_0_bits_vz_valid;
  sc_out<bool> io_in_bits_0_bits_sv_valid;
  sc_out<bool> io_in_bits_0_bits_cmdsync;
  sc_out<bool> io_in_bits_1_valid;
  sc_out<bool> io_in_bits_1_bits_m;
  sc_out<bool> io_in_bits_1_bits_vd_valid;
  sc_out<bool> io_in_bits_1_bits_ve_valid;
  sc_out<bool> io_in_bits_1_bits_vf_valid;
  sc_out<bool> io_in_bits_1_bits_vg_valid;
  sc_out<bool> io_in_bits_1_bits_vs_valid;
  sc_out<bool> io_in_bits_1_bits_vt_valid;
  sc_out<bool> io_in_bits_1_bits_vu_valid;
  sc_out<bool> io_in_bits_1_bits_vx_valid;
  sc_out<bool> io_in_bits_1_bits_vy_valid;
  sc_out<bool> io_in_bits_1_bits_vz_valid;
  sc_out<bool> io_in_bits_1_bits_sv_valid;
  sc_out<bool> io_in_bits_1_bits_cmdsync;
  sc_out<bool> io_in_bits_2_valid;
  sc_out<bool> io_in_bits_2_bits_m;
  sc_out<bool> io_in_bits_2_bits_vd_valid;
  sc_out<bool> io_in_bits_2_bits_ve_valid;
  sc_out<bool> io_in_bits_2_bits_vf_valid;
  sc_out<bool> io_in_bits_2_bits_vg_valid;
  sc_out<bool> io_in_bits_2_bits_vs_valid;
  sc_out<bool> io_in_bits_2_bits_vt_valid;
  sc_out<bool> io_in_bits_2_bits_vu_valid;
  sc_out<bool> io_in_bits_2_bits_vx_valid;
  sc_out<bool> io_in_bits_2_bits_vy_valid;
  sc_out<bool> io_in_bits_2_bits_vz_valid;
  sc_out<bool> io_in_bits_2_bits_sv_valid;
  sc_out<bool> io_in_bits_2_bits_cmdsync;
  sc_out<bool> io_in_bits_3_valid;
  sc_out<bool> io_in_bits_3_bits_m;
  sc_out<bool> io_in_bits_3_bits_vd_valid;
  sc_out<bool> io_in_bits_3_bits_ve_valid;
  sc_out<bool> io_in_bits_3_bits_vf_valid;
  sc_out<bool> io_in_bits_3_bits_vg_valid;
  sc_out<bool> io_in_bits_3_bits_vs_valid;
  sc_out<bool> io_in_bits_3_bits_vt_valid;
  sc_out<bool> io_in_bits_3_bits_vu_valid;
  sc_out<bool> io_in_bits_3_bits_vx_valid;
  sc_out<bool> io_in_bits_3_bits_vy_valid;
  sc_out<bool> io_in_bits_3_bits_vz_valid;
  sc_out<bool> io_in_bits_3_bits_sv_valid;
  sc_out<bool> io_in_bits_3_bits_cmdsync;
  sc_out<bool> io_out_ready;
  sc_in<bool> io_out_valid;
  sc_in<bool> io_out_bits_vd_valid;
  sc_in<bool> io_out_bits_vs_valid;
  sc_in<bool> io_nempty;
  sc_out<sc_bv<7> > io_in_bits_0_bits_op;
  sc_out<sc_bv<3> > io_in_bits_0_bits_f2;
  sc_out<sc_bv<3> > io_in_bits_0_bits_sz;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vd_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_ve_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vf_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vg_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vs_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vt_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vu_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vx_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vy_addr;
  sc_out<sc_bv<6> > io_in_bits_0_bits_vz_addr;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vs_tag;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vt_tag;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vu_tag;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vx_tag;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vy_tag;
  sc_out<sc_bv<4> > io_in_bits_0_bits_vz_tag;
  sc_out<sc_bv<32> > io_in_bits_0_bits_sv_addr;
  sc_out<sc_bv<32> > io_in_bits_0_bits_sv_data;
  sc_out<sc_bv<7> > io_in_bits_1_bits_op;
  sc_out<sc_bv<3> > io_in_bits_1_bits_f2;
  sc_out<sc_bv<3> > io_in_bits_1_bits_sz;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vd_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_ve_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vf_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vg_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vs_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vt_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vu_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vx_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vy_addr;
  sc_out<sc_bv<6> > io_in_bits_1_bits_vz_addr;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vs_tag;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vt_tag;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vu_tag;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vx_tag;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vy_tag;
  sc_out<sc_bv<4> > io_in_bits_1_bits_vz_tag;
  sc_out<sc_bv<32> > io_in_bits_1_bits_sv_addr;
  sc_out<sc_bv<32> > io_in_bits_1_bits_sv_data;
  sc_out<sc_bv<7> > io_in_bits_2_bits_op;
  sc_out<sc_bv<3> > io_in_bits_2_bits_f2;
  sc_out<sc_bv<3> > io_in_bits_2_bits_sz;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vd_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_ve_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vf_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vg_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vs_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vt_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vu_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vx_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vy_addr;
  sc_out<sc_bv<6> > io_in_bits_2_bits_vz_addr;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vs_tag;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vt_tag;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vu_tag;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vx_tag;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vy_tag;
  sc_out<sc_bv<4> > io_in_bits_2_bits_vz_tag;
  sc_out<sc_bv<32> > io_in_bits_2_bits_sv_addr;
  sc_out<sc_bv<32> > io_in_bits_2_bits_sv_data;
  sc_out<sc_bv<7> > io_in_bits_3_bits_op;
  sc_out<sc_bv<3> > io_in_bits_3_bits_f2;
  sc_out<sc_bv<3> > io_in_bits_3_bits_sz;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vd_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_ve_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vf_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vg_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vs_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vt_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vu_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vx_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vy_addr;
  sc_out<sc_bv<6> > io_in_bits_3_bits_vz_addr;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vs_tag;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vt_tag;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vu_tag;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vx_tag;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vy_tag;
  sc_out<sc_bv<4> > io_in_bits_3_bits_vz_tag;
  sc_out<sc_bv<32> > io_in_bits_3_bits_sv_addr;
  sc_out<sc_bv<32> > io_in_bits_3_bits_sv_data;
  sc_in<sc_bv<7> > io_out_bits_op;
  sc_in<sc_bv<3> > io_out_bits_sz;
  sc_in<sc_bv<6> > io_out_bits_vd_addr;
  sc_in<sc_bv<6> > io_out_bits_vs_addr;
  sc_in<sc_bv<4> > io_out_bits_vs_tag;
  sc_in<sc_bv<32> > io_out_bits_data;
  sc_in<sc_bv<64> > io_active;

  using Sysc_tb::Sysc_tb;

  void posedge() {
    // Active.
    uint64_t aref = Active();
    uint64_t adut = io_active.read().get_word(0) |
                    (uint64_t(io_active.read().get_word(1)) << 32);
    if (aref != adut) {
      printf("**error::Active %08x:%08x : %08x:%08x\n", uint32_t(aref >> 32),
             uint32_t(aref), uint32_t(adut >> 32), uint32_t(adut));
      check(false);
    }

    check(!cmdq_.empty() == io_nempty, "io.nempty");

    // Inputs.
#define IN_RAND(idx)                                  \
  {                                                   \
    const bool m = rand_bool();                       \
    const int vd = rand_uint32() & (m ? 60 : 63);     \
    const int vs = rand_uint32() & (m ? 60 : 63);     \
    const bool vd_valid = rand_bool();                \
    const bool vs_valid = rand_bool();                \
    io_in_bits_##idx##_valid = rand_bool();           \
    io_in_bits_##idx##_bits_m = m;                    \
    io_in_bits_##idx##_bits_vd_valid = vd_valid;      \
    io_in_bits_##idx##_bits_ve_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vf_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vg_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vs_valid = vs_valid;      \
    io_in_bits_##idx##_bits_vt_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vu_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vx_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vy_valid = rand_bool();   \
    io_in_bits_##idx##_bits_vz_valid = rand_bool();   \
    io_in_bits_##idx##_bits_sv_valid = rand_bool();   \
    io_in_bits_##idx##_bits_cmdsync = rand_bool();    \
    io_in_bits_##idx##_bits_op = rand_uint32();       \
    io_in_bits_##idx##_bits_sz = 1 << rand_int(0, 2); \
    io_in_bits_##idx##_bits_vd_addr = vd;             \
    io_in_bits_##idx##_bits_ve_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vf_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vg_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vs_addr = vs;             \
    io_in_bits_##idx##_bits_vt_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vu_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vx_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vy_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vz_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_vs_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_vt_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_vu_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_vx_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_vy_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_vz_tag = rand_uint32();   \
    io_in_bits_##idx##_bits_sv_addr = rand_uint32();  \
    io_in_bits_##idx##_bits_sv_data = rand_uint32();  \
  }

    io_in_valid = rand_int(0, 7) == 0;  // Try to hit both full and empty.
    io_out_ready = rand_bool();
    IN_RAND(0);
    IN_RAND(1);
    IN_RAND(2);
    IN_RAND(3);

#define IN_READ(idx)                                        \
  Input(io_in_bits_##idx##_bits_m,                          \
        io_in_bits_##idx##_bits_op.read().get_word(0),      \
        io_in_bits_##idx##_bits_sz.read().get_word(0),      \
        io_in_bits_##idx##_bits_vd_valid,                   \
        io_in_bits_##idx##_bits_vd_addr.read().get_word(0), \
        io_in_bits_##idx##_bits_vs_valid,                   \
        io_in_bits_##idx##_bits_vs_addr.read().get_word(0), \
        io_in_bits_##idx##_bits_vs_tag.read().get_word(0),  \
        io_in_bits_##idx##_bits_sv_data.read().get_word(0));

    if (io_in_valid && io_in_ready) {
      if (io_in_bits_0_valid) IN_READ(0);
      if (io_in_bits_1_valid) IN_READ(1);
      if (io_in_bits_2_valid) IN_READ(2);
      if (io_in_bits_3_valid) IN_READ(3);
    }

    // Outputs.
    cmdq_t dut, ref;
    dut.op = io_out_bits_op.read().get_word(0);
    dut.sz = io_out_bits_sz.read().get_word(0);
    dut.vd.valid = io_out_bits_vd_valid;
    dut.vd.addr = io_out_bits_vd_addr.read().get_word(0);
    dut.vs.valid = io_out_bits_vs_valid;
    dut.vs.addr = io_out_bits_vs_addr.read().get_word(0);
    dut.vs.tag = io_out_bits_vs_tag.read().get_word(0);
    dut.data = io_out_bits_data.read().get_word(0);

    if (io_out_valid && io_out_ready) {
      // Outputs must match.
      check(cmdq_.read(ref), "cmdq fifo empty");
      if (ref != dut) {
        printf("**error::vcmdq\n");
        ref.print();
        dut.print();
        check(false);
      }
    } else if (!io_out_valid) {
      // Outputs must be zero.
      memset(&ref, 0, sizeof(ref));
      if (ref != dut) {
        printf("**error::vcmdq(2)\n");
        ref.print();
        dut.print();
        check(false);
      }
    }
  }

 private:
  struct cmdq_t {
    uint8_t op : 7;
    uint8_t sz : 3;
    struct {
      bool valid;
      uint8_t addr : 6;
    } vd;
    struct {
      bool valid;
      uint8_t addr : 6;
      uint8_t tag : 4;
    } vs;
    uint32_t data;

    bool operator!=(const cmdq_t& rhs) const {
      if (op != rhs.op) return true;
      if (sz != rhs.sz) return true;
      if (vd.valid != rhs.vd.valid) return true;
      if (vd.valid && vd.addr != rhs.vd.addr) return true;
      if (vs.valid != rhs.vs.valid) return true;
      if (vs.valid && vs.addr != rhs.vs.addr) return true;
      if (vs.valid && vs.tag != rhs.vs.tag) return true;
      if (data != rhs.data) return true;
      return false;
    }

    void print() {
      printf("cmdq_t: op=%d sz=%d vd=(%d,%d) vs=(%d,%d,%d) data=%08x\n", op,
             sz, vd.valid, vd.addr, vs.valid, vs.addr, vs.tag, data);
    }
  };

  fifo_t<cmdq_t> cmdq_;

  void Input(bool m, uint8_t op, uint8_t sz, bool vd_valid, uint8_t vd_addr,
             bool vs_valid, uint8_t vs_addr, uint8_t vs_tag, uint32_t data) {
    cmdq_t c;
    c.op = op;
    c.sz = sz;
    c.vd.valid = vd_valid;
    c.vd.addr = vd_addr;
    c.vs.valid = vs_valid;
    c.vs.addr = vs_addr;
    c.vs.tag = vs_tag;
    c.data = data;

    for (int i = 0; i < (m ? 4 : 1); ++i) {
      cmdq_.write(c);
      if (c.vd.valid) c.vd.addr++;
      if (c.vs.valid) c.vs.addr++;
    }
  }

  uint64_t Active(bool m, bool valid, uint8_t index) {
    if (!valid) return 0;
    assert(!(m && (index & 3)));
    return (m ? 15 : 1) << index;
  }

  uint64_t Active() {
    uint64_t active = 0;
    for (int i = 0; i < cmdq_.count(); ++i) {
      cmdq_t v;
      check(cmdq_.next(v, i), "values fifo next");
      if (v.vd.valid) active |= 1ull << v.vd.addr;
      if (v.vs.valid) active |= 1ull << v.vs.addr;
    }
    return active;
  }
};

static void VCmdq_test(char* name, int loops, bool random, bool trace) {
  sc_signal<bool> io_in_ready;
  sc_signal<bool> io_in_valid;
  sc_signal<bool> io_in_bits_0_valid;
  sc_signal<bool> io_in_bits_0_bits_m;
  sc_signal<bool> io_in_bits_0_bits_vd_valid;
  sc_signal<bool> io_in_bits_0_bits_ve_valid;
  sc_signal<bool> io_in_bits_0_bits_vf_valid;
  sc_signal<bool> io_in_bits_0_bits_vg_valid;
  sc_signal<bool> io_in_bits_0_bits_vs_valid;
  sc_signal<bool> io_in_bits_0_bits_vt_valid;
  sc_signal<bool> io_in_bits_0_bits_vu_valid;
  sc_signal<bool> io_in_bits_0_bits_vx_valid;
  sc_signal<bool> io_in_bits_0_bits_vy_valid;
  sc_signal<bool> io_in_bits_0_bits_vz_valid;
  sc_signal<bool> io_in_bits_0_bits_sv_valid;
  sc_signal<bool> io_in_bits_0_bits_cmdsync;
  sc_signal<bool> io_in_bits_1_valid;
  sc_signal<bool> io_in_bits_1_bits_m;
  sc_signal<bool> io_in_bits_1_bits_vd_valid;
  sc_signal<bool> io_in_bits_1_bits_ve_valid;
  sc_signal<bool> io_in_bits_1_bits_vf_valid;
  sc_signal<bool> io_in_bits_1_bits_vg_valid;
  sc_signal<bool> io_in_bits_1_bits_vs_valid;
  sc_signal<bool> io_in_bits_1_bits_vt_valid;
  sc_signal<bool> io_in_bits_1_bits_vu_valid;
  sc_signal<bool> io_in_bits_1_bits_vx_valid;
  sc_signal<bool> io_in_bits_1_bits_vy_valid;
  sc_signal<bool> io_in_bits_1_bits_vz_valid;
  sc_signal<bool> io_in_bits_1_bits_sv_valid;
  sc_signal<bool> io_in_bits_1_bits_cmdsync;
  sc_signal<bool> io_in_bits_2_valid;
  sc_signal<bool> io_in_bits_2_bits_m;
  sc_signal<bool> io_in_bits_2_bits_vd_valid;
  sc_signal<bool> io_in_bits_2_bits_ve_valid;
  sc_signal<bool> io_in_bits_2_bits_vf_valid;
  sc_signal<bool> io_in_bits_2_bits_vg_valid;
  sc_signal<bool> io_in_bits_2_bits_vs_valid;
  sc_signal<bool> io_in_bits_2_bits_vt_valid;
  sc_signal<bool> io_in_bits_2_bits_vu_valid;
  sc_signal<bool> io_in_bits_2_bits_vx_valid;
  sc_signal<bool> io_in_bits_2_bits_vy_valid;
  sc_signal<bool> io_in_bits_2_bits_vz_valid;
  sc_signal<bool> io_in_bits_2_bits_sv_valid;
  sc_signal<bool> io_in_bits_2_bits_cmdsync;
  sc_signal<bool> io_in_bits_3_valid;
  sc_signal<bool> io_in_bits_3_bits_m;
  sc_signal<bool> io_in_bits_3_bits_vd_valid;
  sc_signal<bool> io_in_bits_3_bits_ve_valid;
  sc_signal<bool> io_in_bits_3_bits_vf_valid;
  sc_signal<bool> io_in_bits_3_bits_vg_valid;
  sc_signal<bool> io_in_bits_3_bits_vs_valid;
  sc_signal<bool> io_in_bits_3_bits_vt_valid;
  sc_signal<bool> io_in_bits_3_bits_vu_valid;
  sc_signal<bool> io_in_bits_3_bits_vx_valid;
  sc_signal<bool> io_in_bits_3_bits_vy_valid;
  sc_signal<bool> io_in_bits_3_bits_vz_valid;
  sc_signal<bool> io_in_bits_3_bits_sv_valid;
  sc_signal<bool> io_in_bits_3_bits_cmdsync;
  sc_signal<bool> io_out_ready;
  sc_signal<bool> io_out_valid;
  sc_signal<bool> io_out_bits_vd_valid;
  sc_signal<bool> io_out_bits_vs_valid;
  sc_signal<bool> io_nempty;
  sc_signal<sc_bv<7> > io_in_bits_0_bits_op;
  sc_signal<sc_bv<3> > io_in_bits_0_bits_f2;
  sc_signal<sc_bv<3> > io_in_bits_0_bits_sz;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vd_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_ve_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vf_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vg_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vs_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vt_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vu_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vx_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vy_addr;
  sc_signal<sc_bv<6> > io_in_bits_0_bits_vz_addr;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vs_tag;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vt_tag;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vu_tag;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vx_tag;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vy_tag;
  sc_signal<sc_bv<4> > io_in_bits_0_bits_vz_tag;
  sc_signal<sc_bv<32> > io_in_bits_0_bits_sv_addr;
  sc_signal<sc_bv<32> > io_in_bits_0_bits_sv_data;
  sc_signal<sc_bv<7> > io_in_bits_1_bits_op;
  sc_signal<sc_bv<3> > io_in_bits_1_bits_f2;
  sc_signal<sc_bv<3> > io_in_bits_1_bits_sz;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vd_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_ve_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vf_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vg_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vs_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vt_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vu_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vx_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vy_addr;
  sc_signal<sc_bv<6> > io_in_bits_1_bits_vz_addr;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vs_tag;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vt_tag;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vu_tag;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vx_tag;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vy_tag;
  sc_signal<sc_bv<4> > io_in_bits_1_bits_vz_tag;
  sc_signal<sc_bv<32> > io_in_bits_1_bits_sv_addr;
  sc_signal<sc_bv<32> > io_in_bits_1_bits_sv_data;
  sc_signal<sc_bv<7> > io_in_bits_2_bits_op;
  sc_signal<sc_bv<3> > io_in_bits_2_bits_f2;
  sc_signal<sc_bv<3> > io_in_bits_2_bits_sz;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vd_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_ve_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vf_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vg_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vs_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vt_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vu_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vx_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vy_addr;
  sc_signal<sc_bv<6> > io_in_bits_2_bits_vz_addr;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vs_tag;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vt_tag;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vu_tag;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vx_tag;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vy_tag;
  sc_signal<sc_bv<4> > io_in_bits_2_bits_vz_tag;
  sc_signal<sc_bv<32> > io_in_bits_2_bits_sv_addr;
  sc_signal<sc_bv<32> > io_in_bits_2_bits_sv_data;
  sc_signal<sc_bv<7> > io_in_bits_3_bits_op;
  sc_signal<sc_bv<3> > io_in_bits_3_bits_f2;
  sc_signal<sc_bv<3> > io_in_bits_3_bits_sz;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vd_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_ve_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vf_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vg_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vs_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vt_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vu_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vx_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vy_addr;
  sc_signal<sc_bv<6> > io_in_bits_3_bits_vz_addr;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vs_tag;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vt_tag;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vu_tag;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vx_tag;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vy_tag;
  sc_signal<sc_bv<4> > io_in_bits_3_bits_vz_tag;
  sc_signal<sc_bv<32> > io_in_bits_3_bits_sv_addr;
  sc_signal<sc_bv<32> > io_in_bits_3_bits_sv_data;
  sc_signal<sc_bv<7> > io_out_bits_op;
  sc_signal<sc_bv<3> > io_out_bits_sz;
  sc_signal<sc_bv<6> > io_out_bits_vd_addr;
  sc_signal<sc_bv<6> > io_out_bits_vs_addr;
  sc_signal<sc_bv<4> > io_out_bits_vs_tag;
  sc_signal<sc_bv<32> > io_out_bits_data;
  sc_signal<sc_bv<64> > io_active;

  VCmdq_tb tb("VCmdq_tb", loops, random);
  VVCmdq cmdq(name);

  cmdq.clock(tb.clock);
  cmdq.reset(tb.reset);
  BIND2(tb, cmdq, io_in_ready);
  BIND2(tb, cmdq, io_in_valid);
  BIND2(tb, cmdq, io_in_bits_0_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_m);
  BIND2(tb, cmdq, io_in_bits_0_bits_vd_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_ve_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vf_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vg_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vs_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vt_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vu_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vx_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vy_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_vz_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_sv_valid);
  BIND2(tb, cmdq, io_in_bits_0_bits_cmdsync);
  BIND2(tb, cmdq, io_in_bits_1_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_m);
  BIND2(tb, cmdq, io_in_bits_1_bits_vd_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_ve_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vf_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vg_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vs_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vt_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vu_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vx_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vy_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_vz_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_sv_valid);
  BIND2(tb, cmdq, io_in_bits_1_bits_cmdsync);
  BIND2(tb, cmdq, io_in_bits_2_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_m);
  BIND2(tb, cmdq, io_in_bits_2_bits_vd_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_ve_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vf_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vg_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vs_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vt_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vu_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vx_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vy_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_vz_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_sv_valid);
  BIND2(tb, cmdq, io_in_bits_2_bits_cmdsync);
  BIND2(tb, cmdq, io_in_bits_3_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_m);
  BIND2(tb, cmdq, io_in_bits_3_bits_vd_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_ve_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vf_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vg_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vs_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vt_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vu_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vx_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vy_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_vz_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_sv_valid);
  BIND2(tb, cmdq, io_in_bits_3_bits_cmdsync);
  BIND2(tb, cmdq, io_out_ready);
  BIND2(tb, cmdq, io_out_valid);
  BIND2(tb, cmdq, io_out_bits_vd_valid);
  BIND2(tb, cmdq, io_out_bits_vs_valid);
  BIND2(tb, cmdq, io_nempty);
  BIND2(tb, cmdq, io_in_bits_0_bits_op);
  BIND2(tb, cmdq, io_in_bits_0_bits_f2);
  BIND2(tb, cmdq, io_in_bits_0_bits_sz);
  BIND2(tb, cmdq, io_in_bits_0_bits_vd_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_ve_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vf_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vg_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vs_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vt_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vu_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vx_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vy_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vz_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_vs_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_vt_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_vu_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_vx_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_vy_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_vz_tag);
  BIND2(tb, cmdq, io_in_bits_0_bits_sv_addr);
  BIND2(tb, cmdq, io_in_bits_0_bits_sv_data);
  BIND2(tb, cmdq, io_in_bits_1_bits_op);
  BIND2(tb, cmdq, io_in_bits_1_bits_f2);
  BIND2(tb, cmdq, io_in_bits_1_bits_sz);
  BIND2(tb, cmdq, io_in_bits_1_bits_vd_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_ve_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vf_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vg_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vs_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vt_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vu_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vx_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vy_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vz_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_vs_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_vt_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_vu_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_vx_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_vy_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_vz_tag);
  BIND2(tb, cmdq, io_in_bits_1_bits_sv_addr);
  BIND2(tb, cmdq, io_in_bits_1_bits_sv_data);
  BIND2(tb, cmdq, io_in_bits_2_bits_op);
  BIND2(tb, cmdq, io_in_bits_2_bits_f2);
  BIND2(tb, cmdq, io_in_bits_2_bits_sz);
  BIND2(tb, cmdq, io_in_bits_2_bits_vd_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_ve_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vf_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vg_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vs_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vt_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vu_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vx_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vy_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vz_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_vs_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_vt_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_vu_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_vx_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_vy_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_vz_tag);
  BIND2(tb, cmdq, io_in_bits_2_bits_sv_addr);
  BIND2(tb, cmdq, io_in_bits_2_bits_sv_data);
  BIND2(tb, cmdq, io_in_bits_3_bits_op);
  BIND2(tb, cmdq, io_in_bits_3_bits_f2);
  BIND2(tb, cmdq, io_in_bits_3_bits_sz);
  BIND2(tb, cmdq, io_in_bits_3_bits_vd_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_ve_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vf_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vg_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vs_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vt_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vu_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vx_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vy_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vz_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_vs_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_vt_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_vu_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_vx_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_vy_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_vz_tag);
  BIND2(tb, cmdq, io_in_bits_3_bits_sv_addr);
  BIND2(tb, cmdq, io_in_bits_3_bits_sv_data);
  BIND2(tb, cmdq, io_out_bits_op);
  BIND2(tb, cmdq, io_out_bits_sz);
  BIND2(tb, cmdq, io_out_bits_vd_addr);
  BIND2(tb, cmdq, io_out_bits_vs_addr);
  BIND2(tb, cmdq, io_out_bits_vs_tag);
  BIND2(tb, cmdq, io_out_bits_data);
  BIND2(tb, cmdq, io_active);

  if (trace) {
    tb.trace(cmdq);
  }

  tb.start();
}

int sc_main(int argc, char* argv[]) {
  VCmdq_test(Sysc_tb::get_name(argv[0]), 1000000, true, false);
  return 0;
}
