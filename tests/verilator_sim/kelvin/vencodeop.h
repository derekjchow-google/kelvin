// Copyright 2023 Google LLC.

#ifndef TESTS_VERILATOR_SIM_KELVIN_VENCODEOP_H_
#define TESTS_VERILATOR_SIM_KELVIN_VENCODEOP_H_

namespace encode {

constexpr int kOpBits = 7;
constexpr int kOpEntries = 72;

enum VEncodeOp {
  undef,
  vdup,
  vld,
  vst,
  vstq,
  vcget,
  vadd,
  vsub,
  vrsub,
  veq,
  vne,
  vlt,
  vle,
  vgt,
  vge,
  vabsd,
  vmax,
  vmin,
  vadd3,
  vand,
  vor,
  vxor,
  vnot,
  vrev,
  vror,
  vclb,
  vclz,
  vcpop,
  vmv,
  vmv2,
  vmvp,
  acset,
  actr,
  adwinit,
  vshl,
  vshr,
  vshf,
  vsrans,
  vsraqs,
  vmul,
  vmul2,
  vmuls,
  vmuls2,
  vmulh,
  vmulh2,
  vdmulh,
  vdmulh2,
  vmulw,
  vmadd,
  vadds,
  vsubs,
  vaddw,
  vsubw,
  vacc,
  vpadd,
  vpsub,
  vhadd,
  vhsub,
  vslidevn,
  vslidehn,
  vslidehn2,
  vslidevp,
  vslidehp,
  vslidehp2,
  vsel,
  vevn,
  vodd,
  vevnodd,
  vzip,
  aconv,
  vdwconv,
  adwconv,
};

static_assert(kOpEntries == (adwconv + 1));

}  // namespace encode

#endif  // TESTS_VERILATOR_SIM_KELVIN_VENCODEOP_H_
