# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@kelvin_hw//rules:autogen.bzl", "autogen_scm_info")
load(
    "@kelvin_hw//rules:chisel.bzl",
    "chisel_binary",
    "chisel_cc_library",
    "chisel_library",
    "chisel_test",
)
load("@kelvin_hw//rules:verilog.bzl", "verilog_zip_bundle")

package(default_visibility = ["//visibility:public"])

chisel_library(
    name = "kelvin_float",
    srcs = [
        "scalar/FRegfile.scala",
        "scalar/Fpu.scala",
    ],
    deps = [
        ":kelvin",
        ":kelvin_base",
        "//hdl/chisel/src/common:fma",
        "//hdl/chisel/src/common:fp",
    ],
)

chisel_test(
    name = "kelvin_float_tests",
    srcs = [
        "scalar/FRegfileTest.scala",
        "scalar/FpuTest.scala",
    ],
    deps = [
        ":kelvin",
        ":kelvin_base",
        ":kelvin_float",
        "//hdl/chisel/src/common:fp",
    ],
)

chisel_library(
    name = "kelvin_rvv",
    srcs = [
        "rvv/RvvAlu.scala",
        "rvv/RvvDecode.scala",
    ],
    deps = [
        "//hdl/chisel/src/common:library",
    ],
)

chisel_test(
    name = "kelvin_rvv_tests",
    srcs = [
        "rvv/RvvDecodeTest.scala",
    ],
    args = [
        "-P",  # Allows parallel tests.
    ],
    deps = [
        ":kelvin_rvv",
        "//hdl/chisel/src/common:testing",
    ],
)

chisel_test(
    name = "kelvin_scalar_tests",
    srcs = [
        "scalar/AluTest.scala",
        "scalar/MluTest.scala",
    ],
    args = [
        "-P",  # Allows parallel tests.
    ],
    deps = [
        ":kelvin_base",
        ":kelvin_scalar",
        "//hdl/chisel/src/common:testing",
    ],
)

chisel_test(
    name = "kelvin_uncached_fetch_tests",
    srcs = [
        "scalar/UncachedFetchTest.scala",
    ],
    deps = [
        ":kelvin_base",
        ":kelvin_scalar",
        "//hdl/chisel/src/common",
        "//hdl/chisel/src/common:instruction_buffer",
    ],
)

chisel_test(
    name = "kelvin_core_axi_tests",
    srcs = [
        "AxiSlave2SRAMTest.scala",
        "CoreAxiCSRTest.scala",
        "CoreAxiSlaveMuxTest.scala",
        "DBus2AxiTest.scala",
    ],
    deps = [
        ":kelvin",
        ":kelvin_base",
        "//hdl/chisel/src/bus",
    ],
    size = "medium",
)

chisel_library(
    name = "clock_gate",
    srcs = [
        "ClockGate.scala",
    ],
    resource_strip_prefix = "hdl/verilog",
    resources = [
        "//hdl/verilog:ClockGate.sv",
    ],
)

chisel_test(
    name = "clock_gate_test",
    srcs = [
        "ClockGateTest.scala",
    ],
    tags = ["verilator"],
    deps = [
        ":clock_gate",
        ":kelvin_base",
    ],
)

chisel_library(
    name = "RstSync",
    srcs = [
        "RstSync.scala",
    ],
    resource_strip_prefix = "hdl/verilog",
    resources = [
        "//hdl/verilog:RstSync.sv",
    ],
)

autogen_scm_info(name = "KelvinScmInfo")

chisel_library(
    name = "kelvin_base",
    srcs = [
        "DBus2Axi.scala",
        "DBusMux.scala",
        "IBus2Axi.scala",
        "Interfaces.scala",
        "L1DCache.scala",
        "L1ICache.scala",
        "Library.scala",
        "Parameters.scala",
    ],
    deps = [
        ":clock_gate",
        "//hdl/chisel/src/bus",
        "//hdl/chisel/src/common",
    ],
)

chisel_library(
    name = "kelvin_vector",
    srcs = [
        "vector/VAlu.scala",
        "vector/VAluInt.scala",
        "vector/VCmdq.scala",
        "vector/VCommon.scala",
        "vector/VConvAlu.scala",
        "vector/VConvCtrl.scala",
        "vector/VCore.scala",
        "vector/VDecode.scala",
        "vector/VDecodeInstruction.scala",
        "vector/VDecodeOp.scala",
        "vector/VDot.scala",
        "vector/VEncodeOp.scala",
        "vector/VInst.scala",
        "vector/VLdSt.scala",
        "vector/VRegfile.scala",
        "vector/VRegfileSegment.scala",
    ],
    deps = [
        ":kelvin_base",
        "//hdl/chisel/src/bus",
        "//hdl/chisel/src/common",
    ],
)

chisel_library(
    name = "kelvin_scalar",
    srcs = [
        "scalar/Alu.scala",
        "scalar/Bru.scala",
        "scalar/Csr.scala",
        "scalar/Debug.scala",
        "scalar/Decode.scala",
        "scalar/Dvu.scala",
        "scalar/Fetch.scala",
        "scalar/Flush.scala",
        "scalar/Lsu.scala",
        "scalar/Mlu.scala",
        "scalar/Regfile.scala",
        "scalar/SCore.scala",
        "scalar/SLog.scala",
        "scalar/UncachedFetch.scala",
    ],
    deps = [
        ":KelvinScmInfo",
        ":kelvin_base",
        "//hdl/chisel/src/common",
        "//hdl/chisel/src/common:instruction_buffer",
    ],
)

chisel_library(
    name = "srams",
    srcs = [
        "SramNx128.scala",
        "Sram_12ffcp_128x128.scala",
        "Sram_12ffcp_2048x128.scala",
        "Sram_12ffcp_512x128.scala",
    ],
    resources = [
        "//hdl/verilog:Sram_12ffcp_128x128.v",
        "//hdl/verilog:Sram_12ffcp_2048x128.v",
        "//hdl/verilog:Sram_12ffcp_512x128.v",
    ],
    deps = [
        ":kelvin_base",
    ],
)

chisel_library(
    name = "kelvin",
    srcs = [
        "AxiSlave.scala",
        "AxiSlave2SRAM.scala",
        "Core.scala",
        "CoreAxi.scala",
        "CoreAxiCSR.scala",
        "CoreAxiSlaveMux.scala",
        "TCM.scala",
    ],
    deps = [
        ":RstSync",
        ":clock_gate",
        ":kelvin_base",
        ":kelvin_scalar",
        ":kelvin_vector",
        ":srams",
        "//hdl/chisel/src/bus",
        "//hdl/chisel/src/common",
        "//hdl/chisel/src/common:instruction_buffer",
    ],
)

chisel_library(
    name = "kelvin_params",
    srcs = [
        "Parameters.scala",
    ],
    deps = [
    ],
)

chisel_cc_library(
    name = "alu_cc_library",
    chisel_lib = ":kelvin_scalar",
    emit_class = "kelvin.EmitAlu",
    module_name = "Alu",
)

chisel_cc_library(
    name = "core_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitCore",
    extra_outs = [
        "VCore_parameters.h",
    ],
    module_name = "Core",
)

chisel_cc_library(
    name = "core_scalar_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitCore",
    extra_outs = [
        "VCoreScalar_parameters.h",
    ],
    gen_flags = [
        "--enableVector=False",
        "--moduleName=CoreScalar",
    ],
    module_name = "CoreScalar",
    verilog_file_path = "CoreScalar.sv",
)

chisel_cc_library(
    name = "core_mini_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitCore",
    extra_outs = [
        "VCoreMini_parameters.h",
    ],
    gen_flags = [
        "--enableFetchL0=False",
        "--fetchDataBits=128",
        "--lsuDataBits=128",
        "--enableVector=False",
        "--moduleName=CoreMini",
    ],
    module_name = "CoreMini",
    verilog_file_path = "CoreMini.sv",
)

chisel_cc_library(
    name = "core_mini_axi_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitCore",
    extra_outs = [
        "VCoreMiniAxi_parameters.h",
        "CoreMiniAxi.zip",
    ],
    gen_flags = [
        "--enableFetchL0=False",
        "--fetchDataBits=128",
        "--lsuDataBits=128",
        "--enableVector=False",
        "--moduleName=CoreMini",
        "--useAxi",
    ],
    module_name = "CoreMiniAxi",
    verilog_file_path = "CoreMiniAxi.sv",
    vopts = [
        "-DUSE_GENERIC",
    ],
)

verilog_zip_bundle(
    name = "core_mini_axi_bundle",
    lib = ":core_mini_axi_cc_library_verilog",
)

chisel_cc_library(
    name = "dbus2axi_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitDBus2Axi",
    module_name = "DBus2Axi",
)

chisel_cc_library(
    name = "dvu_cc_library",
    chisel_lib = ":kelvin_scalar",
    emit_class = "kelvin.EmitDvu",
    module_name = "Dvu",
)

chisel_cc_library(
    name = "mlu_cc_library",
    chisel_lib = ":kelvin_scalar",
    emit_class = "kelvin.EmitMlu",
    module_name = "Mlu",
)

chisel_cc_library(
    name = "l1dcache_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitL1DCache",
    module_name = "L1DCache",
    verilog_deps = [
        "//hdl/verilog:sram_1rw_256x288",
    ],
)

chisel_cc_library(
    name = "l1dcachebank_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitL1DCacheBank",
    module_name = "L1DCacheBank",
    verilog_deps = [
        "//hdl/verilog:sram_1rw_256x288",
    ],
)

chisel_cc_library(
    name = "l1icache_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitL1ICache",
    module_name = "L1ICache",
    verilog_deps = [
        "//hdl/verilog:sram_1rw_256x256",
    ],
)

chisel_cc_library(
    name = "valu_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVAlu",
    module_name = "VAlu",
)

chisel_cc_library(
    name = "valuint_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVAluInt",
    module_name = "VAluInt",
)

chisel_cc_library(
    name = "vcmdq_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVCmdq",
    module_name = "VCmdq",
)

chisel_cc_library(
    name = "vconvalu_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVConvAlu",
    module_name = "VConvAlu",
)

chisel_cc_library(
    name = "vconvctrl_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVConvCtrl",
    module_name = "VConvCtrl",
)

chisel_cc_library(
    name = "vdecode_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVDecode",
    module_name = "VDecode",
)

chisel_cc_library(
    name = "vdecodeinstruction_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVDecodeInstruction",
    module_name = "VDecodeInstruction",
)

chisel_cc_library(
    name = "vldst_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVLdSt",
    module_name = "VLdSt",
)

chisel_cc_library(
    name = "vregfile_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVRegfile",
    module_name = "VRegfile",
)

chisel_cc_library(
    name = "vregfilesegment_cc_library",
    chisel_lib = ":kelvin",
    emit_class = "kelvin.EmitVRegfileSegment",
    module_name = "VRegfileSegment",
)
