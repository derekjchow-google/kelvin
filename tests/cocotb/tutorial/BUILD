load("//rules:coco_tb.bzl", "verilator_cocotb_test")

verilator_cocotb_test(
    name = "tutorial",
    defines = {
        "USE_GENERIC" : "",
    },
    waves = True,
    hdl_toplevel = "CoreMiniAxi",
    seed = "42",
    test_module = ["tutorial.py"],
    verilog_sources = [
        "//hdl/chisel/src/kelvin:core_mini_axi_cc_library_verilog"
    ],
    deps = [
        "//tests/cocotb:core_mini_axi_sim_interface",
    ],
    data = glob(["**/*.elf"]),
)
