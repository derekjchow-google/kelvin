load("//rules:coco_tb.bzl", "verilator_cocotb_test")
load("//rules:utils.bzl", "template_rule")
load("@kelvin_pip_deps//:requirements.bzl", "requirement")

template_rule(
    verilator_cocotb_test,
    {
        "core_mini_axi_sim_cocotb": {
            "hdl_toplevel": "CoreMiniAxi",
            "verilog_sources": [
                "//hdl/chisel/src/kelvin:core_mini_axi_cc_library_verilog"
            ],
        },
        "rvv_core_mini_axi_sim_cocotb": {
            "hdl_toplevel": "RvvCoreMiniAxi",
            "verilog_sources": [
                "//hdl/chisel/src/kelvin:rvv_core_mini_axi_cc_library_verilog"
            ],
        },
    },
    defines = {
        "USE_GENERIC" : "",
    },
    waves = True,
    seed = "42",
    test_module = ["core_mini_axi_sim.py"],
    deps = [
        requirement("pyelftools"),
        requirement("tqdm"),
    ],
    data = glob(["*.elf"]),
)
