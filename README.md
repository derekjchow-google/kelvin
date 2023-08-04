# Kelvin

Kelvin is a RISC-V32IM core with a custom instruction set.

## Building

Kelvin uses [bazel](https://bazel.build/) as it's build system. The Verilated
simulator for Kelvin can be generated using:

```bash
bazel build //tests/verilator_sim:core_sim
```

The verilog source for the Kelvin core can be generated using:

```bash
bazel build //hdl/chisel:core_cc_library_emit_verilog
```
