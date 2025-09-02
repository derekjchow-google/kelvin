# Embedding System Verilog in Chisel (Chip of Thesus)

Kelvin is a core with components implemented in both Chisel and SystemVerilog.
For example, the Kelvin fetch, dispatch and scalar stages are implemented in
Chisel, and vector instructions are fowarded to an RVV backend.

This document serves as both a description and tutorial for how components
built in the two languages are glued together in the Kelvin core. The intended
audience is for SystemVerilog developers who want to add/rewrite components to
the project.

## Type System

Here's a high level overview of translating concepts between SystemVerilog
primitives and Chisel primitives:

| SystemVerilog        | Chisel                   |
|----------------------|--------------------------|
| `logic`              | `Bool`                   |
| `logic [N-1:0]`      | `UInt(N.W)`              |
| `logic [N-1:0]`      | `SInt(N.W)`              |
| `enum`               | `ChiselEnum`             |
| `ArrayElemT [N-1:0]` | `Vec(N, new ArrayElemT)` |
| `struct`             | `Bundle`*                |
| `interface`          | `Bundle`*                |
| `module`             | `Module`                 |

> **NOTE:**  The `Bundle` keyword in Chisel works double duty to describe
structs and "interfaces". We'll describe this in more detail in a later section
below.

## Modules and Interfaces

### Simple Example

Below is a simple example of a Chisel module and it's generated SystemVerilog
equivalent. There is a mechanical translation of the `io` in a Chisel `Module`
into the ports of a SystemVerilog `module` via code generation.

<div style="display: flex;">
<div style="flex: 1; padding-right: 10px;">

**Chisel**
```
class ExampleModule extends Module {
  val io = IO(new Bundle {
    val in_a = Input(UInt(32.W))
    val in_b = Input(UInt(32.W))
    val out = Output(UInt(32.W))
  })

  io.out := io.in_a + io.in_b
}
```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (generated)**
```
module ExampleModule(
  input logic clock,
  input logic reset,
  input logic [31:0] io_in_a,
  input logic [31:0] io_in_b,
  output logic [31:0] io_out,
);
  assign io_out = io_in_a + io_in_b;
endmodule
```
</div>
</div>

> **NOTE:**  The `Module` keyword in Chisel will automatically add a `clock`
and `reset` ports to a module. For usage in Kelvin, assume `reset` is a
positive edge asynchronous reset. When finer grained control over ports in
Chisel are necessary,
[`RawModule`](https://www.chisel-lang.org/docs/explanations/modules#rawmodule)
is used.

### Translating "struct" Bundles

Chisel `Bundles` get generated differently into SystemVerilog when used in
modules:

<div style="display: flex;">
<div style="flex: 1; padding-right: 10px;">

**Chisel**
```
class ComplexNum extends Bundle {
    val real = UInt(32.W)
    val imag = UInt(32.W)
}

class ComplexExampleModule extends Module {
  val io = IO(new Bundle {
    val in_a = Input(new ComplexNum())
    val in_a = Input(new ComplexNum())
    val out = Output(new ComplexNum())
  })

  ...
}
```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (generated)**
```



module ComplexExampleModule(
  input logic clock,
  input logic reset,
  input logic [31:0] io_in_a_real,
  input logic [31:0] io_in_a_imag,
  input logic [31:0] io_in_b_real,
  input logic [31:0] io_in_b_imag,
  output logic [31:0] io_out_real,
  output logic [31:0] io_out_image,
);
  ...
endmodule
```
</div>
</div>

Chisel `Bundle`'s get decomposed into multiple primitive SystemVerilog ports.

### Translating "interface" Bundles

It's very common to define "interface" `Bundle`s, which also encodes direction
of each element in a `Bundle`. Here's an examples of a simple ready-valid
interface:

```
class IntReadyValidIO extends Bundle {
    val valid = Output(Bool())
    val bits = Output(UInt(32.W))
    val ready = Input(Bool())
}
```

Using this definition, we can define a single-input, single-output queue with
back pressure as follows:

<div style="display: flex;">
<div style="flex: 1; padding-right: 10px;">

**Chisel**
```
class IntReadyValidIO extends Bundle {
    val valid = Output(Bool())
    val bits = Output(UInt(32.W))
    val ready = Input(Bool())
}

class IntegerQueue extends Module {
  val io = IO(new Bundle {
    val in = Flipped(new IntReadyValidIO())
    val out = new IntReadyValidIO()
  })
  ...
}
```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (generated)**
```
module IntegerQueue(
  input logic clock,
  input logic reset,
  input logic io_in_valid,
  input logic [31:0] io_in_bits,
  output logic io_in_ready,
  output logic io_out_valid,
  output logic [31:0] io_out_bits,
  input logic io_out_ready,
);
  ...
endmodule


```
</div>
</div>

Note how in this example, we include two `IntReadyValidIO` interface `Bundle`s
in the io `Bundle` of the Chisel `Module`. The fields of each interface
`Bundle` gets expanded into sub-fields with the correct direction.

In Kelvin, we make an effort to try to differentiate between "struct `Bundle`s"
and "interface `Bundle`s". Structs should not have any directions annontated on
sub-fields, and interface `Bundle` should end with "IO".

### Vectors

Each element in a Chisel `Module` vector field will populate a new port in the
SystemVerilog generated module:

<div style="display: flex;">
<div style="flex: 1; padding-right: 10px;">

**Chisel**
```
class VectorAdder extends Module {
  val io = IO(new Bundle {
    val in = Input(Vec(4, UInt(32.W)))
    val out = Output(Vec(4, UInt(32.W)))
  })
  ...
}







```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (generated)**
```
module VectorAdder(
  input logic clock,
  input logic reset,
  input logic [31:0] io_in_0,
  input logic [31:0] io_in_1,
  input logic [31:0] io_in_2,
  input logic [31:0] io_in_3,
  output logic [31:0] io_out_0,
  output logic [31:0] io_out_1,
  output logic [31:0] io_out_2,
  output logic [31:0] io_out_3
);
  ...
endmodule
```
</div>
</div>

## Checking a Generated Interface

We maintain a small function `GenerateInterface` in
`hdl/chisel/src/common/SvGenerationUtils.scala` to create the string for the
generated interface (less clock and reset). Typical usage is as below:

```
class ValidModule extends Module {
  val io = IO(new Bundle {
    val in  = Input(Valid(UInt(32.W)))
    val out = Output(Valid(UInt(32.W)))
  })

  io.out := io.in.map(_ + 1.U)
}

class GenerateInterfaceSpec extends AnyFreeSpec with ChiselSim {
    "ValidModule" in {
        simulate(new ValidModule()) { dut =>
            val interface = GenerateInterface(dut.io, "io")
            println(interface)
        }
    }
}

>>>
input  logic io_in_valid,
input  logic [31:0] io_in_bits,
output logic io_out_valid,
output logic [31:0] io_out_bits
```

See [examples/GenerateInterfacePlayground.scala](../../examples/GenerateInterfacePlayground.scala)
for a example test that generates the interface. This test can be run with
`bazel run //examples:generate_interface_playground_test`. You're encouraged to
modify the `PlaygroundModule` module interface to see how the generated
interface changes.

## Wrapping a SystemVerilog module using a Chisel Blackbox

Recall from the above sections that the SystemVerilog generated by Chisel will
typically have more ports than the Chisel module due to:

1) **Bundles**: Each primitive member of a `Bundle` will get it's own port.
2) **Vectors**: Each element of a vector is expanded into one port for each
                primitive element, or a set of ports for `Bundle` elements.

This peculiar generated interface is often incompatible with the idiomatic
native SystemVerilog equivalent. To address this mismatch, Kelvin uses
Chisel's [BlackBox](https://www.chisel-lang.org/docs/explanations/blackboxes)
mechanism, combined with code generation techniques to "wrap" the idiomtic
SV Module.

<div style="display: flex;">
<div style="flex: 1; padding-right: 10px;">

**Chisel "BlackBox**
```
class Instruction extends Bundle {
  val inst = UInt(32.W)
  val addr = UInt(32.W)
}














class FetchBufferBB extends BlackBox with HasBlackBoxInline
                                     with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val clock = Input(AsyncReset())

    val io = new Bundle {
        val in = Vec(4, Flipped(Decoupled(new Instruction)))
        val out = Vec(4, Decoupled(new Instruction))
    }
  })
  addResource(...)
  ...
  setInline("FetchBufferBB.sv", GenerateSvSource())
}
```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (generated from BlackBox)**
```
module FetchBufferBB(
  input logic clock,
  input logic reset,
  output logic io_in_0_ready,
  input  logic io_in_0_valid,
  input  logic [31:0] io_in_0_bits_inst,
  input  logic [31:0] io_in_0_bits_addr,
  output logic io_in_1_ready,
  input  logic io_in_1_valid,
  ...
  input  logic io_out_2_ready,
  output logic io_out_2_valid,
  output logic [31:0] io_out_2_bits_inst,
  output logic [31:0] io_out_2_bits_addr,
  input  logic io_out_3_ready,
  output logic io_out_3_valid,
  output logic [31:0] io_out_3_bits_inst,
  output logic [31:0] io_out_3_bits_addr
);
  logic [3:0] io_in_valid;
  always_comb begin
    io_in_valid[0] = io_in_0_valid;
    io_in_valid[1] = io_in_1_valid;
    io_in_valid[2] = io_in_2_valid;
    io_in_valid[3] = io_in_3_valid;
  end
  ...
  FetchBuffer buffer(
      .clock(clock),
      .reset(reset),
      .io_in_valid(io_in_valid),
      ...
  );
endmodule
```
</div>
<div style="flex: 1; padding-right: 10px;">

**SystemVerilog (to be wrapped)**
```
typedef struct packed {
  logic [31:0] addr;
  logic [31:0] inst;
} Instruction_t;


















module FetchBuffer(
  input  logic clock,
  input  logic reset,
  input  logic [3:0] io_in_valid,
  input  Instruction_t io_in_data,
  output logic [3:0] io_in_ready,
  output logic [3:0] io_out_valid,
  output Instruction_t io_out_data,
  input  logic [3:0] io_out_ready
);
  ...
endmodule
```
</div>
</div>
