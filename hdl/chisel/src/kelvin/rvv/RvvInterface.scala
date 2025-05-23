// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package kelvin.rvv

import chisel3._
import chisel3.util._
import kelvin.{RegfileReadDataIO, RegfileWriteDataIO, Parameters}

class RvvConfigState(p: Parameters) extends Bundle {
  // TODO(derekjchow): Parameterize correctly
  val vl = Output(UInt(8.W))
  val vstart = Output(UInt(7.W))
  val ma = Output(Bool())
  val ta = Output(Bool())
  val xrm = Output(UInt(2.W))
  val sew = Output(UInt(3.W))
  val lmul = Output(UInt(3.W))
}

class Lsu2Rvv(p: Parameters) extends Bundle {
  // TODO(derekjchow): Parameterize correctly
  val addr = UInt(5.W)
  val data = UInt(128.W)
  val last = Bool()
}

class Rvv2Lsu(p: Parameters) extends Bundle {
  // TODO(derekjchow): Parameterize correctly
  val idx = Valid(new Bundle {
    val addr = UInt(5.W)
    val data = UInt(128.W)
  })
  val vregfile = Valid(new Bundle {
    val addr = UInt(5.W)
    val data = UInt(128.W)
  })
  val mask = Valid(UInt(16.W))
}

class RvvCoreIO(p: Parameters) extends Bundle {
    // Decode Cycle.
    val inst = Vec(p.instructionLanes,
        Flipped(Decoupled(new RvvCompressedInstruction)))

    // Execute cycle.
    val rs = Vec(p.instructionLanes * 2, Flipped(new RegfileReadDataIO))
    val rd = Vec(p.instructionLanes, Valid(new RegfileWriteDataIO))

    val rvv2lsu = Vec(2, Decoupled(new Rvv2Lsu(p)))
    val lsu2rvv = Vec(2, Flipped(Decoupled(new Lsu2Rvv(p))))

    // Config state.
    val configState = Output(Valid(new RvvConfigState(p)))

    // Async scalar regfile writes.
    val async_rd = Decoupled(new RegfileWriteDataIO)
}