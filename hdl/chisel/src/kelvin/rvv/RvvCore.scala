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
import common.{MakeInvalid}
import kelvin.{Parameters, RegfileWriteDataIO}

object RvvCore {
  def apply(p: Parameters): RvvCore = {
    return Module(new RvvCore(p))
  }
}

class RvvCore(p: Parameters) extends Module {
  val io = IO(new RvvCoreIO(p))

  // Stub out signals for the time being
  io.inst.foreach(_.ready := false.B)
  io.rd.foreach(_ := MakeInvalid(new RegfileWriteDataIO))
  io.async_rd.valid := false.B
  io.async_rd.bits.addr := 0.U
  io.async_rd.bits.data := 0.U
}