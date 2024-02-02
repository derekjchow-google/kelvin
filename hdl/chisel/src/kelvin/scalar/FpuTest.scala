// Copyright 2024 Google LLC
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

package kelvin

import chisel3._
import chisel3.util._
import chiseltest._
import org.scalatest.freespec.AnyFreeSpec
import chisel3.experimental.BundleLiterals._
import common.Fp32

class FpuSpec extends AnyFreeSpec with ChiselScalatestTester {
  def Float2Bits(x: Float): (Boolean, Int, Int) = {
    val abs = x.abs
    var int = java.lang.Float.floatToIntBits(abs)

    val sign: Boolean = (x < 0)
    val exponent: Int = int >> 23
    val mantissa: Int = int & ((1 << 23) - 1)

    (sign, exponent, mantissa)
  }

  def EnqueueValid(fpu: Fpu, op: FpuOptype.Type, ina: Float, inb: Float, inc: Float, waddr: Int) = {
    fpu.io.cmd.valid.poke(1)
    fpu.io.cmd.bits.optype.poke(op)
    fpu.io.cmd.bits.waddr.poke(waddr)

    val ina_bits = Float2Bits(ina)
    val inb_bits = Float2Bits(inb)
    val inc_bits = Float2Bits(inc)
    fpu.io.cmd.bits.ina.sign.poke(ina_bits._1)
    fpu.io.cmd.bits.ina.exponent.poke(ina_bits._2)
    fpu.io.cmd.bits.ina.mantissa.poke(ina_bits._3)
    fpu.io.cmd.bits.inb.sign.poke(inb_bits._1)
    fpu.io.cmd.bits.inb.exponent.poke(inb_bits._2)
    fpu.io.cmd.bits.inb.mantissa.poke(inb_bits._3)
    fpu.io.cmd.bits.inc.sign.poke(inc_bits._1)
    fpu.io.cmd.bits.inc.exponent.poke(inc_bits._2)
    fpu.io.cmd.bits.inc.mantissa.poke(inc_bits._3)
  }

  def GetFloat(fpu: Fpu): Float = {
    val sign = fpu.io.output.bits.bits.sign.peekInt().toInt
    val exponent = fpu.io.output.bits.bits.exponent.peekInt().toInt
    val mantissa = fpu.io.output.bits.bits.mantissa.peekInt().toInt
    val int_val = (exponent << 23) + mantissa
    val negate = if (sign == 1) -1.0f else 1.0f

    negate * java.lang.Float.intBitsToFloat(int_val)
  }
    
  "Pipeline" in {
    test(new Fpu) { dut =>
      dut.io.output.ready.poke(1)
      EnqueueValid(dut, FpuOptype.FpuAdd, 1.0f, 0.0f, 1.0f, 1)
      dut.clock.step()
      EnqueueValid(dut, FpuOptype.FpuSub, 1.0f, 0.0f, -1.0f, 2)
      dut.clock.step()

      assertResult(1) { dut.io.output.valid.peekInt() }
      assertResult(1) { dut.io.output.bits.addr.peekInt() }
      assertResult(2.0f) { GetFloat(dut) }
      EnqueueValid(dut, FpuOptype.FpuMul, 1.0f, 1.0f, 0.0f, 3)
      dut.clock.step()

      assertResult(1) { dut.io.output.valid.peekInt() }
      assertResult(2) { dut.io.output.bits.addr.peekInt() }
      assertResult(2.0f) { GetFloat(dut) }
      dut.io.cmd.valid.poke(0)
      dut.clock.step()


      assertResult(1) { dut.io.output.valid.peekInt() }
      assertResult(3) { dut.io.output.bits.addr.peekInt() }
      assertResult(1.0f) { GetFloat(dut) }
      dut.clock.step()

      assertResult(0) { dut.io.output.valid.peekInt() }
    }
  }
}