// Copyright 2023 Google LLC
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
import common._

object Bru {
  def apply(p: Parameters): Bru = {
    return Module(new Bru(p))
  }
}

object BruOp extends ChiselEnum {
  val JAL  = Value
  val JALR = Value
  val BEQ  = Value
  val BNE  = Value
  val BLT  = Value
  val BGE  = Value
  val BLTU = Value
  val BGEU = Value
  val EBREAK = Value
  val ECALL = Value
  val EEXIT = Value
  val EYIELD = Value
  val ECTXSW = Value
  val MPAUSE = Value
  val MRET = Value
  val FENCEI = Value
  val UNDEF = Value
}

class BruCmd(p: Parameters) extends Bundle {
  val fwd = Bool()
  val op = BruOp()
  val pc = UInt(p.programCounterBits.W)
  val target = UInt(p.programCounterBits.W)
  val link = UInt(5.W)
}

class BranchTakenIO(p: Parameters) extends Bundle {
  val valid = Output(Bool())
  val value = Output(UInt(p.programCounterBits.W))
}

class BranchState(p: Parameters) extends Bundle {
  val fwd = Bool()
  val op = BruOp()
  val target = UInt(p.programCounterBits.W)
  val linkValid = Bool()
  val linkAddr = UInt(5.W)
  val linkData = UInt(p.programCounterBits.W)
  val pcEx = UInt(32.W)
}

object BranchState {
  def default(p: Parameters): BranchState = {
    val result = Wire(new BranchState(p))
    result.fwd := false.B
    result.op := BruOp.JAL
    result.target := 0.U
    result.linkValid := false.B
    result.linkAddr := 0.U
    result.linkData := 0.U
    result.pcEx := 0.U
    result
  }
}

class Bru(p: Parameters) extends Module {
  val io = IO(new Bundle {
    // Decode cycle.
    val req = Flipped(Valid(new BruCmd(p)))

    // Execute cycle.
    val csr = new CsrBruIO(p)
    val rs1 = Input(new RegfileReadDataIO)
    val rs2 = Input(new RegfileReadDataIO)
    val rd  = Flipped(new RegfileWriteDataIO)
    val taken = new BranchTakenIO(p)
    val target = Flipped(new RegfileBranchTargetIO)
    val interlock = Output(Bool())
    val iflush = Output(Bool())
  })

  // Interlock
  val interlock = RegInit(false.B)
  interlock := io.req.valid && io.req.bits.op.isOneOf(
      BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT, BruOp.EYIELD, BruOp.ECTXSW,
      BruOp.MPAUSE, BruOp.MRET)
  io.interlock := interlock

  // Assign state
  val mode = io.csr.out.mode  // (0) machine, (1) user

  val pcDe  = io.req.bits.pc
  val pc4De = io.req.bits.pc + 4.U

  val mret = (io.req.bits.op === BruOp.MRET) && !mode
  val call = ((io.req.bits.op === BruOp.MRET) && mode) ||
      io.req.bits.op.isOneOf(BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT,
                             BruOp.EYIELD, BruOp.ECTXSW, BruOp.MPAUSE)

  val stateReg = RegInit(MakeValid(false.B, BranchState.default(p)))
  val nextState = Wire(new BranchState(p))
  nextState.linkValid := io.req.valid && (io.req.bits.link =/= 0.U) &&
               (io.req.bits.op.isOneOf(BruOp.JAL, BruOp.JALR))

  nextState.op := io.req.bits.op
  nextState.fwd := io.req.valid && io.req.bits.fwd

  nextState.linkAddr := io.req.bits.link
  nextState.linkData := pc4De
  nextState.pcEx := pcDe

  nextState.target := MuxCase(io.req.bits.target, Seq(
      mret -> io.csr.out.mepc,
      call -> io.csr.out.mepc,
      (io.req.bits.fwd || (io.req.bits.op === BruOp.FENCEI)) -> pc4De,
      (io.req.bits.op === BruOp.JALR) -> io.target.data,
  ))
  stateReg.valid := io.req.valid
  stateReg.bits := nextState

  // This mux sits on the critical path.
  // val rs1 = Mux(readRs, io.rs1.data, 0.U)
  // val rs2 = Mux(readRs, io.rs2.data, 0.U)
  val rs1 = io.rs1.data
  val rs2 = io.rs2.data

  val eq  = rs1 === rs2
  val neq = !eq
  val lt  = rs1.asSInt < rs2.asSInt
  val ge  = !lt
  val ltu = rs1 < rs2
  val geu = !ltu

  val op = stateReg.bits.op

  io.taken.valid := stateReg.valid && MuxLookup(op, false.B)(Seq(
    BruOp.EBREAK -> mode,
    BruOp.ECALL  -> mode,
    BruOp.EEXIT  -> mode,
    BruOp.EYIELD -> mode,
    BruOp.ECTXSW -> mode,
    BruOp.MPAUSE -> mode,  // fault
    BruOp.MRET   -> true.B,  // fault if user mode.
    BruOp.FENCEI -> true.B,
    BruOp.JAL    -> (true.B =/= stateReg.bits.fwd),
    BruOp.JALR   -> (true.B =/= stateReg.bits.fwd),
    BruOp.BEQ    -> (eq  =/= stateReg.bits.fwd),
    BruOp.BNE    -> (neq =/= stateReg.bits.fwd),
    BruOp.BLT    -> (lt  =/= stateReg.bits.fwd),
    BruOp.BGE    -> (ge  =/= stateReg.bits.fwd),
    BruOp.BLTU   -> (ltu =/= stateReg.bits.fwd),
    BruOp.BGEU   -> (geu =/= stateReg.bits.fwd),
  ))
  io.taken.value := stateReg.bits.target

  io.rd.valid := stateReg.valid && stateReg.bits.linkValid
  io.rd.addr := stateReg.bits.linkAddr
  io.rd.data := stateReg.bits.linkData

  // Undefined Fault.
  val undefFault = stateReg.valid && (op === BruOp.UNDEF)

  // Usage Fault.
  val usageFault = stateReg.valid && Mux(
      mode, op.isOneOf(BruOp.MPAUSE, BruOp.MRET),
            op.isOneOf(BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT, BruOp.EYIELD,
                       BruOp.ECTXSW))

  io.csr.in.mode.valid := stateReg.valid && Mux(
      mode, op.isOneOf(BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT, BruOp.EYIELD,
                       BruOp.ECTXSW, BruOp.MPAUSE, BruOp.MRET),
            (op === BruOp.MRET))
  io.csr.in.mode.bits := ((op === BruOp.MRET) && !mode)

  io.csr.in.mepc.valid := stateReg.valid && mode &&
      op.isOneOf(BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT, BruOp.EYIELD,
                 BruOp.ECTXSW, BruOp.MPAUSE, BruOp.MRET)
  io.csr.in.mepc.bits := Mux(op === BruOp.EYIELD, stateReg.bits.linkData,
                                                  stateReg.bits.pcEx)

  io.csr.in.mcause.valid := stateReg.valid && (undefFault || usageFault ||
      (mode && op.isOneOf(BruOp.EBREAK, BruOp.ECALL, BruOp.EEXIT, BruOp.EYIELD,
                          BruOp.ECTXSW)))

  val faultMsb = 1.U << 31
  io.csr.in.mcause.bits := MuxCase(0.U, Seq(
      undefFault        -> (2.U  | faultMsb),
      usageFault        -> (16.U | faultMsb),
      (op === BruOp.EBREAK) -> 1.U,
      (op === BruOp.ECALL)  -> 2.U,
      (op === BruOp.EEXIT)  -> 3.U,
      (op === BruOp.EYIELD) -> 4.U,
      (op === BruOp.ECTXSW) -> 5.U,
  ))

  io.csr.in.mtval.valid := stateReg.valid && (undefFault || usageFault)
  io.csr.in.mtval.bits := stateReg.bits.pcEx

  io.iflush := stateReg.valid && (op === BruOp.FENCEI)

  // Pipeline will be halted.
  io.csr.in.halt := (stateReg.valid && (op === BruOp.MPAUSE) && !mode) ||
                    io.csr.in.fault
  io.csr.in.fault := (undefFault && !mode) || (usageFault && !mode)

  // Assertions.
  val ignore = op.isOneOf(BruOp.JAL, BruOp.JALR, BruOp.EBREAK, BruOp.ECALL,
                          BruOp.EEXIT, BruOp.EYIELD, BruOp.ECTXSW, BruOp.MPAUSE,
                          BruOp.MRET, BruOp.FENCEI, BruOp.UNDEF)

  assert(!(stateReg.valid && !io.rs1.valid) || ignore)
  assert(!(stateReg.valid && !io.rs2.valid) || ignore)
}
