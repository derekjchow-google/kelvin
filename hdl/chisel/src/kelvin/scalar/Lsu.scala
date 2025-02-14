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

object Lsu {
  def apply(p: Parameters): Lsu = {
    return Module(new Lsu(p))
  }
}

object LsuOp extends ChiselEnum {
  val LB  = Value
  val LH  = Value
  val LW  = Value
  val LBU = Value
  val LHU = Value
  val SB  = Value
  val SH  = Value
  val SW  = Value
  val FENCEI = Value
  val FLUSHAT = Value
  val FLUSHALL = Value
  val VLDST = Value
}

class LsuCmd extends Bundle {
  val store = Bool()
  val addr = UInt(5.W)
  val op = LsuOp()
}

class LsuCtrl(p: Parameters) extends Bundle {
  val addr = UInt(32.W)
  val adrx = UInt(32.W)
  val data = UInt(32.W)
  val index = UInt(5.W)
  val size = UInt((log2Ceil(p.lsuDataBits / 8) + 1).W)
  val write = Bool()
  val sext = Bool()
  val iload = Bool()
  val fencei = Bool()
  val flushat = Bool()
  val flushall = Bool()
  val sldst = Bool()  // scalar load/store cached
  val vldst = Bool()  // vector load/store
  val regionType = MemoryRegionType()
}

class LsuReadData(p: Parameters) extends Bundle {
  val addr = UInt(32.W)
  val index = UInt(5.W)
  val size = UInt((log2Ceil(p.lsuDataBits / 8) + 1).W)
  val sext = Bool()
  val iload = Bool()
  val sldst = Bool()
  val regionType = MemoryRegionType()
}

class Lsu(p: Parameters) extends Module {
  val io = IO(new Bundle {
    // Decode cycle.
    val req = Vec(p.instructionLanes, Flipped(Decoupled(new LsuCmd)))
    val busPort = Flipped(new RegfileBusPortIO(p))

    // Execute cycle(s).
    val rd = Valid(Flipped(new RegfileWriteDataIO))

    // Cached interface.
    val ibus = new IBusIO(p)
    val dbus = new DBusIO(p)
    val flush = new DFlushFenceiIO(p)

    // DBus that will eventually reach an external bus.
    // Intended for sending a transaction to an external
    // peripheral, likely on TileLink or AXI.
    val ebus = new EBusIO(p)

    // Vector switch.
    val vldst = Output(Bool())

    val storeCount = Output(UInt(2.W))
    val active = Output(Bool())
  })

  // AXI Queues.
  val n = 8
  val ctrl = FifoX(new LsuCtrl(p), p.instructionLanes, n)
  val data = Slice(new LsuReadData(p), true, true)

  // Match and mask.
  io.active :=
    (ctrl.io.count =/= 0.U || data.io.count =/= 0.U)
  val ctrlready = (1 to p.instructionLanes).reverse.map(x => ctrl.io.count <= (n - x).U)

  for (i <- 0 until p.instructionLanes) {
    io.req(i).ready := ctrlready(i) && data.io.in.ready
  }

  // Address phase must use simple logic to resolve mask for unaligned address.
  val linebit = log2Ceil(p.lsuDataBits / 8)
  val lineoffset = (p.lsuDataBits / 8)

  // ---------------------------------------------------------------------------
  // Control Port Inputs.
  ctrl.io.in.valid := io.req.map(_.valid).reduce(_||_)

  for (i <- 0 until p.instructionLanes) {
    val itcm = p.m.filter(_.memType == MemoryRegionType.IMEM)
                  .map(_.contains(io.busPort.addr(i))).reduceOption(_ || _).getOrElse(false.B)
    val dtcm = p.m.filter(_.memType == MemoryRegionType.DMEM)
                  .map(_.contains(io.busPort.addr(i))).reduceOption(_ || _).getOrElse(true.B)
    val peri = p.m.filter(_.memType == MemoryRegionType.Peripheral)
                  .map(_.contains(io.busPort.addr(i))).reduceOption(_ || _).getOrElse(false.B)
    val external = !(itcm || dtcm || peri)
    assert(PopCount(Cat(itcm | dtcm | peri)) <= 1.U)

    val opstore = io.req(i).bits.op.isOneOf(LsuOp.SW, LsuOp.SH, LsuOp.SB)
    val opiload = io.req(i).bits.op.isOneOf(LsuOp.LW, LsuOp.LH, LsuOp.LB, LsuOp.LHU, LsuOp.LBU)
    val opload  = opiload
    val opfencei   = (io.req(i).bits.op === LsuOp.FENCEI)
    val opflushat  = (io.req(i).bits.op === LsuOp.FLUSHAT)
    val opflushall = (io.req(i).bits.op === LsuOp.FLUSHALL)
    val opsldst = opstore || opload
    val opvldst = (io.req(i).bits.op === LsuOp.VLDST)
    val opsext = io.req(i).bits.op.isOneOf(LsuOp.LB, LsuOp.LH)
    val opsize = Cat(io.req(i).bits.op.isOneOf(LsuOp.LW, LsuOp.SW),
                     io.req(i).bits.op.isOneOf(LsuOp.LH, LsuOp.LHU, LsuOp.SH),
                     io.req(i).bits.op.isOneOf(LsuOp.LB, LsuOp.LBU, LsuOp.SB))

    ctrl.io.in.bits(i).valid := io.req(i).valid && ctrlready(i)

    ctrl.io.in.bits(i).bits.addr := io.busPort.addr(i)
    ctrl.io.in.bits(i).bits.adrx := io.busPort.addr(i) + lineoffset.U
    ctrl.io.in.bits(i).bits.data := io.busPort.data(i)
    ctrl.io.in.bits(i).bits.index := io.req(i).bits.addr
    ctrl.io.in.bits(i).bits.sext := opsext
    ctrl.io.in.bits(i).bits.size := opsize
    ctrl.io.in.bits(i).bits.iload := opiload
    ctrl.io.in.bits(i).bits.fencei   := opfencei
    ctrl.io.in.bits(i).bits.flushat  := opflushat
    ctrl.io.in.bits(i).bits.flushall := opflushall
    ctrl.io.in.bits(i).bits.sldst := opsldst
    ctrl.io.in.bits(i).bits.vldst := opvldst
    ctrl.io.in.bits(i).bits.write := !opload
    ctrl.io.in.bits(i).bits.regionType := MuxCase(MemoryRegionType.External, Array(
      dtcm -> MemoryRegionType.DMEM,
      itcm -> MemoryRegionType.IMEM,
      peri -> MemoryRegionType.Peripheral,
    ))
  }

  // ---------------------------------------------------------------------------
  // Control Port Outputs.
  val wsel = ctrl.io.out.bits.addr(1,0)
  val wda = ctrl.io.out.bits.data
  val wdataS =
    MuxOR(wsel === 0.U, wda(31,0)) |
    MuxOR(wsel === 1.U, Cat(wda(23,16), wda(15,8), wda(7,0), wda(31,24))) |
    MuxOR(wsel === 2.U, Cat(wda(15,8), wda(7,0), wda(31,24), wda(23,16))) |
    MuxOR(wsel === 3.U, Cat(wda(7,0), wda(31,24), wda(23,16), wda(15,8)))
  val wmaskB = p.lsuDataBits / 8
  val wmaskT = (~0.U(wmaskB.W)) >> (wmaskB.U - ctrl.io.out.bits.size)
  val wmaskS = (wmaskT << ctrl.io.out.bits.addr(linebit-1,0)) |
               (wmaskT >> (lineoffset.U - ctrl.io.out.bits.addr(linebit-1,0)))
  val wdata = Wire(UInt(p.lsuDataBits.W))
  val wmask = wmaskS(lineoffset - 1, 0)

  if (p.lsuDataBits == 128) {
    wdata := Cat(wdataS, wdataS, wdataS, wdataS)
  } else if (p.lsuDataBits == 256) {
    wdata := Cat(wdataS, wdataS, wdataS, wdataS,
                 wdataS, wdataS, wdataS, wdataS)
  } else if (p.lsuDataBits == 512) {
    wdata := Cat(wdataS, wdataS, wdataS, wdataS,
                 wdataS, wdataS, wdataS, wdataS,
                 wdataS, wdataS, wdataS, wdataS,
                 wdataS, wdataS, wdataS, wdataS)
  } else {
    assert(false)
  }

  io.dbus.valid := ctrl.io.out.valid && ctrl.io.out.bits.sldst && (ctrl.io.out.bits.regionType === MemoryRegionType.DMEM)
  io.dbus.write := ctrl.io.out.bits.write
  io.dbus.addr  := Cat(0.U(1.W), ctrl.io.out.bits.addr(30,0))
  io.dbus.adrx  := Cat(0.U(1.W), ctrl.io.out.bits.adrx(30,0))
  io.dbus.size  := ctrl.io.out.bits.size
  io.dbus.wdata := wdata
  io.dbus.wmask := wmask

  assert(!(io.dbus.valid && ctrl.io.out.bits.addr(31)))
  assert(!(io.dbus.valid && io.dbus.addr(31)))
  assert(!(io.dbus.valid && io.dbus.adrx(31)))

  io.ebus.dbus.valid := ctrl.io.out.valid && ctrl.io.out.bits.sldst &&
    ((ctrl.io.out.bits.regionType === MemoryRegionType.External) || (ctrl.io.out.bits.regionType === MemoryRegionType.Peripheral))
  io.ebus.dbus.write := ctrl.io.out.bits.write
  io.ebus.dbus.addr := ctrl.io.out.bits.addr
  io.ebus.dbus.adrx := ctrl.io.out.bits.adrx
  io.ebus.dbus.size := ctrl.io.out.bits.size
  io.ebus.dbus.wdata := wdata
  io.ebus.dbus.wmask := wmask
  io.ebus.internal := ctrl.io.out.bits.regionType === MemoryRegionType.Peripheral

  io.ibus.valid := ctrl.io.out.valid && ctrl.io.out.bits.sldst && (ctrl.io.out.bits.regionType === MemoryRegionType.IMEM)
  // TODO(atv): This should actually raise some sort of error, and trigger a store fault
  assert(!(io.ibus.valid && (ctrl.io.out.bits.regionType === MemoryRegionType.IMEM) && ctrl.io.out.bits.write))
  io.ibus.addr := ctrl.io.out.bits.addr


  io.storeCount := PopCount(Cat(
    io.dbus.valid && io.dbus.write,
    io.ebus.dbus.valid && io.ebus.dbus.write
  ))

  io.flush.valid  := ctrl.io.out.valid && (ctrl.io.out.bits.fencei || ctrl.io.out.bits.flushat || ctrl.io.out.bits.flushall)
  io.flush.all    := ctrl.io.out.bits.fencei || ctrl.io.out.bits.flushall
  io.flush.clean  := true.B
  io.flush.fencei := ctrl.io.out.bits.fencei

  ctrl.io.out.ready := io.flush.valid && io.flush.ready ||
                       io.dbus.valid && io.dbus.ready ||
                       io.ebus.dbus.valid && io.ebus.dbus.ready ||
                       io.ibus.valid && io.ibus.ready ||
                       ctrl.io.out.bits.vldst && io.dbus.ready

  io.vldst := ctrl.io.out.valid && ctrl.io.out.bits.vldst

  // ---------------------------------------------------------------------------
  // Load response.
  data.io.in.valid := io.dbus.valid && io.dbus.ready && !io.dbus.write ||
                      io.ebus.dbus.valid && io.ebus.dbus.ready && !io.ebus.dbus.write ||
                      io.ibus.valid && io.ibus.ready

  data.io.in.bits.addr  := ctrl.io.out.bits.addr
  data.io.in.bits.index := ctrl.io.out.bits.index
  data.io.in.bits.sext  := ctrl.io.out.bits.sext
  data.io.in.bits.size  := ctrl.io.out.bits.size
  data.io.in.bits.iload := ctrl.io.out.bits.iload
  data.io.in.bits.sldst := ctrl.io.out.bits.sldst
  data.io.in.bits.regionType := ctrl.io.out.bits.regionType

  data.io.out.ready := true.B

  assert(!(ctrl.io.in.valid && !data.io.in.ready))

  // ---------------------------------------------------------------------------
  // Register file ports.
  val rvalid = data.io.out.valid
  val rsext = data.io.out.bits.sext
  val rsize = data.io.out.bits.size
  val rsel  = data.io.out.bits.addr(linebit - 1, 0)

  // Rotate and sign extend.
  def RotSignExt(datain: UInt, dataout: UInt = 0.U(p.lsuDataBits.W), i: Int = 0): UInt = {
    assert(datain.getWidth  == p.lsuDataBits)
    assert(dataout.getWidth == p.lsuDataBits)

    if (i < p.lsuDataBits / 8) {
      val mod = p.lsuDataBits

      val rdata = Cat(datain((8 * (i + 3) + 7) % mod, (8 * (i + 3)) % mod),
                      datain((8 * (i + 2) + 7) % mod, (8 * (i + 2)) % mod),
                      datain((8 * (i + 1) + 7) % mod, (8 * (i + 1)) % mod),
                      datain((8 * (i + 0) + 7) % mod, (8 * (i + 0)) % mod))

      val sizeMask = Mux(rsize === 4.U, 0xffffffff.S(32.W).asUInt,
                     Mux(rsize === 2.U, 0x0000ffff.U(32.W), 0x000000ff.U(32.W)))

      val signExtend = Mux(rsext,
                         Mux(rsize === 2.U,
                           Mux(rdata(15), 0xffff0000.S(32.W).asUInt, 0.U(32.W)),
                           Mux(rdata(7),  0xffffff00.S(32.W).asUInt, 0.U(32.W))),
                         0.U)
      assert(sizeMask.getWidth == 32)
      assert(signExtend.getWidth == 32)

      val sdata = MuxOR(rsel === i.U, rdata & sizeMask | signExtend)
      RotSignExt(datain, dataout | sdata, i + 1)
    } else {
      dataout
    }
  }

  val regionType = data.io.out.bits.regionType
  val srdata = MuxLookup(regionType, 0.U.asTypeOf(io.dbus.rdata))(Seq(
    MemoryRegionType.DMEM -> io.dbus.rdata,
    MemoryRegionType.IMEM -> io.ibus.rdata,
    MemoryRegionType.External -> io.ebus.dbus.rdata,
    MemoryRegionType.Peripheral -> io.ebus.dbus.rdata,
  ))
  val rdata = RotSignExt(MuxOR(data.io.out.bits.sldst, srdata))

  // pass-through
  val io_rd_pre_pipe = Wire(Valid(Flipped(new RegfileWriteDataIO)))
  io_rd_pre_pipe.valid := rvalid && data.io.out.bits.iload
  io_rd_pre_pipe.bits.addr  := data.io.out.bits.index
  io_rd_pre_pipe.bits.data  := rdata

  // Add one cycle pipeline delay to io.rd passthrough for timing
  val io_rd_pipe = Pipe(io_rd_pre_pipe, p.lsuDelayPipelineLen)
  io.rd := io_rd_pipe

  assert(!ctrl.io.out.valid || PopCount(Cat(ctrl.io.out.bits.sldst, ctrl.io.out.bits.vldst)) <= 1.U)
  assert(!data.io.out.valid || PopCount(Cat(data.io.out.bits.sldst)) <= 1.U)
}
