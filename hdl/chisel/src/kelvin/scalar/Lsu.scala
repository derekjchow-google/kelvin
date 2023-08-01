package kelvin

import chisel3._
import chisel3.util._
import common._

object Lsu {
  def apply(p: Parameters): Lsu = {
    return Module(new Lsu(p))
  }
}

class DBusIO(p: Parameters, bank: Boolean = false) extends Bundle {
  // Control Phase.
  val valid = Output(Bool())
  val ready = Input(Bool())
  val write = Output(Bool())
  val addr = Output(UInt((p.lsuAddrBits - (if (bank) 1 else 0)).W))
  val adrx = Output(UInt((p.lsuAddrBits - (if (bank) 1 else 0)).W))
  val size = Output(UInt((log2Ceil(p.lsuDataBits / 8) + 1).W))
  val wdata = Output(UInt(p.lsuDataBits.W))
  val wmask = Output(UInt((p.lsuDataBits / 8).W))
  // Read Phase.
  val rdata = Input(UInt(p.lsuDataBits.W))
}

case class LsuOp() {
  val LB  = 0
  val LH  = 1
  val LW  = 2
  val LBU = 3
  val LHU = 4
  val SB  = 5
  val SH  = 6
  val SW  = 7
  val FENCEI = 8
  val FLUSHAT = 9
  val FLUSHALL = 10
  val VLDST = 11
  val Entries = 12
}

class LsuIO(p: Parameters) extends Bundle {
  val valid = Input(Bool())
  val ready = Output(Bool())
  val store = Input(Bool())
  val addr = Input(UInt(5.W))
  val op = Input(UInt(new LsuOp().Entries.W))
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
  val suncd = Bool()  // scalar load/store uncached
}

class LsuReadData(p: Parameters) extends Bundle {
  val addr = UInt(32.W)
  val index = UInt(5.W)
  val size = UInt((log2Ceil(p.lsuDataBits / 8) + 1).W)
  val sext = Bool()
  val iload = Bool()
  val sldst = Bool()
  val suncd = Bool()
}

class Lsu(p: Parameters) extends Module {
  val io = IO(new Bundle {
    // Decode cycle.
    val req = Vec(4, new LsuIO(p))
    val busPort = Flipped(new RegfileBusPortIO)

    // Execute cycle(s).
    val rd = Flipped(new RegfileWriteDataIO)

    // Cached interface.
    val dbus = new DBusIO(p)
    val flush = new DFlushFenceiIO(p)

    // Uncached interface.
    val ubus = new DBusIO(p)

    // Vector switch.
    val vldst = Output(Bool())
  })

  val lsu = new LsuOp()

  // AXI Queues.
  val n = 8
  val ctrl = Fifo4(new LsuCtrl(p), n)
  val data = Slice(new LsuReadData(p), true, true)

  // Match and mask.
  val ctrlready = Cat(ctrl.io.count <= (n - 4).U,
                      ctrl.io.count <= (n - 3).U,
                      ctrl.io.count <= (n - 2).U,
                      ctrl.io.count <= (n - 1).U)

  io.req(0).ready := ctrlready(0) && data.io.in.ready
  io.req(1).ready := ctrlready(1) && data.io.in.ready
  io.req(2).ready := ctrlready(2) && data.io.in.ready
  io.req(3).ready := ctrlready(3) && data.io.in.ready

  // Address phase must use simple logic to resolve mask for unaligned address.
  val linebit = log2Ceil(p.lsuDataBits / 8)
  val lineoffset = (p.lsuDataBits / 8)

  // ---------------------------------------------------------------------------
  // Control Port Inputs.
  ctrl.io.in.valid := io.req(0).valid || io.req(1).valid ||
                      io.req(2).valid || io.req(3).valid

  for (i <- 0 until 4) {
    val uncached = io.busPort.addr(i)(31)

    val opstore = io.req(i).op(lsu.SW) || io.req(i).op(lsu.SH) || io.req(i).op(lsu.SB)
    val opiload = io.req(i).op(lsu.LW) || io.req(i).op(lsu.LH) || io.req(i).op(lsu.LB) || io.req(i).op(lsu.LHU) || io.req(i).op(lsu.LBU)
    val opload  = opiload
    val opfencei   = io.req(i).op(lsu.FENCEI)
    val opflushat  = io.req(i).op(lsu.FLUSHAT)
    val opflushall = io.req(i).op(lsu.FLUSHALL)
    val opsldst = opstore || opload
    val opvldst = io.req(i).op(lsu.VLDST)
    val opsext = io.req(i).op(lsu.LB) || io.req(i).op(lsu.LH)
    val opsize = Cat(io.req(i).op(lsu.LW) || io.req(i).op(lsu.SW),
                     io.req(i).op(lsu.LH) || io.req(i).op(lsu.LHU) || io.req(i).op(lsu.SH),
                     io.req(i).op(lsu.LB) || io.req(i).op(lsu.LBU) || io.req(i).op(lsu.SB))

    ctrl.io.in.bits(i).valid := io.req(i).valid && ctrlready(i) && !(opvldst && uncached)

    ctrl.io.in.bits(i).bits.addr := io.busPort.addr(i)
    ctrl.io.in.bits(i).bits.adrx := io.busPort.addr(i) + lineoffset.U
    ctrl.io.in.bits(i).bits.data := io.busPort.data(i)
    ctrl.io.in.bits(i).bits.index := io.req(i).addr
    ctrl.io.in.bits(i).bits.sext := opsext
    ctrl.io.in.bits(i).bits.size := opsize
    ctrl.io.in.bits(i).bits.iload := opiload
    ctrl.io.in.bits(i).bits.fencei   := opfencei
    ctrl.io.in.bits(i).bits.flushat  := opflushat
    ctrl.io.in.bits(i).bits.flushall := opflushall
    ctrl.io.in.bits(i).bits.sldst := opsldst && !uncached
    ctrl.io.in.bits(i).bits.vldst := opvldst
    ctrl.io.in.bits(i).bits.suncd := opsldst && uncached
    ctrl.io.in.bits(i).bits.write := !opload
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

  io.dbus.valid := ctrl.io.out.valid && ctrl.io.out.bits.sldst
  io.dbus.write := ctrl.io.out.bits.write
  io.dbus.addr  := Cat(0.U(1.W), ctrl.io.out.bits.addr(30,0))
  io.dbus.adrx  := Cat(0.U(1.W), ctrl.io.out.bits.adrx(30,0))
  io.dbus.size  := ctrl.io.out.bits.size
  io.dbus.wdata := wdata
  io.dbus.wmask := wmask
  assert(!(io.dbus.valid && ctrl.io.out.bits.addr(31)))
  assert(!(io.dbus.valid && io.dbus.addr(31)))
  assert(!(io.dbus.valid && io.dbus.adrx(31)))

  io.ubus.valid := ctrl.io.out.valid && ctrl.io.out.bits.suncd
  io.ubus.write := ctrl.io.out.bits.write
  io.ubus.addr  := Cat(0.U(1.W), ctrl.io.out.bits.addr(30,0))
  io.ubus.adrx  := Cat(0.U(1.W), ctrl.io.out.bits.adrx(30,0))
  io.ubus.size  := ctrl.io.out.bits.size
  io.ubus.wdata := wdata
  io.ubus.wmask := wmask
  assert(!(io.ubus.valid && !ctrl.io.out.bits.addr(31)))
  assert(!(io.ubus.valid && io.dbus.addr(31)))
  assert(!(io.ubus.valid && io.dbus.adrx(31)))

  io.flush.valid  := ctrl.io.out.valid && (ctrl.io.out.bits.fencei || ctrl.io.out.bits.flushat || ctrl.io.out.bits.flushall)
  io.flush.all    := ctrl.io.out.bits.fencei || ctrl.io.out.bits.flushall
  io.flush.clean  := true.B
  io.flush.fencei := ctrl.io.out.bits.fencei

  ctrl.io.out.ready := io.flush.valid && io.flush.ready ||
                       io.dbus.valid && io.dbus.ready ||
                       io.ubus.valid && io.ubus.ready ||
                       ctrl.io.out.bits.vldst && io.dbus.ready

  io.vldst := ctrl.io.out.valid && ctrl.io.out.bits.vldst

  // ---------------------------------------------------------------------------
  // Load response.
  data.io.in.valid := io.dbus.valid && io.dbus.ready && !io.dbus.write ||
                      io.ubus.valid && io.ubus.ready && !io.ubus.write

  data.io.in.bits.addr  := ctrl.io.out.bits.addr
  data.io.in.bits.index := ctrl.io.out.bits.index
  data.io.in.bits.sext  := ctrl.io.out.bits.sext
  data.io.in.bits.size  := ctrl.io.out.bits.size
  data.io.in.bits.iload := ctrl.io.out.bits.iload
  data.io.in.bits.sldst := ctrl.io.out.bits.sldst
  data.io.in.bits.suncd := ctrl.io.out.bits.suncd

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

  val rdata = RotSignExt(MuxOR(data.io.out.bits.sldst, io.dbus.rdata) |
                         MuxOR(data.io.out.bits.suncd, io.ubus.rdata))

  // pass-through
  io.rd.valid := rvalid && data.io.out.bits.iload
  io.rd.addr  := data.io.out.bits.index
  io.rd.data  := rdata

  assert(!ctrl.io.out.valid || PopCount(Cat(ctrl.io.out.bits.sldst, ctrl.io.out.bits.vldst, ctrl.io.out.bits.suncd)) <= 1.U)
  assert(!data.io.out.valid || PopCount(Cat(data.io.out.bits.sldst, data.io.out.bits.suncd)) <= 1.U)
}
