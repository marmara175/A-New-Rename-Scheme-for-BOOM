//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename BusyTable
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Christopher Celio

package boom

import Chisel._
import config.Parameters

// internally bypasses newly busy registers (.write) to the read ports (.read)
// num_operands is the maximum number of operands per instruction (.e.g., 2 normally, but 3 if FMAs are supported)
class WakeupUnBusy(preg_sz: Int) extends Bundle
{
   val valid = Bool()
   val vdst  = UInt(width = preg_sz)
   val state = UInt(2.W)

   override def cloneType: this.type = new WakeupUnBusy(preg_sz).asInstanceOf[this.type]
}

class BusyTableIo(
   pipeline_width:Int,
   num_pregs: Int,
   num_read_ports:Int,
   num_wb_ports:Int)
   (implicit p: Parameters) extends BoomBundle()(p)
{
   private val preg_sz = TPREG_SZ

   // reading out the busy bits
   val p_rs           = Vec(num_read_ports, UInt(width=preg_sz)).asInput
   val p_rs_busy      = Vec(num_read_ports, Bool()).asOutput

   def prs(i:Int, w:Int):UInt      = p_rs     (w+i*pipeline_width)
   def prs2_busy(i:Int, w:Int):Bool = p_rs_busy(w+i*pipeline_width)

   // marking new registers as busy
   val allocated_vdst = Vec(pipeline_width, new ValidIO(UInt(width=preg_sz))).flip

   // marking registers being written back as unbusy
   val unbusy_vdst    = Vec(num_wb_ports, new WakeupUnBusy(preg_sz)).asInput
   //val unbusy_state   = Vec(num_wb_ports, UInt(2.W))

   val debug = new Bundle { val busytable= Bits(width=num_pregs).asOutput }
}

// Register P0 is always NOT_BUSY, and cannot be set to BUSY
// Note: I do NOT bypass from newly busied registers to the read ports.
// That bypass check should be done elsewhere (this is to get it off the
// critical path).
class BusyTableHelper(
   pipeline_width:Int,
   num_pregs: Int,
   num_read_ports:Int,
   num_wb_ports:Int)
   (implicit p: Parameters) extends BoomModule()(p)
{
   val io = new BusyTableIo(pipeline_width, num_pregs, num_read_ports, num_wb_ports)

   def BUSY     = Bool(true)
   def NOT_BUSY = Bool(false)

   //TODO BUG chisel3
   val table_bsy = Reg(init=Vec.fill(num_pregs){Bool(false)})

   for (wb_idx <- 0 until num_wb_ports)
   {
      when (io.unbusy_vdst(wb_idx).valid && (io.unbusy_vdst(wb_idx).state =/= UInt(2)))
      {
         table_bsy(io.unbusy_vdst(wb_idx).vdst) := NOT_BUSY
      }
	  .elsewhen (io.unbusy_vdst(wb_idx).valid && (io.unbusy_vdst(wb_idx).state === UInt(2)))
	  {
	     table_bsy(io.unbusy_vdst(wb_idx).vdst) := BUSY
	  }
   }

   for (w <- 0 until pipeline_width)
   {
      when (io.allocated_vdst(w).valid && io.allocated_vdst(w).bits =/= UInt(0))
      {
         table_bsy(io.allocated_vdst(w).bits) := BUSY
      }
   }

   // handle bypassing a clearing of the busy-bit
   for (ridx <- 0 until num_read_ports)
   {
      val just_cleared = io.unbusy_vdst.map(p => p.valid && (p.vdst === io.p_rs(ridx)) && (p.state =/= UInt(2))).reduce(_|_)
	  val just_seted   = io.unbusy_vdst.map(p => p.valid && (p.vdst === io.p_rs(ridx)) && (p.state === UInt(2))).reduce(_|_)
      // note: no bypassing of the newly busied (that is done outside this module)
      io.p_rs_busy(ridx) := (table_bsy(io.p_rs(ridx)) && !just_cleared) || just_seted
   }

   io.debug.busytable := table_bsy.toBits
}


class BusyTableOutput extends Bundle
{
   val prs1_busy = Bool()
   val prs2_busy = Bool()
   val prs3_busy = Bool()
}


class BusyTable(
   pl_width:Int,
   rtype: BigInt,
   num_pregs: Int,
   num_read_ports:Int,
   num_wb_ports:Int)
   (implicit p: Parameters) extends BoomModule()(p)
{
   private val preg_sz = TPREG_SZ

   val io = new Bundle
   {
      // Inputs
      val ren_will_fire         = Vec(pl_width, Bool()).asInput
      val ren_uops              = Vec(pl_width, new MicroOp()).asInput

      val map_table             = Vec(pl_width, new L2VMapTableOutput(preg_sz)).asInput

      val wb_valids             = Vec(num_wb_ports, Bool()).asInput
      val wb_vdsts              = Vec(num_wb_ports, UInt(width=preg_sz)).asInput
      val wb_state              = Vec(num_wb_ports, UInt(2.W)).asInput

      // Outputs
      val values                = Vec(pl_width, new BusyTableOutput()).asOutput

      val debug                 = new Bundle { val busytable= Bits(width=num_pregs).asOutput }
   }

   val busy_table = Module(new BusyTableHelper(
      pipeline_width = pl_width,
      num_pregs = num_pregs,
      num_read_ports = num_read_ports,
      num_wb_ports = num_wb_ports))


   // figure out if we need to bypass a newly allocated physical register from a previous instruction in this cycle.
   val vrs1_was_bypassed = Wire(init = Vec.fill(pl_width) {Bool(false)})
   val vrs2_was_bypassed = Wire(init = Vec.fill(pl_width) {Bool(false)})
   val vrs3_was_bypassed = Wire(init = Vec.fill(pl_width) {Bool(false)})
   for {
      w <- 0 until pl_width
      xx <- w-1 to 0 by -1
   }{
      when (io.ren_uops(w).lrs1_rtype === UInt(rtype) && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs1 === io.ren_uops(xx).ldst))
         { vrs1_was_bypassed(w) := Bool(true) }
      when (io.ren_uops(w).lrs2_rtype === UInt(rtype) && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs2 === io.ren_uops(xx).ldst))
         { vrs2_was_bypassed(w) := Bool(true) }

      if (rtype == RT_FLT.litValue) {
         when (io.ren_uops(w).frs3_en && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs3 === io.ren_uops(xx).ldst))
            { vrs3_was_bypassed(w) := Bool(true) }
      }
   }


   for (w <- 0 until pl_width)
   {
      // Reading the Busy Bits
      // for critical path reasons, we speculatively read out the busy-bits assuming no dependencies between uops
      // then verify if the uop actually uses a register and if it depends on a newly unfreed register
      busy_table.io.prs(0,w) := io.map_table(w).vrs1
      busy_table.io.prs(1,w) := io.map_table(w).vrs2

      io.values(w).prs1_busy := io.ren_uops(w).lrs1_rtype === UInt(rtype) && (busy_table.io.prs2_busy(0,w) || vrs1_was_bypassed(w))
      io.values(w).prs2_busy := io.ren_uops(w).lrs2_rtype === UInt(rtype) && (busy_table.io.prs2_busy(1,w) || vrs2_was_bypassed(w))

      if (rtype == RT_FLT.litValue)
      {
         busy_table.io.prs(2,w) := io.map_table(w).vrs3
         io.values(w).prs3_busy := (io.ren_uops(w).frs3_en) && (busy_table.io.prs2_busy(2,w) || vrs3_was_bypassed(w))
      }
      else
      {
         io.values(w).prs3_busy := Bool(false)
      }


       // Updating the Table (new busy register)
      busy_table.io.allocated_vdst(w).valid := io.ren_will_fire(w) &&
                                               io.ren_uops(w).ldst_val &&
                                               io.ren_uops(w).dst_rtype === UInt(rtype)
      busy_table.io.allocated_vdst(w).bits  := io.ren_uops(w).vdst
   }

   // Clear Busy-bit
   for (i <- 0 until num_wb_ports)
   {
      busy_table.io.unbusy_vdst(i).valid := io.wb_valids(i)
      busy_table.io.unbusy_vdst(i).vdst  := io.wb_vdsts(i)
	  busy_table.io.unbusy_vdst(i).state := io.wb_state(i)
   }

   // scalastyle:on
   io.debug := busy_table.io.debug
}
