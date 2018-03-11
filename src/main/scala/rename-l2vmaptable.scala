//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename Map Table
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom

import Chisel._
import config.Parameters


class RenameL2VMapTableElementIo(pl_width: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val element            = UInt(OUTPUT, VPREG_SZ)

   val wens               = Vec(pl_width, Bool()).asInput
   val ren_vdsts          = Vec(pl_width, UInt(width=VPREG_SZ)).asInput

   val ren_br_vals        = Vec(pl_width, Bool()).asInput
   val ren_br_tags        = Vec(pl_width, UInt(width=BR_TAG_SZ)).asInput

   val br_mispredict      = Bool(INPUT)
   val br_mispredict_tag  = UInt(INPUT, BR_TAG_SZ)

   // rollback (on exceptions)
   // TODO REMOVE THIS ROLLBACK PORT, since wens is mutually exclusive with rollback_wens
   val rollback_wen        = Bool(INPUT)
   val rollback_stale_vdst = UInt(INPUT, VPREG_SZ)

   // TODO scr option
   val flush_pipeline      = Bool(INPUT)
   val commit_wen          = Bool(INPUT)
   val commit_vdst         = UInt(INPUT, VPREG_SZ)
   val committed_element   = UInt(OUTPUT, VPREG_SZ)

   override def cloneType: this.type = new RenameL2VMapTableElementIo(pl_width).asInstanceOf[this.type]
}

class RenameL2VMapTableElement(pipeline_width: Int, always_zero: Boolean)(implicit p: Parameters) extends BoomModule()(p)
{
   val io = new RenameL2VMapTableElementIo(pipeline_width)

   // Note: I don't use a "valid" signal, since it's annoying to deal with and
   // only necessary until the map tables are filled. So instead I reset the
   // map table to all point to P0. I'm not sure which is less expensive.  The
   // corner-case to deal with is the "stale" register needs to be correct and
   // valid, so that freeing it won't free an actual register that was handed
   // out in the meantime. A software solution is also possible, but I'm
   // unwilling to trust that.

   val element = Reg(init = UInt(0, VPREG_SZ))

   // handle branch speculation
   val element_br_copies = Mem(MAX_BR_COUNT, UInt(width = VPREG_SZ))


   // this is possibly the hardest piece of code I have ever had to reason about in my LIFE.
   // Or maybe that's the 5am talking.
   // on every branch, make a copy of the rename map table state
   // if jal/jalr, we want to capture our own setting of this register
   // We need to know the AGE of the branch!
   // 1st, is "wen" (incoming)
   // 2nd, is older instructions in same bundle
   // 3rd, current element

   for (w <- 0 until pipeline_width)
   {
      var elm_cases = Array((Bool(false),  UInt(0,VPREG_SZ)))

      for (xx <- w to 0 by -1)
      {
         elm_cases ++= Array((io.wens(xx),  io.ren_vdsts(xx)))
      }

      when (io.ren_br_vals(w))
      {
         element_br_copies(io.ren_br_tags(w)) := MuxCase(element, elm_cases)
      }
   }


   // reset table on mispredict
   when(io.br_mispredict)
   {
      element := element_br_copies(io.br_mispredict_tag)
   }
   // rollback to the previous mapping
   .elsewhen (io.rollback_wen)
   {
      element := io.rollback_stale_vdst
   }
   // free list is giving us a new vdst
   .elsewhen (io.wens.reduce(_|_))
   {
      // give write priority to the last instruction in the bundle
      element := PriorityMux(io.wens.reverse, io.ren_vdsts.reverse)
   }

   if (ENABLE_COMMIT_MAP_TABLE)
   {
      val committed_element = Reg(init=UInt(0,VPREG_SZ))
      when (io.commit_wen)
      {
         committed_element := io.commit_vdst
      }
      when (io.flush_pipeline)
      {
         element := committed_element
      }
      io.committed_element := committed_element
   }

   // outputs
   io.element := element

   if (always_zero) io.element := UInt(0)
}


// Pass out the new virtual physical register specifiers.
class L2VMapTableOutput(vreg_sz: Int) extends Bundle
{
   val vrs1              = UInt(width = vreg_sz)
   val vrs2              = UInt(width = vreg_sz)
   val vrs3              = UInt(width = vreg_sz)
   val stale_vdst        = UInt(width = vreg_sz)
   override def cloneType: this.type = new L2VMapTableOutput(vreg_sz).asInstanceOf[this.type]
}

class RenameL2VMapTable(
   pl_width: Int,
   rtype: BigInt,
   num_logical_registers: Int,
   num_virtual_registers: Int
   )(implicit p: Parameters) extends BoomModule()(p)
   with HasBoomCoreParameters
{
   private val vreg_sz = log2Up(num_virtual_registers)

   val io = new BoomBundle()(p)
   {
      // Inputs
      val brinfo           = new BrResolutionInfo().asInput
      val kill             = Bool(INPUT)

      val ren_will_fire    = Vec(pl_width, Bool()).asInput
      val ren_uops         = Vec(pl_width, new MicroOp()).asInput
      val ren_br_vals      = Vec(pl_width, Bool()).asInput

      val com_valids       = Vec(pl_width, Bool()).asInput
      val com_uops         = Vec(pl_width, new MicroOp()).asInput
      val com_rbk_valids   = Vec(pl_width, Bool()).asInput
      val flush_pipeline   = Bool(INPUT) // only used for SCR (single-cycle reset)

      val debug_inst_can_proceed = Vec(pl_width, Bool()).asInput
      val debug_freelist_can_allocate = Vec(pl_width, Bool()).asInput

      // Outputs
      val values           = Vec(pl_width, new L2VMapTableOutput(vreg_sz)).asOutput
   }


   val entries = for (i <- 0 until num_logical_registers) yield
   {
      val entry = Module(new RenameL2VMapTableElement(pl_width, always_zero = (i==0 && rtype == RT_FIX.litValue)))
      entry
   }
   val map_table_io = Vec(entries.map(_.io))

   map_table_io.zipWithIndex.map{ case (entry, i) =>
   {
      // TODO get rid of this extra, init logic
      entry.rollback_wen := Bool(false)
      entry.rollback_stale_vdst := io.com_uops(0).stale_vdst
      entry.commit_wen := Bool(false)
      entry.commit_vdst := io.com_uops(0).vdst

      for (w <- 0 until pl_width)
      {
         entry.wens(w)        := io.ren_uops(w).ldst === UInt(i) &&
                                           io.ren_will_fire(w) &&
                                           io.ren_uops(w).ldst_val &&
                                           io.ren_uops(w).dst_rtype === UInt(rtype) &&
                                           !io.kill

         assert (!(entry.wens(w) && !io.debug_inst_can_proceed(w)), "[maptable] wen shouldn't be high.")
         assert (!(entry.wens(w) && !io.debug_freelist_can_allocate(w)), "[maptable] wen shouldn't be high.")

         entry.ren_vdsts(w)   := io.ren_uops(w).vdst
         entry.ren_br_tags(w) := io.ren_uops(w).br_tag
      }
      entry.ren_br_vals := io.ren_br_vals

      entry.br_mispredict     := io.brinfo.mispredict
      entry.br_mispredict_tag := io.brinfo.tag

      entry.flush_pipeline    := io.flush_pipeline
   }}

   // backwards, because rollback must give highest priority to 0 (the oldest instruction)
   for (w <- pl_width-1 to 0 by -1)
   {
      val ldst = io.com_uops(w).ldst
      when (io.com_rbk_valids(w) && io.com_uops(w).dst_rtype === UInt(rtype))
      {
         map_table_io(ldst).rollback_wen        := Bool(true)
         map_table_io(ldst).rollback_stale_vdst := io.com_uops(w).stale_vdst
      }
   }

   if (ENABLE_COMMIT_MAP_TABLE)
   {
      for (w <- 0 until pl_width)
      {
         val ldst = io.com_uops(w).ldst
         when (io.com_valids(w) && (io.com_uops(w).dst_rtype === UInt(rtype)))
         {
            map_table_io(ldst).commit_wen := Bool(true)
            map_table_io(ldst).commit_vdst := io.com_uops(w).vdst
         }
      }
   }

   // Read out the map-table entries ASAP, then deal with bypassing busy-bits later.
   private val map_table_output = Seq.fill(pl_width*3)(Wire(UInt(width=VPREG_SZ)))
   def map_table_vrs1(w:Int) = map_table_output(w+0*pl_width)
   def map_table_vrs2(w:Int) = map_table_output(w+1*pl_width)
   def map_table_vrs3(w:Int) = map_table_output(w+2*pl_width)

   for (w <- 0 until pl_width)
   {
      map_table_vrs1(w) := map_table_io(io.ren_uops(w).lrs1).element
      map_table_vrs2(w) := map_table_io(io.ren_uops(w).lrs2).element
      if (rtype == RT_FLT.litValue) {
         map_table_vrs3(w) := map_table_io(io.ren_uops(w).lrs3).element
      } else {
         map_table_vrs3(w) := UInt(0)
      }
   }


   // Bypass the virtual physical register mappings
   for (w <- 0 until pl_width)
   {
      var rs1_cases =  Array((Bool(false),  UInt(0,VPREG_SZ)))
      var rs2_cases =  Array((Bool(false),  UInt(0,VPREG_SZ)))
      var rs3_cases =  Array((Bool(false),  UInt(0,VPREG_SZ)))
      var stale_cases= Array((Bool(false),  UInt(0,VPREG_SZ)))

      // Handle bypassing new virtual physical destinations to operands (and stale destination)
      // scalastyle:off
      for (xx <- w-1 to 0 by -1)
      {
         rs1_cases  ++= Array((io.ren_uops(w).lrs1_rtype === UInt(rtype) && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs1 === io.ren_uops(xx).ldst), (io.ren_uops(xx).vdst)))
         rs2_cases  ++= Array((io.ren_uops(w).lrs2_rtype === UInt(rtype) && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs2 === io.ren_uops(xx).ldst), (io.ren_uops(xx).vdst)))
         stale_cases++= Array((io.ren_uops(w).dst_rtype === UInt(rtype)  && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).ldst === io.ren_uops(xx).ldst), (io.ren_uops(xx).vdst)))

         if (rtype == RT_FLT.litValue) {
            rs3_cases  ++= Array((
                  io.ren_uops(w).frs3_en && io.ren_will_fire(xx) && io.ren_uops(xx).ldst_val && io.ren_uops(xx).dst_rtype === UInt(rtype) && (io.ren_uops(w).lrs3 === io.ren_uops(xx).ldst),
                  (io.ren_uops(xx).vdst)))
         }
      }

      // add default case where we can just read the map table for our information
      if (rtype == RT_FIX.litValue) {
         rs1_cases ++= Array((io.ren_uops(w).lrs1_rtype === UInt(rtype) && (io.ren_uops(w).lrs1 =/= UInt(0)), map_table_vrs1(w)))
         rs2_cases ++= Array((io.ren_uops(w).lrs2_rtype === UInt(rtype) && (io.ren_uops(w).lrs2 =/= UInt(0)), map_table_vrs2(w)))
      } else {
         rs1_cases ++= Array((io.ren_uops(w).lrs1_rtype === UInt(rtype), map_table_vrs1(w)))
         rs2_cases ++= Array((io.ren_uops(w).lrs2_rtype === UInt(rtype), map_table_vrs2(w)))
      }
      rs3_cases ++= Array((io.ren_uops(w).frs3_en, map_table_vrs3(w)))

      // Set outputs.
      io.values(w).vrs1       := MuxCase(io.ren_uops(w).lrs1, rs1_cases)
      io.values(w).vrs2       := MuxCase(io.ren_uops(w).lrs2, rs2_cases)
      if (rtype == RT_FLT.litValue)
         {io.values(w).vrs3 := MuxCase(io.ren_uops(w).lrs3, rs3_cases)}
      io.values(w).stale_vdst := MuxCase(map_table_io(io.ren_uops(w).ldst).element, stale_cases)


      if (rtype == RT_FIX.litValue) {
         assert (!(io.ren_uops(w).lrs1 === UInt(0) && io.ren_uops(w).lrs1_rtype === RT_FIX && io.values(w).vrs1 =/= UInt(0)), "lrs1==0 but maptable(" + w + ") returning non-zero.")
         assert (!(io.ren_uops(w).lrs2 === UInt(0) && io.ren_uops(w).lrs2_rtype === RT_FIX && io.values(w).vrs2 =/= UInt(0)), "lrs2==0 but maptable(" + w + ") returning non-zero.")
         assert (!(io.ren_uops(w).lrs1 === UInt(0) && io.ren_uops(w).lrs1_rtype === RT_FIX && map_table_vrs1(w) =/= UInt(0)), "lrs1==0 but maptable(" + w + ") returning non-zero.")
         assert (!(io.ren_uops(w).lrs2 === UInt(0) && io.ren_uops(w).lrs2_rtype === RT_FIX && map_table_vrs2(w) =/= UInt(0)), "lrs2==0 but maptable(" + w + ") returning non-zero.")
      }
      // scalastyle:on
   }
}

