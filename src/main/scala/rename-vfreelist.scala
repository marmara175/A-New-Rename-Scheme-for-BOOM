//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename VFreeList
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom

import Chisel._
import config.Parameters


class VFreeListIo(num_virtual_registers: Int, pl_width: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   private val vreg_sz = log2Up(num_virtual_registers)

   val req_vreg_vals = Vec(pl_width, Bool()).asInput
   val req_vregs     = Vec(pl_width, UInt(width=vreg_sz)).asOutput

   // committed and newly freed register
   val enq_vals      = Vec(pl_width, Bool()).asInput
   val enq_vregs     = Vec(pl_width, UInt(width=vreg_sz)).asInput

   // do we have space to service incoming requests? (per inst granularity)
   val can_allocate  = Vec(pl_width, Bool()).asOutput

   // handle branches (save copy of vfreelist on branch, merge on mispredict)
   val ren_br_vals   = Vec(pl_width, Bool()).asInput
   val ren_br_tags   = Vec(pl_width, UInt(width=BR_TAG_SZ)).asInput

   // handle mispredicts
   val br_mispredict_val = Bool(INPUT)
   val br_mispredict_tag = UInt(INPUT, BR_TAG_SZ)

   // rollback (on exceptions)
   val rollback_wens  = Vec(pl_width, Bool()).asInput
   val rollback_vdsts = Vec(pl_width, UInt(width=vreg_sz)).asInput

   // or...
   // TODO there are TWO free-list IOs now, based on constants. What is the best way to handle these two designs?
   // perhaps vfreelist.scala, and instantiate which-ever one I want?
   // TODO naming is inconsistent
   // TODO combine with rollback, whatever?
   val flush_pipeline = Bool(INPUT)
   val com_wens       = Vec(pl_width, Bool()).asInput
   val com_uops       = Vec(pl_width, new MicroOp()).asInput

   val debug = new DebugVFreeListIO(num_virtual_registers).asOutput
}

class DebugVFreeListIO(num_virtual_registers: Int) extends Bundle
{
   val vfreelist = Bits(width=num_virtual_registers)
   val isprlist = Bits(width=num_virtual_registers)
   override def cloneType: this.type = new DebugVFreeListIO(num_virtual_registers).asInstanceOf[this.type]
}

// provide a fixed set of renamed destination registers
// i.e., it doesn't matter if a previous UOP needs a vdst or not
// this prevents a dependency chain from existing between UOPs when trying to
// compute a vdst to give away (as well as computing if an available free
// register exists.
// NOTE: we never give out p0 -- that is the "unitialized" state of the map-table,
// and the pipeline will give any reader of p0 0x0 as read data.
class RenameVFreeListHelper(
   num_virtual_registers: Int, // number of physical registers
   pl_width: Int)           // pipeline width ("dispatch group size")
   (implicit p: Parameters) extends BoomModule()(p)
{
   val io = new VFreeListIo(num_virtual_registers, pl_width)

   // ** FREE LIST TABLE ** //
   val free_list = Reg(init = ~"b11111111111111111111111111111111".U(num_virtual_registers.W))

   //printf("vfreelist = b%b\n", free_list)

   // track all allocations that have occurred since branch passed by
   // can quickly reset pipeline on branch mispredict
   val allocation_lists = Reg(Vec(MAX_BR_COUNT, Bits(width = num_virtual_registers)))

   // TODO why is this a Vec? can I do this all on one bit-vector?
   val enq_mask = Wire(Vec(pl_width, Bits(width = num_virtual_registers)))

   // ------------------------------------------
   // find new,free physical registers

   val requested_vregs_oh_array = Array.fill(pl_width,num_virtual_registers){Bool(false)}
   val requested_vregs_oh       = Wire(Vec(pl_width, Bits(width=num_virtual_registers)))
   val requested_vregs          = Wire(Vec(pl_width, UInt(width=log2Up(num_virtual_registers))))
   var allocated                = Wire(Vec(pl_width, Bool())) // did each inst get allocated a register?

   // init
   for (w <- 0 until pl_width)
   {
      allocated(w) := Bool(false)
   }


   // don't give out p0
   for (i <- 1 until num_virtual_registers)
   {
      val next_allocated = Wire(Vec(pl_width, Bool()))
      var can_allocate = free_list(i)

      for (w <- 0 until pl_width)
      {
         requested_vregs_oh_array(w)(i) = can_allocate && !allocated(w)

         next_allocated(w) := can_allocate | allocated(w)
         can_allocate = can_allocate && allocated(w)
      }

      allocated = next_allocated
   }

   for (w <- 0 until pl_width)
   {
      requested_vregs_oh(w) := Vec(requested_vregs_oh_array(w)).toBits
      requested_vregs(w) := PriorityEncoder(requested_vregs_oh(w))
   }


   // ------------------------------------------
   // Calculate next Free List
   val req_free_list = Wire(Bits(width = num_virtual_registers))
   val enq_free_list = Wire(Bits(width = num_virtual_registers))
   req_free_list := free_list
   enq_free_list := free_list

   // ** Set requested PREG to "Not Free" ** //

   // bit vector of newly allocated physical registers
   var just_allocated_mask = Bits(0, num_virtual_registers)

   // track which allocation_lists just got cleared out by a branch,
   // to enforce a write priority to allocation_lists()
   val br_cleared = Wire(Vec(MAX_BR_COUNT, Bool()))
   for (i <- 0 until MAX_BR_COUNT) { br_cleared(i) := Bool(false) }

   for (w <- pl_width-1 to 0 by -1)
   {
      // When branching, start a fresh copy of the allocation_list
      // but don't forget to bypass in the allocations within our bundle
      when (io.ren_br_vals(w))
      {
         allocation_lists(io.ren_br_tags(w)) := just_allocated_mask
         br_cleared(io.ren_br_tags(w)) := Bool(true)
      }

      // check that we both request a register and was able to allocate a register
      just_allocated_mask = Mux(io.req_vreg_vals(w) && allocated(w), requested_vregs_oh(w) | just_allocated_mask,
                                                                     just_allocated_mask)
   }

   for (i <- 0 until MAX_BR_COUNT)
   {
      when (!br_cleared(i))
      {
         allocation_lists(i) := allocation_lists(i) | just_allocated_mask
      }
   }


   // ** Set enqueued PREG to "Free" ** //
   for (w <- 0 until pl_width)
   {
      enq_mask(w) := Bits(0,num_virtual_registers)
      when (io.enq_vals(w))
      {
         enq_mask(w) := UInt(1) << io.enq_vregs(w)
      }
      .elsewhen (io.rollback_wens(w))
      {
         enq_mask(w) := UInt(1) << io.rollback_vdsts(w)
      }
   }


   // Update the Free List
   when (!io.br_mispredict_val)
   {
      free_list := (free_list & ~(just_allocated_mask)) | (enq_mask.reduce(_|_))
   }

   // Handle Misprediction
   //merge misspeculated allocation_list with free_list
   val allocation_list = Wire(Bits(width = num_virtual_registers))
   allocation_list := allocation_lists(io.br_mispredict_tag)

   when (io.br_mispredict_val)
   {
      // include newly freed register as well!
      free_list := allocation_list | free_list | (enq_mask.reduce(_|_))

      // set other branch allocation_lists to zero where allocation_list(j) == 1...
      for (i <- 0 until MAX_BR_COUNT)
      {
         allocation_lists(i) := allocation_lists(i) & ~allocation_list
      }
   }


   // OPTIONALLY: handle single-cycle resets
   // Committed Free List tracks what the free list is at the commit point,
   // allowing for a single-cycle reset of the rename state on a pipeline flush.
   if (ENABLE_COMMIT_MAP_TABLE)
   {
      val committed_free_list = Reg(init=(~Bits(1,num_virtual_registers)))

      val com_mask = Wire(Vec(pl_width, Bits(width=num_virtual_registers)))
      val stale_mask = Wire(Vec(pl_width, Bits(width=num_virtual_registers)))
      for (w <- 0 until pl_width)
      {
         com_mask(w) := Bits(0,width=num_virtual_registers)
         stale_mask(w) := Bits(0,width=num_virtual_registers)
         when (io.com_wens(w))
         {
            com_mask(w) := UInt(1) << io.com_uops(w).vdst
            stale_mask(w) := UInt(1) << io.com_uops(w).stale_vdst
         }
      }

      committed_free_list := (committed_free_list & ~(com_mask.reduce(_|_))) | stale_mask.reduce(_|_)

      when (io.flush_pipeline)
      {
         free_list := committed_free_list
      }
      io.debug.isprlist := committed_free_list
   }



   // ** SET OUTPUTS ** //
   io.req_vregs := requested_vregs

   io.can_allocate := allocated

   io.debug.vfreelist := free_list
}

class RenameVFreeList(
   pl_width: Int,           // Pipeline width ("dispatch group size").
   rtype: BigInt,           // What type of register are we in charge of?
   num_virtual_registers: Int) // Number of physical registers.
   (implicit p: Parameters) extends BoomModule()(p)
{
   private val vreg_sz = log2Up(num_virtual_registers)

   val io = new Bundle
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


      val flush_pipeline   = Bool(INPUT)

      // Outputs
      val can_allocate     = Vec(pl_width, Bool()).asOutput
      val req_vregs        = Vec(pl_width, UInt(width=vreg_sz)).asOutput

      val debug            = new DebugVFreeListIO(num_virtual_registers).asOutput
      val debug_rob_empty  = Bool(INPUT)
   }

   val vfreelist = Module(new RenameVFreeListHelper(
      num_virtual_registers,
      pl_width))

   vfreelist.io.br_mispredict_val := io.brinfo.mispredict
   vfreelist.io.br_mispredict_tag := io.brinfo.tag
   vfreelist.io.flush_pipeline    := io.flush_pipeline

   for (w <- 0 until pl_width)
   {
      vfreelist.io.req_vreg_vals(w)  := !io.kill &&
                                       io.ren_will_fire(w) &&
                                       io.ren_uops(w).ldst_val &&
                                       io.ren_uops(w).dst_rtype === UInt(rtype)

      vfreelist.io.enq_vals(w)       := io.com_valids(w) &&
                                       io.com_uops(w).dst_rtype === UInt(rtype) &&
                                       (io.com_uops(w).stale_vdst =/= UInt(0) || UInt(rtype) === RT_FLT)
      vfreelist.io.enq_vregs(w)      := io.com_uops(w).stale_vdst

      vfreelist.io.ren_br_vals(w)    := io.ren_br_vals(w)
      vfreelist.io.ren_br_tags(w)    := io.ren_uops(w).br_tag


      vfreelist.io.rollback_wens(w)  := io.com_rbk_valids(w) &&
                                       (io.com_uops(w).vdst =/= UInt(0) || UInt(rtype) === RT_FLT) &&
                                       io.com_uops(w).dst_rtype === UInt(rtype)
      vfreelist.io.rollback_vdsts(w) := io.com_uops(w).vdst

      vfreelist.io.com_wens(w)       := io.com_valids(w) &&
                                       (io.com_uops(w).vdst =/= UInt(0) || UInt(rtype) === RT_FLT) &&
                                       io.com_uops(w).dst_rtype === UInt(rtype)
      vfreelist.io.com_uops(w)       := io.com_uops(w)

      if (rtype == RT_FIX.litValue) {
         // x0 is a special-case and should not be renamed
         io.req_vregs(w) := Mux(io.ren_uops(w).ldst === UInt(0), UInt(0), vfreelist.io.req_vregs(w))
      } else {
         io.req_vregs(w) := vfreelist.io.req_vregs(w)
      }
   }

   io.can_allocate := vfreelist.io.can_allocate
   io.debug := vfreelist.io.debug

   when (io.debug_rob_empty) {
      assert (PopCount(vfreelist.io.debug.vfreelist) >= UInt(num_virtual_registers - 32),
         "[vfreelist] We're leaking physical registers")
   }
}
