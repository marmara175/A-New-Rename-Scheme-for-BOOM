//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Slot Logic
//--------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Note: stores (and AMOs) are "broken down" into 2 uops, but stored within a single issue-slot.
// TODO XXX make a separate issueSlot for MemoryIssueSlots, and only they break apart stores.

package boom

import Chisel._
import FUConstants._
import config.Parameters

class IssueSlotIO(num_wakeup_ports: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val valid          = Bool(OUTPUT)
   val will_be_valid  = Bool(OUTPUT) // TODO code review, do we need this signal so explicitely?
   val request        = Bool(OUTPUT)
   val request_hp     = Bool(OUTPUT)
   val grant          = Bool(INPUT)

   val brinfo         = new BrResolutionInfo().asInput
   val kill           = Bool(INPUT) // pipeline flush
   val clear          = Bool(INPUT) // entry being moved elsewhere (not mutually exclusive with grant)

   val wakeup_vdsts   = Vec(num_wakeup_ports, Valid(UInt(width = TPREG_SZ))).flip
   val wakeup_pdsts   = Vec(num_wakeup_ports, UInt(width = TPREG_SZ)).asInput
   val wakeup_masks   = Vec(num_wakeup_ports, UInt(width = numIntPhysRegsParts)).asInput
   val wakeup_rb_state= Vec(num_wakeup_ports, UInt(2.W)).asInput

   val across_rb_val  = Bool().asInput
   val across_rb_state= UInt(2.W).asInput
   val across_rb_vdst = UInt(width = TPREG_SZ).asInput

   val in_uop         = Valid(new MicroOp()).flip // if valid, this WILL overwrite an entry!
   val updated_uop    = new MicroOp().asOutput // the updated slot uop; will be shifted upwards in a collasping queue.
   val uop            = new MicroOp().asOutput // the current Slot's uop. Sent down the pipeline when issued.

   val debug = {
     val result = new Bundle {
       val p1 = Bool()
       val p2 = Bool()
       val p3 = Bool()
	   val state = UInt(3.W)
    }
    result.asOutput
  }

   override def cloneType = new IssueSlotIO(num_wakeup_ports)(p).asInstanceOf[this.type]
}

class IssueSlot(num_slow_wakeup_ports: Int, iqType: BigInt)(implicit p: Parameters) extends BoomModule()(p)
{
   val io = new IssueSlotIO(num_slow_wakeup_ports)

   // slot invalid?
   // slot is valid, holding 1 uop
   // slot is valid, holds 2 uops (like a store)
   def isInvalid = slot_state === s_invalid
   def isValid = slot_state =/= s_invalid

   val updated_state = Wire(UInt()) // the next state of this slot (which might then get moved to a new slot)
   val updated_uopc  = Wire(Bits()) // the next uopc of this slot (which might then get moved to a new slot)
   val updated_lrs1_rtype = Wire(Bits()) // the next reg type of this slot (which might then get moved to a new slot)
   val updated_lrs2_rtype = Wire(Bits()) // the next reg type of this slot (which might then get moved to a new slot)
   val next_p1  = Wire(Bool())
   val next_p2  = Wire(Bool())
   val next_p3  = Wire(Bool())

   val slot_state    = Reg(init = s_invalid)
   val slot_p1       = Reg(init = Bool(false), next = next_p1)
   val slot_p2       = Reg(init = Bool(false), next = next_p2)
   val slot_p3       = Reg(init = Bool(false), next = next_p3)
   val slot_is_2uops = Reg(Bool())

   val slotUop = Reg(init = NullMicroOp)


   //-----------------------------------------------------------------------------
   // next slot state computation
   // compute the next state for THIS entry slot (in a collasping queue, the
   // current uop may get moved elsewhere, and a new slot can get moved into
   // here

   when (io.kill)
   {
      slot_state := s_invalid
   }
   .elsewhen (io.in_uop.valid)
   {
      slot_state := io.in_uop.bits.iw_state
   }
   .elsewhen (io.clear)
   {
      slot_state := s_invalid
   }
   .otherwise
   {
      slot_state := updated_state
   }

   //-----------------------------------------------------------------------------
   // "update" state
   // compute the next state for the micro-op in this slot. This micro-op may
   // be moved elsewhere, so the "updated_state" travels with it.

   // defaults
   updated_state := slot_state
   updated_uopc := slotUop.uopc
   updated_lrs1_rtype := slotUop.lrs1_rtype
   updated_lrs2_rtype := slotUop.lrs2_rtype

   val has_dst   = slotUop.ldst_val
   val cond_fail = Wire(Bool())//异常slow唤醒发生
   val cond_suc  = Wire(Bool())//正常slow唤醒发生
   cond_fail     := Bool(false)
   cond_suc      := Bool(false)

   when (io.kill ||
        (io.grant && (slot_state === s_valid_1) && !has_dst) ||
        (io.grant && (slot_state === s_valid_2) && slot_p1 && slot_p2 && !has_dst) ||
		(slot_state === s_pending) && cond_suc)
   {
      updated_state := s_invalid
   }
   .elsewhen ((io.grant && (slot_state === s_valid_1) && has_dst) ||
              (io.grant && (slot_state === s_valid_2) && has_dst))
   {
      updated_state := s_pending
   }
   .elsewhen ((slot_state === s_pending) && cond_fail)
   {
      when (slotUop.uopc === uopAMO_AG)
	  {
	     updated_state := s_valid_2
	  }
	  .otherwise
	  {
	     updated_state := s_valid_1
	  }
   }
   .elsewhen (io.grant && (slot_state === s_valid_2))
   {
      updated_state := s_valid_1
      when (slot_p1)
      {
         slotUop.uopc := uopSTD
         updated_uopc := uopSTD
         slotUop.lrs1_rtype := RT_X
         updated_lrs1_rtype := RT_X

      }
      .otherwise
      {
         slotUop.lrs2_rtype := RT_X
         updated_lrs2_rtype := RT_X
      }
   }

   when (io.in_uop.valid)
   {
      slotUop := io.in_uop.bits
      assert (isInvalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
   }

   // Wakeup Compare Logic
   next_p1 := Bool(false)
   next_p2 := Bool(false)
   next_p3 := Bool(false)

   // these signals are the "next_p*" for the current slot's micro-op.
   // they are important for shifting the current slotUop up to an other entry.
   // TODO need a better name for these signals
   val out_p1 = Wire(Bool()); out_p1 := slot_p1
   val out_p2 = Wire(Bool()); out_p2 := slot_p2
   val out_p3 = Wire(Bool()); out_p3 := slot_p3

   // yqh debug

   val updated_mask1	= Wire(UInt())
   val updated_mask2 	= Wire(UInt())
   val updated_mask3    = Wire(UInt())
   val updated_pop1	    = Wire(UInt())
   val updated_pop2	    = Wire(UInt())
   val updated_pop3	    = Wire(UInt())

   updated_mask1 := slotUop.rs1_mask 
   updated_mask2 := slotUop.rs2_mask
   updated_mask3 := slotUop.rs3_mask
   updated_pop1  := slotUop.pop1
   updated_pop2  := slotUop.pop2
   updated_pop3  := slotUop.pop3

   when (io.in_uop.valid)
   {
      next_p1 := !(io.in_uop.bits.prs_busy(0))
	  next_p2 := !(io.in_uop.bits.prs_busy(1))
	  next_p3 := !(io.in_uop.bits.prs_busy(2))

	  // yqh debug2
      slotUop.pdst := UInt(0)
      slotUop.dst_mask := Bits(0, width = numIntPhysRegsParts)
   }
   .otherwise
   {
      next_p1 := out_p1
      next_p2 := out_p2
      next_p3 := out_p3
      slotUop.rs1_mask := updated_mask1
      slotUop.rs2_mask := updated_mask2
      slotUop.rs3_mask := updated_mask3
      slotUop.pop1 := updated_pop1
      slotUop.pop2 := updated_pop2
      slotUop.pop3 := updated_pop3
   }

   val cond_ = Wire(Bool())
   cond_ := (iqType.asUInt === IQT_MEM) && (slotUop.dst_rtype === RT_FLT)

   when ((slotUop.fu_code === FU_F2I || slotUop.fu_code === FU_I2F || cond_) &&
          io.across_rb_val && (slotUop.vdst === io.across_rb_vdst))
   {
      when (io.across_rb_state === UInt(1))
	  {
	     cond_suc  := Bool(true)
	  }
	  .elsewhen (io.across_rb_state === UInt(2))
	  {
	     cond_fail  := Bool(true)
	  }
   }

   for (i <- 0 until num_slow_wakeup_ports)
   {
    //  printf ("cond = %d, io.wakeup_vdsts(i).valid = %d, io.wakeup_vdsts(i).bits = %d, io.wakeup_rb_state(i) = %d\n",
	  //        ((slotUop.fu_code =/= FU_F2I && slotUop.fu_code =/= FU_I2F && !cond_) && io.wakeup_vdsts(i).valid && io.wakeup_vdsts(i).bits === slotUop.vdst).asUInt,
		//	  io.wakeup_vdsts(i).valid,
		//	  io.wakeup_vdsts(i).bits,
		//	  io.wakeup_rb_state(i))

      when ((slotUop.fu_code =/= FU_F2I && slotUop.fu_code =/= FU_I2F && !cond_) &&
	         io.wakeup_vdsts(i).valid && io.wakeup_vdsts(i).bits === slotUop.vdst)
	  {
	     when (io.wakeup_rb_state(i) === UInt(1))
		 {
		    cond_suc  := Bool(true)
		 }
		 .elsewhen (io.wakeup_rb_state(i) === UInt(2))
		 {
		    cond_fail := Bool(true)
		 }
		// printf ("io.wakeup_vdsts(i).bits = %d, io.wakeup_rb_state(i) = %d, cond_fail = %d\n",
		//          io.wakeup_vdsts(i).bits,
		//		  io.wakeup_rb_state(i),
		//		  cond_fail)
	  }

      when (io.wakeup_vdsts(i).valid && io.wakeup_vdsts(i).bits === slotUop.vop1)
      {
	     when (io.wakeup_rb_state(i) =/= UInt(2))
		 {
		    out_p1        := Bool(true)
			updated_mask1 := io.wakeup_masks(i)
			updated_pop1  := io.wakeup_pdsts(i)
		 }
		 .elsewhen (slotUop.lrs1_rtype =/= RT_X)
		 {
		    out_p1        := Bool(false) 
		 }
      }

      when (io.wakeup_vdsts(i).valid && io.wakeup_vdsts(i).bits === slotUop.vop2)
      {
	     when (io.wakeup_rb_state(i) =/= UInt(2))
		 {
		    out_p2        := Bool(true)
			updated_mask2 := io.wakeup_masks(i)
			updated_pop2  := io.wakeup_pdsts(i)
		 }
		 .elsewhen (slotUop.lrs2_rtype =/= RT_X)
		 {
		    out_p2        := Bool(false)
		 }
      }

      when (io.wakeup_vdsts(i).valid && io.wakeup_vdsts(i).bits === slotUop.vop3)
      {
	     when (io.wakeup_rb_state(i) =/= UInt(2))
		 {
		    out_p3        := Bool(true)
			updated_mask3 := io.wakeup_masks(i)
			updated_pop3  := io.wakeup_pdsts(i)
		 }
		 .elsewhen (slotUop.frs3_en)
		 {
		    out_p3        := Bool(false)
		 }
      }
   }

   // Handle branch misspeculations
   val out_br_mask = GetNewBrMask(io.brinfo, slotUop)

   // was this micro-op killed by a branch? if yes, we can't let it be valid if
   // we compact it into an other entry
   when (IsKilledByBranch(io.brinfo, slotUop))
   {
      updated_state := s_invalid
   }

   when (!io.in_uop.valid)
   {
      slotUop.br_mask := out_br_mask
   }


   //-------------------------------------------------------------
   // Request Logic
   io.request := ((slot_state === s_valid_1) || (slot_state === s_valid_2)) && slot_p1 && slot_p2 && slot_p3 && !io.kill
   val high_priority = slotUop.is_br_or_jmp
//   io.request_hp := io.request && high_priority
   io.request_hp := Bool(false)

   when (slot_state === s_valid_1)
   {
      io.request := slot_p1 && slot_p2 && slot_p3 && !io.kill
   }
   .elsewhen (slot_state === s_valid_2)
   {
      io.request := (slot_p1 || slot_p2)  && !io.kill
   }
   .otherwise
   {
      io.request := Bool(false)
   }


   //assign outputs
   io.valid         := isValid
   io.uop           := slotUop
   io.will_be_valid := isValid && !((slot_state === s_pending) && cond_suc ||
					     io.grant && !has_dst && ((slot_state === s_valid_1) || (slot_state === s_valid_2) && slot_p1 && slot_p2))

   io.updated_uop           := slotUop
   io.updated_uop.iw_state  := updated_state
   io.updated_uop.uopc      := updated_uopc
   io.updated_uop.lrs1_rtype:= updated_lrs1_rtype
   io.updated_uop.lrs2_rtype:= updated_lrs2_rtype
   io.updated_uop.br_mask   := out_br_mask
   // yqh
   io.updated_uop.prs_busy := ((~out_p3) << 2) | ((~out_p2) << 1) | (~out_p1)
   // yqh
   io.updated_uop.rs1_mask  := updated_mask1
   io.updated_uop.rs2_mask  := updated_mask2
   io.updated_uop.rs3_mask  := updated_mask3
   // yqh todo
   io.updated_uop.pop1      := updated_pop1
   io.updated_uop.pop2      := updated_pop2
   io.updated_uop.pop3      := updated_pop3

   when (slot_state === s_valid_2)
   {
      when (slot_p1 && slot_p2)
      {
         ; // send out the entire instruction as one uop
      }
      .elsewhen (slot_p1)
      {
         io.uop.uopc := slotUop.uopc
         io.uop.lrs2_rtype := RT_X
      }
      .elsewhen (slot_p2)
      {
         io.uop.uopc := uopSTD
         io.uop.lrs1_rtype := RT_X
      }
   }

   // debug outputs
   io.debug.p1 := slot_p1
   io.debug.p2 := slot_p2
   io.debug.p3 := slot_p3
   io.debug.state := slot_state
}

