//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Datapath: Rename Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Christopher Celio
// 2012 Feb 14
//
// Supports 1-cycle and 2-cycle latencies. (aka, passthrough versus registers between ren1 and ren2).
//    - ren1: read the map tables and allocate a new physical register from the freelist.
//    - ren2: read the busy table for the physical operands.
//
// Ren1 data is provided as an output to be fed directly into the ROB.


package boom

import Chisel._
import config.Parameters


class RenameStageIO(
   pl_width: Int,
   num_int_pregs: Int,
   num_fp_pregs: Int,
   num_int_wb_ports: Int,
   num_fp_wb_ports: Int)
   (implicit p: Parameters) extends BoomBundle()(p)
{
   private val int_preg_sz = log2Up(num_int_pregs)
   private val fp_preg_sz = log2Up(num_fp_pregs)

   val inst_can_proceed = Vec(pl_width, Bool()).asOutput

   val kill      = Bool(INPUT)

   val dec_will_fire = Vec(pl_width, Bool()).asInput // will commit state updates
   val dec_uops  = Vec(pl_width, new MicroOp()).asInput

   // physical specifiers now available (but not the busy/ready status of the operands).
   val ren1_mask = Vec(pl_width, Bool().asOutput) // mask of valid instructions
   val ren1_uops = Vec(pl_width, new MicroOp().asOutput)

   // physical specifiers available AND busy/ready status available.
   val ren2_mask  = Vec(pl_width, Bool().asOutput) // mask of valid instructions
   val ren2_uops  = Vec(pl_width, new MicroOp().asOutput)

   val ren_pred_info = Vec(pl_width, new BranchPredInfo()).asInput

   // branch resolution (execute)
   val brinfo    = new BrResolutionInfo().asInput
   val get_pred  = new GetPredictionInfo().flip

   val dis_inst_can_proceed = Vec(DISPATCH_WIDTH, Bool()).asInput

   // issue stage (fast wakeup)
   val int_wakeups = Vec(num_int_wb_ports, Valid(new ExeUnitResp(xLen))).flip
   val fp_wakeups = Vec(num_fp_wb_ports, Valid(new ExeUnitResp(fLen+1))).flip

   // commit stage
   val com_valids = Vec(pl_width, Bool()).asInput
   val com_uops   = Vec(pl_width, new MicroOp()).asInput
   val com_rbk_valids = Vec(pl_width, Bool()).asInput

   val flush_pipeline = Bool(INPUT) // only used for SCR (single-cycle reset)

   val debug_rob_empty = Bool(INPUT)
   val debug = new DebugRenameStageIO(num_int_pregs, num_fp_pregs).asOutput
}


class DebugRenameStageIO(int_num_pregs: Int, fp_num_pregs: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val i_vfreelist = Bits(width=int_num_pregs)
   val iisprlist = Bits(width=int_num_pregs)
   val ibusytable = UInt(width=int_num_pregs)
   val f_vfreelist = Bits(width=fp_num_pregs)
   val fisprlist = Bits(width=fp_num_pregs)
   val fbusytable = UInt(width=fp_num_pregs)
   override def cloneType: this.type = new DebugRenameStageIO(int_num_pregs, fp_num_pregs).asInstanceOf[this.type]
}


class RenameStage(
   pl_width: Int,
   num_int_wb_ports: Int,
   num_fp_wb_ports: Int)
(implicit p: Parameters) extends BoomModule()(p)
{
   val io = new RenameStageIO(pl_width, numIntVPhysRegs, numFpVPhysRegs, num_int_wb_ports, num_fp_wb_ports)

   // integer registers
   val i_l2v_maptable = Module(new RenameL2VMapTable(
      pl_width,
      RT_FIX.litValue,
      32,
      numIntVPhysRegs))
   val i_vfreelist = Module(new RenameVFreeList(
      pl_width,
      RT_FIX.litValue,
      numIntVPhysRegs))
   val i_v2p_maptable = Module(new RenameV2PMapTable(
       pl_width,
       RT_FIX.litValue,
       numIntVPhysRegs,
       numIntPPhysRegs,
       pl_width*2,
       num_wb_ports = pl_width))
   val i_pfreelist = Module(new RenamePFreeList(
       num_write_ports = pl_width,
	   pl_width,
	   RT_FIX.litValue,
	   numIntPPhysRegs))
   val ibusytable = Module(new BusyTable(
      pl_width,
      RT_FIX.litValue,
      num_pregs = numIntVPhysRegs,
      num_read_ports = pl_width*2,
      num_wb_ports = num_int_wb_ports))
   // floating point registers
   val f_l2v_maptable = Module(new RenameL2VMapTable(
      pl_width,
      RT_FLT.litValue,
      32,
      numFpVPhysRegs))
   val f_vfreelist = Module(new RenameVFreeList(
      pl_width,
      RT_FLT.litValue,
      numFpVPhysRegs))
   val fbusytable = Module(new BusyTable(
      pl_width,
      RT_FLT.litValue,
      num_pregs = numFpVPhysRegs,
      num_read_ports = pl_width*3,
      num_wb_ports = num_fp_wb_ports))

   //-------------------------------------------------------------
   // Pipeline State & Wires

   val ren1_br_vals   = Wire(Vec(pl_width, Bool()))
   val ren1_will_fire = Wire(Vec(pl_width, Bool()))
   val ren1_uops      = Wire(Vec(pl_width, new MicroOp()))

   val ren2_valids    = Wire(Vec(pl_width, Bool()))
   val ren2_uops      = Wire(Vec(pl_width, new MicroOp()))

   for (w <- 0 until pl_width)
   {
      // TODO silly, we've already verified this beforehand on the inst_can_proceed
      ren1_will_fire(w) := io.dec_will_fire(w) && io.inst_can_proceed(w) && !io.kill
      ren1_uops(w)      := GetNewUopAndBrMask(io.dec_uops(w), io.brinfo)
      ren1_br_vals(w)   := io.dec_will_fire(w) && io.dec_uops(w).allocate_brtag
   }

   //-------------------------------------------------------------
   // Branch Predictor Snapshots

   // Each branch prediction must snapshot the predictor (history state, etc.).
   // On a mispredict, the snapshot must be used to reset the predictor.
   // TODO use Mem(), but it chokes on the undefines in VCS
   // val prediction_copies = Reg(Vec(MAX_BR_COUNT, new BranchPredictionResp))
   // This info is sent to the BRU and deallocated after Execute.
   val prediction_copies = Reg(Vec(MAX_BR_COUNT, new BranchPredInfo))

   for (w <- 0 until pl_width)
   {
      when(ren1_br_vals(w)) {
         prediction_copies(ren1_uops(w).br_tag) := io.ren_pred_info(w)
      }
   }

   io.get_pred.info := prediction_copies(io.get_pred.br_tag)

   val temp = Wire(new BranchPredInfo)
   println("\t\tPrediction Snapshots: " + temp.toBits.getWidth + "-bits, " + MAX_BR_COUNT + " entries")

   //-------------------------------------------------------------
   // Free List

   for (list <- Seq(i_vfreelist, f_vfreelist))
   {
      list.io.brinfo := io.brinfo
      list.io.kill := io.kill
      list.io.ren_will_fire := ren1_will_fire
      list.io.ren_uops := ren1_uops
      list.io.ren_br_vals := ren1_br_vals
      list.io.com_valids := io.com_valids
      list.io.com_uops := io.com_uops
      list.io.com_rbk_valids := io.com_rbk_valids
      list.io.flush_pipeline := io.flush_pipeline
      list.io.debug_rob_empty := io.debug_rob_empty
   }

   for ((uop, w) <- ren1_uops.zipWithIndex)
   {
      val i_vreg = i_vfreelist.io.req_vregs(w)
      val f_vreg = f_vfreelist.io.req_vregs(w)
      uop.vdst := Mux(uop.dst_rtype === RT_FLT, f_vreg, i_vreg)
   }

   //-------------------------------------------------------------
   // Rename Table

   for (table <- Seq(i_l2v_maptable, f_l2v_maptable))
   {
      table.io.brinfo := io.brinfo
      table.io.kill := io.kill
      table.io.ren_will_fire := ren1_will_fire
      table.io.ren_uops := ren1_uops // expects vdst to be set up
      table.io.ren_br_vals := ren1_br_vals
      table.io.com_valids := io.com_valids
      table.io.com_uops := io.com_uops
      table.io.com_rbk_valids := io.com_rbk_valids
      table.io.flush_pipeline := io.flush_pipeline
      table.io.debug_inst_can_proceed := io.inst_can_proceed
   }
   i_l2v_maptable.io.debug_freelist_can_allocate := i_vfreelist.io.can_allocate
   f_l2v_maptable.io.debug_freelist_can_allocate := f_vfreelist.io.can_allocate

   for ((uop, w) <- ren1_uops.zipWithIndex)
   {
      val imap = i_l2v_maptable.io.values(w)
      val fmap = f_l2v_maptable.io.values(w)

      uop.vop1       := Mux(uop.lrs1_rtype === RT_FLT, fmap.vrs1, imap.vrs1)
      uop.vop2       := Mux(uop.lrs2_rtype === RT_FLT, fmap.vrs2, imap.vrs2)
      uop.vop3       := f_l2v_maptable.io.values(w).vrs3 // only FP has 3rd operand
      uop.stale_vdst := Mux(uop.dst_rtype === RT_FLT,  fmap.stale_vdst, imap.stale_vdst)
   }

   //-------------------------------------------------------------
   // pipeline registers

   val ren2_will_fire = ren2_valids zip io.dis_inst_can_proceed map {case (v,c) => v && c && !io.kill}

   // will ALL ren2 uops proceed to dispatch?
   val ren2_will_proceed = 
      if (renameLatency == 2) (ren2_valids zip ren2_will_fire map {case (v,f) => (v === f)}).reduce(_&_)
      else io.dis_inst_can_proceed.reduce(_&_)



   val ren2_imapvalues = if (renameLatency == 2) RegEnable(i_l2v_maptable.io.values, ren2_will_proceed)
                         else i_l2v_maptable.io.values
   val ren2_fmapvalues = if (renameLatency == 2) RegEnable(f_l2v_maptable.io.values, ren2_will_proceed)
                         else f_l2v_maptable.io.values

   for (w <- 0 until pl_width)
   {
      if (renameLatency == 1)
      {
         ren2_valids(w) := ren1_will_fire(w)
         ren2_uops(w)   := GetNewUopAndBrMask(ren1_uops(w), io.brinfo)
      }
      else
      {
         require (renameLatency == 2)
         val r_valids = Reg(Vec.fill(pl_width) {Bool(false)})
         val r_uops   = Reg(Vec(pl_width, new MicroOp()))

         when (io.kill)
         {
            r_valids(w) := Bool(false)
         }
         .elsewhen (ren2_will_proceed)
         {
            r_valids(w) := ren1_will_fire(w)
            r_uops(w) := GetNewUopAndBrMask(ren1_uops(w), io.brinfo)
         }
         .otherwise
         {
            r_valids(w) := r_valids(w) && !ren2_will_fire(w) // clear bit if uop gets dispatched
            r_uops(w) := GetNewUopAndBrMask(r_uops(w), io.brinfo)
         }

         ren2_valids(w) := r_valids(w)
         ren2_uops  (w) := r_uops(w)
      }
   }

   //-------------------------------------------------------------
   // V2P MapTable by yqh

   i_v2p_maptable.io.ren_will_fire 	:= ren2_will_fire
   i_v2p_maptable.io.ren_uops 		:= ren2_uops
   i_v2p_maptable.io.map_table 		:= ren2_imapvalues

   i_v2p_maptable.io.com_valids		:= io.com_valids
   i_v2p_maptable.io.com_uops		:= io.com_uops
   i_v2p_maptable.io.com_rbk_valids	:= io.com_rbk_valids

   for (w <- 0 until pl_width)
   { 
       i_v2p_maptable.io.allocpregs_valids(w)	:= i_pfreelist.io.can_allocate(w)
       i_v2p_maptable.io.allocpregs_vregs(w)	:= ren2_uops(w).vdst // ??? 
       i_v2p_maptable.io.allocpregs_pregs(w)	:= i_pfreelist.io.req_pregs(w)
       i_v2p_maptable.io.allocpregs_masks(w)	:= i_pfreelist.io.req_masks(w)
   }

   for ((uop, w) <- ren2_uops.zipWithIndex) {
       val i_v2p_map = i_v2p_maptable.io.values(w)

       when (uop.lrs1_rtype === RT_FIX) 
	   {
           uop.pop1 		:= i_v2p_map.prs1
	       //uop.rs1_mask 	:= i_v2p_map.prs1_mask === UInt(0)
	   }

	   when (uop.lrs2_rtype === RT_FIX) 
	   {
           uop.pop2 		:= i_v2p_map.prs2
		   //uop.rs2_mask 	:= i_v2p_map.prs2_mask === UInt(0)
	   }
   }

   //-------------------------------------------------------------
   // Physical Register FreeList
   // i_pfreelist

   i_pfreelist.io.brinfo := io.brinfo

   for (w <- 0 until pl_width) {
	   i_pfreelist.io.ren_uops(w)       	:= ren2_uops(w)
       i_pfreelist.io.ren_br_vals(w) 		:= ren2_will_fire(w) & ren2_uops(w).allocate_brtag

	   //???
       i_pfreelist.io.req_preg_vals(w)     	:= !io.kill &&                        
                                               ren2_will_fire(w) &&
                                               ren2_uops(w).ldst_val &&
                                               ren2_uops(w).dst_rtype === RT_FIX

       i_pfreelist.io.req_part_nums(w) 		:= Wire(init = UInt(numIntPhysRegsParts))
   }

   i_pfreelist.io.enq_vals 			:= i_v2p_maptable.io.enq_valids 
   i_pfreelist.io.enq_pregs 		:= i_v2p_maptable.io.enq_pregs
   i_pfreelist.io.enq_masks 		:= i_v2p_maptable.io.enq_masks

   i_pfreelist.io.rollback_wens 	:= i_v2p_maptable.io.rollback_valids 
   i_pfreelist.io.rollback_pdsts 	:= i_v2p_maptable.io.rollback_pdsts
   i_pfreelist.io.rollback_masks 	:= i_v2p_maptable.io.rollback_masks

   //-------------------------------------------------------------
   // Busy Table

   ibusytable.io.ren_will_fire := ren2_will_fire
   ibusytable.io.ren_uops := ren2_uops  // expects vdst to be set up.
   ibusytable.io.map_table := ren2_imapvalues
   ibusytable.io.wb_valids := io.int_wakeups.map(_.valid)
   ibusytable.io.wb_vdsts := io.int_wakeups.map(_.bits.uop.vdst)
   
   assert (!(io.int_wakeups.map(x => x.valid && x.bits.uop.dst_rtype =/= RT_FIX).reduce(_|_)),
      "[rename] int wakeup is not waking up a Int register.")

   for (w <- 0 until pl_width)
   {
      assert (!(
         ren2_will_fire(w) &&
         ren2_uops(w).lrs1_rtype === RT_FIX &&
         ren2_uops(w).vop1 =/= ibusytable.io.map_table(w).vrs1),
         "[rename] ren2 maptable vrs1 value don't match uop's values.")
      assert (!(
         ren2_will_fire(w) &&
         ren2_uops(w).lrs2_rtype === RT_FIX &&
         ren2_uops(w).vop2 =/= ibusytable.io.map_table(w).vrs2),
         "[rename] ren2 maptable vrs2 value don't match uop's values.")
   }

   fbusytable.io.ren_will_fire := ren2_will_fire
   fbusytable.io.ren_uops := ren2_uops  // expects vdst to be set up.
   fbusytable.io.map_table := ren2_fmapvalues
   fbusytable.io.wb_valids := io.fp_wakeups.map(_.valid)
   fbusytable.io.wb_vdsts := io.fp_wakeups.map(_.bits.uop.vdst)

   assert (!(io.fp_wakeups.map(x => x.valid && x.bits.uop.dst_rtype =/= RT_FLT).reduce(_|_)),
      "[rename] fp wakeup is not waking up a FP register.")

   for ((uop, w) <- ren2_uops.zipWithIndex)
   {
      val ibusy = ibusytable.io.values(w)
      val fbusy = fbusytable.io.values(w)

	  // yqh
	  val ibusy_rs1_mask = Mux(ibusy.rs1_busy, Bits(0, width = numIntPhysRegsParts), ~Bits(0, width = numIntPhysRegsParts))
	  val ibusy_rs2_mask = Mux(ibusy.rs2_busy, Bits(0, width = numIntPhysRegsParts), ~Bits(0, width = numIntPhysRegsParts))

	  val fbusy_rs1_mask = Mux(fbusy.rs1_busy, Bits(0, width = numIntPhysRegsParts), ~Bits(0, width = numIntPhysRegsParts))
	  val fbusy_rs2_mask = Mux(fbusy.rs2_busy, Bits(0, width = numIntPhysRegsParts), ~Bits(0, width = numIntPhysRegsParts))
	  val fbusy_rs3_mask = Mux(fbusy.rs3_busy, Bits(0, width = numIntPhysRegsParts), ~Bits(0, width = numIntPhysRegsParts))

      uop.rs1_mask := Mux(uop.lrs1_rtype === RT_FLT, fbusy_rs1_mask, ibusy_rs1_mask)
      uop.rs2_mask := Mux(uop.lrs2_rtype === RT_FLT, fbusy_rs2_mask, ibusy_rs2_mask)
      uop.rs3_mask := fbusy_rs3_mask

      val valid = ren2_valids(w)
      assert (!(valid && ibusy.rs1_busy && uop.lrs1_rtype === RT_FIX && uop.lrs1 === UInt(0)), "[rename] x0 is busy??")
      assert (!(valid && ibusy.rs2_busy && uop.lrs2_rtype === RT_FIX && uop.lrs2 === UInt(0)), "[rename] x0 is busy??")
   }

   //-------------------------------------------------------------
   // Outputs

   io.ren1_mask := ren1_will_fire
   io.ren1_uops := ren1_uops

   io.ren2_mask := ren2_will_fire
   io.ren2_uops := ren2_uops map {u => GetNewUopAndBrMask(u, io.brinfo)}

   for (w <- 0 until pl_width)
   {
      // Push back against Decode stage if Rename1 can't proceed (and Rename2/Dispatch can't receive).
      io.inst_can_proceed(w) :=
         ren2_will_proceed &&
         ((ren1_uops(w).dst_rtype =/= RT_FIX && ren1_uops(w).dst_rtype =/= RT_FLT) ||
         (i_vfreelist.io.can_allocate(w) && ren1_uops(w).dst_rtype === RT_FIX) ||
         (f_vfreelist.io.can_allocate(w) && ren1_uops(w).dst_rtype === RT_FLT))
   }


   //-------------------------------------------------------------
   // Debug signals

   io.debug.i_vfreelist  := i_vfreelist.io.debug.vfreelist
   io.debug.iisprlist  := i_vfreelist.io.debug.isprlist
   io.debug.ibusytable := ibusytable.io.debug.busytable
   io.debug.f_vfreelist  := f_vfreelist.io.debug.vfreelist
   io.debug.fisprlist  := f_vfreelist.io.debug.isprlist
   io.debug.fbusytable := fbusytable.io.debug.busytable
}

