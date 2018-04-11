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

class AllocToRenameIO(
   vreg_sz: Int,
   preg_sz: Int,
   mask_sz: Int)
   (implicit p: Parameters) extends BoomBundle()(p)
{
   val valid     = Bool(INPUT)
   val vreg      = UInt(INPUT, width=vreg_sz)
   val nums      = UInt(INPUT, width=mask_sz)
   val br_mask   = UInt(INPUT, width=MAX_BR_COUNT)
   val is_rob_head = Bool(INPUT)   

   val can_alloc = Bool(OUTPUT)
   val preg      = UInt(OUTPUT, width=preg_sz)
   val mask      = UInt(OUTPUT, width=mask_sz)

   override def cloneType: this.type = new AllocToRenameIO(vreg_sz,preg_sz,mask_sz).asInstanceOf[this.type]
}

class RenameStageIO(
   pl_width: Int,
   num_int_pregs: Int,
   num_fp_pregs: Int,
   num_int_wb_ports: Int,
   num_fp_wb_ports: Int,
   num_myint_wb_ports: Int)
   (implicit p: Parameters) extends BoomBundle()(p)
{
   private val int_preg_sz = TPREG_SZ
   private val fp_preg_sz = TPREG_SZ

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

   // alloc physical register stage, yqh
   // yqh debug
   val int_alloc_pregs = Vec(num_myint_wb_ports, new AllocToRenameIO(TPREG_SZ, TPREG_SZ, numIntPhysRegsParts))
   val fp_alloc_pregs  = Vec(num_fp_wb_ports, new AllocToRenameIO(TPREG_SZ, TPREG_SZ, numIntPhysRegsParts))

   // issue stage (fast wakeup)
   val int_wakeups = Vec(num_int_wb_ports, Valid(new ExeUnitResp(xLen))).flip
   val fp_wakeups = Vec(num_fp_wb_ports, Valid(new ExeUnitResp(fLen+1))).flip

   // commit stage
   val com_valids = Vec(pl_width, Bool()).asInput
   val com_uops   = Vec(pl_width, new MicroOp()).asInput
   val com_rbk_valids = Vec(pl_width, Bool()).asInput

   val flush_pipeline = Bool(INPUT) // only used for SCR (single-cycle reset)

   val nrr_used   = UInt(INPUT, NRR)

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
   num_fp_wb_ports: Int,
   num_myint_wb_ports: Int)
(implicit p: Parameters) extends BoomModule()(p)
{
   val io = new RenameStageIO(pl_width, numIntVPhysRegs, numFpVPhysRegs, num_int_wb_ports, num_fp_wb_ports, num_myint_wb_ports)

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
   // yqh debug
   val i_v2p_maptable = Module(new RenameV2PMapTable(
       pl_width,
       RT_FIX.litValue,
       numIntVPhysRegs,
       numIntPPhysRegs,
       pl_width*2,
       num_myint_wb_ports))
   val i_pfreelist = Module(new RenamePFreeList(
       num_myint_wb_ports,
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
   // yqh debug
   val f_vfreelist = Module(new RenameVFreeList(
      pl_width,
      RT_FLT.litValue,
      numFpVPhysRegs))
   val f_v2p_maptable = Module(new RenameV2PMapTable(
       pl_width,
       RT_FLT.litValue,
       numFpVPhysRegs,
       numFpPPhysRegs,
       pl_width*3,
       num_fp_wb_ports))
   val f_pfreelist = Module(new RenamePFreeList(
       num_fp_wb_ports,
	   pl_width,
	   RT_FLT.litValue,
	   numFpPPhysRegs))
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

   // yqh debug

   // i_v2p_maptable
   i_v2p_maptable.io.ren_will_fire 	:= ren2_will_fire
   i_v2p_maptable.io.ren_uops 		:= ren2_uops
   i_v2p_maptable.io.map_table 		:= ren2_imapvalues

   i_v2p_maptable.io.com_valids		:= io.com_valids
   i_v2p_maptable.io.com_uops		:= io.com_uops
   i_v2p_maptable.io.com_rbk_valids	:= io.com_rbk_valids

   for (w <- 0 until num_myint_wb_ports)
   { 
       i_v2p_maptable.io.allocpregs_valids(w)	:= i_pfreelist.io.can_allocate(w)// | io.int_alloc_pregs(w).valid // yangqinghong
	   i_v2p_maptable.io.allocpregs_vregs(w)	:= io.int_alloc_pregs(w).vreg
       i_v2p_maptable.io.allocpregs_pregs(w)	:= i_pfreelist.io.req_pregs(w)
       i_v2p_maptable.io.allocpregs_masks(w)	:= i_pfreelist.io.req_masks(w)
   }

   // f_v2p_maptable
   f_v2p_maptable.io.ren_will_fire 	:= ren2_will_fire
   f_v2p_maptable.io.ren_uops 		:= ren2_uops
   f_v2p_maptable.io.map_table 		:= ren2_fmapvalues

   f_v2p_maptable.io.com_valids		:= io.com_valids
   f_v2p_maptable.io.com_uops		:= io.com_uops
   f_v2p_maptable.io.com_rbk_valids	:= io.com_rbk_valids
   
   for (w <- 0 until num_fp_wb_ports)
   { 
       f_v2p_maptable.io.allocpregs_valids(w)	:= f_pfreelist.io.can_allocate(w)// | io.fp_alloc_pregs(w).valid // yangqinghong
       f_v2p_maptable.io.allocpregs_vregs(w)	:= io.fp_alloc_pregs(w).vreg
       f_v2p_maptable.io.allocpregs_pregs(w)	:= f_pfreelist.io.req_pregs(w)
       f_v2p_maptable.io.allocpregs_masks(w)	:= f_pfreelist.io.req_masks(w)
   }

   //-------------------------------------------------------------
   // Physical Register FreeList
   // i_pfreelist

   for (w <- 0 until pl_width) {
       i_pfreelist.io.ren_uops(w)           := ren2_uops(w)
       i_pfreelist.io.ren_br_vals(w)        := ren2_will_fire(w) & ren2_uops(w).allocate_brtag
   }

   for (w <- 0 until num_myint_wb_ports) {
       i_pfreelist.io.req_preg_vals(w)      := io.int_alloc_pregs(w).valid
	   i_pfreelist.io.req_is_rob_head(w)    := io.int_alloc_pregs(w).is_rob_head
       i_pfreelist.io.req_part_nums(w)      := io.int_alloc_pregs(w).nums
	   i_pfreelist.io.req_br_mask(w)        := io.int_alloc_pregs(w).br_mask
       io.int_alloc_pregs(w).can_alloc      := i_pfreelist.io.can_allocate(w)// | io.int_alloc_pregs(w).valid //yangqinghong
       io.int_alloc_pregs(w).preg           := i_pfreelist.io.req_pregs(w)
       io.int_alloc_pregs(w).mask           := i_pfreelist.io.req_masks(w)
   }

   i_pfreelist.io.nrr_used          := io.nrr_used
   i_pfreelist.io.brinfo 			:= io.brinfo
   i_pfreelist.io.enq_vals 			:= i_v2p_maptable.io.enq_valids 
   i_pfreelist.io.enq_pregs 		:= i_v2p_maptable.io.enq_pregs
   i_pfreelist.io.enq_masks 		:= i_v2p_maptable.io.enq_masks

   i_pfreelist.io.rollback_wens 	:= i_v2p_maptable.io.rollback_valids 
   i_pfreelist.io.rollback_pdsts 	:= i_v2p_maptable.io.rollback_pdsts
   i_pfreelist.io.rollback_masks 	:= i_v2p_maptable.io.rollback_masks

   // f_pfreelist
   for (w <- 0 until pl_width) {
       f_pfreelist.io.ren_uops(w)           := ren2_uops(w)
       f_pfreelist.io.ren_br_vals(w)        := ren2_will_fire(w) & ren2_uops(w).allocate_brtag
   }

   for (w <- 0 until num_fp_wb_ports) {
       f_pfreelist.io.req_preg_vals(w)      := io.fp_alloc_pregs(w).valid
       f_pfreelist.io.req_part_nums(w)      := io.fp_alloc_pregs(w).nums
	   f_pfreelist.io.req_br_mask(w)        := io.fp_alloc_pregs(w).br_mask
	   f_pfreelist.io.req_is_rob_head(w)    := io.fp_alloc_pregs(w).is_rob_head
       io.fp_alloc_pregs(w).can_alloc       := f_pfreelist.io.can_allocate(w)// | io.fp_alloc_pregs(w).valid //yangqinghong
       io.fp_alloc_pregs(w).preg            := f_pfreelist.io.req_pregs(w)
       io.fp_alloc_pregs(w).mask            := f_pfreelist.io.req_masks(w)
   }

   f_pfreelist.io.nrr_used          := UInt(0) // ???
   f_pfreelist.io.brinfo 			:= io.brinfo
   f_pfreelist.io.enq_vals 			:= f_v2p_maptable.io.enq_valids 
   f_pfreelist.io.enq_pregs 		:= f_v2p_maptable.io.enq_pregs
   f_pfreelist.io.enq_masks 		:= f_v2p_maptable.io.enq_masks

   f_pfreelist.io.rollback_wens 	:= f_v2p_maptable.io.rollback_valids 
   f_pfreelist.io.rollback_pdsts 	:= f_v2p_maptable.io.rollback_pdsts
   f_pfreelist.io.rollback_masks 	:= f_v2p_maptable.io.rollback_masks
   //-------------------------------------------------------------
   // Busy Table

   ibusytable.io.ren_will_fire := ren2_will_fire
   ibusytable.io.ren_uops := ren2_uops  // expects vdst to be set up.
   ibusytable.io.map_table := ren2_imapvalues
   ibusytable.io.wb_valids := io.int_wakeups.map(_.valid)
   ibusytable.io.wb_vdsts := io.int_wakeups.map(_.bits.uop.vdst)
   ibusytable.io.wb_state := io.int_wakeups.map(_.bits.state)

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
   fbusytable.io.wb_state := io.fp_wakeups.map(_.bits.state)

   assert (!(io.fp_wakeups.map(x => x.valid && x.bits.uop.dst_rtype =/= RT_FLT).reduce(_|_)),
      "[rename] fp wakeup is not waking up a FP register.")

   for ((uop, w) <- ren2_uops.zipWithIndex)
   {
	  uop.pop1  := Mux(uop.lrs1_rtype === RT_FLT, f_v2p_maptable.io.values(w).prs1, i_v2p_maptable.io.values(w).prs1)
	  uop.pop2  := Mux(uop.lrs2_rtype === RT_FLT, f_v2p_maptable.io.values(w).prs2, i_v2p_maptable.io.values(w).prs2)
      uop.pop3  := f_v2p_maptable.io.values(w).prs3// only FP has 3rd operand

	  uop.rs1_mask  := Mux(uop.lrs1_rtype === RT_FLT, f_v2p_maptable.io.values(w).prs1_mask, i_v2p_maptable.io.values(w).prs1_mask)
	  uop.rs2_mask  := Mux(uop.lrs1_rtype === RT_FLT, f_v2p_maptable.io.values(w).prs2_mask, i_v2p_maptable.io.values(w).prs2_mask)
	  uop.rs3_mask  := f_v2p_maptable.io.values(w).prs3_mask
   }

   for ((uop, w) <- ren2_uops.zipWithIndex)
   {
      val ibusy = ibusytable.io.values(w)
	  val fbusy = fbusytable.io.values(w)

	  val b1 = Mux(uop.lrs1_rtype === RT_FLT, fbusy.prs1_busy, ibusy.prs1_busy)
	  val b2 = Mux(uop.lrs2_rtype === RT_FLT, fbusy.prs2_busy, ibusy.prs2_busy)
	  val b3 = fbusy.prs3_busy
	  uop.prs_busy := b3 << 2 | b2 << 1 | b1

	  val valid = ren2_valids(w)
	  assert (!(valid && ibusy.prs1_busy && uop.lrs1_rtype === RT_FIX && uop.lrs1 === UInt(0)), "[rename] x0 is busy??")
	  assert (!(valid && ibusy.prs2_busy && uop.lrs2_rtype === RT_FIX && uop.lrs2 === UInt(0)), "[rename] x0 is busy??")
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

