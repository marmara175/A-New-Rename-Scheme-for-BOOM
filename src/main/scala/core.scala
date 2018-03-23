//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Processor Core
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// BOOM has the following (conceptual) stages:
//   if1 - Instruction Fetch 1 (I$ access)
//   if2 - Instruction Fetch 2 (instruction return)
//   bp1 - Branch Predict      (in parallel with IF1)
//   bp2 - Branch Decode       (in parallel with IF2)
//   dec - Decode
//   ren - Rename
//   dis - Dispatch
//   iss - Issue
//   rrd - Register Read
//   exe - Execute
//   mem - Memory
//   wb  - Writeback
//   com - Commit
//
//


package boom

import Chisel._
import config.Parameters

import rocket.Instructions._
import boom.FUConstants._
import util.Str


abstract class BoomModule(implicit p: Parameters) extends tile.CoreModule()(p)
  with HasBoomCoreParameters

class BoomBundle(implicit val p: Parameters) extends util.ParameterizedBundle()(p)
  with HasBoomCoreParameters

//-------------------------------------------------------------
//-------------------------------------------------------------
//-------------------------------------------------------------


class BoomCore(implicit p: Parameters, edge: uncore.tilelink2.TLEdgeOut) extends BoomModule()(p)
   with tile.HasCoreIO
{
   //**********************************
   // construct all of the modules

   // Only holds integer-registerfile execution units.
   val exe_units        = new boom.ExecutionUnits(fpu=false)
   // Meanwhile, the FP pipeline holds the FP issue window, FP regfile, and FP arithmetic units.
   var fp_pipeline: FpPipeline = null
   if (usingFPU) {
      fp_pipeline       = Module(new FpPipeline())
   }

   val num_irf_write_ports = exe_units.map(_.num_rf_write_ports).sum
   val num_fast_wakeup_ports = exe_units.count(_.isBypassable)
   val num_wakeup_ports = num_irf_write_ports + num_fast_wakeup_ports
   val fetch_unit       = Module(new FetchUnit(fetchWidth))
   val bpd_stage        = Module(new BranchPredictionStage(fetchWidth))
   val dec_serializer   = Module(new FetchSerializerNtoM)
   val decode_units     = for (w <- 0 until DECODE_WIDTH) yield { val d = Module(new DecodeUnit); d }
   val dec_brmask_logic = Module(new BranchMaskGenerationLogic(DECODE_WIDTH))
   val rename_stage     = Module(new RenameStage(DECODE_WIDTH, num_wakeup_ports, fp_pipeline.io.wakeups.length, num_irf_write_ports))
   val issue_units      = new boom.IssueUnits(num_wakeup_ports)
   val iregfile         = if (regreadLatency == 1 && enableCustomRf) {
                              Module(new RegisterFileSeqCustomArray(numIntPPhysRegs,
                                 exe_units.withFilter(_.usesIRF).map(e => e.num_rf_read_ports).sum,
                                 exe_units.withFilter(_.usesIRF).map(e => e.num_rf_write_ports).sum,
                                 xLen,
                                 exe_units.bypassable_write_port_mask))
                          } else {
                              Module(new RegisterFileBehavorial(numIntPPhysRegs,
                                 exe_units.withFilter(_.usesIRF).map(e => e.num_rf_read_ports).sum,
                                 exe_units.withFilter(_.usesIRF).map(e => e.num_rf_write_ports).sum,
                                 xLen,
                                 exe_units.bypassable_write_port_mask))
                          }
   val ll_wbarb         = Module(new Arbiter(new ExeUnitResp(xLen), 2))
   val iregister_read   = Module(new RegisterRead(
                                 issue_units.map(_.issue_width).sum,
                                 exe_units.withFilter(_.usesIRF).map(_.supportedFuncUnits),
                                 exe_units.withFilter(_.usesIRF).map(_.num_rf_read_ports).sum,
                                 exe_units.withFilter(_.usesIRF).map(_.num_rf_read_ports),
                                 exe_units.num_total_bypass_ports,
                                 xLen))
   val csr              = Module(new rocket.CSRFile())
   val dc_shim          = Module(new DCacheShim())
   val lsu              = Module(new LoadStoreUnit(DECODE_WIDTH))
   val rob              = Module(new Rob(
                                 DECODE_WIDTH,
                                 NUM_ROB_ENTRIES,
                                 num_irf_write_ports + fp_pipeline.io.wakeups.length,
                                 exe_units.num_fpu_ports + fp_pipeline.io.wakeups.length))
   // Used to wakeup registers in rename and issue. ROB needs to listen to something else.
   val int_wakeups      = Wire(Vec(num_wakeup_ports, Valid(new ExeUnitResp(xLen))))

   require (exe_units.length == issue_units.map(_.issue_width).sum)

   //***********************************
   // Pipeline State Registers and Wires

   // Instruction Decode Stage
   val dec_valids     = Wire(Vec(DECODE_WIDTH, Bool()))  // are the decoded instruction valid? It may be held up though.
   val dec_uops       = Wire(Vec(DECODE_WIDTH, new MicroOp()))
   val dec_will_fire  = Wire(Vec(DECODE_WIDTH, Bool()))  // can the instruction fire beyond decode?
                                                         // (can still be stopped in ren or dis)
   val dec_rdy        = Wire(Bool())

   // Dispatch Stage
   val dis_valids     = Wire(Vec(DISPATCH_WIDTH, Bool())) // true if uop WILL enter IW
   val dis_uops       = Wire(Vec(DISPATCH_WIDTH, new MicroOp()))

   // Issue Stage/Register Read
   val iss_valids     = Wire(Vec(exe_units.length, Bool()))
   val iss_uops       = Wire(Vec(exe_units.length, new MicroOp()))
   val bypasses       = Wire(new BypassData(exe_units.num_total_bypass_ports, xLen))

   // Branch Unit
   val br_unit = Wire(new BranchUnitResp())
   val brunit_idx = exe_units.br_unit_idx
   br_unit <> exe_units.br_unit_io

   // Shim to DCache
   io.dmem <> dc_shim.io.dmem
   dc_shim.io.core <> exe_units.memory_unit.io.dmem
   dc_shim.io.core.invalidate_lr := rob.io.com_xcpt.valid

   // Load/Store Unit & ExeUnits
   exe_units.memory_unit.io.lsu_io := lsu.io


   //****************************************
   // Time Stamp Counter & Retired Instruction Counter
   // (only used for printf and vcd dumps - the actual counters are in the CSRFile)
   val debug_tsc_reg  = Reg(init = UInt(0, xLen))
   val debug_irt_reg  = Reg(init = UInt(0, xLen))
   debug_tsc_reg  := debug_tsc_reg + Mux(Bool(O3PIPEVIEW_PRINTF), UInt(O3_CYCLE_TIME), UInt(1))
   debug_irt_reg  := debug_irt_reg + PopCount(rob.io.commit.valids.toBits)
   debug(debug_tsc_reg)
   debug(debug_irt_reg)


   //****************************************
   // Print-out information about the machine

   if (usingFPU)         println ("\n    FPU Unit Enabled")
   else                  println ("\n    FPU Unit Disabled")
   if (usingVM)          println ("    VM       Enabled")
   else                  println ("    VM       Disabled")
   if (usingFDivSqrt)    println ("    FDivSqrt Enabled\n")
   else                  println ("    FDivSqrt Disabled\n")

   val iss_str = if (enableAgePriorityIssue) " (Age-based Priority)"
                 else " (Unordered Priority)"
   println("\n   Fetch Width           : " + fetchWidth)
   println("   Issue Width           : " + issueParams.map(_.issueWidth).sum)
   println("   ROB Size              : " + NUM_ROB_ENTRIES)
   println("   Issue Window Size     : " + issueParams.map(_.numEntries) + iss_str)
   println("   Load/Store Unit Size  : " + NUM_LSU_ENTRIES + "/" + NUM_LSU_ENTRIES)
   println("   Num Int Phys Registers: " + numIntPPhysRegs)
   println("   Num FP  Phys Registers: " + numFpPPhysRegs)
   println("   Max Branch Count      : " + MAX_BR_COUNT)
   println("   BTB Size              : " +
      (if (enableBTB) ("" + boomParams.btb.nSets * boomParams.btb.nWays + " entries (" +
         boomParams.btb.nSets + " x " + boomParams.btb.nWays + " ways)") else 0))
   println("   RAS Size              : " + (if (enableBTB) boomParams.btb.nRAS else 0))
   println("   Rename  Stage Latency : " + renameLatency)
   println("   RegRead Stage Latency : " + regreadLatency)

   print(iregfile)
   println("\n   Num Slow Wakeup Ports : " + num_irf_write_ports)
   println("   Num Fast Wakeup Ports : " + exe_units.count(_.isBypassable))
   println("   Num Bypass Ports      : " + exe_units.num_total_bypass_ports)

   print(fp_pipeline)

   println("\n   DCache Ways           : " + dcacheParams.nWays)
   println("   DCache Sets           : " + dcacheParams.nSets)
   println("   ICache Ways           : " + icacheParams.nWays)
   println("   ICache Sets           : " + icacheParams.nSets)

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Fetch Stage/Frontend ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   io.imem <> fetch_unit.io.imem
   // TODO: work-around rocket-chip issue #183, broken imem.mask for fetchWidth=1
   // TODO: work-around rocket-chip issue #184, broken imem.mask for fetchWidth=1
   if (fetchWidth == 1)
   {
      fetch_unit.io.imem.resp.bits.mask := UInt(1)
      fetch_unit.io.imem.resp.bits.btb.bits.bridx := UInt(0)
   }
   fetch_unit.io.br_unit <> br_unit
   fetch_unit.io.tsc_reg           := debug_tsc_reg

   fetch_unit.io.f1_btb            := bpd_stage.io.f1_btb
   fetch_unit.io.f2_bpu_request    := bpd_stage.io.f2_bpu_request
   fetch_unit.io.f2_btb_resp       := bpd_stage.io.f2_btb_resp
   fetch_unit.io.f2_bpd_resp       := bpd_stage.io.f2_bpd_resp

   fetch_unit.io.f3_bpd_resp       := bpd_stage.io.f3_bpd_resp

   fetch_unit.io.clear_fetchbuffer := br_unit.brinfo.mispredict ||
                                       rob.io.flush.valid
   fetch_unit.io.flush_take_pc     := rob.io.flush.valid
   fetch_unit.io.flush_pc          := rob.io.flush.bits.pc

   io.imem.flush_icache :=
      Range(0,DECODE_WIDTH).map{i => rob.io.commit.valids(i) && rob.io.commit.uops(i).is_fencei}.reduce(_|_) ||
      (br_unit.brinfo.mispredict && br_unit.brinfo.is_jr &&  csr.io.status.debug)

   io.imem.flush_tlb := csr.io.fatc

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Branch Prediction ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // These stages are effectively in parallel with instruction fetch and
   // decode.  BHT look-up is in parallel with I$ access, and Branch Decode
   // occurs before fetch buffer insertion.

   bpd_stage.io.ext_btb_req := io.imem.ext_btb.req
   bpd_stage.io.icmiss := io.imem.ext_btb.icmiss
   bpd_stage.io.ext_btb_req.valid := io.imem.ext_btb.req.valid || io.imem.req.valid

   bpd_stage.io.br_unit := br_unit
   bpd_stage.io.redirect := io.imem.req.valid
   bpd_stage.io.flush := rob.io.flush.valid

   bpd_stage.io.fetch_stalled := fetch_unit.io.stalled

   bpd_stage.io.f2_ras_update := fetch_unit.io.f2_ras_update
   bpd_stage.io.f3_btb_update := fetch_unit.io.f3_btb_update
   bpd_stage.io.f3_hist_update:= fetch_unit.io.f3_hist_update
   bpd_stage.io.f3_bim_update := fetch_unit.io.f3_bim_update
   bpd_stage.io.status_prv   := csr.io.status.prv
   bpd_stage.io.status_debug := csr.io.status.debug

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Decode Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // track mask of finished instructions in the bundle
   // use this to mask out insts coming from FetchBuffer that have been finished
   // for example, back pressure may cause us to only issue some instructions from FetchBuffer
   // but on the next cycle, we only want to retry a subset
   val dec_finished_mask = Reg(init = Bits(0, DECODE_WIDTH))

   // TODO need to generalize this logic to other width disparities
   require (DECODE_WIDTH == fetchWidth)

   //-------------------------------------------------------------
   // Pull out instructions and send to the Decoders

   dec_serializer.io.enq <> fetch_unit.io.resp

   dec_serializer.io.kill := fetch_unit.io.clear_fetchbuffer
   dec_serializer.io.deq.ready := dec_rdy

   val fetched_inst_valid = dec_serializer.io.deq.valid
   val dec_fbundle        = dec_serializer.io.deq.bits

   //-------------------------------------------------------------
   // Decoders

   // allow early instructions to stall later instructions
   var dec_stall_next_inst = Bool(false)
   var dec_last_inst_was_stalled = Bool(false)

   // stall fetch/dcode because we ran out of branch tags
   val branch_mask_full = Wire(Vec(DECODE_WIDTH, Bool()))

   for (w <- 0 until DECODE_WIDTH)
   {
      dec_valids(w)                      := fetched_inst_valid && dec_fbundle.uops(w).valid && !dec_finished_mask(w)
      decode_units(w).io.enq.uop         := dec_fbundle.uops(w)
      decode_units(w).io.status          := csr.io.status
      decode_units(w).io.interrupt       := csr.io.interrupt
      decode_units(w).io.interrupt_cause := csr.io.interrupt_cause

      val prev_insts_in_bundle_valid = Range(0,w).map{i => dec_valids(i)}.foldLeft(Bool(false))(_|_)

      // stall this instruction?
      // TODO tailor this to only care if a given instruction uses a resource?
      val stall_me = (dec_valids(w) &&
                        (  !(rename_stage.io.inst_can_proceed(w))
                        || (dec_valids(w) && dec_uops(w).is_unique &&
                           (!(rob.io.empty) || !lsu.io.lsu_fencei_rdy || prev_insts_in_bundle_valid))
                        || !rob.io.ready
                        || lsu.io.laq_full
                        || lsu.io.stq_full
                        || branch_mask_full(w)
                        || br_unit.brinfo.mispredict
                        || rob.io.flush.valid
                        || dec_stall_next_inst
                        || !bpd_stage.io.brob.allocate.ready
                        || (dec_valids(w) && dec_uops(w).is_fencei && !lsu.io.lsu_fencei_rdy)
                        )) ||
                     dec_last_inst_was_stalled

      // stall the next instruction following me in the decode bundle?
      dec_last_inst_was_stalled = stall_me
      dec_stall_next_inst  = stall_me || (dec_valids(w) && dec_uops(w).is_unique)

      dec_will_fire(w) := dec_valids(w) && !stall_me && !fetch_unit.io.clear_fetchbuffer
      dec_uops(w)      := decode_units(w).io.deq.uop
   }

   // all decoders are empty and ready for new instructions
   dec_rdy := !(dec_stall_next_inst)

   when (dec_rdy || fetch_unit.io.clear_fetchbuffer)
   {
      dec_finished_mask := Bits(0)
   }
   .otherwise
   {
      dec_finished_mask := dec_will_fire.toBits | dec_finished_mask
   }

   //-------------------------------------------------------------
   // Branch Mask Logic


   dec_brmask_logic.io.brinfo := br_unit.brinfo
   dec_brmask_logic.io.flush_pipeline := rob.io.flush.valid

   for (w <- 0 until DECODE_WIDTH)
   {
      dec_brmask_logic.io.is_branch(w) := !dec_finished_mask(w) && dec_uops(w).allocate_brtag
      dec_brmask_logic.io.will_fire(w) :=  dec_will_fire(w) && dec_uops(w).allocate_brtag // ren, dis can back pressure us

      dec_uops(w).br_tag  := dec_brmask_logic.io.br_tag(w)
      dec_uops(w).br_mask := dec_brmask_logic.io.br_mask(w)
   }

   branch_mask_full := dec_brmask_logic.io.is_full

   //-------------------------------------------------------------
   // LD/ST Unit Allocation Logic

   // TODO this is dupliciated logic with the the LSU... do we need ldq_idx/stq elsewhere?
   val new_ldq_idx = Wire(UInt())
   val new_stq_idx = Wire(UInt())

   var new_lidx = new_ldq_idx
   var new_sidx = new_stq_idx

   for (w <- 0 until DECODE_WIDTH)
   {
      dec_uops(w).ldq_idx := new_lidx
      dec_uops(w).stq_idx := new_sidx

      new_lidx = Mux(dec_will_fire(w) && dec_uops(w).is_load,  WrapInc(new_lidx, NUM_LSU_ENTRIES), new_lidx)
      new_sidx = Mux(dec_will_fire(w) && dec_uops(w).is_store, WrapInc(new_sidx, NUM_LSU_ENTRIES), new_sidx)
   }

   //-------------------------------------------------------------
   // Rob Allocation Logic

   for (w <- 0 until DECODE_WIDTH)
   {
      // note: this assumes uops haven't been shifted - there's a 1:1 match between PC's LSBs and "w" here
      // (thus the LSB of the rob_idx gives part of the PC)
      if (DECODE_WIDTH == 1)
         dec_uops(w).rob_idx := rob.io.curr_rob_tail
      else
         dec_uops(w).rob_idx := Cat(rob.io.curr_rob_tail, UInt(w, log2Up(DECODE_WIDTH)))

      dec_uops(w).brob_idx := bpd_stage.io.brob.allocate_brob_tail
   }

   val dec_has_br_or_jalr_in_packet =
      (dec_valids zip dec_uops map {case(v,u) => v && u.is_br_or_jmp && !u.is_jal}).reduce(_|_)

   bpd_stage.io.brob.allocate.valid := dec_will_fire.reduce(_|_) &&
                                       dec_finished_mask === Bits(0) &&
                                       dec_has_br_or_jalr_in_packet
   bpd_stage.io.brob.allocate.bits.ctrl.executed.map{_ := Bool(false)}
   bpd_stage.io.brob.allocate.bits.ctrl.taken.map{_ := Bool(false)}
   bpd_stage.io.brob.allocate.bits.ctrl.mispredicted.map{_ := Bool(false)}
   bpd_stage.io.brob.allocate.bits.ctrl.debug_executed := Bool(false)
   bpd_stage.io.brob.allocate.bits.ctrl.debug_rob_idx := dec_uops(0).rob_idx
   bpd_stage.io.brob.allocate.bits.ctrl.brob_idx := dec_uops(0).brob_idx
   bpd_stage.io.brob.allocate.bits.info := dec_fbundle.bpd_resp



   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Register Rename Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // TODO for now, assume worst-case all instructions will dispatch towards one issue unit.
   val dis_readys = issue_units.map(_.io.dis_readys.toBits).reduce(_&_) & fp_pipeline.io.dis_readys.toBits
   rename_stage.io.dis_inst_can_proceed := dis_readys.toBools
   rename_stage.io.ren_pred_info := Vec(dec_fbundle.uops.map(_.br_prediction))

   rename_stage.io.kill     := fetch_unit.io.clear_fetchbuffer // mispredict or flush
   rename_stage.io.brinfo   := br_unit.brinfo
   rename_stage.io.get_pred.br_tag        := (if (regreadLatency == 1) RegNext(iss_uops(brunit_idx).br_tag)
                                             else iss_uops(brunit_idx).br_tag)
   exe_units(brunit_idx).io.get_pred.info := RegNext(rename_stage.io.get_pred.info)

   rename_stage.io.flush_pipeline := rob.io.flush.valid
   rename_stage.io.debug_rob_empty := rob.io.empty

   rename_stage.io.dec_will_fire := dec_will_fire
   rename_stage.io.dec_uops := dec_uops

   //yqh
   //def MyEncode(data: UInt, data_size: Int, sub_size: Int): UInt =
   //{
   //   val nums = data_size / sub_size
   //   val sub_datas = Wire(Vec(nums, UInt(sub_size.W)))
   //
   //   val this_data = Wire(UInt(width=data_size.W))
   //   this_data := data
   //
   //   for (i <- 0 until nums)
   //   {
   //       val offset = i * sub_size
   //	   sub_datas(i) := (this_data >> offset)(sub_size-1, 0)
   //   }
   //
   //   val sign = sub_datas(nums-1)(sub_size-1).toBool
   //   val base = Wire(UInt(width=sub_size.W))
   //
   //   when (sign)
   //   {
   //       base := (-1.S(sub_size.W)).asUInt()
   //   }
   //   .otherwise
   //   {
   //       base := 0.U(sub_size.W)
   //   }
   //
   //   var res = 0.U(width=nums.W)
   //   var j   = 0.U(width=nums.W)
   //   for (i <- nums-1 to 1 by -1)
   //   {
   //       val sub = Wire(init = false.B)
   //	   when (res === j && sub_datas(i) === base && sub_datas(i-1)(sub_size-1).toBool === sign)
   //	   {
   //	       sub := true.B
   //	   }
   //
   //	   j = j + 1.U
   //	   res = res + sub.asUInt()
   //   }
   //
   //   nums.asUInt()-res
   //}
   //
   //def ShiftByMask(data: UInt, data_size: Int, mask: UInt, mask_size: Int, mask_cnt_one: Int): UInt =
   //{
   //    val output = Wire(UInt(width=data_size.W))
   //	val sub_size = data_size / mask_size
   //	val sub_data = Wire(Vec(mask_cnt_one, UInt(width=sub_size.W)))
   //
   //	for (i <- 0 until mask_cnt_one)
   //	{
   //	    val left  = sub_size * i
   //		val right = sub_size * (i+1) - 1
   //		sub_data(i) := data(right, left)
   //	}
   //
   //	var idx = 0.U
   //	val tmp = Wire(Vec(mask_size, UInt(width=data_size.W)))
   //	for (i <- 0 until mask_size)
   //	{
   //	    val asc = Wire(init=0.U)
   //		tmp(i) := 0.U(width=data_size.W)
   //		when (mask(i))
   //		{
   //		    tmp(i)  := sub_data(idx) << (i * sub_size)
   //			asc     := 1.U
   //		}
   //
   //		idx = idx + asc
   //	}
   //
   //	output := tmp.reduce(_|_)
   //	output
   //}
  
   // fp

   for (i <- 0 until fp_pipeline.io.wakeups.length)
   {
      rename_stage.io.fp_alloc_pregs(i).valid := fp_pipeline.io.fp_alloc_pregs(i).valid
      rename_stage.io.fp_alloc_pregs(i).vreg  := fp_pipeline.io.fp_alloc_pregs(i).vreg
      rename_stage.io.fp_alloc_pregs(i).nums  := fp_pipeline.io.fp_alloc_pregs(i).nums
      rename_stage.io.fp_alloc_pregs(i).br_mask  := fp_pipeline.io.fp_alloc_pregs(i).br_mask 
	  fp_pipeline.io.fp_alloc_pregs(i).can_alloc := rename_stage.io.fp_alloc_pregs(i).can_alloc
      fp_pipeline.io.fp_alloc_pregs(i).preg   := rename_stage.io.fp_alloc_pregs(i).preg
      fp_pipeline.io.fp_alloc_pregs(i).mask   := rename_stage.io.fp_alloc_pregs(i).mask

	  printf("fp_req: valid = b%b, vreg = d%d, nums = d%d, br_mask = b%b, can alloc = b%b, preg = d%d, mask = b%b\n",
	  rename_stage.io.fp_alloc_pregs(i).valid,
	  rename_stage.io.fp_alloc_pregs(i).vreg,
	  rename_stage.io.fp_alloc_pregs(i).nums,
	  rename_stage.io.fp_alloc_pregs(i).br_mask,
	  fp_pipeline.io.fp_alloc_pregs(i).can_alloc,
	  fp_pipeline.io.fp_alloc_pregs(i).preg,
	  fp_pipeline.io.fp_alloc_pregs(i).mask)
   }

   // yqh
   val can_alloc  = Wire(Vec(num_irf_write_ports, Bool()))
   val alloc_pdst = Wire(Vec(num_irf_write_ports, UInt(width=TPREG_SZ)))
   val alloc_mask = Wire(Vec(num_irf_write_ports, UInt(width=numIntPhysRegsParts)))
   val shift_data = Wire(Vec(num_irf_write_ports, UInt(width=64)))

   var al_idx = 0
   for (i <- 0 until exe_units.length)
   {
      if (exe_units(i).is_mem_unit)
	  {
	     //要写回的
	     rename_stage.io.int_alloc_pregs(al_idx).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.vdst != UInt(0)
		 rename_stage.io.int_alloc_pregs(al_idx).vreg  := ll_wbarb.io.out.bits.uop.vdst
		 rename_stage.io.int_alloc_pregs(al_idx).nums  := 4.U//MyEncode(ll_wbarb.io.out.bits.data)
		 rename_stage.io.int_alloc_pregs(al_idx).br_mask := ll_wbarb.io.out.bits.uop.br_mask 
		 when (ll_wbarb.io.out.bits.uop.vdst != UInt(0))
		 {
		    alloc_mask(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).mask
		    alloc_pdst(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).preg 
		    can_alloc(al_idx) := rename_stage.io.int_alloc_pregs(al_idx).can_alloc
		 }
		 .otherwise
		 {
		    alloc_mask(al_idx):= ~Bits(0, numIntPhysRegsParts)
			alloc_pdst(al_idx):= UInt(0)
			can_alloc(al_idx) := true.B
		 }
		 shift_data(al_idx):= ll_wbarb.io.out.bits.data//ShiftByMask(ll_wbarb.io.out.bits.data, alloc_mask(al_idx))

	     when (rename_stage.io.int_alloc_pregs(al_idx).valid)
	     {
           printf("1111: valid(%d) = b%b, vreg(%d) = d%d, nums(%d) = d%d, can_alloc(%d)=d%d, alloc_pdst(%d)=d%d, alloc_mask(%d)=b%b\n", 
	       al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).valid,
	       al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).vreg,
	       al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).nums,
	       al_idx.asUInt(), can_alloc(al_idx),
	       al_idx.asUInt(), alloc_pdst(al_idx),
	       al_idx.asUInt(), alloc_mask(al_idx))
         }
         when (rename_stage.io.int_alloc_pregs(al_idx).valid && !can_alloc(al_idx))
		 {
		    printf("error1 b%b\n", can_alloc(al_idx))
		 }
		 al_idx += 1
	  }
	  else
	  {
	     for (j <- 0 until exe_units(i).num_rf_write_ports)
		 {
		    val wbresp = exe_units(i).io.resp(j)
            
            def wbIsValid(rtype: UInt) =
			   wbresp.valid && wbresp.bits.uop.ctrl.rf_wen && wbresp.bits.uop.dst_rtype === rtype
			val wbReadsCSR = wbresp.bits.uop.ctrl.csr_cmd =/= rocket.CSR.N

			if (exe_units(i).uses_csr_wport && (j == 0))
			{
			   rename_stage.io.int_alloc_pregs(al_idx).valid := wbIsValid(RT_FIX) && wbresp.bits.uop.vdst != UInt(0)
			   rename_stage.io.int_alloc_pregs(al_idx).vreg  := wbresp.bits.uop.vdst
			   rename_stage.io.int_alloc_pregs(al_idx).nums  := 4.U
			   rename_stage.io.int_alloc_pregs(al_idx).br_mask := wbresp.bits.uop.br_mask 

		       when (wbresp.bits.uop.vdst != UInt(0))
		       {
		          alloc_mask(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).mask
		          alloc_pdst(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).preg 
		          can_alloc(al_idx) := rename_stage.io.int_alloc_pregs(al_idx).can_alloc
		       }
		       .otherwise
		       {
		          alloc_mask(al_idx):= ~Bits(0, numIntPhysRegsParts)
		          alloc_pdst(al_idx):= UInt(0)
		          can_alloc(al_idx) := true.B
		       }
			   
			   shift_data(al_idx):= Mux(wbReadsCSR, csr.io.rw.rdata, wbresp.bits.data)
               
			   when (rename_stage.io.int_alloc_pregs(al_idx).valid)
         	   {
                 printf("2222: valid(%d) = b%b, vreg(%d) = d%d, nums(%d) = d%d, can_alloc(%d)=d%d, alloc_pdst(%d)=d%d, alloc_mask(%d)=b%b, wbReadsCSR = b%b\n", 
	             al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).valid,
	             al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).vreg,
	             al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).nums,
         	     al_idx.asUInt(), can_alloc(al_idx),
         	     al_idx.asUInt(), alloc_pdst(al_idx),
         	     al_idx.asUInt(), alloc_mask(al_idx),
			     wbReadsCSR)
         	   }

               when (rename_stage.io.int_alloc_pregs(al_idx).valid && !can_alloc(al_idx))
		       {
		          printf("error1 b%b\n", can_alloc(al_idx))
		       }

               //require( !wbIsValid(RT_FIX) || can_alloc(al_idx))
		    }
			else
			{
			   rename_stage.io.int_alloc_pregs(al_idx).valid := wbIsValid(RT_FIX) && wbresp.bits.uop.vdst != UInt(0)
			   rename_stage.io.int_alloc_pregs(al_idx).vreg  := wbresp.bits.uop.vdst
			   rename_stage.io.int_alloc_pregs(al_idx).nums  := 4.U
			   rename_stage.io.int_alloc_pregs(al_idx).br_mask := wbresp.bits.uop.br_mask

		       when (wbresp.bits.uop.vdst != UInt(0))
		       {
		          alloc_mask(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).mask
		          alloc_pdst(al_idx):= rename_stage.io.int_alloc_pregs(al_idx).preg 
		          can_alloc(al_idx) := rename_stage.io.int_alloc_pregs(al_idx).can_alloc
		       }
		       .otherwise
		       {
		          alloc_mask(al_idx):= ~Bits(0, numIntPhysRegsParts)
		          alloc_pdst(al_idx):= UInt(0)
		          can_alloc(al_idx) := true.B
		       }

			   shift_data(al_idx):= wbresp.bits.data

               when (rename_stage.io.int_alloc_pregs(al_idx).valid)
			   {

                    printf("3333: valid(%d) = b%b, vreg(%d) = d%d, nums(%d) = d%d, can_alloc(%d)=d%d, alloc_pdst(%d)=d%d, alloc_mask(%d)=b%b\n", 
	                al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).valid,
	             	al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).vreg,
	             	al_idx.asUInt(), rename_stage.io.int_alloc_pregs(al_idx).nums,
                  	al_idx.asUInt(), can_alloc(al_idx),
                	al_idx.asUInt(), alloc_pdst(al_idx),
                	al_idx.asUInt(), alloc_mask(al_idx))
               }

               when (rename_stage.io.int_alloc_pregs(al_idx).valid && !can_alloc(al_idx))
		       {
		          printf("error1 b%b\n", can_alloc(al_idx))
		       }
			   //require( !wbIsValid(RT_FIX) || can_alloc(al_idx))
			}

            al_idx += 1
		 }
	  }
   }

   var wu_idx = 0
   var swu_idx = 0
   // loop through each issue-port (exe_units are statically connected to an issue-port)
   for (i <- 0 until exe_units.length)
   {
      if (exe_units(i).is_mem_unit)
      {
         // If Memory, it's the shared long-latency port.
         int_wakeups(wu_idx).valid := ll_wbarb.io.out.fire()
         int_wakeups(wu_idx).bits  := ll_wbarb.io.out.bits
         
		 // yqh
		 int_wakeups(wu_idx).bits.uop.pdst := alloc_pdst(swu_idx)
		 int_wakeups(wu_idx).bits.uop.dst_mask := alloc_mask(swu_idx)

		 //when (int_wakeups(wu_idx).valid)
		 //{
		 //	printf("wakeup 1111: valid(%d) = b%b, vdst(%d) = d%d, pdst(%d) = d%d, dst_mask(%d) = d%d\n", 
		 //   	wu_idx.asUInt(), int_wakeups(wu_idx).valid,
		 //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.vdst,
		 //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.pdst,
		 //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.dst_mask)
		 //}

		 wu_idx += 1
		 swu_idx += 1
      }
      else
      {
         // Fast Wakeup (uses just-issued uops) that have known latencies
         if (exe_units(i).isBypassable)
         {
            int_wakeups(wu_idx).valid := iss_valids(i) &&
                                         iss_uops(i).bypassable &&
                                         iss_uops(i).dst_rtype === RT_FIX &&
                                         iss_uops(i).ldst_val
            int_wakeups(wu_idx).bits.uop := iss_uops(i)
			
		    //when (int_wakeups(wu_idx).valid)
		    //{
		    //	printf("wakeup 3333: valid(%d) = b%b, vdst(%d) = d%d, dst_mask(%d) = d%d\n", 
		    //   	wu_idx.asUInt(), int_wakeups(wu_idx).valid,
			//	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.vdst,
		    //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.dst_mask)
		    //}

            wu_idx += 1
            assert (!(iss_uops(i).dst_rtype === RT_FLT && iss_uops(i).bypassable), "Bypassing FP is not supported.")
         
		 }

         // Slow Wakeup (uses write-port to register file)
         for (j <- 0 until exe_units(i).num_rf_write_ports)
         {
            val resp = exe_units(i).io.resp(j)
            int_wakeups(wu_idx).valid := resp.valid &&
                                         resp.bits.uop.ctrl.rf_wen &&
                                         //resp.bits.uop.bypassable &&
                                         resp.bits.uop.dst_rtype === RT_FIX
            int_wakeups(wu_idx).bits := exe_units(i).io.resp(j).bits

            // yqh
		    int_wakeups(wu_idx).bits.uop.pdst := alloc_pdst(swu_idx)
		    int_wakeups(wu_idx).bits.uop.dst_mask := alloc_mask(swu_idx)
   
		    //when (int_wakeups(wu_idx).valid)
		    //{
		    //	printf("wakeup 2222: valid(%d) = b%b, vdst(%d) = d%d, pdst(%d) = d%d, dst_mask(%d) = d%d\n", 
		    //   	wu_idx.asUInt(), int_wakeups(wu_idx).valid,
			//	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.vdst,
		    //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.pdst,
		    //   	wu_idx.asUInt(), int_wakeups(wu_idx).bits.uop.dst_mask)
		    //}

			wu_idx += 1
			swu_idx += 1
         }
      }
      require (exe_units(i).usesIRF)
   }
   require (wu_idx == num_wakeup_ports)

   for ((renport, intport) <- rename_stage.io.int_wakeups zip int_wakeups)
   {
      renport <> intport
   }
   for ((renport, fpport) <- rename_stage.io.fp_wakeups zip fp_pipeline.io.wakeups)
   {
      renport <> fpport
   }

   rename_stage.io.com_valids := rob.io.commit.valids
   rename_stage.io.com_uops := rob.io.commit.uops
   rename_stage.io.com_rbk_valids := rob.io.commit.rbk_valids

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Dispatch Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   for (w <- 0 until DECODE_WIDTH)
   {
      dis_valids(w)       := rename_stage.io.ren2_mask(w)
      dis_uops(w)         := GetNewUopAndBrMask(rename_stage.io.ren2_uops(w), br_unit.brinfo)
   }


   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Issue Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   require (issue_units.map(_.issue_width).sum == exe_units.length)

   // Input (Dispatch)
   for {
      iu <- issue_units
      w <- 0 until DISPATCH_WIDTH
   }{
      iu.io.dis_valids(w) := dis_valids(w) && dis_uops(w).iqtype === UInt(iu.iqType)
      iu.io.dis_uops(w) := dis_uops(w)

      when (dis_uops(w).uopc === uopSTA && dis_uops(w).lrs2_rtype === RT_FLT) {
         iu.io.dis_uops(w).lrs2_rtype := RT_X
		 iu.io.dis_uops(w).prs_busy := dis_uops(w).prs_busy & "b101".U
		 // yqh
         // iu.io.dis_uops(w).rs2_mask := ~Bits(0, width = numIntPhysRegsParts)
      }
   }

   fp_pipeline.io.dis_valids <> dis_valids
   fp_pipeline.io.dis_uops <> dis_uops

   // Output (Issue)

   val ifpu_idx = exe_units.length-1 // TODO hack; need more disciplined manner to hook up ifpu
   require (exe_units(ifpu_idx).supportedFuncUnits.ifpu)

   var iss_idx = 0
   var iss_cnt = 0
   for (w <- 0 until exe_units.length)
   {
      iss_valids(w) := issue_units(iss_idx).io.iss_valids(iss_cnt)
      iss_uops(w)   := issue_units(iss_idx).io.iss_uops(iss_cnt)

      var fu_types = exe_units(w).io.fu_types

      if (w == ifpu_idx) {
         // TODO hack, need a more disciplined way to connect to an issue port
         // TODO XXX need to also apply back-pressure.
         fu_types = fu_types | FUConstants.FU_I2F
      }

      if (exe_units(w).supportedFuncUnits.muld && regreadLatency > 0)
      {
         // Supress just-issued divides from issuing back-to-back, since it's an iterative divider.
         // But it takes a cycle to get to the Exe stage, so it can't tell us it is busy yet.
         val idiv_issued = iss_valids(w) && iss_uops(w).fu_code_is(FU_DIV)
         fu_types = fu_types & RegNext(~Mux(idiv_issued, FU_DIV, Bits(0)))
      }
      require (!exe_units(w).supportedFuncUnits.fdiv)

      issue_units(iss_idx).io.fu_types(iss_cnt) := fu_types

      // TODO this is super fragile -- check the issue-units match the exe-units on instruction types.
      require ((issue_units(iss_idx).iqType == IQT_MEM.litValue) ^ !exe_units(w).is_mem_unit)
      require (issueParams(iss_idx).iqType != IQT_FP.litValue)

      iss_cnt += 1
      val iwidths = issueParams.map(_.issueWidth)
      if (iss_cnt >= iwidths(iss_idx)) {
         iss_idx += 1
         iss_cnt = 0
      }
   }


   issue_units.map(_.io.tsc_reg := debug_tsc_reg)
   issue_units.map(_.io.brinfo := br_unit.brinfo)
   issue_units.map(_.io.flush_pipeline := rob.io.flush.valid)

   // Wakeup (Issue & Writeback)

   // yqh debug

   for {
      iu <- issue_units
      (issport, wakeup) <- iu.io.wakeup_vdsts zip int_wakeups
   }{
      issport.valid := wakeup.valid
      issport.bits  := wakeup.bits.uop.vdst

      //printf ("core.wakeup: issport.valid = b%b, vdst = d%d\n", issport.valid, issport.bits)

      require (iu.io.wakeup_vdsts.length == int_wakeups.length)
   }

   for {iu <- issue_units
        (pdst, wakeup) <- iu.io.wakeup_pdsts zip int_wakeups
   }{
      pdst    := wakeup.bits.uop.pdst
      //printf ("core.wakeup: issport.pdst = d%d\n", pdst)
   }

   for {iu <- issue_units
        (mask, wakeup) <- iu.io.wakeup_masks zip int_wakeups
   }{
      mask    := wakeup.bits.uop.dst_mask
	  //printf ("core.wakeup: issport.dst_mask = d%d\n", mask)
   }
   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Register Read Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // Register Read <- Issue (rrd <- iss)
   iregister_read.io.rf_read_ports <> iregfile.io.read_ports

   iregister_read.io.iss_valids <> iss_valids
   iregister_read.io.iss_uops := iss_uops

   iregister_read.io.brinfo := br_unit.brinfo
   iregister_read.io.kill   := rob.io.flush.valid

   iregister_read.io.bypass := bypasses

   //-------------------------------------------------------------
   // Privileged Co-processor 0 Register File
   // Note: Normally this would be bad in that I'm writing state before
   // committing, so to get this to work I stall the entire pipeline for
   // CSR instructions so I never speculate these instructions.

   val csr_exe_unit = exe_units.csr_unit

   // for critical path reasons, we aren't zero'ing this out if resp is not valid
   val csr_rw_cmd = csr_exe_unit.io.resp(0).bits.uop.ctrl.csr_cmd
   val wb_wdata = csr_exe_unit.io.resp(0).bits.data

   csr.io.rw.addr  := csr_exe_unit.io.resp(0).bits.uop.csr_addr
   csr.io.rw.cmd   := Mux(csr_exe_unit.io.resp(0).valid, csr_rw_cmd, rocket.CSR.N)
   csr.io.rw.wdata :=wb_wdata

   // Extra I/O
   csr.io.retire    := PopCount(rob.io.commit.valids.toBits)
   csr.io.exception := rob.io.com_xcpt.valid && !csr.io.csr_xcpt
   csr.io.pc        := rob.io.com_xcpt.bits.pc
   csr.io.cause     := rob.io.com_xcpt.bits.cause
   csr.io.badaddr   := rob.io.com_xcpt.bits.badvaddr


   // reading requires serializing the entire pipeline
   csr.io.fcsr_flags.valid := rob.io.commit.fflags.valid
   csr.io.fcsr_flags.bits  := rob.io.commit.fflags.bits

   exe_units.map(_.io.fcsr_rm := csr.io.fcsr_rm)
   fp_pipeline.io.fcsr_rm := csr.io.fcsr_rm

   csr.io.hartid := io.hartid
   csr.io.interrupts := io.interrupts

// TODO can we add this back in, but handle reset properly and save us the mux above on csr.io.rw.cmd?
//   assert (!(csr_rw_cmd =/= rocket.CSR.N && !exe_units(0).io.resp(0).valid), "CSRFile is being written to spuriously.")


   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Execute Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   var idx = 0
   for (w <- 0 until exe_units.length)
   {
      exe_units(w).io.req <> iregister_read.io.exe_reqs(w)
      exe_units(w).io.brinfo := br_unit.brinfo
      exe_units(w).io.com_exception := rob.io.flush.valid

      if (exe_units(w).isBypassable)
      {
         for (i <- 0 until exe_units(w).numBypassPorts)
         {
            bypasses.valid(idx) := exe_units(w).io.bypass.valid(i)
            bypasses.uop(idx)   := exe_units(w).io.bypass.uop(i)
            bypasses.data(idx)  := exe_units(w).io.bypass.data(i)
            idx = idx + 1
         }
      }

   }
   require (idx == exe_units.num_total_bypass_ports)


   // don't send IntToFP moves to integer execution units.
   when (iregister_read.io.exe_reqs(ifpu_idx).bits.uop.fu_code === FUConstants.FU_I2F) {
      exe_units(ifpu_idx).io.req.valid := Bool(false)
   }
   fp_pipeline.io.fromint := iregister_read.io.exe_reqs(ifpu_idx)
   fp_pipeline.io.fromint.valid :=
      iregister_read.io.exe_reqs(ifpu_idx).valid &&
      iregister_read.io.exe_reqs(ifpu_idx).bits.uop.fu_code === FUConstants.FU_I2F

   fp_pipeline.io.brinfo := br_unit.brinfo


   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Load/Store Unit ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // enqueue basic load/store info in Decode
   lsu.io.dec_uops := dec_uops

   for (w <- 0 until DECODE_WIDTH)
   {
      lsu.io.dec_st_vals(w) := dec_will_fire(w) && rename_stage.io.inst_can_proceed(w) && !rob.io.flush.valid &&
                               dec_uops(w).is_store
      lsu.io.dec_ld_vals(w) := dec_will_fire(w) && rename_stage.io.inst_can_proceed(w) && !rob.io.flush.valid &&
                               dec_uops(w).is_load

      lsu.io.dec_uops(w).rob_idx := dec_uops(w).rob_idx // for debug purposes (comit logging)
   }

   lsu.io.commit_store_mask := rob.io.commit.st_mask
   lsu.io.commit_load_mask  := rob.io.commit.ld_mask
   lsu.io.commit_load_at_rob_head := rob.io.com_load_is_at_rob_head

   //com_xcpt.valid comes too early, will fight against a branch that resolves same cycle as an exception
   lsu.io.exception := rob.io.flush.valid

   // Handle Branch Mispeculations
   lsu.io.brinfo := br_unit.brinfo
   dc_shim.io.core.brinfo := br_unit.brinfo

   new_ldq_idx := lsu.io.new_ldq_idx
   new_stq_idx := lsu.io.new_stq_idx

   lsu.io.debug_tsc := debug_tsc_reg

   dc_shim.io.core.flush_pipe := rob.io.flush.valid

   lsu.io.nack <> dc_shim.io.core.nack

   lsu.io.dmem_req_ready := dc_shim.io.core.req.ready
   lsu.io.dmem_is_ordered:= dc_shim.io.core.ordered

   lsu.io.fp_stdata <> fp_pipeline.io.tosdq


   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Writeback Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------


   var w_cnt = 0
   var llidx = -1 // find which rf port corresponds to the long latency memory port.
   for (i <- 0 until exe_units.length)
   {
      for (j <- 0 until exe_units(i).num_rf_write_ports)
      {
         val wbresp = exe_units(i).io.resp(j)

		 val wbpdst = alloc_pdst(w_cnt)
		 val wbmask = alloc_mask(w_cnt)
		 val wbdata = shift_data(w_cnt)
		 //yangqinghong
		 val wbvdst = wbresp.bits.uop.vdst

         def wbIsValid(rtype: UInt) =
            wbresp.valid && wbresp.bits.uop.ctrl.rf_wen && wbresp.bits.uop.dst_rtype === rtype
         val wbReadsCSR = wbresp.bits.uop.ctrl.csr_cmd =/= rocket.CSR.N

         if (exe_units(i).data_width > 64)
         {
				require (exe_units(i).is_mem_unit)
            assert (!(wbIsValid(RT_FIX) && exe_units(i).io.resp(j).bits.data(64).toBool),
               "the 65th bit was set on a fixed point write-back to the regfile.")
         }

         if (exe_units(i).uses_csr_wport && (j == 0))
         {
            iregfile.io.write_ports(w_cnt).valid     := wbIsValid(RT_FIX)
            iregfile.io.write_ports(w_cnt).bits.addr := wbpdst //yangqinghong 1
            iregfile.io.write_ports(w_cnt).bits.mask := wbmask 
            iregfile.io.write_ports(w_cnt).bits.data := wbdata
            wbresp.ready := iregfile.io.write_ports(w_cnt).ready
         }
         else if (exe_units(i).is_mem_unit)
         {
            assert (llidx == -1) // should only hit this once
            require (j == 0) // only support one port on memory unit for now.
            llidx = w_cnt

            // connect to FP pipeline's long latency writeport.
            fp_pipeline.io.ll_wport.valid     := wbIsValid(RT_FLT)
            fp_pipeline.io.ll_wport.bits.uop  := wbresp.bits.uop
            fp_pipeline.io.ll_wport.bits.data := wbresp.bits.data
            fp_pipeline.io.ll_wport.bits.fflags.valid := Bool(false)
         }
         else
         {
            iregfile.io.write_ports(w_cnt).valid     := wbIsValid(RT_FIX)
            iregfile.io.write_ports(w_cnt).bits.addr := wbpdst // yangqinghong 1
            iregfile.io.write_ports(w_cnt).bits.mask := wbmask
            iregfile.io.write_ports(w_cnt).bits.data := wbdata
            wbresp.ready := iregfile.io.write_ports(w_cnt).ready
         }


         if (!exe_units(i).is_mem_unit) {
            assert (!wbIsValid(RT_FLT), "[fppipeline] An FP writeback is being attempted to the Int Regfile.")
         }

         assert (!(exe_units(i).io.resp(j).valid &&
            !exe_units(i).io.resp(j).bits.uop.ctrl.rf_wen &&
            exe_units(i).io.resp(j).bits.uop.dst_rtype === RT_FIX),
            "[fppipeline] An Int writeback is being attempted with rf_wen disabled.")

         if (!exe_units(i).is_mem_unit) {
            assert (!(exe_units(i).io.resp(j).valid &&
               exe_units(i).io.resp(j).bits.uop.ctrl.rf_wen &&
               exe_units(i).io.resp(j).bits.uop.dst_rtype =/= RT_FIX),
               "[fppipeline] writeback being attempted to Int RF with dst != Int type exe_units("+i+").resp("+j+")")
         }

         w_cnt += 1
      }
   }

   // Share the memory port with other long latency operations.
   val mem_unit = exe_units.memory_unit
   require (mem_unit.num_rf_write_ports == 1)
   val mem_resp = mem_unit.io.resp(0)

   ll_wbarb.io.in(0).valid := mem_resp.valid && mem_resp.bits.uop.ctrl.rf_wen && mem_resp.bits.uop.dst_rtype === RT_FIX
   ll_wbarb.io.in(0).bits  := mem_resp.bits

   assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
   ll_wbarb.io.in(1) <> fp_pipeline.io.toint
   iregfile.io.write_ports(llidx) <> WritePort(ll_wbarb.io.out, TPREG_SZ, xLen)
   //yqh
   iregfile.io.write_ports(llidx).bits.addr := alloc_pdst(llidx) // yangqinghong 1
   iregfile.io.write_ports(llidx).bits.mask := alloc_mask(llidx)
   iregfile.io.write_ports(llidx).bits.data := shift_data(llidx)

/*
   val alloc_pdst = Wire(Vec(num_irf_write_ports, UInt(width=TPREG_SZ)))
   val alloc_mask = Wire(Vec(num_irf_write_ports, UInt(width=numIntPhysRegsParts)))
   val shift_data = Wire(Vec(num_irf_write_ports, UInt(width=64)))
*/

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Commit Stage ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // Dispatch
   rob.io.enq_valids := rename_stage.io.ren1_mask
   rob.io.enq_uops   := rename_stage.io.ren1_uops
   rob.io.enq_has_br_or_jalr_in_packet := dec_has_br_or_jalr_in_packet
   rob.io.enq_partial_stall := !dec_rdy && !dec_will_fire(DECODE_WIDTH-1)
   rob.io.enq_new_packet := dec_finished_mask === Bits(0)
   rob.io.debug_tsc := debug_tsc_reg

   assert ((dec_will_fire zip rename_stage.io.ren1_mask map {case(d,r) => d === r}).reduce(_|_),
      "[core] Assumption that dec_will_fire and ren1_mask are equal is being violated.")

   // Writeback
   var cnt = 0
   var f_cnt = 0 // rob fflags port index
   for (eu <- exe_units)
   {
      for ((resp, j) <- eu.io.resp.zipWithIndex)
      {
         val wb_uop = resp.bits.uop

         if (eu.is_mem_unit)
         {
            val ll_uop = ll_wbarb.io.out.bits.uop
            rob.io.wb_resps(cnt).valid := ll_wbarb.io.out.valid && !(ll_uop.is_store && !ll_uop.is_amo)
            rob.io.wb_resps(cnt).bits <> ll_wbarb.io.out.bits
         }
         else
         {
            rob.io.wb_resps(cnt).valid := resp.valid && !(wb_uop.is_store && !wb_uop.is_amo)
            rob.io.wb_resps(cnt).bits <> resp.bits
         }

         // for commit logging...
         rob.io.debug_wb_valids(cnt) := resp.valid &&
                                        wb_uop.ctrl.rf_wen &&
                                        (wb_uop.dst_rtype === RT_FIX || wb_uop.dst_rtype === RT_FLT)

         val data = resp.bits.data
         if (eu.hasFFlags || (eu.is_mem_unit && usingFPU))
         {
            if (eu.hasFFlags)
            {
               rob.io.fflags(f_cnt) <> resp.bits.fflags
               f_cnt += 1
            }
            val unrec_s = hardfloat.fNFromRecFN(8, 24, data)
            val unrec_d = hardfloat.fNFromRecFN(11, 53, data)
            val unrec_out     = Mux(wb_uop.fp_single, Cat(UInt(0,32), unrec_s), unrec_d)
            if (eu.uses_csr_wport && (j == 0))
            {
               rob.io.debug_wb_wdata(cnt) := Mux(wb_uop.ctrl.csr_cmd =/= rocket.CSR.N, csr.io.rw.rdata,
                                             Mux(wb_uop.fp_val && wb_uop.dst_rtype === RT_FLT, unrec_out,
                                                                                               data))
            }
            else
            {
               rob.io.debug_wb_wdata(cnt) := Mux(resp.bits.uop.fp_val, unrec_out, data)
            }
         }
         else
         {
            if (eu.uses_csr_wport && (j == 0))
            {
               rob.io.debug_wb_wdata(cnt) := Mux(wb_uop.ctrl.csr_cmd =/= rocket.CSR.N, csr.io.rw.rdata, data)
            }
            else
            {
               rob.io.debug_wb_wdata(cnt) := data
            }
         }
         cnt += 1
      }
   }

   for (wakeup <- fp_pipeline.io.wakeups)
   {
      rob.io.wb_resps(cnt) <> wakeup
      rob.io.fflags(f_cnt) <> wakeup.bits.fflags
      rob.io.debug_wb_valids(cnt) := Bool(false) // TODO XXX add back commit logging for FP instructions.
      cnt += 1
      f_cnt += 1
   }
   assert (cnt == rob.num_wakeup_ports)


   // branch resolution
   rob.io.brinfo <> br_unit.brinfo

   // branch unit requests PCs and predictions from ROB during register read
   // (fetch PC from ROB cycle earlier than needed for critical path reasons)
   rob.io.get_pc.rob_idx := (if (regreadLatency == 1) RegNext(iss_uops(brunit_idx).rob_idx)
                            else iss_uops(brunit_idx).rob_idx)
   exe_units(brunit_idx).io.get_rob_pc.curr_pc        := RegNext(rob.io.get_pc.curr_pc)
   exe_units(brunit_idx).io.get_rob_pc.curr_brob_idx  := RegNext(rob.io.get_pc.curr_brob_idx)
   exe_units(brunit_idx).io.get_rob_pc.next_val       := RegNext(rob.io.get_pc.next_val)
   exe_units(brunit_idx).io.get_rob_pc.next_pc        := RegNext(rob.io.get_pc.next_pc)
   exe_units(brunit_idx).io.status := csr.io.status

   // LSU <> ROB
   rob.io.lsu_clr_bsy_valid   := lsu.io.lsu_clr_bsy_valid
   rob.io.lsu_clr_bsy_rob_idx := lsu.io.lsu_clr_bsy_rob_idx
   rob.io.lxcpt <> lsu.io.xcpt

   rob.io.cxcpt.valid := csr.io.csr_xcpt
   rob.io.csr_eret := csr.io.eret
   rob.io.csr_evec := csr.io.evec

   rob.io.bxcpt <> br_unit.xcpt

   bpd_stage.io.brob.deallocate <> rob.io.brob_deallocate
   bpd_stage.io.brob.bpd_update <> br_unit.bpd_update
   bpd_stage.io.brob.flush := rob.io.flush.valid || rob.io.clear_brob

   //-------------------------------------------------------------
   // **** Flush Pipeline ****
   //-------------------------------------------------------------
   // flush on exceptions, miniexeptions, and after some special instructions

   fp_pipeline.io.flush_pipeline := rob.io.flush.valid
   for (w <- 0 until exe_units.length)
   {
      exe_units(w).io.req.bits.kill := rob.io.flush.valid
   }

   assert (!(RegNext(rob.io.com_xcpt.valid) && !rob.io.flush.valid),
      "[core] exception occurred, but pipeline flush signal not set!")

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Outputs to the External World ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // detect pipeline freezes and throw error
   val idle_cycles = util.WideCounter(32)
   when (rob.io.commit.valids.toBits.orR || reset.toBool) { idle_cycles := UInt(0) }
   assert (!(idle_cycles.value(13)), "Pipeline has hung.")

   fp_pipeline.io.debug_tsc_reg := debug_tsc_reg

   //-------------------------------------------------------------
   // Uarch Hardware Performance Events (HPEs)

   csr.io.events.map(_ := UInt(0))

   require (nPerfEvents > 29)
   println ("   " + nPerfCounters + " HPM counters enabled (with " + nPerfEvents + " events).")

   // Execution-time branch prediction accuracy.
   csr.io.events(0) := br_unit.brinfo.valid
   csr.io.events(1) := br_unit.brinfo.mispredict

   // User-level instruction count.
   csr.io.events(2) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && (csr.io.status.prv === UInt(rocket.PRV.U))})

   // L1 cache stats.
   // TODO add back in cache-miss counters.
//   csr.io.events(3) := io.dmem.acquire // D$ miss
//   csr.io.events(4) := io.imem.acquire // I$ miss

   csr.io.events(5)  := csr.io.status.prv === UInt(rocket.PRV.U)

   // Instruction mixes.
   csr.io.events(6)  := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal})
   csr.io.events(7)  := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_jal})
   csr.io.events(8)  := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_jump && !rob.io.commit.uops(w).is_jal})
   csr.io.events(9)  := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_load})
   csr.io.events(10) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_store})
   csr.io.events(11) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).fp_val})

   // Decode stall causes.
   csr.io.events(12) := !rob.io.ready
   csr.io.events(13) := lsu.io.laq_full
   csr.io.events(14) := lsu.io.stq_full
   csr.io.events(15) := !dis_readys.toBools.reduce(_&_) // issue queues
   csr.io.events(16) := branch_mask_full.reduce(_|_)
   csr.io.events(17) := rob.io.flush.valid

   // LSU Speculation stats.
   csr.io.events(18) := lsu.io.counters.ld_valid
   csr.io.events(19) := lsu.io.counters.stld_order_fail
   csr.io.events(20) := lsu.io.counters.ldld_order_fail

   // Branch prediction stats.
   csr.io.events(21)  := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_brjmp_mispredicted})
   csr.io.events(22) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_btb_made_pred})
   csr.io.events(23) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_btb_mispredicted})
   csr.io.events(24) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_bpd_made_pred})
   csr.io.events(25) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_bpd_mispredicted})

   // Branch prediction - no prediction made.
   csr.io.events(26) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      !rob.io.commit.uops(w).stat_btb_made_pred && !rob.io.commit.uops(w).stat_bpd_made_pred})

   // Branch prediction - no predition made & a mispredict occurred.
   csr.io.events(27) := PopCount((Range(0,COMMIT_WIDTH)).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && !rob.io.commit.uops(w).is_jal &&
      !rob.io.commit.uops(w).stat_btb_made_pred && !rob.io.commit.uops(w).stat_bpd_made_pred &&
      rob.io.commit.uops(w).stat_brjmp_mispredicted})


   // Count user-level branches (subtract from total to get privilege branch accuracy)
   csr.io.events(28) := br_unit.brinfo.valid && (csr.io.status.prv === UInt(rocket.PRV.U))
   csr.io.events(29) := br_unit.brinfo.mispredict && (csr.io.status.prv === UInt(rocket.PRV.U))

   // count change of privilege modes
   csr.io.events(30) := csr.io.status.prv =/= RegNext(csr.io.status.prv)

   csr.io.events(31) := !issue_units(0).io.dis_readys.reduce(_&_)
   csr.io.events(32) := !issue_units(1).io.dis_readys.reduce(_&_)
   csr.io.events(33) := !fp_pipeline.io.dis_readys.reduce(_&_)

   assert (!(Range(0,COMMIT_WIDTH).map{w =>
      rob.io.commit.valids(w) && rob.io.commit.uops(w).is_br_or_jmp && rob.io.commit.uops(w).is_jal &&
      rob.io.commit.uops(w).stat_brjmp_mispredicted}.reduce(_|_)),
      "[dpath] A committed JAL was marked as having been mispredicted.")

   // Count issued instructions (only integer currently).
   require (log2Ceil(1+iss_valids.length) <= csr.io.events(0).getWidth) // CSR.scala sets increment width.
   csr.io.events(34) := PopCount(iss_valids)

   // Count not-issued slots due to empty issue windows (only integer currently).
   val not_issued_and_empty = for {iu <- issue_units; iss_valid <- iu.io.iss_valids} yield {
         !iss_valid && iu.io.event_empty }
   csr.io.events(35) := PopCount(not_issued_and_empty)

   // Count not-issued slots due to backend hazards/unsatisified dependencies (only integer currently).
   val not_issued_and_not_empty = for {iu <- issue_units; iss_valid <- iu.io.iss_valids} yield {
         !iss_valid && !iu.io.event_empty}
   csr.io.events(36) := PopCount(not_issued_and_not_empty)


   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // **** Handle Cycle-by-Cycle Printouts ****
   //-------------------------------------------------------------
   //-------------------------------------------------------------

   if (false) {
      // 每个有效操作数
	  // busy=0
	  // mask=0
	  // bypass口一定能看到
      // dis_uops
      // bypasses.valid uop

	  //when (debug_tsc_reg < 150.U)
	  //{
         for (i <- 0 until DECODE_WIDTH)
	     {
		    when (dis_valids(i))
			{
		       printf("Dispatch: pc:x%x [[DASM(%x)] OP1_Int:%b, vop1 = d%d, DST_INT:%b, vdst:%d, cycles=%d\n", 
			   dis_uops(i).pc(31,0), dis_uops(i).inst, 
			   dis_uops(i).lrs1_rtype === RT_FIX, dis_uops(i).vop1,
			   dis_uops(i).dst_rtype === RT_FIX, dis_uops(i).vdst, debug_tsc_reg)
			}
	     }
         
		 for (i <- 0 until exe_units.length)
		 {
		    when (iss_valids(i))
			{
			   printf("Issue: pc:x%x [[DASM(%x)], cycles=%d\n", iss_uops(i).pc(31,0), iss_uops(i).inst, debug_tsc_reg)
			}
		 }
      //}

      //iss_valids(w) 
      //iss_uops(w)   
      /*
	  for (i <- 0 until DECODE_WIDTH)
	  {
	     when (dis_valids(i) && dis_uops(i).lrs1_rtype === RT_FIX && dis_uops(i).prs_busy(0) === 0.U && dis_uops(i).rs1_mask === 0.U)
		 {
		    printf("------------------------yqh------------------------------\n")
			printf("[[DASM(%x)] \n", dis_uops(i).inst)
			printf("dis_valid = b%b, prs_busy = b%b, mask = b%b\n", dis_valids(i), dis_uops(i).prs_busy, dis_uops(i).rs1_mask)
			printf("dis_uop.vop1 = d%d, cycle = d%d\n", dis_uops(i).vop1, debug_tsc_reg)
			printf("iss(0).valid = b%b, bypasses(0).uop.vdst = d%d\n", iss_valids(0), iss_uops(0).vdst)
			printf("iss(1).valid = b%b, bypasses(1).uop.vdst = d%d\n", iss_valids(1), iss_uops(1).vdst)
			printf("iss(2).valid = b%b, bypasses(2).uop.vdst = d%d\n", iss_valids(2), iss_uops(2).vdst)

			printf("bypasses(0).valid = b%b, bypasses(0).uop.vdst = d%d\n", bypasses.valid(0), bypasses.uop(0).vdst)
			printf("bypasses(1).valid = b%b, bypasses(1).uop.vdst = d%d\n", bypasses.valid(1), bypasses.uop(1).vdst)
			printf("bypasses(2).valid = b%b, bypasses(2).uop.vdst = d%d\n", bypasses.valid(2), bypasses.uop(2).vdst)
			printf("bypasses(3).valid = b%b, bypasses(3).uop.vdst = d%d\n", bypasses.valid(3), bypasses.uop(3).vdst)
			printf("bypasses(4).valid = b%b, bypasses(4).uop.vdst = d%d\n", bypasses.valid(4), bypasses.uop(4).vdst)
			printf("bypasses(5).valid = b%b, bypasses(5).uop.vdst = d%d\n", bypasses.valid(5), bypasses.uop(5).vdst)
		 }
	  }
	  */

   }

   if (DEBUG_PRINTF)
   {
      println("\n Chisel Printout Enabled\n")

      val numBrobWhitespace = if (DEBUG_PRINTF_BROB) NUM_BROB_ENTRIES else 0
//      val screenheight = 103 - 4 - 10
      val screenheight = 85 - 4 - 10
//      val screenheight = 62-8
       var whitespace = (screenheight - 11 + 3 - NUM_LSU_ENTRIES -
         issueParams.map(_.numEntries).sum - issueParams.length - (NUM_ROB_ENTRIES/COMMIT_WIDTH) - numBrobWhitespace
     )

      println("Whitespace padded: " + whitespace)

      printf("--- Cyc=%d , ----------------- Ret: %d ----------------------------------"
         , debug_tsc_reg
         , debug_irt_reg & UInt(0xffffff))

      for (w <- 0 until DECODE_WIDTH)
      {
         if (w == 0) {
            printf("\n  Dec:  ([0x%x]                        ", dec_uops(w).pc(19,0))
         } else {
            printf("[0x%x]                        ", dec_uops(w).pc(19,0))
         }
      }

      for (w <- 0 until DECODE_WIDTH)
      {
         printf("(%c%c) " + "DASM(%x)" + " |  "
            , Mux(fetched_inst_valid && dec_fbundle.uops(w).valid && !dec_finished_mask(w), Str("v"), Str("-"))
            , Mux(dec_will_fire(w), Str("V"), Str("-"))
            , dec_fbundle.uops(w).inst
            )
      }

      for (w <- 0 until DECODE_WIDTH)
      {
         if (w == 0) {
            printf("\n  Ren:  ([0x%x]                        ", rename_stage.io.ren2_uops(w).pc(19,0))
         } else {
            printf("[0x%x]                        ", rename_stage.io.ren2_uops(w).pc(19,0))
         }
      }

      for (w <- 0 until DECODE_WIDTH)
      {
         printf(" (%c) " + "DASM(%x)" + " |  "
            , Mux(rename_stage.io.ren2_mask(w), Str("V"), Str("-"))
            , rename_stage.io.ren2_uops(w).inst
            )
      }

      printf(") fin(%x)\n", dec_finished_mask)
      for (w <- 0 until DECODE_WIDTH)
      {
         printf("        [ISA:%d,%d,%d,%d] [Phs:%d(%c)%d[%c](%c)%d[%c](%c)%d[%c](%c)] "
            , dis_uops(w).ldst
            , dis_uops(w).lrs1
            , dis_uops(w).lrs2
            , dis_uops(w).lrs3
            , dis_uops(w).vdst
            , Mux(dis_uops(w).dst_rtype   === RT_FIX, Str("X")
              , Mux(dis_uops(w).dst_rtype === RT_X  , Str("-")
              , Mux(dis_uops(w).dst_rtype === RT_FLT, Str("f")
              , Mux(dis_uops(w).dst_rtype === RT_PAS, Str("C"), Str("?")))))
            , dis_uops(w).vop1
            , Mux(rename_stage.io.ren2_uops(w).rs1_mask === Bits(0), Str("B"), Str("R"))
            , Mux(dis_uops(w).lrs1_rtype    === RT_FIX, Str("X")
               , Mux(dis_uops(w).lrs1_rtype === RT_X  , Str("-")
               , Mux(dis_uops(w).lrs1_rtype === RT_FLT, Str("f")
               , Mux(dis_uops(w).lrs1_rtype === RT_PAS, Str("C"), Str("?")))))
            , dis_uops(w).vop2
            , Mux(rename_stage.io.ren2_uops(w).rs2_mask === Bits(0), Str("B"), Str("R"))
            , Mux(dis_uops(w).lrs2_rtype    === RT_FIX, Str("X")
               , Mux(dis_uops(w).lrs2_rtype === RT_X  , Str("-")
               , Mux(dis_uops(w).lrs2_rtype === RT_FLT, Str("f")
               , Mux(dis_uops(w).lrs2_rtype === RT_PAS, Str("C"), Str("?")))))
            , dis_uops(w).vop3
			, Mux(rename_stage.io.ren2_uops(w).rs3_mask === Bits(0), Str("B"), Str("R"))
            , Mux(dis_uops(w).frs3_en, Str("f"), Str("-"))
            )
      }

      if (DEBUG_PRINTF_ROB)
      {
         printf("\n) ctate: (%c: %c %c %c %c %c %c) BMsk:%x Mode:%c\n"
         , Mux(rob.io.debug.state === UInt(0), Str("R"),
           Mux(rob.io.debug.state === UInt(1), Str("N"),
           Mux(rob.io.debug.state === UInt(2), Str("B"),
           Mux(rob.io.debug.state === UInt(3), Str("W"),
                                               Str(" ")))))
         , Mux(rob.io.ready,Str("_"), Str("!"))
         , Mux(lsu.io.laq_full, Str("L"), Str("_"))
         , Mux(lsu.io.stq_full, Str("S"), Str("_"))
         , Mux(rob.io.flush.valid, Str("F"), Str(" "))
         , Mux(branch_mask_full.reduce(_|_), Str("B"), Str(" "))
         , Mux(dc_shim.io.core.req.ready, Str("R"), Str("B"))
         , dec_brmask_logic.io.debug.branch_mask
         , Mux(csr.io.status.prv === Bits(0x3), Str("M"),
           Mux(csr.io.status.prv === Bits(0x0), Str("U"),
           Mux(csr.io.status.prv === Bits(0x1), Str("S"),  //2 is H
                                                 Str("?"))))
         )
      }

      printf("Exct(%c%d) Commit(%x) fl: 0x%x (%d) is: 0x%x (%d)\n"
         , Mux(rob.io.com_xcpt.valid, Str("E"), Str("-"))
         , rob.io.com_xcpt.bits.cause
         , rob.io.commit.valids.toBits
         , rename_stage.io.debug.i_vfreelist
         , PopCount(rename_stage.io.debug.i_vfreelist)
         , rename_stage.io.debug.iisprlist
         , PopCount(rename_stage.io.debug.iisprlist)
         )

      printf("                                      fl: 0x%x (%d) is: 0x%x (%d)\n"
         , rename_stage.io.debug.f_vfreelist
         , PopCount(rename_stage.io.debug.f_vfreelist)
         , rename_stage.io.debug.fisprlist
         , PopCount(rename_stage.io.debug.fisprlist)
         )

      // branch unit
//      printf("                          Branch Unit: %c,%c,%d PC=0x%x, %d Targ=0x%x NPC=%d,0x%x %d%d\n"
      printf("                          Branch Unit: %c,%c,%d  NPC=%d,0x%x\n"
         , Mux(br_unit.brinfo.valid,Str("V"), Str(" "))
         , Mux(br_unit.brinfo.mispredict, Str("M"), Str(" "))
         , br_unit.brinfo.taken
//         , br_unit.btb_update.bits.br_pc(19,0)
//         , br_unit.btb_update.valid
//         , br_unit.btb_update.bits.target(19,0)
         , exe_units(brunit_idx).io.get_rob_pc.next_val
         , exe_units(brunit_idx).io.get_rob_pc.next_pc(19,0)
//         , br_unit.btb_update.isJump
//         , br_unit.btb_update.isReturn
      )

      // Rename Map Tables / ISA Register File
      val xpr_to_string =
              Vec(Str(" x0"), Str(" ra"), Str(" sp"), Str(" gp"),
                   Str(" tp"), Str(" t0"), Str(" t1"), Str(" t2"),
                   Str(" s0"), Str(" s1"), Str(" a0"), Str(" a1"),
                   Str(" a2"), Str(" a3"), Str(" a4"), Str(" a5"),
                   Str(" a6"), Str(" a7"), Str(" s2"), Str(" s3"),
                   Str(" s4"), Str(" s5"), Str(" s6"), Str(" s7"),
                   Str(" s8"), Str(" s9"), Str("s10"), Str("s11"),
                   Str(" t3"), Str(" t4"), Str(" t5"), Str(" t6"))

      val fpr_to_string =
              Vec( Str("ft0"), Str("ft1"), Str("ft2"), Str("ft3"),
                   Str("ft4"), Str("ft5"), Str("ft6"), Str("ft7"),
                   Str("fs0"), Str("fs1"), Str("fa0"), Str("fa1"),
                   Str("fa2"), Str("fa3"), Str("fa4"), Str("fa5"),
                   Str("fa6"), Str("fa7"), Str("fs2"), Str("fs3"),
                   Str("fs4"), Str("fs5"), Str("fs6"), Str("fs7"),
                   Str("fs8"), Str("fs9"), Str("fs10"), Str("fs11"),
                   Str("ft8"), Str("ft9"), Str("ft10"), Str("ft11"))

      for (x <- 0 until whitespace)
      {
         printf("|\n")
      }
   } // End DEBUG_PRINTF



   if (COMMIT_LOG_PRINTF)
   {
      var new_commit_cnt = UInt(0)
      for (w <- 0 until COMMIT_WIDTH)
      {
         val priv = csr.io.status.prv

         when (rob.io.commit.valids(w))
         {
            when (rob.io.commit.uops(w).dst_rtype === RT_FIX && rob.io.commit.uops(w).ldst =/= UInt(0))
            {
               printf("%d 0x%x (0x%x) x%d 0x%x\n",
                  priv, Sext(rob.io.commit.uops(w).pc(vaddrBits,0), xLen), rob.io.commit.uops(w).inst,
                  rob.io.commit.uops(w).inst(RD_MSB,RD_LSB), rob.io.commit.uops(w).debug_wdata)
            }
            .elsewhen (rob.io.commit.uops(w).dst_rtype === RT_FLT)
            {
               printf("%d 0x%x (0x%x) f%d 0x%x\n",
                  priv, Sext(rob.io.commit.uops(w).pc(vaddrBits,0), xLen), rob.io.commit.uops(w).inst,
                  rob.io.commit.uops(w).inst(RD_MSB,RD_LSB), rob.io.commit.uops(w).debug_wdata)
            }
            .otherwise
            {
               printf("%d 0x%x (0x%x)\n",
                  priv, Sext(rob.io.commit.uops(w).pc(vaddrBits,0), xLen), rob.io.commit.uops(w).inst)
            }
         }
      }
   }

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // Pipeview Visualization

   if (O3PIPEVIEW_PRINTF)
   {
      println("   O3Pipeview Visualization Enabled\n")

      // did we already print out the instruction sitting at the front of the fetchbuffer/decode stage?
      val dec_printed_mask = Reg(init = Bits(0, DECODE_WIDTH))

      for (w <- 0 until DECODE_WIDTH)
      {
         when (dec_valids(w) && !dec_printed_mask(w)) {
            printf("%d; O3PipeView:decode:%d\n", dec_uops(w).debug_events.fetch_seq, debug_tsc_reg)
         }
         // Rename begins when uop leaves fetch buffer (Dec+Ren1 are in same stage).
         when (dec_will_fire(w)) {
            printf("%d; O3PipeView:rename: %d\n", dec_uops(w).debug_events.fetch_seq, debug_tsc_reg)
         }
         when (dis_valids(w)) {
            printf("%d; O3PipeView:dispatch: %d\n", dis_uops(w).debug_events.fetch_seq, debug_tsc_reg)
         }

         when (dec_rdy || fetch_unit.io.clear_fetchbuffer)
         {
            dec_printed_mask := UInt(0)
         }
         .otherwise
         {
            dec_printed_mask := dec_valids.toBits | dec_printed_mask
         }
      }

      for (i <- 0 until COMMIT_WIDTH)
      {
         when (rob.io.commit.valids(i))
         {
            printf("%d; O3PipeView:retire:%d:store: 0\n",
               rob.io.commit.uops(i).debug_events.fetch_seq,
               debug_tsc_reg)
         }
      }
   }

   //-------------------------------------------------------------
   //-------------------------------------------------------------
   // Page Table Walker

   io.ptw_tlb <> lsu.io.ptw
   io.ptw.ptbr       := csr.io.ptbr
   io.ptw.invalidate := csr.io.fatc
   io.ptw.status     := csr.io.status

   //-------------------------------------------------------------
   //-------------------------------------------------------------

   // we do not support RoCC (yet)
   io.rocc.cmd.valid := Bool(false)
   io.rocc.resp.ready := Bool(false)
}

