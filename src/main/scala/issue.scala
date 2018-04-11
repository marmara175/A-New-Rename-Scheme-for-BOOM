//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom

import Chisel._
import config.Parameters

import FUConstants._
import util.Str

import scala.collection.mutable.ArrayBuffer

//------------------------------across_rb_state-------------------------------
//-------------------------------------------------------------

case class IssueParams(
   issueWidth: Int = 1,
   numEntries: Int = 8,
   iqType: BigInt
)

class IssueUnitIO(issue_width: Int, num_wakeup_ports: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val dis_valids     = Vec(DISPATCH_WIDTH, Bool()).asInput
   val dis_uops       = Vec(DISPATCH_WIDTH, new MicroOp()).asInput
   val dis_readys     = Vec(DISPATCH_WIDTH, Bool()).asOutput

   val iss_valids     = Vec(issue_width, Bool()).asOutput
   val iss_uops       = Vec(issue_width, new MicroOp().asOutput)
   val iss_readys     = Vec(issue_width, Bool()).asInput

   val wakeup_vdsts   = Vec(num_wakeup_ports, Valid(UInt(width=TPREG_SZ))).flip
   val wakeup_pdsts   = Vec(num_wakeup_ports, UInt(width=TPREG_SZ)).asInput
   val wakeup_masks   = Vec(num_wakeup_ports, UInt(width=numIntPhysRegsParts)).asInput
   val wakeup_rb_state= Vec(num_wakeup_ports, UInt(2.W)).asInput

   val across_rb_val  = Bool(INPUT)
   val across_rb_state= UInt(2.W).asInput
   val across_rb_vdst = UInt(width = TPREG_SZ).asInput

   // tell the issue unit what each execution pipeline has in terms of functional units
   val fu_types       = Vec(issue_width, Bits(width=FUC_SZ)).asInput

   val brinfo         = new BrResolutionInfo().asInput
   val flush_pipeline = Bool(INPUT)

   val event_empty    = Bool(OUTPUT) // used by HPM events; is the issue unit empty?

   val tsc_reg        = UInt(INPUT, xLen)
}

abstract class IssueUnit(
   val num_issue_slots: Int,
   val issue_width: Int,
   num_wakeup_ports: Int,
   val iqType: BigInt)
   (implicit p: Parameters)
   extends BoomModule()(p)
{
   val io = new IssueUnitIO(issue_width, num_wakeup_ports)

   //-------------------------------------------------------------
   // Set up the dispatch uops
   // special case "storing" 2 uops within one issue slot.

   val dis_uops = Array.fill(DISPATCH_WIDTH) {Wire(new MicroOp())}
   for (w <- 0 until DISPATCH_WIDTH)
   {
      dis_uops(w) := io.dis_uops(w)
      dis_uops(w).iw_state := s_valid_1
      when ((dis_uops(w).uopc === uopSTA && dis_uops(w).lrs2_rtype === RT_FIX) || dis_uops(w).uopc === uopAMO_AG)
      {
         dis_uops(w).iw_state := s_valid_2
      }
   }

   //-------------------------------------------------------------
   // Issue Table

   val issue_slots = Vec.fill(num_issue_slots) {Module(new IssueSlot(num_wakeup_ports, iqType)).io}

   //for (i <- 0 until num_wakeup_ports)
   //{
   //   printf ("io.wakeup_rb_state(%d) = %d, io.wakeup_vdsts = %d\n", 
   //            i.asUInt, io.wakeup_rb_state(i), io.wakeup_vdsts(i).bits)
   //}
   //printf ("issue_slot(0).rob_idx = d%d, issue_slot(0).state = d%d\n", issue_slots(0).uop.rob_idx, issue_slots(0).debug.state)

   //printf ("across_rb_val = %d, across_rb_state = %d, across_rb_vdst = %d\n", io.across_rb_val, io.across_rb_state, io.across_rb_vdst)

   io.event_empty := !(issue_slots.map(s => s.valid).reduce(_|_))

   //-------------------------------------------------------------

   assert (PopCount(issue_slots.map(s => s.grant)) <= UInt(issue_width), "Issue window giving out too many grants.")

   //-------------------------------------------------------------

   if (O3PIPEVIEW_PRINTF)
   {
      for (i <- 0 until issue_width)
      {
         // only print stores once!
         when (io.iss_valids(i) && io.iss_uops(i).uopc =/= uopSTD)
         {
            printf("%d; O3PipeView:issue: %d\n",
               io.iss_uops(i).debug_events.fetch_seq,
               io.tsc_reg)
         }
      }
   }

   if (DEBUG_PRINTF)
   //if (true)
   //if (iqType != IQT_FP.litValue)
   {
      val typ_str = if (iqType == IQT_INT.litValue) "int"
                    else if (iqType == IQT_MEM.litValue) "mem"
                    else if (iqType == IQT_FP.litValue) " fp"
                    else "unknown"
      for (i <- 0 until num_issue_slots)
      {
         printf("  " + typ_str + "_issue_slot[%d](%c)(Req:%c):wen=%c P:(%c,%c,%c) VALID_OP:(%b, %b, %b) VOP:(%d,%d,%d) POP:(%d,%d,%d) RS_MASK:(%x, %x, %x) PDST:%d VDST:%d DST_MASK:%x %c [[DASM(%x)]" +
               " 0x%x: %d] ri:%d bm=%d imm=0x%x state = d%d rob_idx=%d\n"
            , UInt(i, log2Up(num_issue_slots))
            , Mux(issue_slots(i).valid, Str("V"), Str("-"))
            , Mux(issue_slots(i).request, Str("R"), Str("-"))
            , Mux(issue_slots(i).in_uop.valid, Str("W"),  Str(" "))
            , Mux(issue_slots(i).debug.p1, Str("!"), Str(" "))
            , Mux(issue_slots(i).debug.p2, Str("!"), Str(" "))
            , Mux(issue_slots(i).debug.p3, Str("!"), Str(" "))
            , issue_slots(i).uop.lrs1_rtype =/= RT_X
            , issue_slots(i).uop.lrs2_rtype =/= RT_X
            , issue_slots(i).uop.frs3_en
            , issue_slots(i).uop.vop1
            , issue_slots(i).uop.vop2
            , issue_slots(i).uop.vop3
            , issue_slots(i).uop.pop1
            , issue_slots(i).uop.pop2
            , issue_slots(i).uop.pop3
            , issue_slots(i).uop.rs1_mask
            , issue_slots(i).uop.rs2_mask
            , issue_slots(i).uop.rs3_mask
            , issue_slots(i).uop.pdst
            , issue_slots(i).uop.vdst
            , issue_slots(i).uop.dst_mask
            , Mux(issue_slots(i).uop.dst_rtype === RT_FIX, Str("X"),
              Mux(issue_slots(i).uop.dst_rtype === RT_X, Str("-"),
              Mux(issue_slots(i).uop.dst_rtype === RT_FLT, Str("f"),
              Mux(issue_slots(i).uop.dst_rtype === RT_PAS, Str("C"), Str("?")))))
            , issue_slots(i).uop.inst
            , issue_slots(i).uop.pc(31,0)
            , issue_slots(i).uop.uopc
            , issue_slots(i).uop.rob_idx
            , issue_slots(i).uop.br_mask
            , issue_slots(i).uop.imm_packed
			, issue_slots(i).debug.state
			, issue_slots(i).uop.rob_idx
            )
	  }

      printf("-----------------------------------------------------------------------------------------\n")
   }
}

class IssueUnits(num_wakeup_ports: Int)(implicit val p: Parameters)
   extends HasBoomCoreParameters
   with IndexedSeq[IssueUnit]
{
   //*******************************
   // Instantiate the IssueUnits

   private val iss_units = ArrayBuffer[IssueUnit]()

   //*******************************
   // Act like a collection

   def length = iss_units.length

   def apply(n: Int): IssueUnit = iss_units(n)

   //*******************************
   // Construct.

   require (enableAgePriorityIssue) // unordered is currently unsupported.

//      issue_Units =issueConfigs colect {if iqType=....)
   iss_units += Module(new IssueUnitCollasping(issueParams.find(_.iqType == IQT_MEM.litValue).get, num_wakeup_ports))
   iss_units += Module(new IssueUnitCollasping(issueParams.find(_.iqType == IQT_INT.litValue).get, num_wakeup_ports))

}

