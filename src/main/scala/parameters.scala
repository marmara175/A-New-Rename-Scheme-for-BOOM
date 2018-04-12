//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
package boom
{

import Chisel._
import rocket._
import tile._
import config.{Parameters, Field}

case object BoomKey extends Field[BoomCoreParams]

case class BoomCoreParams(
   numRobEntries: Int = 16,
   issueParams: Seq[IssueParams] = Seq(
         IssueParams(issueWidth=1, numEntries=16, iqType=IQT_MEM.litValue),
         IssueParams(issueWidth=2, numEntries=16, iqType=IQT_INT.litValue),
         IssueParams(issueWidth=1, numEntries=16, iqType=IQT_FP.litValue)),
   numLsuEntries: Int = 8,
   numIntVPhysRegisters: Int = 96,
   numIntPPhysRegisters: Int = 96,
   numFpVPhysRegisters: Int = 64,
   numFpPPhysRegisters: Int = 64,
   enableCustomRf: Boolean = false,
   enableCustomRfModel: Boolean = true,
   maxBrCount: Int = 4,
   fetchBufferSz: Int = 8,
   enableAgePriorityIssue: Boolean = true,
   enablePrefetching: Boolean = false,
   enableFetchBufferFlowThrough: Boolean = true,
   enableBrResolutionRegister: Boolean = true,
   enableCommitMapTable: Boolean = false,
   enableBTBContainsBranches: Boolean = true,
   enableBIM: Boolean = true,
   enableBranchPredictor: Boolean = false,
   enableBpdUModeOnly: Boolean = false,
   enableBpdUSModeHistory: Boolean = false,
   enableBpdF2Redirect: Boolean = false,
   enableBpdF3Redirect: Boolean = true,
   btb: BTBsaParameters = BTBsaParameters(),
   tage: Option[TageParameters] = None,
   gshare: Option[GShareParameters] = None,
   gskew: Option[GSkewParameters] = None,
   intToFpLatency: Int = 2,
   imulLatency: Int = 3,
   fetchLatency: Int = 3,
   renameLatency: Int = 2,
   regreadLatency: Int = 0//yangqinghong
)

trait HasBoomCoreParameters extends tile.HasCoreParameters
{
   // HACK this is a bit hacky since BoomParams can't extend RocketParams.
   val rocketParams: RocketCoreParams = tileParams.core.asInstanceOf[RocketCoreParams]
   val boomParams: BoomCoreParams = p(BoomKey)
   require(xLen == 64)

   val nPerfCounters    = rocketParams.nPerfCounters
   val nPerfEvents      = rocketParams.nPerfEvents
   //************************************
   // Superscalar Widths
   val FETCH_WIDTH      = rocketParams.fetchWidth       // number of insts we can fetch
   val DECODE_WIDTH     = rocketParams.decodeWidth
   val DISPATCH_WIDTH   = DECODE_WIDTH                // number of insts put into the IssueWindow
   val COMMIT_WIDTH     = rocketParams.retireWidth

   require (DECODE_WIDTH == COMMIT_WIDTH)
   require (DISPATCH_WIDTH == COMMIT_WIDTH)
   require (isPow2(FETCH_WIDTH))
   require (DECODE_WIDTH <= FETCH_WIDTH)

   //************************************
   // Data Structure Sizes
   val NUM_ROB_ENTRIES  = boomParams.numRobEntries     // number of ROB entries (e.g., 32 entries for R10k)
   val NUM_LSU_ENTRIES  = boomParams.numLsuEntries     // number of LD/ST entries
   val MAX_BR_COUNT     = boomParams.maxBrCount        // number of branches we can speculate simultaneously
   val fetchBufferSz    = boomParams.fetchBufferSz     // number of instructions that stored between fetch&decode

   val numIntVPhysRegs  = boomParams.numIntVPhysRegisters // size of the integer virtual register file
   val numIntPPhysRegs  = boomParams.numIntPPhysRegisters // size of the integer physical register file
   val numFpVPhysRegs   = boomParams.numFpVPhysRegisters  // size of the floating point physical register file
   val numFpPPhysRegs   = boomParams.numFpPPhysRegisters   

   val numIntPhysRegsParts = 4

   val enableFetchBufferFlowThrough = boomParams.enableFetchBufferFlowThrough

   //************************************
   // Functional Units
   val usingFDivSqrt = rocketParams.fpu.isDefined && rocketParams.fpu.get.divSqrt

   val mulDivParams = rocketParams.mulDiv.getOrElse(MulDivParams())

   //************************************
   // Pipelining

   val imulLatency = boomParams.imulLatency
   val dfmaLatency = if (rocketParams.fpu.isDefined) rocketParams.fpu.get.dfmaLatency else 3
   val sfmaLatency = if (rocketParams.fpu.isDefined) rocketParams.fpu.get.sfmaLatency else 3
   // All FPU ops padded out to same delay for writeport scheduling.
   require (sfmaLatency == dfmaLatency)

   val intToFpLatency = boomParams.intToFpLatency

   val fetchLatency = boomParams.fetchLatency // how many cycles does fetch occupy?
   require (fetchLatency == 3) // do not currently support changing this
   val renameLatency = boomParams.renameLatency // how many cycles does rename occupy?
   val regreadLatency = boomParams.regreadLatency // how many cycles does rrd occupy?
   require (regreadLatency == 0 || regreadLatency == 1)

   val enableBrResolutionRegister = boomParams.enableBrResolutionRegister

   //************************************
   // Issue Units

   val issueParams: Seq[IssueParams] = boomParams.issueParams
   val enableAgePriorityIssue = boomParams.enableAgePriorityIssue

   // currently, only support one of each.
   require (issueParams.count(_.iqType == IQT_FP.litValue) == 1)
   require (issueParams.count(_.iqType == IQT_MEM.litValue) == 1)
   require (issueParams.count(_.iqType == IQT_INT.litValue) == 1)

   //************************************
   // Load/Store Unit
   val dcacheParams: DCacheParams = tileParams.dcache.get
   val nTLBEntries = dcacheParams.nTLBEntries

   val icacheParams: ICacheParams = tileParams.icache.get

   //************************************
   // Branch Prediction

   val enableBTB = true
   val enableBTBContainsBranches = boomParams.enableBTBContainsBranches
   val enableBIM = boomParams.enableBIM

   val ENABLE_BRANCH_PREDICTOR = boomParams.enableBranchPredictor

   // allow the BPD to redirect the PC in the F2 stage (hurts critical path).
   val enableBpdF2Redirect = boomParams.enableBpdF2Redirect
   val enableBpdF3Redirect = boomParams.enableBpdF3Redirect

   val ENABLE_BPD_UMODE_ONLY = boomParams.enableBpdUModeOnly
   val ENABLE_BPD_USHISTORY = boomParams.enableBpdUSModeHistory
   // What is the maximum length of global history tracked?
   var GLOBAL_HISTORY_LENGTH = 0
   // What is the physical length of the VeryLongHistoryRegister? This must be
   // able to handle the GHIST_LENGTH as well as being able hold all speculative
   // updates well beyond the GHIST_LENGTH (i.e., +ROB_SZ and other buffering).
   var VLHR_LENGTH = 0
   var BPD_INFO_SIZE = 0
   var ENABLE_VLHR = false

   val tageParams = boomParams.tage
   val gshareParams = boomParams.gshare
   val gskewParams = boomParams.gskew

   if (!ENABLE_BRANCH_PREDICTOR)
   {
      BPD_INFO_SIZE = 1
      GLOBAL_HISTORY_LENGTH = 1
   }
   else if (tageParams.isDefined && tageParams.get.enabled)
   {
      GLOBAL_HISTORY_LENGTH = tageParams.get.history_lengths.max
      BPD_INFO_SIZE = TageBrPredictor.GetRespInfoSize(p, fetchWidth)
      ENABLE_VLHR = true
   }
   else if (gskewParams.isDefined && gskewParams.get.enabled)
   {
      GLOBAL_HISTORY_LENGTH = gskewParams.get.history_length
      BPD_INFO_SIZE = GSkewBrPredictor.GetRespInfoSize(p, fetchWidth)
   }
   else if (gshareParams.isDefined && gshareParams.get.enabled)
   {
      GLOBAL_HISTORY_LENGTH = gshareParams.get.history_length
      BPD_INFO_SIZE = GShareBrPredictor.GetRespInfoSize(p, GLOBAL_HISTORY_LENGTH)
   }
   else if (p(SimpleGShareKey).enabled)
   {
      GLOBAL_HISTORY_LENGTH = p(SimpleGShareKey).history_length
      BPD_INFO_SIZE = SimpleGShareBrPredictor.GetRespInfoSize(p)
   }
   else if (p(RandomBpdKey).enabled)
   {
      GLOBAL_HISTORY_LENGTH = 1
      BPD_INFO_SIZE = RandomBrPredictor.GetRespInfoSize(p)
   }
   else
   {
      require(!ENABLE_BRANCH_PREDICTOR) // set branch predictor in configs.scala
      BPD_INFO_SIZE = 1
      GLOBAL_HISTORY_LENGTH = 1
   }
   VLHR_LENGTH = GLOBAL_HISTORY_LENGTH+2*NUM_ROB_ENTRIES


   //************************************
   // Extra Knobs and Features
   val ENABLE_COMMIT_MAP_TABLE = boomParams.enableCommitMapTable

   //************************************
   // Implicitly calculated constants
   val NUM_ROB_ROWS      = NUM_ROB_ENTRIES/DECODE_WIDTH
   val ROB_ADDR_SZ       = log2Up(NUM_ROB_ENTRIES)
   // the f-registers are mapped into the space above the x-registers
   val LOGICAL_REG_COUNT = if (usingFPU) 64 else 32
   val LREG_SZ           = log2Up(LOGICAL_REG_COUNT)
   val IVPREG_SZ         = log2Up(numIntVPhysRegs)
   val IPPREG_SZ	 = log2Up(numIntPPhysRegs)
   val FVPREG_SZ         = log2Up(numFpVPhysRegs)
   val FPPREG_SZ         = log2Up(numFpPPhysRegs)
   val VPREG_SZ          = IVPREG_SZ max FVPREG_SZ
   val PPREG_SZ          = IPPREG_SZ max FPPREG_SZ
   val TPREG_SZ          = VPREG_SZ max PPREG_SZ
   val MEM_ADDR_SZ       = log2Up(NUM_LSU_ENTRIES)
   val MAX_ST_COUNT      = (1 << MEM_ADDR_SZ)
   val MAX_LD_COUNT      = (1 << MEM_ADDR_SZ)
   val BR_TAG_SZ         = log2Up(MAX_BR_COUNT)
   val NUM_BROB_ENTRIES  = NUM_ROB_ROWS //TODO explore smaller BROBs
   val BROB_ADDR_SZ      = log2Up(NUM_BROB_ENTRIES)

   val NRR               = 10

   require (numIntVPhysRegs >= (32 + DECODE_WIDTH))
   require (numFpVPhysRegs >= (32 + DECODE_WIDTH))
   require (MAX_BR_COUNT >=2)
   require (NUM_ROB_ROWS % 2 == 0)
   require (NUM_ROB_ENTRIES % DECODE_WIDTH == 0)
   require (isPow2(NUM_LSU_ENTRIES))
   require ((NUM_LSU_ENTRIES-1) > DECODE_WIDTH)


   //************************************
   // Custom Logic

   val enableCustomRf      = boomParams.enableCustomRf
   val enableCustomRfModel = boomParams.enableCustomRfModel

   //************************************
   // Non-BOOM parameters

   val corePAddrBits = paddrBits
   val corePgIdxBits = pgIdxBits
}


}
