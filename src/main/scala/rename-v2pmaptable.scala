//*********************************************************
//rename v2p map Table
//yangqinghong@pku.edu.cn
//2018.01.13
//---------------------------------------------------------

package boom

import Chisel._
import config.Parameters

class V2PMapTableIo(
    pipeline_width: Int,
    num_vregs: Int,
    num_pregs: Int,
    num_read_ports: Int,
    num_wb_ports: Int,
    mask_sz: Int)
    (implicit p: Parameters) extends BoomBundle()(p)
{
    private val preg_sz = TPREG_SZ
    private val vreg_sz = TPREG_SZ

    val v_rs		= Vec(num_read_ports, UInt(width=vreg_sz)).asInput
    val p_rs		= Vec(num_read_ports, UInt(width=preg_sz)).asOutput
    val masks		= Vec(num_read_ports, UInt(width=mask_sz)).asOutput

    def vrs(i:Int, w:Int):UInt		= v_rs	(w+i*pipeline_width)
    def prs(i:Int, w:Int):UInt		= p_rs	(w+i*pipeline_width)
    def mask(i:Int, w:Int):UInt		= masks	(w+i*pipeline_width)

	val enq_vregs     	= Vec(pipeline_width, UInt(width=vreg_sz)).asInput
	val enq_pregs		= Vec(pipeline_width, UInt(width=preg_sz)).asOutput
	val enq_masks		= Vec(pipeline_width, UInt(width=mask_sz)).asOutput

	val rollback_vdsts 	= Vec(pipeline_width, UInt(width=vreg_sz)).asInput
	val rollback_pdsts	= Vec(pipeline_width, UInt(width=preg_sz)).asOutput
	val rollback_masks	= Vec(pipeline_width, UInt(width=mask_sz)).asOutput

    //clean new virtual register mapping
    val allocated_vdst 	= Vec(pipeline_width, new ValidIO(UInt(width=vreg_sz))).flip

    //build virtual register mapping
    val allocpregs_valids = Vec(num_wb_ports, Bool()).asInput
    val allocpregs_vregs = Vec(num_wb_ports, UInt(width=vreg_sz)).asInput
    val allocpregs_pregs = Vec(num_wb_ports, UInt(width=preg_sz)).asInput
    val allocpregs_masks = Vec(num_wb_ports, UInt(width=mask_sz)).asInput
}

class V2PMapTableHelper(
    pipeline_width: Int,
    num_vregs: Int,
    num_pregs: Int,
    num_read_ports: Int,
    num_wb_ports: Int,
    mask_sz: Int)
    (implicit p: Parameters) extends BoomModule()(p)
{
    private val preg_sz = TPREG_SZ
    private val vreg_sz = TPREG_SZ

    val io = new V2PMapTableIo(pipeline_width, num_vregs, num_pregs, num_read_ports, num_wb_ports, mask_sz)

    val v2p_maptable_pregs = Reg(init = Vec.fill(num_vregs) {UInt(0, preg_sz)})
    val v2p_maptable_masks = Reg(init = Vec.fill(num_vregs) {UInt(0, mask_sz)})

    for (idx <- 0 until pipeline_width)
    {
        when (io.allocated_vdst(idx).valid)
		{
		    val vreg = io.allocated_vdst(idx).bits
			v2p_maptable_masks(vreg) := Wire(init = UInt(0, width=mask_sz))
			v2p_maptable_pregs(vreg) := Wire(init = UInt(0, width=preg_sz))
		}
    }

    for (idx <- 0 until num_wb_ports)
    {
        when (io.allocpregs_valids(idx))
		{
		    val vreg = io.allocpregs_vregs(idx)
		    val preg = io.allocpregs_pregs(idx)
		    val mask = io.allocpregs_masks(idx)

			v2p_maptable_pregs(vreg) := preg
			v2p_maptable_masks(vreg) := mask
		}
    }

    //handle bypassing allocated physical register
    //handle bypassing allocated virtual register
    for (ridx <- 0 until num_read_ports) 
    {
        val r_vreg = io.v_rs(ridx)

        io.p_rs(ridx) := v2p_maptable_pregs(r_vreg)
        io.masks(ridx) := v2p_maptable_masks(r_vreg)

        when (r_vreg === UInt(0))
		{
		   io.p_rs(ridx) := UInt(0, width = preg_sz) 
		   io.masks(ridx) := ~UInt(0, width = mask_sz) 
		}

		for (widx <- 0 until num_wb_ports)
		{
		    val w_valid = io.allocpregs_valids(widx)
		    val w_vreg = io.allocpregs_vregs(widx)
		    val w_preg = io.allocpregs_pregs(widx)
		    val w_mask = io.allocpregs_masks(widx)

		    when (w_valid && (w_vreg === r_vreg))
		    {
		        io.p_rs(ridx) := w_preg
				io.masks(ridx) := w_mask
		    }
		}

		//同一周期的指令，前面指令写、后面指令读
		//分配给前面指令的虚拟寄存器的映射还未清空
		//所以需要旁路逻辑
		for (cidx <- 0 until pipeline_width)
		{
		    val c_valid = io.allocated_vdst(cidx).valid
		    val c_vreg = io.allocated_vdst(cidx).bits

		    when (c_valid && (c_vreg === r_vreg))
		    {
				io.masks(ridx) := UInt(0, width=mask_sz)
		    }
		}
    }

	//ROB发起“回收”或者“回滚”，虚拟寄存器到物理寄存器的映射
	//要么被清除了，要么被建立了新的映射，所以，不要旁路逻辑
	//回滚，流水线一切工作暂停
	//handle enq & rollback vr->pr
	for (ridx <- 0 until pipeline_width)
	{
		val enq_vreg 		= io.enq_vregs(ridx)
		val rollback_vreg 	= io.rollback_vdsts(ridx)

		//enq_pregs enq_masks rollback_pdsts rollback_masks
		io.enq_pregs(ridx)	:= v2p_maptable_pregs(enq_vreg)
		io.enq_masks(ridx)	:= v2p_maptable_masks(enq_vreg)

		io.rollback_pdsts(ridx)	:= v2p_maptable_pregs(rollback_vreg)
		io.rollback_masks(ridx)	:= v2p_maptable_masks(rollback_vreg)
	}
}

class V2PMapTableOutput(
    preg_sz: Int,
    mask_sz: Int) extends Bundle
{
    val prs1 = UInt(width=preg_sz)
    val prs2 = UInt(width=preg_sz)
    val prs3 = UInt(width=preg_sz)

    val prs1_mask = UInt(width=mask_sz)
    val prs2_mask = UInt(width=mask_sz)
    val prs3_mask = UInt(width=mask_sz)
    
	override def cloneType: this.type = new V2PMapTableOutput(preg_sz,mask_sz).asInstanceOf[this.type]
}

class RenameV2PMapTable(
    pl_width: Int,
    rtype: BigInt,
    num_virtual_registers: Int,
    num_physical_registers: Int,
    num_read_ports: Int,
    num_wb_ports: Int
    )(implicit p: Parameters) extends BoomModule()(p)
    with HasBoomCoreParameters
{
    private val vreg_sz = TPREG_SZ
    private val preg_sz = TPREG_SZ
	private val mask_sz = numIntPhysRegsParts 

    val io = new BoomBundle()(p)
    {
        //Inputs
		val ren_will_fire	= Vec(pl_width, Bool()).asInput
		val ren_uops		= Vec(pl_width, new MicroOp()).asInput

		val map_table		= Vec(pl_width, new L2VMapTableOutput(vreg_sz)).asInput

		val com_valids		= Vec(pl_width, Bool()).asInput
		val com_uops		= Vec(pl_width, new MicroOp()).asInput
		val com_rbk_valids	= Vec(pl_width, Bool()).asInput

		val enq_valids     	= Vec(pl_width, Bool()).asOutput
		val enq_pregs		= Vec(pl_width, UInt(width=preg_sz)).asOutput
		val enq_masks		= Vec(pl_width, UInt(width=mask_sz)).asOutput
                                                                        
		val rollback_valids = Vec(pl_width, Bool()).asOutput
		val rollback_pdsts	= Vec(pl_width, UInt(width=preg_sz)).asOutput
		val rollback_masks	= Vec(pl_width, UInt(width=mask_sz)).asOutput

		//allocate physical registers
		val allocpregs_valids	= Vec(num_wb_ports, Bool()).asInput
		val allocpregs_vregs	= Vec(num_wb_ports, UInt(width=vreg_sz)).asInput
		val allocpregs_pregs	= Vec(num_wb_ports, UInt(width=preg_sz)).asInput
		val allocpregs_masks	= Vec(num_wb_ports, UInt(width=mask_sz)).asInput

		//Outputs
		val values	= Vec(pl_width, new V2PMapTableOutput(preg_sz,mask_sz)).asOutput
    }

    val v2p_maptable = Module(new V2PMapTableHelper(
        pipeline_width = pl_width,
		num_vregs = num_virtual_registers,
		num_pregs = num_physical_registers,
		num_read_ports = num_read_ports,
		num_wb_ports = num_wb_ports,
		mask_sz = numIntPhysRegsParts))

    for (w <- 0 until pl_width)
    {
        v2p_maptable.io.vrs(0,w) := io.map_table(w).vrs1
		v2p_maptable.io.vrs(1,w) := io.map_table(w).vrs2

		io.values(w).prs1 := v2p_maptable.io.prs(0,w)
		io.values(w).prs2 := v2p_maptable.io.prs(1,w)

		io.values(w).prs1_mask := v2p_maptable.io.mask(0,w)
		io.values(w).prs2_mask := v2p_maptable.io.mask(1,w) 

		if (rtype == RT_FLT.litValue) 
		{
		    v2p_maptable.io.vrs(2,w) := io.map_table(w).vrs3

		    io.values(w).prs3 := v2p_maptable.io.prs(2,w)
		    io.values(w).prs3_mask := v2p_maptable.io.mask(2,w)
		} 
		else 
		{
		    io.values(w).prs3 := UInt(0, width=preg_sz)
		    io.values(w).prs3_mask := UInt(0, width=numIntPhysRegsParts)
		}

		v2p_maptable.io.allocated_vdst(w).valid := io.ren_will_fire(w) &&
							io.ren_uops(w).ldst_val &&
							io.ren_uops(w).dst_rtype === UInt(rtype)
		v2p_maptable.io.allocated_vdst(w).bits := io.ren_uops(w).vdst//保证ren1就对ren_uops(w).vdst赋值
    }

    v2p_maptable.io.allocpregs_valids := io.allocpregs_valids //???
    v2p_maptable.io.allocpregs_vregs := io.allocpregs_vregs
    v2p_maptable.io.allocpregs_pregs := io.allocpregs_pregs
    v2p_maptable.io.allocpregs_masks := io.allocpregs_masks

	for (w <- 0 until pl_width)
	{
		v2p_maptable.io.enq_vregs(w) := io.com_uops(w).stale_vdst
		io.enq_valids(w):= 	io.com_valids(w) &&
							io.com_uops(w).dst_rtype === UInt(rtype) &&
							(io.com_uops(w).stale_vdst =/= UInt(0) || UInt(rtype) === RT_FLT)
		io.enq_pregs(w) := v2p_maptable.io.enq_pregs(w)
		io.enq_masks(w) := v2p_maptable.io.enq_masks(w)

		v2p_maptable.io.rollback_vdsts(w) := io.com_uops(w).vdst
		io.rollback_valids(w)	:= 	io.com_rbk_valids(w) &&
									(io.com_uops(w).vdst =/= UInt(0) || UInt(rtype) === RT_FLT) &&
									io.com_uops(w).dst_rtype === UInt(rtype)
		io.rollback_pdsts(w) 	:= v2p_maptable.io.rollback_pdsts(w)
		io.rollback_masks(w) 	:= v2p_maptable.io.rollback_masks(w)
	}
}


