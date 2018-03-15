//******************************************************************************
// Rename PFreelist
// 2018.01.17
//------------------------------------------------------------------------------

package boom

import Chisel._
import config.Parameters

class PFreeListIO(
    num_write_ports: Int,
    pl_width: Int,
    num_physical_registers: Int)
    (implicit p: Parameters) extends BoomBundle()(p)
{
    private val preg_sz		= log2Up(num_physical_registers)
    private val part_num_sz	= log2Up(numIntPhysRegsParts)

    val req_preg_vals 		= Vec(num_write_ports, Bool()).asInput
    val req_part_nums		= Vec(num_write_ports, UInt(width=part_num_sz)).asInput
    val req_br_mask 		= Vec(num_write_ports, UInt(width = MAX_BR_COUNT)).asInput

    // do we have space to service incoming requests?
	val can_allocate    	= Vec(num_write_ports, Bool()).asOutput
    val req_pregs			= Vec(num_write_ports, UInt(width=preg_sz)).asOutput
    val req_masks			= Vec(num_write_ports, UInt(width=numIntPhysRegsParts)).asOutput

    // committed and newly freed register
    val enq_vals			= Vec(pl_width, Bool()).asInput
    val enq_pregs			= Vec(pl_width, UInt(width=preg_sz)).asInput
    val enq_masks			= Vec(pl_width, UInt(width=numIntPhysRegsParts)).asInput

    // handle branches (save copy of pfreelist on branch, merge on mispredict)
    val ren_br_vals			= Vec(pl_width, Bool()).asInput
    val ren_br_tags			= Vec(pl_width, UInt(width=BR_TAG_SZ)).asInput

    // handle mispredicts
    val br_mispredict_val 	= Bool(INPUT)
    val br_mispredict_tag 	= UInt(INPUT, BR_TAG_SZ)

    // rollback (on exceptions)
    val rollback_wens		= Vec(pl_width, Bool()).asInput
    val rollback_pdsts		= Vec(pl_width, UInt(width=preg_sz)).asInput
    val rollback_masks		= Vec(pl_width, UInt(width=numIntPhysRegsParts)).asInput

}

class RenamePFreeListHelper(
    num_write_ports: Int,
    pl_width: Int,
    num_physical_registers: Int)
    (implicit p: Parameters) extends BoomModule()(p)
{

	def count_one(a: Bits): UInt = {
	    var x = UInt(0, a.getWidth)
	
	    for(idx <- 0 until a.getWidth) {
		    x = x + a(idx).asUInt() 
	    }
	
	    x
	}
	
	def alloc_parts(a: Bits, n: UInt): Bits = {
		var mask	= Vec.fill(a.getWidth) {Bool(false)}
		var cnt		= n

	    for (idx <- 0 until a.getWidth) 
		{
			val cur = Wire(init = false.B)
			when(cnt =/= 0.U && a(idx) === 1.U)
			{
				cur := true.B
				mask(idx) := true.B
			}
			cnt = cnt - cur.asUInt()
	    }

		mask.asUInt()
	}

	val io = new PFreeListIO(num_write_ports, pl_width, num_physical_registers)

    // ** FREE LIST TABLE **//
	val freelist = Reg(init = ~Bits(0, width = num_physical_registers * numIntPhysRegsParts))
	def free_list(w: Int) = freelist((w+1)*numIntPhysRegsParts-1, w*numIntPhysRegsParts)

    // track all allocations that have occurred since branch passed by
    // can quickly reset pipeline on branch mispredict
    val allocation_lists = Mem(MAX_BR_COUNT, Bits(width = num_physical_registers * numIntPhysRegsParts))

    // ------------------------------------------
    // find new,free physical registers

    var allocated           = Vec.fill(num_write_ports) {Bool(false)}
    var request_pregs		= Vec.fill(num_write_ports) {UInt(0, width=log2Up(num_physical_registers))}
    var request_masks		= Vec.fill(num_write_ports) {Bits(0, width=numIntPhysRegsParts)}

    for (preg_idx <- 1 until num_physical_registers)
    {
        var idle_mask = free_list(preg_idx)
		var idle_size = count_one(idle_mask)
		
		val next_allocated      = Wire(Vec(num_write_ports, Bool()))
		val next_request_pregs  = Wire(Vec(num_write_ports, UInt(width=log2Up(num_physical_registers))))
		val next_request_masks  = Wire(Vec(num_write_ports, Bits(width=numIntPhysRegsParts)))

		for (req_idx <- 0 until num_write_ports)
		{
			next_allocated(req_idx) := allocated(req_idx)
			next_request_pregs(req_idx) := request_pregs(req_idx)
			next_request_masks(req_idx) := request_masks(req_idx)

	   		val nums = io.req_part_nums(req_idx)

			val alloc_nums = Wire(init = UInt(0, width=numIntPhysRegsParts))
			val alloc_mask = Wire(init = Bits(0, width=numIntPhysRegsParts))

	    	when (io.req_preg_vals(req_idx) && !allocated(req_idx) && idle_size >= nums)
	    	{
				next_request_pregs(req_idx)     := preg_idx.asUInt()
				next_request_masks(req_idx)     := alloc_parts(idle_mask, nums)
				next_allocated(req_idx)         := true.B

				alloc_nums                      := nums
				alloc_mask                      := next_request_masks(req_idx)
	    	}

			idle_size = idle_size - alloc_nums
			idle_mask = idle_mask & ~alloc_mask
		}

		allocated       = next_allocated
		request_pregs   = next_request_pregs
		request_masks   = next_request_masks
    }

	var req_free_list	= Bits(0, width = num_physical_registers * numIntPhysRegsParts)
	var enq_free_list	= Bits(0, width = num_physical_registers * numIntPhysRegsParts)

	// ** Set requested PREG to "Not Free" ** //
	for (w <- 0 until num_write_ports)
	{
		val cur_preg    = Wire(init = UInt(0, width = log2Up(num_physical_registers)))
		val cur_mask	= Wire(init = Bits(0, width = numIntPhysRegsParts))

		when (allocated(w) === true.B)
		{
			cur_preg	:= request_pregs(w)
			cur_mask	:= request_masks(w)
		}

		req_free_list = req_free_list | cur_mask << (cur_preg * UInt(numIntPhysRegsParts))
	}	

	// ** Set enqueued PREG to "Free" ** //
	for (w <- 0 until pl_width)
	{
		val cur_preg    = Wire(init = UInt(0, width = log2Up(num_physical_registers)))
		val cur_mask    = Wire(init = Bits(0, width = numIntPhysRegsParts))

		when (io.enq_vals(w) === true.B)
		{
			cur_preg	:= io.enq_pregs(w)
			cur_mask	:= io.enq_masks(w)
		}
		.elsewhen (io.rollback_wens(w) === true.B)
		{
			cur_preg	:= io.rollback_pdsts(w)
			cur_mask	:= io.rollback_masks(w)
		}

		enq_free_list = enq_free_list | cur_mask << (cur_preg * UInt(numIntPhysRegsParts))
	}

    // ------------------------------------------
    // clear allocation list for ren-branch inst
    
    val is_br_clear = Wire(Vec(MAX_BR_COUNT, Bool()))
	for (i <- 0 until MAX_BR_COUNT) { is_br_clear(i) := Bool(false) }

    for (w <- 0 until pl_width)
    {
        when (io.ren_br_vals(w))
		{
		    allocation_lists(io.ren_br_tags(w)) := Wire(init = Bits(0, width = num_physical_registers * (numIntPhysRegsParts)))
	    	is_br_clear(io.ren_br_tags(w)) := true.B
		}
    }

	// --------------------------------------------------
	// ******put new alloc phys to allocation lists******

	val just_allocation_lists = Wire(Vec(MAX_BR_COUNT, Vec(num_write_ports, Bits(width = num_physical_registers * numIntPhysRegsParts))))

	for (i <- 0 until MAX_BR_COUNT)
	{
		for (j <- 0 until num_write_ports)
		{
			just_allocation_lists(i)(j) := Wire(init = Bits(0, width = num_physical_registers * numIntPhysRegsParts))
		}
	}

	for (w <- 0 until num_write_ports)
    {
		val cur_mask = request_masks(w)
		val cur_preg = request_pregs(w)

		for (idx <- 0 until MAX_BR_COUNT)
		{
			when (allocated(w) === true.B && io.req_br_mask(w)(idx) === true.B)
			{
				just_allocation_lists(idx)(w) := cur_mask << (cur_preg * 4.U)
			}
		}
    }

	for (idx <- 0 until MAX_BR_COUNT)
	{
		when(is_br_clear(idx) === false.B)
		{
			allocation_lists(idx) := allocation_lists(idx) | just_allocation_lists(idx).reduce(_|_)
		}
	}

	// Handle Misprediction
	// merge misspeculated allocation_list with free_list
	val allocation_list = Wire(Bits(width = num_physical_registers * numIntPhysRegsParts))
	allocation_list := allocation_lists(io.br_mispredict_tag)

    // Update the Free List
	when (!io.br_mispredict_val)
	{
		freelist := (freelist & ~req_free_list) | enq_free_list
	}
	.otherwise
	{	
		freelist := (freelist & ~req_free_list) | enq_free_list | allocation_list
	}

	// set other branch allocation_lists to zero where allocation_list(j) == 1...
	when (io.br_mispredict_val)
	{
		for (i <- 0 until MAX_BR_COUNT)
		{
			allocation_lists(i) := allocation_lists(i) & ~allocation_list
		}
	}

	// ** SET OUTPUTS ** //
	io.req_pregs := request_pregs
	io.req_masks := request_masks
	io.can_allocate := allocated

}

class RenamePFreeList(
    num_write_ports: Int,	// number of write back ports
    pl_width: Int,		// pipeline width
    rtype: BigInt,		// type of pfreelist
    num_physical_registers: Int)// number of physical registers
    (implicit p: Parameters) extends BoomModule()(p)
{
    private val preg_sz = TPREG_SZ
	private val part_num_sz= log2Up(numIntPhysRegsParts)

    val io = new Bundle
    {
		val brinfo			= new BrResolutionInfo().asInput
		
		val ren_uops		= Vec(pl_width, new MicroOp()).asInput	
		val ren_br_vals		= Vec(pl_width, Bool()).asInput //当前指令有效，并且是分支指令
    	
		// request allocate physical register
    	val req_preg_vals   = Vec(num_write_ports, Bool()).asInput
		val req_part_nums   = Vec(num_write_ports, UInt(width=part_num_sz)).asInput
		// allocated physical register
		val can_allocate    = Vec(num_write_ports, Bool()).asOutput
		val req_pregs       = Vec(num_write_ports, UInt(width=preg_sz)).asOutput
		val req_masks       = Vec(num_write_ports, UInt(width=numIntPhysRegsParts)).asOutput
    	
		// commit & freed physical registers 
		val enq_vals        = Vec(pl_width, Bool()).asInput
		val enq_pregs       = Vec(pl_width, UInt(width=preg_sz)).asInput
		val enq_masks       = Vec(pl_width, UInt(width=numIntPhysRegsParts)).asInput
		
		// rollback & freed physical registers
		val rollback_wens   = Vec(pl_width, Bool()).asInput
		val rollback_pdsts  = Vec(pl_width, UInt(width=preg_sz)).asInput
		val rollback_masks  = Vec(pl_width, UInt(width=numIntPhysRegsParts)).asInput
    }

    val pfreelist = Module(new RenamePFreeListHelper(
        num_write_ports,
		pl_width,
		num_physical_registers))

    pfreelist.io.br_mispredict_val := io.brinfo.mispredict
    pfreelist.io.br_mispredict_tag := io.brinfo.tag

    pfreelist.io.enq_vals := io.enq_vals
    pfreelist.io.enq_pregs := io.enq_pregs
    pfreelist.io.enq_masks := io.enq_masks
    pfreelist.io.rollback_wens := io.rollback_wens
    pfreelist.io.rollback_pdsts := io.rollback_pdsts
    pfreelist.io.rollback_masks := io.rollback_masks

    for (w <- 0 until pl_width) {
        pfreelist.io.ren_br_vals(w) := io.ren_br_vals(w)
        pfreelist.io.ren_br_tags(w) := io.ren_uops(w).br_tag
    }

    pfreelist.io.req_preg_vals := io.req_preg_vals
    pfreelist.io.req_part_nums := io.req_part_nums

    io.can_allocate := pfreelist.io.can_allocate
    io.req_pregs := pfreelist.io.req_pregs 
    io.req_masks := pfreelist.io.req_masks 

	// 目的逻辑寄存器是0，ldst_val等于false
	// 不会请求分配物理寄存器空间
}
