//******************************************************************************
// allocate physical register stage
// yangqinghong@pku.edu.cn
// 2018.03.15
//******************************************************************************
/*
package boom

import Chisel._
import config.Parameters

def MyEncode_64(data: UInt, data_size: Int, sub_size: Int): UInt =
{
   val nums = data_size / sub_size
   val sub_datas = Wire(Vec(nums, UInt(sub_size.W)))

   val this_data = Wire(UInt(width=data_size.W))
   this_data := data

   for (i <- 0 until nums)
   {
       val offset = i * sub_size
	   sub_datas(i) := (this_data >> offset)(sub_size-1, 0)
   }

   val sign = sub_datas(nums-1)(sub_size-1).toBool
   val base = Wire(UInt(width=sub_size.W))

   when (sign)
   {
       base := (-1.S(sub_size.W)).asUInt()
   }
   .otherwise
   {
       base := 0.U(sub_size.W)
   }

   var res = 0.U(width=nums.W)
   var j   = 0.U(width=nums.W)
   for (i <- nums-1 to 1 by -1)
   {
       val sub = Wire(init = false.B)
	   when (res === j && sub_datas(i) === base && sub_datas(i-1)(sub_size-1).toBool === sign)
	   {
	       sub := true.B
	   }

	   j = j + 1.U
	   res = res + sub.asUInt()
   }

   nums.asUInt()-res
}

def ShiftByMask_64(data: UInt, data_size: Int, mask: UInt, mask_size: Int, mask_cnt_one: Int): UInt =
{
    val output = Wire(UInt(width=data_size.W))
	val sub_size = data_size / mask_size
	val sub_data = Wire(Vec(mask_cnt_one, UInt(width=sub_size.W)))

	for (i <- 0 until mask_cnt_one)
	{
	    val left  = sub_size * i
		val right = sub_size * (i+1) - 1
		sub_data(i) := data(right, left)
	}

	var idx = 0.U
	val tmp = Wire(Vec(mask_size, UInt(width=data_size.W)))
	for (i <- 0 until mask_size)
	{
	    val asc = Wire(init=0.U)
		tmp(i) := 0.U(width=data_size.W)
		when (mask(i))
		{
		    tmp(i)  := sub_data(idx) << (i * sub_size)
			asc     := 1.U
		}

		idx = idx + asc
	}

	output := tmp.reduce(_|_)
	output
}

class AllocToRenameIO(
   vreg_sz: Int,
   preg_sz: Int,
   mask_sz: Int) extends Bundle
{
   val valid     = Bool(INPUT)
   val nums      = UInt(INPUT,  width=vreg_sz)
   val can_alloc = Bool(OUTPUT)
   val preg      = UInt(OUTPUT, width=preg_sz)
   val mask      = UInt(OUTPUT, width=mask_sz)
}

class AllocPregsStageIO(
   num_write_ports: Int,
   (implicit p: Parameters) extends BoomBundle()(p)
{
   private val vreg_sz = TPREG_SZ
   private val preg_sz = TPREG_SZ
   private val mask_sz = numIntPhysRegsParts

   val req       = Vec(num_write_ports, new DecoupledIO(new ExeUnitResp(data_width))).flip
   val resp      = Vec(num_write_ports, new DecoupledIO(new ExeUnitResp(data_width)))
   val req_reg   = Vec(num_write_ports, new AllocToRenameIO(vreg_sz, preg_sz, mask_sz)).flip
}

class AllocPregsStage(
   rtype: BigInt,
   num_write_ports: Int,
   register_width: Int,
   mask_width: Int
   (implicit p: Parameters) extends BoomModule()(p)
{
   val io = new AllocPregsStageIO(num_write_ports)

   io.resp := io.req

   for (i <- 0 until num_write_ports)
   {
      io.req_preg(i).valid := io.req(i).valid && io.req(i).uop.dst_rtype === UInt(rtype) && io.req(i).uop.ldst_val
      io.req_preg(i).nums  := MyEncode_64(io.req(i).data, register_width, mask_width)
      io.resp(i).uop.pdst  := io.req_preg(i).preg
      io.resp(i).uop.mask  := io.req_preg(i).mask
	  io.resp(i).data      := ShiftByMask_64(io.req(i).data)
	  //:= can_alloc(i)???
   }
}
*/
