//******************************************************************************
// Copyright (c) 2015, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Datapath Register File
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Christopher Celio
// 2013 May 1


package boom

import Chisel._
import config.Parameters

import scala.collection.mutable.ArrayBuffer

class RegisterFileReadPortIO(addr_width: Int, data_width: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val addr = UInt(INPUT, addr_width)
   val mask = UInt(INPUT, numIntPhysRegsParts)
   val data = UInt(OUTPUT, data_width)
   override def cloneType = new RegisterFileReadPortIO(addr_width, data_width)(p).asInstanceOf[this.type]
}

class RegisterFileWritePort(addr_width: Int, data_width: Int)(implicit p: Parameters) extends BoomBundle()(p)
{
   val addr = UInt(width = addr_width)
   val mask = UInt(width = numIntPhysRegsParts)
   val data = UInt(width = data_width)
   override def cloneType = new RegisterFileWritePort(addr_width, data_width)(p).asInstanceOf[this.type]
}


// utility function to turn ExeUnitResps to match the regfile's WritePort I/Os.
object WritePort
{
   def apply(enq: DecoupledIO[ExeUnitResp], addr_width: Int, data_width: Int)
   (implicit p: Parameters): DecoupledIO[RegisterFileWritePort] =
   {
      val wport = Wire(Decoupled(new RegisterFileWritePort(addr_width, data_width)))
      wport.valid := enq.valid
      wport.bits.addr := enq.bits.uop.pdst
      //yqh
      wport.bits.mask := enq.bits.uop.dst_mask
      wport.bits.data := enq.bits.data
      enq.ready := wport.ready

      wport
   }
}


abstract class RegisterFile(
   num_registers: Int,
   num_read_ports: Int,
   num_write_ports: Int,
   register_width: Int,
   bypassable_array: Seq[Boolean]) // which write ports can be bypassed to the read ports?
   (implicit p: Parameters) extends BoomModule()(p)
{
   val io = new BoomBundle()(p)
   {
      val read_ports = Vec(num_read_ports, new RegisterFileReadPortIO(TPREG_SZ, register_width))
      val write_ports = Vec(num_write_ports, Decoupled(new RegisterFileWritePort(TPREG_SZ, register_width))).flip
   }

   private val rf_cost = (num_read_ports+num_write_ports)*(num_read_ports+2*num_write_ports)
   private val type_str = if (register_width == fLen+1) "Floating Point" else "Integer"
   override def toString: String =
      "\n   ==" + type_str + " Regfile==" +
      "\n   Num RF Read Ports     : " + num_read_ports +
      "\n   Num RF Write Ports    : " + num_write_ports +
      "\n   RF Cost (R+W)*(R+2W)  : " + rf_cost
}

// A behavorial model of a Register File. You will likely want to blackbox this for more than modest port counts.
class RegisterFileBehavorial(
   num_registers: Int,
   num_read_ports: Int,
   num_write_ports: Int,
   register_width: Int,
   bypassable_array: Seq[Boolean])
   (implicit p: Parameters)
   extends RegisterFile(num_registers, num_read_ports, num_write_ports, register_width, bypassable_array)
{
   // --------------------------------------------------------------

   val regfile = Mem(num_registers, UInt(width=register_width))

   // --------------------------------------------------------------
   // Write Merge

   val valid_merged	   = Wire(Vec(num_write_ports, Bool()))
   val addr_merged	   = Wire(Vec(num_write_ports, UInt(width = TPREG_SZ)))
   val mask_merged	   = Wire(Vec(num_write_ports, UInt(width = numIntPhysRegsParts)))
   val data_merged	   = Wire(Vec(num_write_ports, UInt(width = register_width)))
   val new_data_merged = Wire(Vec(num_write_ports, UInt(width = register_width)))

   // mask === all(1) : ext_mask === all(1, register_width)
   // mask =/= all(1) : ext_mask === 对应的子块掩码全1
   def extend_mask(mask: UInt, mask_size: Int, register_width: Int): UInt =
   {
      val output = Wire(init = 0.U(register_width.W))

      val sub_width = register_width / mask_size
	  var e_mask = 0.U(register_width.W)

	  for (i <- 0 until mask_size)
	  {
	     val sub_mask = ((mask(i) << 63.U).asSInt() >> 63.U)(sub_width-1,0)
		 e_mask = e_mask | sub_mask.asUInt() << (sub_width * i)
	  }

	  output := e_mask

	  when ((mask === ~0.U(width=mask_size.W)))
	  {
	     output := ~0.U(width=register_width.W)
	  }

	  output
   }

   // mask === all(1) : 读出整个寄存器
   // masl =/= all(1) : 读出对应子块、拼接、符号扩展成reg_width宽度
   def MyDecode(reg_data: UInt, reg_width: Int, mask: UInt, mask_width: Int): UInt =
   {
	  val output      = Wire(init = UInt(0, reg_width.W))
	  val sub_width   = reg_width / mask_width
	  val sub_mask    = (1.U << sub_width) - 1.U

	  val this_mask   = Wire(UInt(mask_width.W))
	  val this_data   = Wire(UInt(reg_width.W))
	  this_mask       := mask
	  this_data       := reg_data

	  var res         = UInt(0, reg_width.W)
	  var cnt         = UInt(0, mask_width.W)
	  for (i <- mask_width-1 to 0 by -1)
	  {
	     val offset  = sub_width * i
		 val sub_res = Wire(init = UInt(0, width = sub_width))
		 val shift   = Wire(init = UInt(0, width = sub_width))
		 val asc     = Wire(init = UInt(0, 1.W))

		 when(this_mask(i))
		 {
		    sub_res := (this_data >> offset) & sub_mask
			shift   := sub_width.asUInt()
			asc     := 1.U
		 }

		 cnt = cnt + asc
		 res = (res << shift) | sub_res
	  }

	  val size = sub_width.asUInt() * cnt
	  val sign = res((size - 1.U) & "b111111".U).toBool
	  val ext1 = (1.U << (reg_width.asUInt() - size)) - 1.U

	  when (sign)
	  {
	     output := (ext1 << size) | res
	  }.otherwise{
	     output := res
	  }
 
	  when (mask === ~0.U(mask_width.W))
	  {
	     output := reg_data
	  }

	  output
   }

   for (i <- 0 until num_write_ports)
   {
      io.write_ports(i).ready := Bool(true)
      valid_merged(i) := io.write_ports(i).valid
	  addr_merged(i)  := io.write_ports(i).bits.addr
	  mask_merged(i)  := io.write_ports(i).bits.mask
	  data_merged(i)  := io.write_ports(i).bits.data
   }

   for (i <- 0 until num_write_ports)
   {
      var next_mask_merged = io.write_ports(i).bits.mask
	  var next_data_merged = io.write_ports(i).bits.data

	  for (j <- i+1 until num_write_ports)
	  {
	     val valid_i = io.write_ports(i).valid
		 val valid_j = io.write_ports(j).valid
		 val addr_i  = io.write_ports(i).bits.addr
		 val addr_j  = io.write_ports(j).bits.addr

		 val updated_mask_i = Wire(init = UInt(0, width = numIntPhysRegsParts))
		 val updated_data_i = Wire(init = UInt(0, width = register_width))

		 when (valid_i && valid_j && addr_i === addr_j)
		 {
		    updated_mask_i  := io.write_ports(j).bits.mask
			updated_data_i  := io.write_ports(j).bits.data
			valid_merged(j) := false.B
		 }
		 next_mask_merged = next_mask_merged | updated_mask_i
		 next_data_merged = next_data_merged | updated_data_i
	  }

	  mask_merged(i)  := next_mask_merged
	  data_merged(i)  := next_data_merged
   }

   val merged_wport = Wire(Vec(num_write_ports, new DecoupledIO(new RegisterFileWritePort(TPREG_SZ, register_width))))
   for (i <- 0 until num_write_ports) 
   {
      val valid   = valid_merged(i)
      val addr    = addr_merged(i)
	  val mask    = mask_merged(i)
	  val data    = data_merged(i)
     
      merged_wport(i).valid     := valid
	  merged_wport(i).bits.addr := addr
	  merged_wport(i).bits.data := data | regfile(addr) & ~extend_mask(mask, numIntPhysRegsParts, register_width)
   }

   // --------------------------------------------------------------
   // Read ports.

   val read_data = Wire(Vec(num_read_ports, UInt(width = register_width)))

   // Register the read port addresses to give a full cycle to the RegisterRead Stage (if desired).
   val read_addrs =
      if (regreadLatency == 0) {
         io.read_ports map {_.addr}
      } else {
         require (regreadLatency == 1)
         io.read_ports.map(p => RegNext(p.addr))
      }

   for (i <- 0 until num_read_ports)
   {
      read_data(i) :=
         Mux(read_addrs(i) === UInt(0),
            UInt(0),
            regfile(read_addrs(i)))
   }

   for (i <- 0 until num_read_ports)
   {
      val bypass_ens = merged_wport.map(x => x.valid && x.bits.addr =/= UInt(0) && x.bits.addr === read_addrs(i)) 
      val bypass_data = Mux1H(Vec(bypass_ens), Vec(merged_wport.map(_.bits.data)))
      val pre_data = Mux(bypass_ens.reduce(_|_), bypass_data, read_data(i))
      //val mask = io.read_ports(i).mask

      io.read_ports(i).data := MyDecode(pre_data, register_width, ~Bits(0, numIntPhysRegsParts), numIntPhysRegsParts)

      //printf ("pre_data = 0x%x, register_width = %d, read_mask = b%b, mask_size = %d, output_data = 0x%x\n",
	  //        pre_data, register_width.asUInt(), mask, numIntPhysRegsParts.asUInt(), io.read_ports(i).data)
   }
  
   // --------------------------------------------------------------
   // Write ports.
   for (wport <- merged_wport)
   {
      when (wport.valid && (wport.bits.addr =/= UInt(0)))
	  {
	     regfile(wport.bits.addr) := wport.bits.data
	  }
   }
   
   /*
   // --------------------------------------------------------------
   // Bypass out of the ALU's write ports.
   // We are assuming we cannot bypass a writer to a reader within the regfile memory
   // for a write that occurs at the end of cycle S1 and a read that returns data on cycle S1.
   // But since these bypasses are expensive, and not all write ports need to bypass their data,
   // only perform the w->r bypass on a select number of write ports.

   require (bypassable_array.length == io.write_ports.length)

   if (bypassable_array.reduce(_||_))
   {
      val bypassable_wports = ArrayBuffer[DecoupledIO[RegisterFileWritePort]]()
      io.write_ports zip bypassable_array map { case (wport, b) => if (b) { bypassable_wports += wport} }

      for (i <- 0 until num_read_ports)
      {
         val bypass_ens = bypassable_wports.map(x => x.valid &&
                                                  x.bits.addr =/= UInt(0) &&
                                                  x.bits.addr === read_addrs(i))

         val bypass_data = Mux1H(Vec(bypass_ens), Vec(bypassable_wports.map(_.bits.data)))

         io.read_ports(i).data := Mux(bypass_ens.reduce(_|_), bypass_data, read_data(i))
      }
   }
   else
   {
      for (i <- 0 until num_read_ports)
      {
         io.read_ports(i).data := read_data(i)
      }
   }


   // --------------------------------------------------------------
   // Write ports.

   for (wport <- io.write_ports)
   {
      wport.ready := Bool(true)
      when (wport.valid && (wport.bits.addr =/= UInt(0)))
      {
         regfile(wport.bits.addr) := wport.bits.data
      }
   }
   */
}
