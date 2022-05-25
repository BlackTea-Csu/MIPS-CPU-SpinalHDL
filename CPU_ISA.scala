package mylib

import spinal.core
import spinal.core._
import spinal.core.internals.Operator
import spinal.lib._

import scala.language.postfixOps
import scala.util.Random
import spinal.lib.com.uart._
import BundleImplicit._
import spinal.lib.fsm._
import spinal.sim._
import spinal.core.sim._

// class中使用的 r_*并非全为Reg，也有可能是类似于Verilog的wire，具体要观测其定义

// 隐式类，协助Master、Slave在不同类的实例中的连接
object BundleImplicit{
  implicit class autoConnect(bus:Bundle){
    def connect(srcBus:Bundle):Unit={
      bus.connectWithSrc(srcBus)
    }
    def connectWithSrc(srcBus:Bundle):Unit={
      for((name,element)<-bus.elements){
        val nameOfBundle1 = srcBus.getName()
        val nameOfBundle2 = bus.getName()
        println(Console.GREEN+s"connecting Bundle " + Console.YELLOW + s"$nameOfBundle1"
          + Console.GREEN + s" & " + Console.YELLOW + s"$nameOfBundle2" + Console.GREEN +s" with port "
          + Console.MAGENTA + s"$name"+Console.RESET)
        val srcPort = srcBus.find(name)
        if(srcPort!=null){
          element match {
            case b:Bundle => b.connect(srcPort.asInstanceOf[Bundle])
            case _ =>{
              (element.getDirection,srcPort.getDirection) match {
                case (`out`,`in`)  => assignWithAdapt(element,srcPort)
                case(`out`,null)   => assignWithAdapt(element,srcPort)
                case(`in`,`out`)   => assignWithAdapt(srcPort,element)
                case(`in`,null)    => assignWithAdapt(srcPort,element)
                case(null,`out`)   => assignWithAdapt(srcPort,element)
                case(null,`in`)    => assignWithAdapt(element,srcPort)
                case (`in`,`in`)  => assignWithAdapt(element,srcPort)   // 为模块嵌套做准备，即子模块的输入也作为其上主模块的输入
                case (`out`,`out`)  => assignWithAdapt(element,srcPort)  //  // 为模块嵌套做准备，即子模块的输出也作为其上主模块的输出
                case _  if element.isAnalog && srcPort.isAnalog => assignWithAdapt(element,srcPort)
                case _             => LocatedPendingError(s"Direction Error")
              }
            }
          }
        }
      }
    }
    def assignWithAdapt(dst:Data,src:Data):Unit={
      if(dst.getBitsWidth != src.getBitsWidth){
        println(Console.RED+s"$dst width is different with $src, auto resize."+Console.RESET)
        dst <> src.resized
      }else
        dst <> src
    }
  }
}

class pcReg extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val pc = out UInt (InstAddrBus bits)
    val ce = out Bool
    val stall = in UInt(stallDir bits)  // 流水线暂停
    val S_branch_id2pcReg = slave(branchDetermineInterface(InstAddrBus))  // 转移/分支指令相关 取值跳转
    val flush = in Bool  // ctrl确认发生异常之信号
    val newPC = in UInt (InstAddrBus bits)  // 重置的新指令地址
  }
  // Configure the clock domain，rst==RstEnable
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_ce = Reg(Bool()) init (ChipDisable)
    r_ce := ChipEnable
    io.ce := r_ce
    val r_pc = Reg(UInt(io.pc.getWidth bits))
    io.pc := r_pc
    when(r_ce === ChipDisable) {
      r_pc := U"32'h0"
    }
      .otherwise {
        when(io.flush === hasException) { // 有异常发生
          r_pc := io.newPC
        }
          .elsewhen(io.stall(0) === NoStop) { // 当pc不被暂停
            when(io.S_branch_id2pcReg.brach_flag === Branch) { // 当上一指令为转移/分支指令
              r_pc := io.S_branch_id2pcReg.brach_targetAddress
            }
              .otherwise {
                r_pc := r_pc + 4
              }
          }
          .otherwise {} // r_pc寄存器保存原值(当r_ce === ChipEnable 且 stall(0) === Stop）
      }
  }
}

class ifId extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val if_pc = in UInt (InstAddrBus bits)
    val if_inst = in UInt (InstBus bits)
    val M_ifId2id = master(ifId2idInterface(InstAddrBus, InstBus))
    val stall = in UInt(stallDir bits)  // 流水线暂停
    val flush = in Bool  // ctrl确认发生异常之信号
  }
  // Configure the clock domain，rst==RstEnable
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
//  r_id_pc := ZeroWord
//  r_id_inst := ZeroWord
  val areaClk = new ClockingArea(thisClockDomain) {
  val r_id_pc = Reg(UInt(io.if_pc.getWidth bits)) init(ZeroWord)
  val r_id_inst = Reg(UInt(io.if_inst.getWidth bits)) init(ZeroWord)
  io.M_ifId2id.pc := r_id_pc
  io.M_ifId2id.inst := r_id_inst
  when(io.flush === hasException){
    r_id_pc := ZeroWord
    r_id_inst := ZeroWord
  }
    .elsewhen(io.stall(1) === Stop && io.stall(2) === NoStop){
      // 当if/id收到stall的stop信号，译码id却收到NoStop,此时视为置入空指令
      // 视作，io.stall(1)以控制ifId，io.stall(2)通过ifId的output影响id
        r_id_pc := ZeroWord
        r_id_inst := ZeroWord
    }
    .elsewhen(io.stall(1) === NoStop){
       r_id_pc := io.if_pc
       r_id_inst := io.if_inst
    }
    .otherwise{}  // 其余（ifId与id均暂停），保持原值
  }
}

class regFile extends Component with Global_parameter with Interface_MS {
   val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    // 与memWb交互端口
    val S_memWb2regfile = slave(wInterface(RegAddrBus, RegBus))
    // 与ID交互端口
    val S_id2regfile = slave(id2regfileInterface(RegAddrBus, RegBus, RegAddrBus, RegBus))
  }

  val init_array = new Array[UInt](RegNum)
  for(i <- 0 until RegNum) {init_array(i)=U"32'h00000000"}
  // 使用初始化的列表,寄存器堆
  val memReg = Mem(UInt(RegBus bits),initialContent = init_array)
//  val memReg = Mem(UInt(RegBus bits), RegNum)

  // 写操作：时序逻辑电路, rst==RstDisable触发
  val wClockDomain = ClockDomain(
    clock = io.clk,
    reset = null,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(wClockDomain) {
    memReg.write(
      address = io.S_memWb2regfile.waddr,
      data = io.S_memWb2regfile.wdata,
      enable = (io.rst === RstDisable) && (io.S_memWb2regfile.we === WriteEnable) && (io.S_memWb2regfile.waddr =/= 0) // 复位rst无效时写入，写入为时序电路
    )
  }

  // 组合逻辑 读端口1（没有ClockArea限制即可？使用\=反而出错）
  when(io.rst === RstEnable) {
    io.S_id2regfile.rdata1 := ZeroWord
  }
    .elsewhen(io.S_id2regfile.raddr1 === 0) { // 当io.rst === RstDisable，才可读取 寄存器对应地址储存的数据
    io.S_id2regfile.rdata1 := ZeroWord  // 设定为regFile的$0寄存器恒为0
  }
    // regFile可写可读且可写地址(回写阶段memWb给出）恰好等于可读（译码阶段id给出）
    .elsewhen((io.S_memWb2regfile.we === WriteEnable) && (io.S_id2regfile.re1 === ReadEnable) && (io.S_id2regfile.raddr1 === io.S_memWb2regfile.waddr)) {
    io.S_id2regfile.rdata1 := io.S_memWb2regfile.wdata
  }
    .elsewhen(io.S_id2regfile.re1 === ReadEnable) { // 仅可读
    io.S_id2regfile.rdata1 := memReg(io.S_id2regfile.raddr1)
  }
    .otherwise {
    io.S_id2regfile.rdata1 := ZeroWord
  }
  // 组合逻辑 读端口2（没有ClockArea限制即可？使用\=反而出错）
  when(io.rst === RstEnable) {
    io.S_id2regfile.rdata2 := ZeroWord
  }.elsewhen(io.S_id2regfile.raddr2 === 0) { // 可读地址为$0
    io.S_id2regfile.rdata2 := ZeroWord
  }.elsewhen((io.S_memWb2regfile.we === WriteEnable) && (io.S_id2regfile.re2 === ReadEnable) && (io.S_id2regfile.raddr2 === io.S_memWb2regfile.waddr)) { // 可写可读且可写地址恰好等于可读
    io.S_id2regfile.rdata2 := io.S_memWb2regfile.wdata
  }.elsewhen(io.S_id2regfile.re2 === ReadEnable) { // 仅可读
    io.S_id2regfile.rdata2 := memReg(io.S_id2regfile.raddr2)
  }.otherwise {
    io.S_id2regfile.rdata2 := ZeroWord
  }
}

class id extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val rst = in Bool
    val S_ifId2id = slave(ifId2idInterface(InstAddrBus, InstBus))
    val M_id2regfile = master(id2regfileInterface(RegAddrBus, RegBus, RegAddrBus, RegBus))
    val M_id2idEx = master(id2idExInterface(AluOpBus, AluSelBus, RegBus, RegBus, RegAddrBus,InstBus))  // 包括了储存指令（InstBus）相关
    // 将mem、ex数据向前推送，解决RAW相关问题
    val S_PullForward_mem2id = slave(wInterface(RegAddrBus, RegBus))
    val S_PullForward_ex2id = slave(wInterface(RegAddrBus, RegBus))
    // val stall = in UInt(stallDir bits)  // 流水线暂停
    val stallCtrl = out Bool
    // 分支/转移指令下的的数据交流
    val M_branch_id2pcReg = master(branchDetermineInterface(InstAddrBus))
    val M_branch_id2idEx = master(branchInterce(InstAddrBus))
    // 通过执行模块，感知上一clk的指令类型
    val aluop_ex2id = in UInt(AluOpBus bits)
    // 异常判断
    val M_except_id2idEx = master(exceptTypeAddrInterface(ExceptTypeBus,InstBus))
  }
  // 指令判断过程：
  // op [31:26] -->  ==SPECIAL  --> op2 [10:6]=0 --> op3 [5:0] 决定 or/and/xor/nor/sllv/srlv/srav/sync
  //                                                              movn/movz/mthi/mtlo/mfhi/mflo
  //                                                              add/addu/sub/subu/slt/sltu/mult/multu
  //                                                              div/divu
  //                                                              jr/jalr
  //                                                              （其余无效）
  //                           --> op3 [5:0] 决定 teq/tge/tgeu/tlt/tltu/tne/syscall
  //                           -->  其余无效
  //
  //            -->  ori/andi/xori/lui/pref
  //                 addi/addiu/slti/sltiu
  //                 j/jal/beq/bgtz/blez/bne
  //                 lb/lbu/lh/lhu/lw/lwl/lwr/sb/sh/sw/swl/swr
  //                 ll/sc
  //            -->  ==COP0      -->   [25:21] 决定 mtc0/mfc0
  //                             -->   [5:0] 决定 eret
  //            -->  ==SPECIAL2  -->   op3 [5:0] 决定 clz/clo/mul
  //                                                 madd/maddu/msub/msubu
  //            -->  ==REGIMM    -->   op4 [20:16] 决定 bltz/bltzal/bgez/bgezal/bal
  //                                                   teqi/tgei/tgeiu/tlti/tltiu/tnei
  //            --> 其余无效
  //
  // [31:21] == 0 --> op3 [5:0] 决定 sll/srl/sra
 //               --> 其余无效

  val op = io.S_ifId2id.inst(31 downto 26)
  val op2 = io.S_ifId2id.inst(10 downto 6)
  val op3 = io.S_ifId2id.inst(5 downto 0)
  val op4 = io.S_ifId2id.inst(20 downto 16)

  // 转移/分支相关
  val r_pc_plus4 = UInt(InstAddrBus bits)  // 当前指令后的第一条指令的地址（转移指令相关，为延迟槽）
  r_pc_plus4 := io.S_ifId2id.pc + 4
  val r_pc_plus8= UInt(InstAddrBus bits)  // 当前指令后的第二条指令的地址（转移指令相关，为延迟槽后的第一条指令）
  r_pc_plus8 := io.S_ifId2id.pc + 8
  val r_targetAddr_J = UInt(InstAddrBus bits)  // 由转移（Jump）指令中的instr_index组成的要转跳的指令地址
  r_targetAddr_J := r_pc_plus4(InstAddrBus-1 downto InstAddrBus-4) @@ io.S_ifId2id.inst(25 downto 0) @@ U"2'b00"
  val r_targetAddr_B = UInt(InstAddrBus bits)  // 由分支（Branch）指令中的instr_index组成的要转跳的指令地址
  // r_targetAddr_B使用符号扩充且左移的io.S_ifId2id.inst(15 downto 0)
  r_targetAddr_B := r_pc_plus4 + (U(14 bits, default->io.S_ifId2id.inst(15)) @@ io.S_ifId2id.inst(15 downto 0) @@ U"2'b00")

  // 立即数
  val r_imm = UInt(RegBus bits)
  // 解决load相关（利用流水线暂停）
  val preInst_isLoad = Bool
  preInst_isLoad := (io.aluop_ex2id === EXE_LB_OP || io.aluop_ex2id === EXE_LBU_OP ||
    io.aluop_ex2id === EXE_LH_OP  || io.aluop_ex2id === EXE_LHU_OP ||
    io.aluop_ex2id === EXE_LW_OP  || io.aluop_ex2id === EXE_LWL_OP ||
    io.aluop_ex2id === EXE_LWR_OP || io.aluop_ex2id === EXE_LL_OP  ||
    io.aluop_ex2id === EXE_SC_OP) ? True | False
  // 解决异常返回问题（上一clk为mtc0写入cpu0，本clk为eret异常返回）
  val isMtc0_beforeEret= Bool
  isMtc0_beforeEret := (io.aluop_ex2id === EXE_MTC0_OP) && (io.M_id2idEx.aluop === EXE_ERET_OP)
  val stallForReg1_load = Bool
  // 若上一clk的指令（上一clk的指令当前周期在执行阶段）为加载指令，且加载指令的加载目标地址（写入普通寄存器地址）
  // 为本指令涉及读取的地址reg1（reg1可读、地址相同）,请求暂停
  // 当preInst_isLoad为false，即成功插入一条空指令后，结束流水线暂停
  stallForReg1_load := (preInst_isLoad && io.M_id2regfile.raddr1 === io.S_PullForward_ex2id.waddr &&
    io.M_id2regfile.re1 === ReadEnable) ? Stop | NoStop
  val stallForReg2_load = Bool
  // 若上一clk的指令（上一clk的指令当前周期在执行阶段）为加载指令，且加载指令的加载目标地址（写入普通寄存器地址）
  // 为本指令涉及读取的地址reg2（reg2可读、地址相同）
  stallForReg2_load := (preInst_isLoad && io.M_id2regfile.raddr2 === io.S_PullForward_ex2id.waddr &&
    io.M_id2regfile.re2 === ReadEnable) ? Stop | NoStop
  // 流水线暂停相关
  io.stallCtrl := stallForReg1_load | stallForReg2_load | isMtc0_beforeEret

  io.M_id2idEx.inst := io.S_ifId2id.inst  // 直接向后派发指令码

  // 防止latch（异常判断）
  // 第0-7bit exceptiontype的低8bit留给外部中断 [7:0]
  // 第8bit表示是否系统调用异常syscall [8]
  // 第9bit表示是否无效指令异常InstInvalid [9]
  // 第10bit表示是否自陷异常trap(自陷异常trap需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [10]
  // 第11bit表示是否溢出异常overflow(溢出异常overflow需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [11]
  // 第12bit表示是否异常返回eret [12]
  val r_isSyscall = Bool  // 系统调用标记位
  val r_isInstValid = Bool  // 指令有效标记位
  val r_isEret = Bool // 异常返回标记位
  io.M_except_id2idEx.exceptType :=  (r_isEret.asUInt @@ U"2'b0" @@ r_isInstValid.asUInt @@ r_isSyscall.asUInt @@ U"8'b0").resized
  io.M_except_id2idEx.currentInst_addr := io.S_ifId2id.pc  // 指令的地址实质就是累加的pc的值

  when(io.rst === RstEnable) {
    r_isInstValid := InstValid  // 指令有效标记位，初始时记为无效指令
    r_isSyscall := noException  // 系统调用异常，初始为无
    r_isEret := noException  // 异常返回，初始不返回
    io.M_id2idEx.alusel := EXE_RES_NOP // 运算类型（逻辑or算数）
    io.M_id2idEx.aluop := EXE_NOP_OP // 运算子类型（如：逻辑类型下的 and、or、not,默认为NOP
    io.M_id2idEx.waddr := NOPRegAddr
    io.M_id2idEx.we := WriteDisable
    io.M_id2regfile.re1 := ReadDisable
    io.M_id2regfile.re2 := ReadDisable
    io.M_id2regfile.raddr1 := NOPRegAddr
    io.M_id2regfile.raddr2 := NOPRegAddr
    r_imm := ZeroWord
    // 转移/分支相关
    // pc相关
    io.M_branch_id2pcReg.brach_flag := NotBranch
    io.M_branch_id2pcReg.brach_targetAddress := ZeroWord
    // idEx相关
    io.M_branch_id2idEx.link_addr := ZeroWord
    io.M_branch_id2idEx.next_inst_inDelayslot := NotInDelaySlot
    io.M_branch_id2idEx.is_inDelayslot := NotInDelaySlot
  }.otherwise { // io.rst === RstDisable 时，若接收到op指令为EXE_ORI，可写入idEx,并自regfile读取
    // 注意，nop、ssnop并不需要特殊实现，只要不满足下列switch的运算类型，一概默认为nop
    r_isInstValid := InstInvalid  // 指令有效标记位，初始时记为无效指令，根据具体指令更新
    r_isSyscall := noException  // 系统调用异常，初始为无
    r_isEret := noException  // 异常返回，初始不返回
    io.M_id2idEx.alusel := EXE_RES_NOP
    io.M_id2idEx.aluop := EXE_NOP_OP  // mthi\mtlo指令，大类类型为默认EXE_NOP_OP
    io.M_id2idEx.waddr := io.S_ifId2id.inst(15 downto 11) // 5 bits 2^5=32
    io.M_id2idEx.we := WriteDisable
    io.M_id2regfile.re1 := ReadDisable
    io.M_id2regfile.re2 := ReadDisable
    io.M_id2regfile.raddr1 := io.S_ifId2id.inst(25 downto 21)  // MIPS指令中的rs索引
    io.M_id2regfile.raddr2 := io.S_ifId2id.inst(20 downto 16)  // MIPS指令中的rt索引
    r_imm := ZeroWord
    // 转移/分支相关 防止latch
    // pc相关
    io.M_branch_id2pcReg.brach_flag := NotBranch
    io.M_branch_id2pcReg.brach_targetAddress := ZeroWord
    // idEx相关
    io.M_branch_id2idEx.link_addr := ZeroWord
    io.M_branch_id2idEx.next_inst_inDelayslot := NotInDelaySlot
    io.M_branch_id2idEx.is_inDelayslot := io.M_branch_id2idEx.check_inDelayslot  // 根据后端idEx的信号判断本条指令是否位于延迟槽并转告后续模块
    switch(op) {
      is(EXE_SPECIAL_INST){  // [31:26] 为特殊指令 SPEICAL
        switch(op2){
          is(U"5'b00000"){
            switch(op3){
              is(EXE_OR){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_LOGIC
                io.M_id2idEx.aluop := EXE_OR_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_AND){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_LOGIC
                io.M_id2idEx.aluop := EXE_AND_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_XOR){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_LOGIC
                io.M_id2idEx.aluop := EXE_XOR_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_NOR){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_LOGIC
                io.M_id2idEx.aluop := EXE_NOR_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SLLV){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_SHIFT
                io.M_id2idEx.aluop := EXE_SLL_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SRLV){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_SHIFT
                io.M_id2idEx.aluop := EXE_SRL_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SRAV){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_SHIFT
                io.M_id2idEx.aluop := EXE_SRA_OP
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SYNC){  // SYNC指令，暂不涉及
                io.M_id2idEx.we := WriteDisable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_NOP_OP
                io.M_id2regfile.re1 := ReadDisable
                io.M_id2regfile.re2 := ReadDisable
              }
              is(EXE_MFHI){  // MFHI指令
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_MOVE
                io.M_id2idEx.aluop := EXE_MFHI_OP
                io.M_id2regfile.re1 := ReadDisable  // MFHI的rs/rt均为0
                io.M_id2regfile.re2 := ReadDisable
              }
              is(EXE_MFLO){
                io.M_id2idEx.we := WriteEnable
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_MOVE
                io.M_id2idEx.aluop := EXE_MFLO_OP
                io.M_id2regfile.re1 := ReadDisable  // MFHI的rs/rt均为0
                io.M_id2regfile.re2 := ReadDisable
              }
              is(EXE_MOVN){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_MOVE
                io.M_id2idEx.aluop := EXE_MOVN_OP
                io.M_id2regfile.re1 := ReadEnable  // 先读取rs、rt，再根据rt（rdata2）判读是否写入
                io.M_id2regfile.re2 := ReadEnable
                io.M_id2idEx.we := (io.M_id2regfile.rdata2 === ZeroWord) ? WriteDisable | WriteEnable
              }
              is(EXE_MOVZ){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_MOVE
                io.M_id2idEx.aluop := EXE_MOVN_OP
                io.M_id2regfile.re1 := ReadEnable  // 先读取rs、rt，再根据rt（rdata2）判读是否写入
                io.M_id2regfile.re2 := ReadEnable
                io.M_id2idEx.we := (io.M_id2regfile.rdata2 === ZeroWord) ? WriteEnable | WriteDisable
              }
              is(EXE_MTHI){
               // MTHI指令， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_MTHI_OP
                io.M_id2idEx.we := WriteDisable
                io.M_id2regfile.re1 := ReadEnable  // hi <- rs(rdata1)
                io.M_id2regfile.re2 := ReadDisable
              }
              is(EXE_MTLO){
                // MTLO指令， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_MTLO_OP
                io.M_id2idEx.we := WriteDisable
                io.M_id2regfile.re1 := ReadEnable  // lo <- rs(rdata1)
                io.M_id2regfile.re2 := ReadDisable
              }
              is(EXE_ADD){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_ADD_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_ADDU){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_ADDU_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SUB){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_SUB_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SUBU){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_SUBU_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SLT){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_SLT_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_SLTU){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
                io.M_id2idEx.aluop := EXE_SLTU_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_MULT){ //涉及hilo， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_MULT_OP
                io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_MULTU){  //涉及hilo， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_MULTU_OP
                io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_DIV){  //涉及hilo， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_DIV_OP
                io.M_id2idEx.we := WriteDisable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_DIVU){  //涉及hilo， alusel为默认值nop，因其不写入普通寄存器
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_NOP
                io.M_id2idEx.aluop := EXE_DIVU_OP
                io.M_id2idEx.we := WriteDisable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadEnable
              }
              is(EXE_JR){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
                io.M_id2idEx.aluop := EXE_JR_OP
                io.M_id2idEx.we := WriteDisable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadDisable
                // pc相关
                io.M_branch_id2pcReg.brach_flag := Branch
                io.M_branch_id2pcReg.brach_targetAddress := io.M_id2idEx.reg1  // 取rs的值作为新的指令地址
                // idEx相关
                io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
              }
              is(EXE_JALR){
                r_isInstValid := InstValid
                io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
                io.M_id2idEx.aluop := EXE_JALR_OP
                io.M_id2idEx.we := WriteEnable
                io.M_id2regfile.re1 := ReadEnable
                io.M_id2regfile.re2 := ReadDisable
                // pc相关
                io.M_branch_id2pcReg.brach_flag := Branch
                io.M_branch_id2pcReg.brach_targetAddress := io.M_id2idEx.reg1  // 取rs的值作为新的指令地址
                // idEx相关
                io.M_branch_id2idEx.link_addr := r_pc_plus8
                io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
              }
              default{}
            }
          }
          default{}
        }
        switch(op3){
          // 自陷异常，等待ex模块予以判断是否触发
          is(EXE_TEQ){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TEQ_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_TGE){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TGE_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_TGEU){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TGEU_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_TLT){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TLT_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_TLTU) {
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TLTU_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_TNE){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TNE_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          // 系统调用，在id模块即予以判断
          is(EXE_SYSCALL){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_SYSCALL_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadDisable
            io.M_id2regfile.re2 := ReadDisable
            r_isSyscall := hasException
          }
        }
      }
      is(EXE_ORI) {
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOGIC
        io.M_id2idEx.aluop := EXE_OR_OP  // 立即数的逻辑运算可视为将reg2替换为immediate的逻辑操作
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)  // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable  // 逻辑“或”操作，re2不可读，对应的rdata2为立即数
        r_imm := io.S_ifId2id.inst(15 downto 0).resize(InstAddrBus bits)  // imm对应MIPS指令中的immediate立即数
      }
      is(EXE_ANDI){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOGIC
        io.M_id2idEx.aluop := EXE_AND_OP // 立即数的逻辑运算可视为将reg2替换为immediate的逻辑操作
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := io.S_ifId2id.inst(15 downto 0).resize(InstAddrBus bits)  // imm对应MIPS指令中的immediate立即数
      }
      is(EXE_XORI){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOGIC
        io.M_id2idEx.aluop := EXE_XOR_OP // 立即数的逻辑运算可视为将reg2替换为immediate的逻辑操作
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := io.S_ifId2id.inst(15 downto 0).resize(InstAddrBus bits)
      }
      is(EXE_LUI){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOGIC
        io.M_id2idEx.aluop := EXE_OR_OP  // LUI可以视作特殊的OR
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable // 默认raddr1读$0，$0恒为0，执行或操作无影响
        io.M_id2regfile.re2 := ReadDisable
        r_imm := io.S_ifId2id.inst(15 downto 0) @@ U"16'h0000"  // 立即数为原立即数即inst[15:0]右移16bits
      }
      is(EXE_PREF){  // 暂无缓存区域，该指令视作无效
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_NOP
        io.M_id2idEx.aluop := EXE_NOP_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable // 默认raddr1读$0，$0恒为0
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_ADDI){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
        io.M_id2idEx.aluop := EXE_ADDI_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
      }
      is(EXE_ADDIU){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
        io.M_id2idEx.aluop := EXE_ADDIU_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
      }
      // 移位相关
      is(EXE_SLTI){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
        io.M_id2idEx.aluop := EXE_SLTI_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
      }
      is(EXE_SLTIU){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
        io.M_id2idEx.aluop := EXE_SLTIU_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16) // 指向操作结果保存的寄存器地址,MIPS ORI指令中的rt索引
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
      }
      // 加载/储存相关
      is(EXE_LB){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LB_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_LBU){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LBU_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_LH){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LH_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_LHU){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LHU_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_LW){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LW_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_LWL){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LWL_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_LWR){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LWR_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_SB){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SB_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_SH){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SH_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_SW){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SW_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_SWL){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SWL_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      is(EXE_SWR){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SWR_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      // 链接状态位
      is(EXE_LL){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_LL_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
      }
      is(EXE_SC){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_LOAD_STORE
        io.M_id2idEx.aluop := EXE_SC_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
      }
      // 转移/分支相关
      is(EXE_J){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_J_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadDisable
        io.M_id2regfile.re2 := ReadDisable
        // pc相关
        io.M_branch_id2pcReg.brach_flag := Branch
        io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_J  // 取r_targetPC的值作为新的指令地址
        // idEx相关
        io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
      }
      is(EXE_JAL){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_JAL_OP
        io.M_id2idEx.we := WriteEnable
        io.M_id2idEx.waddr := Reg_31  // 写入地址为$31
        io.M_id2regfile.re1 := ReadDisable
        io.M_id2regfile.re2 := ReadDisable
        // pc相关
        io.M_branch_id2pcReg.brach_flag := Branch
        io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_J  // 取r_targetAddr_J的值作为新的指令地址
        // idEx相关
        io.M_branch_id2idEx.link_addr := r_pc_plus8
        io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
      }
      is(EXE_BEQ){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_BEQ_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
        when(io.M_id2idEx.reg1 === io.M_id2idEx.reg2) {
          // pc相关
          io.M_branch_id2pcReg.brach_flag := Branch
          io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
          // idEx相关
          io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
        }
          .otherwise()  // 不相等则保持原值
      }
      is(EXE_BGTZ){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_BGTZ_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        when(io.M_id2idEx.reg1.asSInt > 0) {  // 视rs为有符号数，与0进行比较（可以用 io.M_id2idEx.reg1(31)===false && io.M_id2idEx.reg1 =/= ZeroWord 代替）
          // pc相关
          io.M_branch_id2pcReg.brach_flag := Branch
          io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
          // idEx相关
          io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
        }
          .otherwise()  // 不满足条件则保持原值
      }
      is(EXE_BLEZ){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_BLEZ_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadDisable
        when(io.M_id2idEx.reg1.asSInt <= 0) {  // 视rs为有符号数，与0进行比较
          // pc相关
          io.M_branch_id2pcReg.brach_flag := Branch
          io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
          // idEx相关
          io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
        }
          .otherwise()  // 不满足条件则保持原值
      }
      is(EXE_BNE){
        r_isInstValid := InstValid
        io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
        io.M_id2idEx.aluop := EXE_BNE_OP
        io.M_id2idEx.we := WriteDisable
        io.M_id2regfile.re1 := ReadEnable
        io.M_id2regfile.re2 := ReadEnable
        when(io.M_id2idEx.reg1 =/= io.M_id2idEx.reg2) {  // 视rs\rt为有符号数，进行比较
          // pc相关
          io.M_branch_id2pcReg.brach_flag := Branch
          io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
          // idEx相关
          io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
        }
          .otherwise()  // 不满足条件则保持原值
      }
      is(EXE_REGIMM_INST){  // 分支指令相关
        switch(op4){
          is(EXE_BLTZ){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
            io.M_id2idEx.aluop := EXE_BLTZ_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            when(io.M_id2idEx.reg1(RegBus-1) === True) {  // 视rs为有符号数，与0进行比较,则直接取rs高位进行对比则可,为1则rs为负数小于0
              // pc相关
              io.M_branch_id2pcReg.brach_flag := Branch
              io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
              // idEx相关
              io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
            }
              .otherwise()  // 不满足条件则保持原值
          }
          is(EXE_BLTZAL){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
            io.M_id2idEx.aluop := EXE_BLTZAL_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2idEx.waddr := Reg_31
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            io.M_branch_id2idEx.link_addr := r_pc_plus8  // 不管下面的条件是否满足，都将r_pc_plus8写入Reg_31
            when(io.M_id2idEx.reg1(RegBus-1) === True) {  // 视rs为有符号数，与0进行比较,则直接取rs高位进行对比则可,为1则rs为负数小于0
              // pc相关
              io.M_branch_id2pcReg.brach_flag := Branch
              io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
              // idEx相关
              io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
            }
              .otherwise()  // 不满足条件则保持原值
          }
          is(EXE_BGEZ){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
            io.M_id2idEx.aluop := EXE_BGEZ_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            when(io.M_id2idEx.reg1(RegBus-1) === False) {  // 视rs为有符号数，与0进行比较,则直接取rs高位进行对比则可,为0则rs为正数>=0
              // pc相关
              io.M_branch_id2pcReg.brach_flag := Branch
              io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
              // idEx相关
              io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
            }
              .otherwise()  // 不满足条件则保持原值
          }
          is(EXE_BGEZAL){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_JUMP_BRANCH
            io.M_id2idEx.aluop := EXE_BGEZAL_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2idEx.waddr := Reg_31
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            io.M_branch_id2idEx.link_addr := r_pc_plus8  // 不管下面的条件是否满足，都将r_pc_plus8写入Reg_31
            when(io.M_id2idEx.reg1(RegBus-1) === False) {  // 视rs为有符号数，与0进行比较,则直接取rs高位进行对比则可,为0则rs为正数>=0
              // pc相关
              io.M_branch_id2pcReg.brach_flag := Branch
              io.M_branch_id2pcReg.brach_targetAddress := r_targetAddr_B // 取r_targetAddr_B的值作为新的指令地址
              // idEx相关
              io.M_branch_id2idEx.next_inst_inDelayslot := InDelaySlot
            }
              .otherwise()  // 不满足条件则保持原值
          }
          // 异常指令
          is(EXE_TEQI){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TEQI_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          is(EXE_TGEI){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TGEI_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          is(EXE_TGEIU){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TGEIU_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          is(EXE_TLTI){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TLTI_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          is(EXE_TLTIU){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TLTIU_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          is(EXE_TNEI){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_TNEI_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            r_imm := U(16 bits,default -> io.S_ifId2id.inst(15))  @@ io.S_ifId2id.inst(15 downto 0)  // 符号位扩展
          }
          default{}
        }
      }
      is(EXE_SPECIAL2_INST){  // op === EXE_SPECIAL2_INST U"6'b011100" 的情况
        switch(op3){
          is(EXE_CLZ){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
            io.M_id2idEx.aluop := EXE_CLZ_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
          }
          is(EXE_CLO){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_ARITHMETIC
            io.M_id2idEx.aluop := EXE_CLO_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
          }
          is(EXE_MUL){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MUL
            io.M_id2idEx.aluop := EXE_MUL_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_MADD){  // 涉及hilo的乘， alusel为的RES_MUL
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MUL
            io.M_id2idEx.aluop := EXE_MADD_OP
            io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_MADDU){  // 涉及hilo的乘， alusel为的RES_MUL
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MUL
            io.M_id2idEx.aluop := EXE_MADDU_OP
            io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_MSUB){  // 涉及hilo的乘， alusel为的RES_MUL
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MUL
            io.M_id2idEx.aluop := EXE_MSUB_OP
            io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
          is(EXE_MSUBU){  // 涉及hilo的乘， alusel为的RES_MUL
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MUL
            io.M_id2idEx.aluop := EXE_MSUBU_OP
            io.M_id2idEx.we := WriteDisable  // 不必写入普通寄存器
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadEnable
          }
        }
      }
      is(COP0){
        switch(io.S_ifId2id.inst(25 downto 21)){
          is(EXE_MTC0){  // CPR[0,rd] <- GPR[rt]
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP  // 将普通寄存器rt的值赋予到协处理器的对应rd地址的寄存器
            io.M_id2idEx.aluop := EXE_MTC0_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadEnable
            io.M_id2regfile.re2 := ReadDisable
            io.M_id2regfile.raddr1 := io.S_ifId2id.inst(20 downto 16)  // 要读取数值的对应CP0类指令rt地址
          }
          is(EXE_MFC0){  // GPR[rt] <- CPR[0,rd]
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_MOVE  // 将协处理器的对应rd地址的寄存器的值赋予到普通寄存器rt，等同于move
            io.M_id2idEx.aluop := EXE_MFC0_OP
            io.M_id2idEx.we := WriteEnable
            io.M_id2idEx.waddr := io.S_ifId2id.inst(20 downto 16)  // 要写入的普通寄存器区rt地址
            io.M_id2regfile.re1 := ReadDisable
            io.M_id2regfile.re2 := ReadDisable
          }
          default()
        }
        switch(op3){
          is(EXE_ERET){
            r_isInstValid := InstValid
            io.M_id2idEx.alusel := EXE_RES_NOP
            io.M_id2idEx.aluop := EXE_ERET_OP
            io.M_id2idEx.we := WriteDisable
            io.M_id2regfile.re1 := ReadDisable
            io.M_id2regfile.re2 := ReadDisable
            r_isEret := hasException
          }
          default()
        }
      }
      default()
    }
    // 与最上层的switch(op)同级,表示 inst[31:21]==0的特殊情况
    when(io.S_ifId2id.inst(31 downto 21) === U"11'b00000000000"){
      switch(op3){
        is(EXE_SLL){
          io.M_id2idEx.we := WriteEnable
          r_isInstValid := InstValid
          io.M_id2idEx.alusel := EXE_RES_SHIFT
          io.M_id2idEx.aluop := EXE_SLL_OP
          io.M_id2regfile.re1 := ReadDisable  // raddr1对应inst[25:21],在sll指令中，只需用到rt即对应inst[20:16]-->raddr2
          io.M_id2regfile.re2 := ReadEnable
          r_imm := io.S_ifId2id.inst(10 downto 6).resize(RegBus bits)  // 取inst[25:21]作为immediate（需要扩充为32bits）
        }
        is(EXE_SRL){
          io.M_id2idEx.we := WriteEnable
          r_isInstValid := InstValid
          io.M_id2idEx.alusel := EXE_RES_SHIFT
          io.M_id2idEx.aluop := EXE_SRL_OP
          io.M_id2regfile.re1 := ReadDisable
          io.M_id2regfile.re2 := ReadEnable
          r_imm := io.S_ifId2id.inst(10 downto 6).resize(RegBus bits)
        }
        is(EXE_SRA) {
          io.M_id2idEx.we := WriteEnable
          r_isInstValid := InstValid
          io.M_id2idEx.alusel := EXE_RES_SHIFT
          io.M_id2idEx.aluop := EXE_SRA_OP
          io.M_id2regfile.re1 := ReadDisable
          io.M_id2regfile.re2 := ReadEnable
          r_imm := io.S_ifId2id.inst(10 downto 6).resize(RegBus bits)
        }
        default{}
      }
    }.otherwise()
  }
  // 数据前推，将执行阶段、访存阶段即已得出结果（标志为re1==true、we==true）的wd直接推往译码器输出
  when(io.rst === RstEnable) {
    io.M_id2idEx.reg1 := ZeroWord
  }
    // ex数据前推(满足条件：regFile可读、ex表示可写，且要读取的regFIle地址与ex要写入地址相同）
    .elsewhen(io.M_id2regfile.re1 === ReadEnable && io.S_PullForward_ex2id.we === WriteEnable &&
      io.M_id2regfile.raddr1 === io.S_PullForward_ex2id.waddr){
      io.M_id2idEx.reg1 := io.S_PullForward_ex2id.wdata
    }
    // mem数据前推(满足条件：regFile可读、ex表示可写，且要读取的regFIle地址与ex要写入地址相同）
    .elsewhen(io.M_id2regfile.re1 === ReadEnable && io.S_PullForward_mem2id.we === WriteEnable &&
      io.M_id2regfile.raddr1 === io.S_PullForward_mem2id.waddr){
      io.M_id2idEx.reg1 := io.S_PullForward_mem2id.wdata
    }
    .elsewhen(io.M_id2regfile.re1 === ReadEnable) { // io.rst === RstDisable 时，可根据是否可读，取值regFile对应数据，送往idEx
    io.M_id2idEx.reg1 := io.M_id2regfile.rdata1  // 否则读取rdata1，此时，raddr1为MIPS指令中的rs索引，指向要进行操作的寄存器地址
  }
    .elsewhen(io.M_id2regfile.re1 === ReadDisable) {
    io.M_id2idEx.reg1 := r_imm  // ReadDisable，输出立即数
  }
    .otherwise {
    io.M_id2idEx.reg1 := ZeroWord
  }
  // 数据前推，将执行阶段、访存阶段即已得出结果（标志为re1==true、we==true）的wd直接推往译码器输出
  when(io.rst === RstEnable) {
    io.M_id2idEx.reg2 := ZeroWord
  }
    // ex数据前推(满足条件：regFile可读、ex表示可写，且要读取的regFIle地址与ex要写入地址相同）
    .elsewhen(io.M_id2regfile.re2 === ReadEnable && io.S_PullForward_ex2id.we === WriteEnable && io.M_id2regfile.raddr2 === io.S_PullForward_ex2id.waddr){
      io.M_id2idEx.reg2 := io.S_PullForward_ex2id.wdata
    }
    // mem数据前推(满足条件：regFile可读、ex表示可写，且要读取的regFIle地址与ex要写入地址相同）
    .elsewhen(io.M_id2regfile.re2 === ReadEnable && io.S_PullForward_mem2id.we === WriteEnable && io.M_id2regfile.raddr2 === io.S_PullForward_mem2id.waddr){
      io.M_id2idEx.reg2 := io.S_PullForward_mem2id.wdata
    }
    .elsewhen(io.M_id2regfile.re2 === ReadEnable) { // io.rst === RstDisable 时，可根据是否可读，取值regFile对应数据，送往idEx
    io.M_id2idEx.reg2 := io.M_id2regfile.rdata2  // 否则读取rdata2，此时，raddr2为MIPS指令中的rt索引，指向操作结果保存的寄存器地址，在逻辑“或”操作中，re2恒为0（不可读）
  }
    .elsewhen(io.M_id2regfile.re2 === ReadDisable) {
    io.M_id2idEx.reg2 := r_imm  // ReadDisable，输出立即数
  }
    .otherwise {
    io.M_id2idEx.reg2 := ZeroWord
  }
}

class idEx extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val S_id2idEx = slave(id2idExInterface(AluOpBus, AluSelBus, RegBus, RegBus, RegAddrBus,InstBus))
    val M_idEx2ex = master(idEx2exInterface(AluOpBus, AluSelBus, RegBus, RegBus, RegAddrBus,InstBus))
    val stall = in UInt(stallDir bits)  // 流水线暂停
    // 转移/分支指令相关
    val S_branch_id2idEx = slave(branchInterce(InstAddrBus))
    val M_branchLink_idEx2ex = master(branchLinkInterface(InstAddrBus))
    // 异常指令相关
    val S_except_id2idEx = slave(exceptTypeAddrInterface(ExceptTypeBus,InstBus))
    val M_except_idEx2ex = master(exceptTypeAddrInterface(ExceptTypeBus,InstBus))
    val flush = in Bool  // ctrl确认发生异常之信号
  }
  // Configure the clock domain，rst==RstEnable
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_inst = RegNext(io.S_id2idEx.inst) init(ZeroWord)  // 向后派发指令码（储存指令相关）
    val r_aluop = Reg(io.S_id2idEx.aluop) init (EXE_NOP_OP)
    val r_alusel = Reg(io.S_id2idEx.alusel) init (EXE_RES_NOP)
    val r_reg1 = Reg(io.S_id2idEx.reg1) init (ZeroWord)
    val r_reg2 = Reg(io.S_id2idEx.reg2) init (ZeroWord)
    val r_wd = Reg(io.S_id2idEx.waddr) init (NOPRegAddr)  // MIPS指令中的rt索引，指向操作结果保存的寄存器地址
    val r_we = Reg(io.S_id2idEx.we) init (WriteDisable)
    // 转移/分支指令相关缓存
    val r_is_inDelayslot = Reg(io.S_branch_id2idEx.is_inDelayslot) init(NotInDelaySlot)
    val r_link_addr = Reg(io.S_branch_id2idEx.link_addr) init(ZeroWord)
    val r_next_inDelayslot = Reg(io.S_branch_id2idEx.next_inst_inDelayslot) init(NotInDelaySlot)
    // 异常指令相关缓存
    val r_except = Reg(io.S_except_id2idEx)
    r_except.exceptType init(ZeroWord)
    r_except.currentInst_addr init(ZeroWord)

    when(io.flush === hasException){  // 有异常发生
      // 清除流水线缓存
      r_aluop := EXE_NOP_OP
      r_alusel := EXE_RES_NOP
      r_reg1 := ZeroWord
      r_reg2 := ZeroWord
      r_wd := NOPRegAddr
      r_we := WriteDisable
      // 转移/分支指令相关
      r_is_inDelayslot := NotInDelaySlot
      r_link_addr := ZeroWord
      r_next_inDelayslot := NotInDelaySlot
      // 异常相关
      r_except.exceptType := ZeroWord
      r_except.currentInst_addr := ZeroWord
    }
      .otherwise{
        when(io.stall(2) === NoStop){  // 译码阶段（id)继续（id受ifId影响）
          r_aluop := io.S_id2idEx.aluop
          r_alusel := io.S_id2idEx.alusel
          r_reg1 := io.S_id2idEx.reg1
          r_reg2 := io.S_id2idEx.reg2
          r_wd := io.S_id2idEx.waddr
          r_we := io.S_id2idEx.we
          // 转移/分支指令相关
          r_is_inDelayslot := io.S_branch_id2idEx.is_inDelayslot
          r_link_addr := io.S_branch_id2idEx.link_addr
          r_next_inDelayslot := io.S_branch_id2idEx.next_inst_inDelayslot
          // 异常相关
          r_except.exceptType := io.S_except_id2idEx.exceptType
          r_except.currentInst_addr := io.S_except_id2idEx.currentInst_addr
        }
          .elsewhen(io.stall(2)===Stop && io.stall(3)===NoStop) {  // 译码阶段（id)暂停，执行阶段(ex)继续，置入空指令，通过idEx的output影响ex
            r_aluop := EXE_NOP_OP
            r_alusel := EXE_RES_NOP
            r_reg1 := ZeroWord
            r_reg2 := ZeroWord
            r_wd := NOPRegAddr
            r_we := WriteDisable
            // 转移/分支指令相关
            r_is_inDelayslot := NotInDelaySlot
            r_link_addr := ZeroWord
            r_next_inDelayslot := NotInDelaySlot
          }
          .otherwise{} // 其余（译码暂停，执行也暂停），保持上一阶段值
      }
  }
  io.M_idEx2ex.aluop := areaClk.r_aluop
  io.M_idEx2ex.alusel := areaClk.r_alusel
  io.M_idEx2ex.reg1 := areaClk.r_reg1
  io.M_idEx2ex.reg2 := areaClk.r_reg2
  io.M_idEx2ex.waddr := areaClk.r_wd
  io.M_idEx2ex.we := areaClk.r_we
  io.M_idEx2ex.inst := areaClk.r_inst

  io.M_branchLink_idEx2ex.is_inDelayslot := areaClk.r_is_inDelayslot
  io.M_branchLink_idEx2ex.link_addr := areaClk.r_link_addr
  io.S_branch_id2idEx.check_inDelayslot := areaClk.r_next_inDelayslot
  io.M_except_idEx2ex <> areaClk.r_except
}

class ex extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val rst = in Bool
    val S_idEx2ex = slave(idEx2exInterface(AluOpBus, AluSelBus, RegBus, RegBus, RegAddrBus,InstBus))
    val M_ex2exMem = master(wInterface(RegAddrBus, RegBus))
    val S_hiloData_hiloReg2ex = slave(hiloRegInterface(RegBus))
    val M_hilo_ex2exMem = master(hiloInterface(RegBus))
    // 与HI、LO有关的数据前推
    val S_hilo_mem2ex = slave(hiloInterface(RegBus))
    val S_hilo_memWb2ex = slave(hiloInterface(RegBus))
    // 流水线暂停相关
    val stallCtrl = out Bool
    // 流水线暂停相关（多周期计算的hi、lo的数据暂存）
    val M_tmpStall_ex2mem_ex = master(stallHiLoInterface(DoubleRegWidth,cntWidth))
    val S_tmpStall_mem_ex2ex = slave(stallHiLoInterface(DoubleRegWidth,cntWidth))
    // 与Div模块的互联，除法相关
    val M_divConnect_ex2div = master(divConnectInterface(RegBus))
    val S_divResult_div2ex = slave(divResultInterface(DoubleRegBus))
    // 转移/分支指令相关
    val S_branchLink_idEx2ex = slave(branchLinkInterface(InstAddrBus))
    // 加载/储存相关
    val M_loadstore_ex2exMem = master(load_storeInterface(AluOpBus,DataAddrBus,DataBus))
    // 加载指令需要ex模块返回id模块的指令类型
    val aluop_ex2id = out UInt(AluOpBus bits)
    // 协处理器相关
    val M_cpu0Write_ex2exMem = master(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val M_cpu0Read_ex2cpu0Reg = master(cpu0ReadInterface(CPU0AddrBus,CPU0RegBus))
    // 协处理器-数据前推（写相关）
    val S_PullForward_cpu0Write_mem2ex = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val S_PullForword_cpu0Write_memWb2ex = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    // 异常相关
    val S_except_idEx2ex = slave(exceptTypeAddrInterface(ExceptTypeBus,InstBus))
    val M_exception_ex2exMem = master(exceptInterface(ExceptTypeBus,InstBus))
  }
  val r_logicOut = UInt(RegBus bits)  // 逻辑运算结果
  r_logicOut := ZeroWord
  val r_shifRes = UInt(RegBus bits)  // 移位运算结果
  r_shifRes := ZeroWord
  val r_moveRes = UInt(RegBus bits)  // 移动运算结果
  r_moveRes := ZeroWord
  val r_hi = UInt(RegBus bits)
  r_hi := ZeroWord
  val r_lo = UInt(RegBus bits)
  r_lo := ZeroWord
  val r_arithmeticRes = UInt(RegBus bits)
  r_arithmeticRes := ZeroWord  // 算术结果
  val r_tmp_hilo = UInt(DoubleRegWidth bits)  // 准备写入hi、lo特殊寄存器的值
  r_tmp_hilo := ZeroWord @@ ZeroWord
  val r_stallByM_ = Bool  // 流水线暂停控制 与乘累加、乘累减相关
  r_stallByM_ := NoStop
  val r_stallByDiv = Bool  // 与除法相关的流水线暂停
  r_stallByDiv := NoStop
  // 加载指令需要ex模块返回id模块的指令类型
  io.aluop_ex2id := io.S_idEx2ex.aluop

  io.stallCtrl := r_stallByM_ || r_stallByDiv  // 连接到外部的output stallCtrl ，暂停与乘累加/乘累减/除法相关
  // 加载/储存相关
  io.M_loadstore_ex2exMem.aluop := io.S_idEx2ex.aluop  // 将操作类型传递给访存（加载/储存相关）
  io.M_loadstore_ex2exMem.memAddr := io.S_idEx2ex.reg1 +
    (U(16 bits,default -> io.S_idEx2ex.inst(15))  @@ io.S_idEx2ex.inst(15 downto 0))  // 计算加载/储存指令涉及的内存ram地址（基于偏移offset）
  io.M_loadstore_ex2exMem.memReg := io.S_idEx2ex.reg2  // 储存指令中，要写入GPR的数据/初始rt的数据（即rt位置，ex模块中的reg2）

  // 避免latch
  io.M_divConnect_ex2div.start := DivStop
  io.M_divConnect_ex2div.opData1 := ZeroWord
  io.M_divConnect_ex2div.opData2 := ZeroWord
  io.M_divConnect_ex2div.signalDiv := DivNotSignal

  // 避免latch（读取cpu0Reg的数据）
  io.M_cpu0Read_ex2cpu0Reg.addr := NOPRegAddr

  val r_reg2_mux = UInt(RegBus bits)  // 在算术运算中使用，reg2数据选择寄存器（wire形式），若aluop为减法操作、有符号比较操作，保存reg2补码，否则保存reg2原码
  r_reg2_mux := (io.S_idEx2ex.aluop === EXE_SUB_OP |
                 io.S_idEx2ex.aluop === EXE_SUBU_OP |
                 io.S_idEx2ex.aluop === EXE_SLT_OP) ? ((~io.S_idEx2ex.reg2) + U(1,RegBus bits)) | io.S_idEx2ex.reg2
  // 若aluop为加法操作，则 r_result_sum = reg1 + r_reg2_mux = reg1 + reg2
  // 若aluop为减法操作，则 r_result_sum = reg1 + r_reg2_mux = reg1 + (reg2)补 = reg1 - reg2
  // 若aluop为有符号比较操作，则r_result_sum等同于减法操作，通过比较r_result_sum是否小于零得出reg1是否小于reg2
  val r_result_sum = UInt(RegBus bits)
  r_result_sum := io.S_idEx2ex.reg1 + r_reg2_mux
  // 判断是否溢出
  // 溢出情况，对于加法、减法操作只有以下两种：
  // 正数(最高位为0)+正数(最高位为0)，和为负数（最高位为1） 、 负数(最高位为1)+负数(最高位为1)，和为正数（最高位为0）
  // 注意，r_result_sum中，减法被转化为加法，即
  // 若aluop为减法，r_result_sum = reg1 - reg2 = reg1 +(-reg2)= reg1 + r_reg2_mux
  // 若aluop为加法，r_result_sum = reg1 + reg2 = reg1 +(reg2)= reg1 + r_reg2_mux
  // 判断reg1、r_reg2_mux、r_result_sum的最高位即可判断溢出
  // 可以写成 (reg1[31] ⊙ r_reg2_mux) && (reg1[31] ^ r_result_sum[31]),但消耗的门电路可能不如直接写逻辑
  val r_overflow_sum = Bool
  r_overflow_sum := (~(io.S_idEx2ex.reg1(RegBus-1) ^ r_reg2_mux(RegBus-1)))  &&  (io.S_idEx2ex.reg1(RegBus-1) ^ r_result_sum(RegBus-1))
  // 比较运算，分为有符号数比较、无符号数比较
  // 无符号数可直接比较
  // 有符号数比较，r_reg1_slt_reg2 = (reg1<reg2) 满足以下条件为真
  // reg1为负数、reg2为正数
  // reg1为正数、reg2为正数，且 reg1-reg2<0 （等同于r_result_sum为负数）
  // reg1为负数，reg2为负数，且 reg1-reg2<0  （等同于r_result_sum为负数）（可以写成异或表达式,但消耗的门电路可能不如直接写逻辑）
  val r_reg1_lt_reg2 = Bool
  // 加入自陷指令相关
  val r_trapInst = Bool
  r_trapInst := io.S_idEx2ex.aluop.mux(
    EXE_TGE_OP -> (True),
    EXE_TLT_OP -> (True),
    EXE_TGEI_OP -> (True),
    EXE_TLTI_OP -> (True),
    default -> (False)
  )
  r_reg1_lt_reg2 := (io.S_idEx2ex.aluop === EXE_SLT_OP | io.S_idEx2ex.aluop === EXE_SLTI_OP | r_trapInst) ?
    ((io.S_idEx2ex.reg1(RegBus-1) && ~(io.S_idEx2ex.reg2(RegBus-1))) ||
      (~(io.S_idEx2ex.reg1(RegBus-1)^io.S_idEx2ex.reg2(RegBus-1)) && r_result_sum(RegBus-1))) |
    (io.S_idEx2ex.reg1 < io.S_idEx2ex.reg2)

  // rlo\rlz操作 使用递归函数，rawReg中的自最高位自最低位方向检查，直至遇到值为1的位，统计0的个数
  def get_CLZ(index:Int,rawReg:UInt):UInt ={
    if(index==0){
      return rawReg(index) ? U(RegBus-1,RegBus bits) | U(RegBus,RegBus bits)
    }
    else {
      return rawReg(index) ? U(RegBus-1-index,RegBus bits) | get_CLZ(index-1,rawReg)
    }
  }
  val r_clz = get_CLZ(RegBus-1,io.S_idEx2ex.reg1)
  val r_clo = get_CLZ(RegBus-1,~io.S_idEx2ex.reg1)

  // 乘法操作
  // 乘法运算的被乘数，若为负数则取其补码；同理，乘法运算的乘数，若为负数则取其补码，使得乘法运算本质上是正数*正数，最后予以校正
  val r_mul_data1 = UInt(RegBus bits)
  // 当为有符号数相乘(MUL\MULT)(包括乘累加/乘累减运算，MADD、MSUB)且数为负数,取其补码；否则取其原码
  r_mul_data1 := ((io.S_idEx2ex.aluop === EXE_MUL_OP || io.S_idEx2ex.aluop === EXE_MULT_OP ||
    io.S_idEx2ex.aluop === EXE_MADD_OP || io.S_idEx2ex.aluop === EXE_MSUB_OP) &&
    (io.S_idEx2ex.reg1(RegBus-1))) ? (~io.S_idEx2ex.reg1 + U(1,RegBus bits)) | io.S_idEx2ex.reg1
  val r_mul_data2 = UInt(RegBus bits)
  // 当为有符号数相乘(MUL\MULT)(包括乘累加/乘累减运算，MADD、MSUB)且数为负数,取其补码；否则取其原码
  r_mul_data2 := ((io.S_idEx2ex.aluop === EXE_MUL_OP || io.S_idEx2ex.aluop === EXE_MULT_OP ||
    io.S_idEx2ex.aluop === EXE_MADD_OP || io.S_idEx2ex.aluop === EXE_MSUB_OP) &&
    (io.S_idEx2ex.reg2(RegBus-1))) ? (~io.S_idEx2ex.reg2 + U(1,RegBus bits)) | io.S_idEx2ex.reg2
  // 暂时保存的乘法结果
  val r_mulRes_tmp = UInt(DoubleRegBus bits)
  r_mulRes_tmp := r_mul_data1 * r_mul_data2
  // 修正乘法运算
  // 若为有符号运算(MUL\MULT)(包括乘累加/乘累减运算，MADD、MSUB)，且操作数一正一负（异或），则r_mul_tmp取反即取补码；其余都为r_mul_tmp
  val r_mulRes = UInt(DoubleRegBus bits)
  r_mulRes := ((io.S_idEx2ex.aluop === EXE_MUL_OP || io.S_idEx2ex.aluop === EXE_MULT_OP ||
    io.S_idEx2ex.aluop === EXE_MADD_OP || io.S_idEx2ex.aluop === EXE_MSUB_OP) &&
    (io.S_idEx2ex.reg1(RegBus-1) ^ io.S_idEx2ex.reg2(RegBus-1))) ? (~r_mulRes_tmp + U(1,DoubleRegBus bits)) | r_mulRes_tmp

  io.M_ex2exMem.waddr := io.S_idEx2ex.waddr  // MIPS指令中的rt索引，指向操作结果保存的寄存器地址
  io.M_ex2exMem.we := io.S_idEx2ex.we   // Write Enable
  // 根据操作指令
  // 若为加、减法，且为溢出即不写入的操作指令，写入标记位记为0
  when(r_overflow_sum){
    switch(io.S_idEx2ex.aluop){
      is(EXE_ADD_OP,EXE_ADDI_OP,EXE_SUB_OP){
        io.M_ex2exMem.we := WriteDisable
      }
    }
  }
  // 运算的类型：逻辑、移位、nop（空）
  switch(io.S_idEx2ex.alusel) {
    is(EXE_RES_LOGIC) {
      io.M_ex2exMem.wdata := r_logicOut
    }
    is(EXE_RES_SHIFT){
      io.M_ex2exMem.wdata := r_shifRes
    }
    is(EXE_RES_MOVE){
      io.M_ex2exMem.wdata := r_moveRes
    }
    is(EXE_RES_ARITHMETIC){
      io.M_ex2exMem.wdata := r_arithmeticRes
    }
    is(EXE_RES_MUL){  // 仅在指令 MUL取得，MULT、MULTU都是存入hi、lo特殊寄存器，alusel为nop
      io.M_ex2exMem.wdata := r_mulRes(RegBus-1 downto 0)  // 存入低32位
    }
    is(EXE_RES_JUMP_BRANCH){  // 转移/分支指令下，延迟槽后的第一条指令地址写入普通寄存器
      io.M_ex2exMem.wdata := io.S_branchLink_idEx2ex.link_addr
    }
    default {
      io.M_ex2exMem.wdata := ZeroWord
    }
  }
  // 具体运算
  when(io.rst === RstEnable) {
    r_logicOut := ZeroWord
    r_shifRes := ZeroWord
    r_moveRes := ZeroWord
    r_arithmeticRes := ZeroWord
  }.otherwise {
    switch(io.S_idEx2ex.aluop) {
      // 逻辑运算
      is(EXE_OR_OP) {
        r_logicOut := io.S_idEx2ex.reg1 | io.S_idEx2ex.reg2
      }
      is(EXE_AND_OP){
        r_logicOut := io.S_idEx2ex.reg1 & io.S_idEx2ex.reg2
      }
      is(EXE_NOR_OP){
        r_logicOut := ~(io.S_idEx2ex.reg1 | io.S_idEx2ex.reg2)
      }
      is(EXE_XOR_OP){
        r_logicOut := io.S_idEx2ex.reg1 ^ io.S_idEx2ex.reg2
      }
      // 移位运算
      is(EXE_SLL_OP){
        r_shifRes := io.S_idEx2ex.reg2 |<< io.S_idEx2ex.reg1(4 downto 0)  // Logical shift left (output width == input width),算术左移和逻辑左移无区别
      }
      is(EXE_SRL_OP){
        r_shifRes := io.S_idEx2ex.reg2 |>> io.S_idEx2ex.reg1(4 downto 0)  // UInt下，翻译为Verilog为>>>，结果与逻辑右移无差别
      }
      is(EXE_SRA_OP){
        r_shifRes := (io.S_idEx2ex.reg2.asSInt |>> io.S_idEx2ex.reg1(4 downto 0)).asUInt  // 利用SInt实现算术右移
      }
      // 移动运算
      is(EXE_MOVN_OP){
        r_moveRes := io.S_idEx2ex.reg1
      }
      is(EXE_MOVZ_OP){
        r_moveRes := io.S_idEx2ex.reg1
      }
      is(EXE_MFHI_OP){
        r_moveRes := r_hi
      }
      is(EXE_MFLO_OP){
        r_moveRes := r_lo
      }
      // 协处理器MFC0,实质也为写入RPG[rt]的移动操作
      is(EXE_MFC0_OP){  // 不涉及向cpu0Reg的写入
        io.M_cpu0Read_ex2cpu0Reg.addr := io.S_idEx2ex.inst(15 downto 11)
        // 数据前推,解决与mem、memWb的数据相关问题
        when(io.S_PullForword_cpu0Write_memWb2ex.we === WriteEnable &&
          io.M_cpu0Read_ex2cpu0Reg.addr === io.S_PullForword_cpu0Write_memWb2ex.addr){
          r_moveRes := io.S_PullForword_cpu0Write_memWb2ex.data
        }
          .elsewhen(io.S_PullForward_cpu0Write_mem2ex.we === WriteEnable &&
            io.M_cpu0Read_ex2cpu0Reg.addr === io.S_PullForward_cpu0Write_mem2ex.addr){
            r_moveRes := io.S_PullForward_cpu0Write_mem2ex.data
          }
          .otherwise{
            r_moveRes := io.M_cpu0Read_ex2cpu0Reg.data  // 返回从协处理器中读得的对应rd地址(inst[15:11]寄存器值
          }
      }
      // 算术运算
      is(EXE_SLT_OP,EXE_SLTU_OP,EXE_SLTI_OP,EXE_SLTIU_OP){  // 比较运算
        r_arithmeticRes := r_reg1_lt_reg2.asUInt.resized
      }
      is(EXE_ADD_OP,EXE_ADDU_OP,EXE_ADDI_OP,EXE_ADDIU_OP){  // 加法运算
        r_arithmeticRes := r_result_sum
      }
      is(EXE_SUB_OP,EXE_SUBU_OP){  // 减法运算
        r_arithmeticRes := r_result_sum
      }
      is(EXE_CLZ_OP){
        r_arithmeticRes := r_clz
      }
      is(EXE_CLO_OP){
        r_arithmeticRes := r_clo
      }
      // 除法运算
      is(EXE_DIV_OP){
        when(io.S_divResult_div2ex.ready === DivResultNotReady){  // 除法没有计算完成
          io.M_divConnect_ex2div.start := DivStart  // 启动除法计算
          io.M_divConnect_ex2div.opData1 := io.S_idEx2ex.reg1
          io.M_divConnect_ex2div.opData2 := io.S_idEx2ex.reg2
          io.M_divConnect_ex2div.signalDiv := DivIsSignal
          r_stallByDiv := Stop  // 请求暂停流水线
        }
          .elsewhen(io.S_divResult_div2ex.ready === DivResultReady){  // 除法计算完毕
            io.M_divConnect_ex2div.start := DivStop  // 停止除法计算
            io.M_divConnect_ex2div.opData1 := io.S_idEx2ex.reg1
            io.M_divConnect_ex2div.opData2 := io.S_idEx2ex.reg2
            io.M_divConnect_ex2div.signalDiv := DivIsSignal
            r_stallByDiv :=  NoStop // 不暂停流水线
          }
        is(EXE_DIVU_OP){
          when(io.S_divResult_div2ex.ready === DivResultNotReady){  // 除法没有计算完成
            io.M_divConnect_ex2div.start := DivStart  // 启动除法计算
            io.M_divConnect_ex2div.opData1 := io.S_idEx2ex.reg1
            io.M_divConnect_ex2div.opData2 := io.S_idEx2ex.reg2
            io.M_divConnect_ex2div.signalDiv := DivNotSignal
            r_stallByDiv := Stop  // 请求暂停流水线
          }
            .elsewhen(io.S_divResult_div2ex.ready === DivResultReady){  // 除法计算完毕
              io.M_divConnect_ex2div.start := DivStop  // 停止除法计算
              io.M_divConnect_ex2div.opData1 := io.S_idEx2ex.reg1
              io.M_divConnect_ex2div.opData2 := io.S_idEx2ex.reg2
              io.M_divConnect_ex2div.signalDiv := DivNotSignal
              r_stallByDiv :=  NoStop // 不暂停流水线
            }
        }
      }
      default()
    }
  }

  // 特殊寄存器hi、lo的中间连线的执行操作
  when(io.rst === RstEnable){
    r_hi := ZeroWord
    r_lo := ZeroWord
  }
    .elsewhen(io.S_hilo_mem2ex.w_hiloEnable === WriteEnable){
      r_hi := io.S_hilo_mem2ex.hi
      r_lo := io.S_hilo_mem2ex.lo
    }
    .elsewhen(io.S_hilo_memWb2ex.w_hiloEnable === WriteEnable){
      r_hi := io.S_hilo_memWb2ex.hi
      r_lo := io.S_hilo_memWb2ex.lo
    }
    .otherwise{
      r_hi := io.S_hiloData_hiloReg2ex.hi
      r_lo := io.S_hiloData_hiloReg2ex.lo
    }
  // 处理特殊寄存器hi\lo的写入运算
  when(io.rst === RstEnable){
    io.M_hilo_ex2exMem.hi := ZeroWord
    io.M_hilo_ex2exMem.lo := ZeroWord
    io.M_hilo_ex2exMem.w_hiloEnable := WriteDisable
  }
    .elsewhen(io.S_idEx2ex.aluop === EXE_MTHI){
      io.M_hilo_ex2exMem.hi := io.S_idEx2ex.reg1
      io.M_hilo_ex2exMem.lo := r_lo
      io.M_hilo_ex2exMem.w_hiloEnable := WriteEnable
    }
    .elsewhen(io.S_idEx2ex.aluop === EXE_MTLO){
      io.M_hilo_ex2exMem.hi := r_hi
      io.M_hilo_ex2exMem.lo := io.S_idEx2ex.reg1
      io.M_hilo_ex2exMem.w_hiloEnable := WriteEnable
    }
    .elsewhen(io.S_idEx2ex.aluop === EXE_MULT_OP || io.S_idEx2ex.aluop === EXE_MULTU_OP){  // 存入特殊寄存器的乘法结果
      io.M_hilo_ex2exMem.hi := r_mulRes(DoubleRegBus-1 downto RegBus)
      io.M_hilo_ex2exMem.lo := r_mulRes(RegBus-1 downto 0)
      io.M_hilo_ex2exMem.w_hiloEnable := WriteEnable
    }
    // 存入乘累加、乘累减操作的结果
    .elsewhen(io.S_idEx2ex.aluop === EXE_MADD_OP || io.S_idEx2ex.aluop === EXE_MADDU_OP ||
    io.S_idEx2ex.aluop === EXE_MSUB_OP || io.S_idEx2ex.aluop === EXE_MSUBU_OP){
      io.M_hilo_ex2exMem.hi := r_tmp_hilo(DoubleRegBus-1 downto RegBus)
      io.M_hilo_ex2exMem.lo := r_tmp_hilo(RegBus-1 downto 0)
      io.M_hilo_ex2exMem.w_hiloEnable := WriteEnable
    }
    // 存入除法操作的结果
    .elsewhen(io.S_idEx2ex.aluop === EXE_DIV_OP || io.S_idEx2ex.aluop === EXE_DIVU_OP){
      io.M_hilo_ex2exMem.hi := io.S_divResult_div2ex.resultDiv(DoubleRegBus-1 downto RegBus)
      io.M_hilo_ex2exMem.lo := io.S_divResult_div2ex.resultDiv(RegBus-1 downto 0)
      io.M_hilo_ex2exMem.w_hiloEnable := WriteEnable
    }
    .otherwise{  // 这意味着，若当次指令不是mthi/mtlo，hi、lo特殊寄存器即刻清零
      io.M_hilo_ex2exMem.hi := ZeroWord
      io.M_hilo_ex2exMem.lo := ZeroWord
      io.M_hilo_ex2exMem.w_hiloEnable := WriteDisable
    }

  // 流水线暂停相关，ex对exMem的特殊寄存器缓存相关输出
  // 乘累加/乘累减
  io.M_tmpStall_ex2mem_ex.hilo_temp := ZeroWord @@ ZeroWord  // 注意避免latch
  io.M_tmpStall_ex2mem_ex.cnt := U"2'b00"  // 注意避免latch
  when(io.rst === RstEnable){
    io.M_tmpStall_ex2mem_ex.hilo_temp := ZeroWord @@ ZeroWord
    io.M_tmpStall_ex2mem_ex.cnt := U"2'b00"
  }
    .otherwise{
      switch(io.S_idEx2ex.aluop){
        is(EXE_MADD_OP,EXE_MADDU_OP){
          when(io.S_tmpStall_mem_ex2ex.cnt === U"2'b00"){  // 执行第一个时钟周期
            io.M_tmpStall_ex2mem_ex.hilo_temp := r_mulRes
            io.M_tmpStall_ex2mem_ex.cnt := U"2'b01"
            r_tmp_hilo := r_hi @@ r_lo  // 准备写入hi、lo特殊寄存器的值(保存上一时刻值） r_tmp_hilo已在定义时赋默认值避免latch
            r_stallByM_ := Stop  // 触发流水线暂停 r_stallCtrl已在定义时赋默认值避免latch
          }
            .elsewhen(io.S_tmpStall_mem_ex2ex.cnt === U"2'b01"){
              io.M_tmpStall_ex2mem_ex.hilo_temp := ZeroWord @@ ZeroWord  // EX已计算完毕，mem_ex的缓存清零
              io.M_tmpStall_ex2mem_ex.cnt := U"2'b10"
              r_tmp_hilo := io.S_tmpStall_mem_ex2ex.hilo_temp + r_hi @@ r_lo
              r_stallByM_ := NoStop
            }
      }
        is(EXE_MSUB_OP,EXE_MSUBU_OP){
          when(io.S_tmpStall_mem_ex2ex.cnt === U"2'b00"){  // 执行第一个时钟周期
            io.M_tmpStall_ex2mem_ex.hilo_temp := ~r_mulRes + U(1,DoubleRegBus bits)  // 为后续的减法预先做好补码准备
            io.M_tmpStall_ex2mem_ex.cnt := U"2'b01"
            r_tmp_hilo := r_hi @@ r_lo  // 准备写入hi、lo特殊寄存器的值(保存上一时刻值） r_tmp_hilo已在定义时赋默认值避免latch
            r_stallByM_ := Stop  // 触发流水线暂停 r_stallCtrl已在定义时赋默认值避免latch
          }
            .elsewhen(io.S_tmpStall_mem_ex2ex.cnt === U"2'b01"){
              io.M_tmpStall_ex2mem_ex.hilo_temp := ZeroWord @@ ZeroWord  // EX已计算完毕，mem_ex的缓存清零
              io.M_tmpStall_ex2mem_ex.cnt := U"2'b10"
              r_tmp_hilo := io.S_tmpStall_mem_ex2ex.hilo_temp + r_hi @@ r_lo
              r_stallByM_ := NoStop
            }
        }
        default{}
      }
    }

  // 写入协处理器cpu0Reg
  // 避免latch
  io.M_cpu0Write_ex2exMem.we := WriteDisable
  io.M_cpu0Write_ex2exMem.addr := NOPRegAddr
  io.M_cpu0Write_ex2exMem.data := ZeroWord
  when(io.rst === RstEnable){
    io.M_cpu0Write_ex2exMem.we := WriteDisable
    io.M_cpu0Write_ex2exMem.addr := NOPRegAddr
    io.M_cpu0Write_ex2exMem.data := ZeroWord
  }
    .otherwise{
      switch(io.S_idEx2ex.aluop){
        is(EXE_MTC0_OP){
          io.M_cpu0Write_ex2exMem.we := WriteEnable
          io.M_cpu0Write_ex2exMem.addr := io.S_idEx2ex.inst(15 downto 11)
          io.M_cpu0Write_ex2exMem.data := io.S_idEx2ex.reg1
        }
        default()
      }
    }

  // 异常判断（自陷异常、溢出异常）
  val r_isOverflow = Bool  // 溢出异常标记位（跟着r_overflow_sum走，不用特别reset）
  when((io.S_idEx2ex.aluop === EXE_ADD_OP || io.S_idEx2ex.aluop === EXE_ADDI_OP ||
    io.S_idEx2ex.aluop === EXE_SUB_OP) && r_overflow_sum){
    r_isOverflow := hasException
  }
    .otherwise{
      r_isOverflow := noException
    }
  val r_isTrap = Bool  // 自陷异常标记位
  r_isTrap := noException   // 防止latch
  when(io.rst === RstEnable){
    r_isTrap := noException
  }
    .otherwise{
      switch(io.S_idEx2ex.aluop){
        is(EXE_TEQ_OP,EXE_TEQI_OP){
          when(io.S_idEx2ex.reg1 === io.S_idEx2ex.reg2){
            r_isTrap := hasException
          }
        }
        is(EXE_TGE_OP,EXE_TGEU_OP,EXE_TGEI_OP,EXE_TGEIU_OP){
          when(~r_reg1_lt_reg2){  // reg1 不小于 reg2
            r_isTrap := hasException
          }
        }
        is(EXE_TLT_OP,EXE_TLTU_OP,EXE_TLTI_OP,EXE_TLTIU_OP){
          when(r_reg1_lt_reg2){  // reg1 小于 reg2
            r_isTrap := hasException
          }
        }
        is(EXE_TNE_OP,EXE_TNEI_OP){
          when(io.S_idEx2ex.reg1 =/= io.S_idEx2ex.reg2){
            r_isTrap := hasException
          }
        }
      }
    }
  io.M_exception_ex2exMem.is_inDelayslot := io.S_branchLink_idEx2ex.is_inDelayslot  // 将延迟槽标记位传递给下一模块
  io.M_exception_ex2exMem.currentInst_addr := io.S_except_idEx2ex.currentInst_addr  // 将地址传递给下一模块
  // 第0-7bit exceptiontype的低8bit留给外部中断 [7:0]
  // 第8bit表示是否系统调用异常syscall [8]
  // 第9bit表示是否无效指令异常InstInvalid [9]
  // 第10bit表示是否自陷异常trap(自陷异常trap需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [10]
  // 第11bit表示是否溢出异常overflow(溢出异常overflow需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [11]
  // 第12bit表示是否异常返回eret [12]
  io.M_exception_ex2exMem.exceptType := io.S_except_idEx2ex.exceptType(31 downto 12) @@
    r_isOverflow.asUInt @@ r_isTrap @@ io.S_except_idEx2ex.exceptType(9 downto 0)
}

class exMem extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val S_ex2exMem = slave(wInterface(RegAddrBus, RegBus))
    val M_exMem2mem = master(wInterface(RegAddrBus, RegBus))
    val S_hilo_ex2exMem = slave(hiloInterface(RegBus))
    val M_hilo_exMem2mem = master(hiloInterface(RegBus))
    val stall = in UInt(stallDir bits)
    // 涉及流水线暂停的多周期操作相关缓存
    val S_tmpStall_ex2mem_ex = slave(stallHiLoInterface(DoubleRegWidth,cntWidth))
    val M_tmpStall_mem_ex2ex = master(stallHiLoInterface(DoubleRegWidth,cntWidth))
    // 加载/储存相关
    val S_loadstore_ex2exMem = slave(load_storeInterface(AluOpBus,DataAddrBus,DataBus))
    val M_loadstore_exMem2mem = master(load_storeInterface(AluOpBus,DataAddrBus,DataBus))
    // 协处理器写相关
    val S_cpu0Write_ex2exMem = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val M_cpu0Write_exMem2mem = master(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    // 异常相关
    val S_exception_ex2exMem = slave(exceptInterface(ExceptTypeBus,InstBus))
    val M_exception_exMem2mem = master(exceptInterface(ExceptTypeBus,InstBus))
    val flush = in Bool  // ctrl确认发生异常之信号
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_S_ex2exMem = Reg(io.S_ex2exMem)
    r_S_ex2exMem.wdata.init(ZeroWord)
    r_S_ex2exMem.waddr.init(NOPRegAddr)
    r_S_ex2exMem.we.init(WriteDisable)
    val r_S_hilo = Reg(io.S_hilo_ex2exMem)
    r_S_hilo.hi.init(ZeroWord)
    r_S_hilo.lo.init(ZeroWord)
    r_S_hilo.w_hiloEnable.init(WriteDisable)
    val r_M_tmpStall_mem_ex2ex = Reg(io.M_tmpStall_mem_ex2ex)
    r_M_tmpStall_mem_ex2ex.hilo_temp.init(ZeroWord@@ZeroWord)
    r_M_tmpStall_mem_ex2ex.cnt.init(U"2'b00")
    // 加载/储存相关
    val r_loadstore = Reg(io.S_loadstore_ex2exMem)
    r_loadstore.aluop.init(EXE_NOP_OP)
    r_loadstore.memAddr.init(ZeroWord)
    r_loadstore.memReg.init(ZeroWord)
    // 协处理器写相关
    val r_cpu0Write = Reg(io.S_cpu0Write_ex2exMem)
    r_cpu0Write.we init(WriteDisable)
    r_cpu0Write.addr init(NOPRegAddr)
    r_cpu0Write.data init(ZeroWord)
    // 异常相关
    val r_exception = Reg(io.S_exception_ex2exMem)
    r_exception.exceptType init(ZeroWord)
    r_exception.currentInst_addr init(ZeroWord)
    r_exception.is_inDelayslot init(NotInDelaySlot)
    when(io.flush === hasException){  // 有异常发生
      // 缓存寄存器初始化
      r_S_ex2exMem.wdata := ZeroWord
      r_S_ex2exMem.waddr := NOPRegAddr
      r_S_ex2exMem.we := WriteDisable
      r_S_hilo.hi := ZeroWord
      r_S_hilo.lo := ZeroWord
      r_S_hilo.w_hiloEnable := WriteDisable
      r_M_tmpStall_mem_ex2ex <> io.S_tmpStall_ex2mem_ex
      // 加载/储存相关
      r_loadstore.aluop := EXE_NOP_OP
      r_loadstore.memAddr := ZeroWord
      r_loadstore.memReg := ZeroWord
      // 协处理器写
      r_cpu0Write.we := WriteDisable
      r_cpu0Write.addr := NOPRegAddr
      r_cpu0Write.data := ZeroWord
      // 异常相关
      r_exception.exceptType := ZeroWord
      r_exception.currentInst_addr := ZeroWord
      r_exception.is_inDelayslot := NotInDelaySlot
    }
      .otherwise{
        when(io.stall(3) === NoStop){  // 执行阶段（ex）不暂停（ex受idEx的output影响）
          r_S_ex2exMem := io.S_ex2exMem
          r_S_hilo := io.S_hilo_ex2exMem
          r_M_tmpStall_mem_ex2ex.hilo_temp := ZeroWord@@ZeroWord
          r_M_tmpStall_mem_ex2ex.cnt := U"2'b00"
          // 加载/储存相关
          r_loadstore <> io.S_loadstore_ex2exMem
          // 协处理器写
          r_cpu0Write <> io.S_cpu0Write_ex2exMem
          // 异常相关
          r_exception <> io.S_exception_ex2exMem
        }
          .elsewhen(io.stall(3)===Stop && io.stall(4)===NoStop){  // 执行阶段（ex）暂停，访存阶段（mem）继续（通过exMem的output影响mem）
            r_S_ex2exMem.wdata := ZeroWord
            r_S_ex2exMem.waddr := NOPRegAddr
            r_S_ex2exMem.we := WriteDisable
            r_S_hilo.hi := ZeroWord
            r_S_hilo.lo := ZeroWord
            r_S_hilo.w_hiloEnable := WriteDisable
            r_M_tmpStall_mem_ex2ex <> io.S_tmpStall_ex2mem_ex
            // 加载/储存相关
            r_loadstore.aluop := EXE_NOP_OP
            r_loadstore.memAddr := ZeroWord
            r_loadstore.memReg := ZeroWord
            // 协处理器写
            r_cpu0Write.we := WriteDisable
            r_cpu0Write.addr := NOPRegAddr
            r_cpu0Write.data := ZeroWord
            // 异常相关
            r_exception.exceptType := ZeroWord
            r_exception.currentInst_addr := ZeroWord
            r_exception.is_inDelayslot := NotInDelaySlot
          }
          .otherwise{}  // 其余（执行阶段在那头，访存阶段也暂停），则保持原值
      }
  }
  io.M_exMem2mem <> areaClk.r_S_ex2exMem
  io.M_hilo_exMem2mem <> areaClk.r_S_hilo
  io.M_tmpStall_mem_ex2ex <> areaClk.r_M_tmpStall_mem_ex2ex
  io.M_loadstore_exMem2mem <> areaClk.r_loadstore
  io.M_cpu0Write_exMem2mem <> areaClk.r_cpu0Write
  io.M_exception_exMem2mem <> areaClk.r_exception
}

class mem extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val rst = in Bool
    val S_exMem2mem = slave(wInterface(RegAddrBus, RegBus))
    val M_mem2memWb = master(wInterface(RegAddrBus, RegBus))
    val S_hilo_exMem2mem = slave(hiloInterface(RegBus))
    val M_hilo_mem2memWb = master(hiloInterface(RegBus))
    // 加载/储存相关
    val S_loadstore_exMem2mem = slave(load_storeInterface(AluOpBus,DataAddrBus,DataBus))
    val M_mem2ram = master(ramInterface(MemSelBus,DataAddrBus,DataBus))
    // llBit链接状态位相关
    val M_llbit_mem2memWb = master(llbitInterface())
    val S_PullForward_memWb2mem = slave(llbitInterface())  // llbit的写数据前推，为在mem端口读llbit时获取最新数据
    val llbit = in Bool  // 自llbit_reg收到的llbit
    // 协处理器写相关
    val S_cpu0Write_exMem2mem = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val M_cpu0Write_mem2memWb = master(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    // 异常相关（与协处理器CPU0关联）
    val S_exception_exMem2mem = slave(exceptInterface(ExceptTypeBus,InstBus))
    val S_exceptCPU0RegOut_cpu0Reg2mem = slave(exceptCPU0RegoutInterface(CPU0RegBus))  // 来自cpu0的cause、status、epc等数据
    val S_PullForward_cpu0Write_memWb2mem = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))  // 来自memWb的数据前推
    val M_exception_mem2cpu0Reg = master(exceptInterface(ExceptTypeBus,InstBus))
    val epc = out UInt(CPU0RegBus bits)  // 对stallCtrl的异常地址输出
  }

//  异常处理：异常指令地址（cpu0的epc寄存器）与延迟槽标记可以直接从mem送往cpu0（不用经过mem/wb）
//  异常处理：异常类型（exceptType）需要根据优先级进行排列
//  优先等级： 硬件复位>中断（6个硬件中断与2个软件中断）>系统调用/无效/溢出/自陷异常   （硬件复位，通过rst实现）
//  注意，异常指令地址为CPU0Reg及stallCtrl模块需要stallCtrl通过flush控制时序电路的寄存器置位
//  当stallCtrl通过输入的exceptType得到异常类型，根据异常类型决定pc端的指令地址
//  mem模块所接收的current_inst_address只是本次指令地址，并非最初的异常地址，
//  （如，上一clk有mtc0指令修改了epc寄存器，但本clk时，异常指令仍处于memWb)
//  故而需要通过memWb的数据前推来获取cpu0寄存器cause、status、epc的最新值
//  注意，若上一clk的mem模块为mtc0指令，且欲修改cpu0的epc寄存器
//  则mtc0指令会在exe模块处使得cpu0标记位M_cpu0Write_ex2exMem.we 可写（除mtc0指令外，该标志位均为不可写0）
//  同时，上一clk的mem模块的mtc0指令会在本clk处于memWb，等待下一clk写入cpu0
//  本clk的异常指令，于mem模块会一直向cpu0模块assign current_inst_address，同时向stallCtrl模块发送exceptType
//  在CPU0Reg模块中，mtc0指令的对epc值的赋值在异常指令的current_inst_address之前，
//  相当于被异常指令的current_inst_address覆盖，现阶段cpu0内epc保存的是异常指令的地址，而非mtc0写入的地址
//  mtc0写入的地址被输送到stallCtrl，最后更新到pc端执行，cpu0始终保存最新的异常地址
  val r_status = UInt(CPU0RegBus bits)  // 最新的status
  val r_cause = UInt(CPU0RegBus bits)  // 最新的cause
  val r_epc = UInt(CPU0RegBus bits)  // 最新的epc
  val r_exceptType = UInt(ExceptTypeBus bits)  // ExceptTpye缓存
  // 对CPU0的输出
  io.M_exception_mem2cpu0Reg.currentInst_addr <> io.S_exception_exMem2mem.currentInst_addr  // 本指令地址
  io.M_exception_mem2cpu0Reg.is_inDelayslot <> io.S_exception_exMem2mem.is_inDelayslot
  io.M_exception_mem2cpu0Reg.exceptType <> r_exceptType
  // 对stallCtrl的异常地址输出
  io.epc <> r_epc
  // 防止latch
  r_status := ZeroWord
  r_cause := ZeroWord
  r_epc := ZeroWord
  r_exceptType := ZeroWord
  when(io.rst === RstEnable){
    r_status := ZeroWord
    r_cause := ZeroWord
    r_epc := ZeroWord
    r_exceptType := ZeroWord
  }
    .otherwise{
      when(io.S_PullForward_cpu0Write_memWb2mem.addr === CP0_REG_STATUS &&
        io.S_PullForward_cpu0Write_memWb2mem.we === WriteEnable){
        r_status := io.S_PullForward_cpu0Write_memWb2mem.data
      }
        .otherwise{
          r_status := io.S_exceptCPU0RegOut_cpu0Reg2mem.status
        }
      when(io.S_PullForward_cpu0Write_memWb2mem.addr === CP0_REG_CAUSE &&
        io.S_PullForward_cpu0Write_memWb2mem.we === WriteEnable){
        // cause寄存器只有IP[1:0]（软件中断 [9:8]）、IV（中断向量 [23]）、WP（观测挂起 [22]）字段是可写的，外部中断只能由外部中断触发写入
        r_cause(9 downto 8) := io.S_PullForward_cpu0Write_memWb2mem.data(9 downto 8)
        r_cause(23 downto 22) := io.S_PullForward_cpu0Write_memWb2mem.data(23 downto 22)
      }
        .otherwise{
          r_cause := io.S_exceptCPU0RegOut_cpu0Reg2mem.cause
        }
      when(io.S_PullForward_cpu0Write_memWb2mem.addr === CP0_REG_EPC &&
        io.S_PullForward_cpu0Write_memWb2mem.we === WriteEnable){
        r_epc := io.S_PullForward_cpu0Write_memWb2mem.data
      }
        .otherwise{
          r_epc := io.S_exceptCPU0RegOut_cpu0Reg2mem.epc
        }
      // 第0-7bit exceptiontype的低8bit留给外部中断 [7:0]
      // 第8bit表示是否系统调用异常syscall [8]
      // 第9bit表示是否无效指令异常InstInvalid [9]
      // 第10bit表示是否自陷异常trap(自陷异常trap需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [10]
      // 第11bit表示是否溢出异常overflow(溢出异常overflow需要在ex模块中，根据寄存器值进行判断而给出，此处先不给，以0占位) [11]
      // 第12bit表示是否异常返回eret [12]
      when((r_status(15 downto 8) & r_cause(15 downto 8)) =/= U"8'b0" &&
        (r_status(1) === NotInException) && (r_status(0) === AllowException)){
        // 基于异常优先级的异常类型判断
        // (r_status(15 downto 8) & r_cause(15 downto 8) 相当于r_cause以r_status作为mask
        // 只有双方的对应位均为1，方可认为本位置发送中断
        r_exceptType := U"32'h00000001"  // 触发中断 Interrupt
      }
        .elsewhen(io.S_exception_exMem2mem.exceptType(8) === hasException){
          r_exceptType := U"32'h00000008"  // 系统调用异常syscall
        }
        .elsewhen(io.S_exception_exMem2mem.exceptType(9) === hasException){
          r_exceptType := U"32'h0000000a"  // 无效指令异常InstInvalid
        }
        .elsewhen(io.S_exception_exMem2mem.exceptType(11) === hasException){
        r_exceptType := U"32'h0000000c"  // 溢出异常overflow
        }
        .elsewhen(io.S_exception_exMem2mem.exceptType(10) === hasException){
          r_exceptType := U"32'h0000000d"  // 自陷异常trap
        }
       .elsewhen(io.S_exception_exMem2mem.exceptType(12) === hasException){
          r_exceptType := U"32'h0000000e"  // 异常返回eret
        }
    }

  // 防止latch
  io.M_llbit_mem2memWb.llBit := llBitZero
  io.M_llbit_mem2memWb.we := WriteDisable

  io.M_cpu0Write_mem2memWb <> io.S_cpu0Write_exMem2mem

  // llbit的数据前推，先观察memWb(即上一clk的结果)
  val r_llbit = Bool
  when(io.rst === RstEnable){
    r_llbit := llBitZero
  }
    .otherwise{
      when(io.S_PullForward_memWb2mem.we === WriteEnable){
        r_llbit := io.S_PullForward_memWb2mem.llBit
      }
        .otherwise{
          r_llbit := io.llbit
        }
    }

  when(io.rst === RstEnable) {
    io.M_mem2memWb.wdata := ZeroWord
    io.M_mem2memWb.waddr := NOPRegAddr
    io.M_mem2memWb.we := WriteDisable
    io.M_hilo_mem2memWb.hi := ZeroWord
    io.M_hilo_mem2memWb.lo := ZeroWord
    io.M_hilo_mem2memWb.w_hiloEnable := WriteDisable
    // 与内存RAM相关的访存操作
    io.M_mem2ram.ramEn := ChipDisable
    io.M_mem2ram.we := WriteDisable
    io.M_mem2ram.addr := ZeroWord
    io.M_mem2ram.sel := MemSelZero
    io.M_mem2ram.dataWrite := ZeroWord
  }.otherwise {
    io.M_mem2memWb <> io.S_exMem2mem
    io.M_hilo_mem2memWb <> io.S_hilo_exMem2mem
    // 与内存RAM相关的访存操作
    // 防止latch
    io.M_mem2ram.ramEn := ChipDisable
    io.M_mem2ram.we := WriteDisable
    io.M_mem2ram.addr := ZeroWord
    io.M_mem2ram.sel := MemSelOne  // 预期先读全字
    io.M_mem2ram.dataWrite := ZeroWord
    switch(io.S_loadstore_exMem2mem.aluop){
      // 加载指令
      is(EXE_LB_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        // sel根据addr的末两位决定（与实际使用的ram有关，ram通过addr[31:2]、sel[3:0]计算读写数据的地址）
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){  // 读取RAM对应地址字的第1个字节（首字节）
            io.M_mem2ram.sel := Memsel_1
            io.M_mem2memWb.wdata := U(24 bits,default->io.M_mem2ram.dataRead(31)) @@
              io.M_mem2ram.dataRead(31 downto 24)  // 读取RAM的对应数据（并带符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b01"){  // 读取RAM对应地址字的第2个字节
            io.M_mem2ram.sel := Memsel_2
            io.M_mem2memWb.wdata := U(24 bits,default->io.M_mem2ram.dataRead(23)) @@
              io.M_mem2ram.dataRead(23 downto 16)  // 读取RAM的对应数据（并带符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b10"){  // 读取RAM对应地址字的第3个字节
            io.M_mem2ram.sel := Memsel_3
            io.M_mem2memWb.wdata := U(24 bits,default->io.M_mem2ram.dataRead(15)) @@
              io.M_mem2ram.dataRead(15 downto 8)  // 读取RAM的对应数据（并带符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b11"){  // 读取RAM对应地址字的第4个字节
            io.M_mem2ram.sel := Memsel_4
            io.M_mem2memWb.wdata := U(24 bits,default->io.M_mem2ram.dataRead(7)) @@
              io.M_mem2ram.dataRead(7 downto 0)  // 读取RAM的对应数据（并带符号扩展），加载到GPR（普通寄存器）的rt中
          }
          default()
        }
      }
      is(EXE_LBU_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        // sel根据addr的末两位决定（与实际使用的ram有关，ram通过addr[31:2]、sel[3:0]计算读写数据的地址）
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){  // 读取RAM对应地址字的第1个字节（首字节）
            io.M_mem2ram.sel := Memsel_1
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(31 downto 24).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b01"){  // 读取RAM对应地址字的第2个字节
            io.M_mem2ram.sel := Memsel_2
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(23 downto 16).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b10"){  // 读取RAM对应地址字的第3个字节
            io.M_mem2ram.sel := Memsel_3
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(15 downto 8).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b11"){  // 读取RAM对应地址字的第4个字节
            io.M_mem2ram.sel := Memsel_4
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(7 downto 0).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          default()
        }
      }
      is(EXE_LH_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        // sel根据addr的末两位决定（与实际使用的ram有关，ram通过addr[31:2]、sel[3:0]计算读写数据的地址）
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){  // 读取RAM对应地址字的第1个半字
            io.M_mem2ram.sel := Memsel_1H
            io.M_mem2memWb.wdata := U(16 bits,default->io.M_mem2ram.dataRead(31)) @@
              io.M_mem2ram.dataRead(31 downto 16)  // 读取RAM的对应数据（并有符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b10"){  // 读取RAM对应地址字的第2个字节
            io.M_mem2ram.sel := Memsel_2H
            io.M_mem2memWb.wdata := U(16 bits,default->io.M_mem2ram.dataRead(15)) @@
              io.M_mem2ram.dataRead(15 downto 0)  // 读取RAM的对应数据（并有符号扩展），加载到GPR（普通寄存器）的rt中
       }
          default()
        }
      }
      is(EXE_LHU_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        // sel根据addr的末两位决定（与实际使用的ram有关，ram通过addr[31:2]、sel[3:0]计算读写数据的地址）
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){  // 读取RAM对应地址字的第1个半字
            io.M_mem2ram.sel := Memsel_1H
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(31 downto 16).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          is(U"2'b10"){  // 读取RAM对应地址字的第2个字节
            io.M_mem2ram.sel := Memsel_2H
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(15 downto 0).resized  // 读取RAM的对应数据（并无符号扩展），加载到GPR（普通寄存器）的rt中
          }
          default()
        }
      }
      is(EXE_LW_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        io.M_mem2ram.sel := MemSelOne
        io.M_mem2memWb.wdata := io.M_mem2ram.dataRead
      }
      is(EXE_LWL_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr(31 downto 2) @@ U"2'b00" // 舍去末2位，对齐内存地址
        io.M_mem2ram.sel := MemSelOne  // 读全字
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead
          }
          is(U"2'b01"){
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(23 downto 0) @@ io.S_loadstore_exMem2mem.memReg(7 downto 0)
          }
          is(U"2'b10"){
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(15 downto 0) @@ io.S_loadstore_exMem2mem.memReg(15 downto 0)
          }
          is(U"2'b11"){
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead(7 downto 0) @@ io.S_loadstore_exMem2mem.memReg(23 downto 0)
          }
          default()
        }
      }
      is(EXE_LWR_OP){
        // 加载，不涉及向ram的写入，故而we、dataWrite均保持默认值即可
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr(31 downto 2) @@ U"2'b00"  // 舍去末2位，对齐内存地址
        io.M_mem2ram.sel := MemSelOne  // 读全字
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2memWb.wdata := io.S_loadstore_exMem2mem.memReg(31 downto 8) @@ io.M_mem2ram.dataRead(31 downto 24)
          }
          is(U"2'b01"){
            io.M_mem2memWb.wdata := io.S_loadstore_exMem2mem.memReg(31 downto 16) @@ io.M_mem2ram.dataRead(31 downto 16)
          }
          is(U"2'b10"){
            io.M_mem2memWb.wdata := io.S_loadstore_exMem2mem.memReg(31 downto 24) @@ io.M_mem2ram.dataRead(31 downto 8)
          }
          is(U"2'b11"){
            io.M_mem2memWb.wdata := io.M_mem2ram.dataRead
          }
          default()
        }
      }
      // 储存指令
      is(EXE_SB_OP){
        // 储存，涉及向ram的写入，注意we、dataWrite
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteEnable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(7 downto 0) @@
          io.S_loadstore_exMem2mem.memReg(7 downto 0) @@ io.S_loadstore_exMem2mem.memReg(7 downto 0) @@
          io.S_loadstore_exMem2mem.memReg(7 downto 0)  // 使用GPR中rt的后八位组成写入内存ram对应地址的数据
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2ram.sel := Memsel_1
          }
          is(U"2'b01"){
            io.M_mem2ram.sel := Memsel_2
          }
          is(U"2'b10"){
            io.M_mem2ram.sel := Memsel_3
          }
          is(U"2'b11"){
            io.M_mem2ram.sel := Memsel_4
          }
        }
      }
      is(EXE_SH_OP){
        // 储存，涉及向ram的写入，注意we、dataWrite
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteEnable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(15 downto 0) @@
          io.S_loadstore_exMem2mem.memReg(15 downto 0) // 使用GPR中rt的后16位组成写入内存ram对应地址的数据
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2ram.sel := Memsel_1H
          }
          is(U"2'b10"){
            io.M_mem2ram.sel := Memsel_2H
          }
        }
      }
      is(EXE_SW_OP){
        // 储存，涉及向ram的写入，注意we、dataWrite
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteEnable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg
        io.M_mem2ram.sel := MemSelOne
      }
      is(EXE_SWL_OP){
        // 储存，涉及向ram的写入，注意we、dataWrite
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteEnable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr(31 downto 2) @@ U"2'b00"  // 舍去末2位，对齐内存地址
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg
            io.M_mem2ram.sel := MemSelOne
          }
          is(U"2'b01"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(31 downto 8).resized  // 无符号扩充
            io.M_mem2ram.sel := Memsel_maskL1
          }
          is(U"2'b10"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(31 downto 16).resized  // 无符号扩充
            io.M_mem2ram.sel := Memsel_maskL2
          }
          is(U"2'b11"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(31 downto 24).resized  // 无符号扩充
            io.M_mem2ram.sel := Memsel_maskL3
          }
        }
      }
      is(EXE_SWR_OP){
        // 储存，涉及向ram的写入，注意we、dataWrite
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteEnable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr(31 downto 2) @@ U"2'b00"  // 舍去末2位，对齐内存地址
        switch(io.S_loadstore_exMem2mem.memAddr(1 downto 0)){
          is(U"2'b00"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(7 downto 0) @@ U(0,24 bits)
            io.M_mem2ram.sel := Memsel_maskR3
          }
          is(U"2'b01"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(15 downto 0) @@ U(0,16 bits)
            io.M_mem2ram.sel := Memsel_maskR2
          }
          is(U"2'b10"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg(23 downto 0) @@ U(0,8 bits)
            io.M_mem2ram.sel := Memsel_maskR1
          }
          is(U"2'b11"){
            io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg
            io.M_mem2ram.sel := MemSelOne
          }
        }
      }
      is(EXE_LL_OP){
        // 从内存指定地址读取一个字，保存到GPR rt中
        io.M_mem2ram.ramEn := ChipEnable
        io.M_mem2ram.we := WriteDisable
        io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
        io.M_mem2ram.sel := MemSelOne
        io.M_mem2memWb.wdata := io.M_mem2ram.dataRead
        // 设置链接状态位
        io.M_llbit_mem2memWb.we := WriteEnable
        io.M_llbit_mem2memWb.llBit := llBitOne
      }
      is(EXE_SC_OP){
        when(r_llbit === llBitOne){
          // 根据llbit的值，确认是否修改内存/rt写入值为1或0
          io.M_mem2ram.ramEn := ChipEnable
          io.M_mem2ram.addr := io.S_loadstore_exMem2mem.memAddr
          io.M_mem2ram.we := WriteEnable
          io.M_mem2ram.dataWrite := io.S_loadstore_exMem2mem.memReg  // 将GPR普通寄存器rt值存入内存
          io.M_mem2memWb.wdata := U"32'b1"  // 设置rt值为1
          // 设置链接状态位
          io.M_llbit_mem2memWb.we := WriteEnable
          io.M_llbit_mem2memWb.llBit := llBitZero
        }
          .otherwise{  // llbit===false
            // 不修改内存，默认 ramEn===ChipDisable we===WriteDisable addr===ZeroWord
            io.M_mem2memWb.wdata := U"32'b0"  // 设置rt值为0
          }
      }
      default()
    }
  }
}

class memWb extends Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val S_mem2memWb = slave(wInterface(RegAddrBus, RegBus))
    val M_memWb2regfile = master(wInterface(RegAddrBus, RegBus))
    val S_hilo_mem2memWb = slave(hiloInterface(RegBus))
    val M_hilo_memWb2hiloReg = master(hiloInterface(RegBus))
    val stall = in UInt(stallDir bits)
    // 链接状态位
    val S_llbit_mem2memWb = slave(llbitInterface())
    val M_memWb2llbitReg = master(llbitInterface())
    // 协处理器写相关
    val S_cpu0Write_mem2memWb = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val M_cpu0Write_memWb2cpu0Reg = master(slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus)))
    // 异常处理
    val flush = in Bool
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_S_mem2memWb = Reg(io.S_mem2memWb)
    r_S_mem2memWb.wdata.init(ZeroWord)
    r_S_mem2memWb.waddr.init(NOPRegAddr)
    r_S_mem2memWb.we.init(WriteDisable)
    val r_S_hilo = Reg(io.S_hilo_mem2memWb)
    r_S_hilo.hi.init(ZeroWord)
    r_S_hilo.lo.init(ZeroWord)
    r_S_hilo.w_hiloEnable.init(WriteDisable)
    val r_S_llbit_mem2memWb = Reg(io.S_llbit_mem2memWb)
    r_S_llbit_mem2memWb.we.init(WriteDisable)
    r_S_llbit_mem2memWb.llBit.init(llBitZero)
    // 协处理器写相关
    val r_cpu0Write = Reg(io.S_cpu0Write_mem2memWb)
    r_cpu0Write.we init(WriteDisable)
    r_cpu0Write.addr init(NOPRegAddr)
    r_cpu0Write.data init(ZeroWord)
    // 异常触发
    when(io.flush === hasException){
      r_S_mem2memWb.wdata := ZeroWord
      r_S_mem2memWb.waddr := NOPRegAddr
      r_S_mem2memWb.we := WriteDisable
      r_S_hilo.hi := ZeroWord
      r_S_hilo.lo := ZeroWord
      r_S_hilo.w_hiloEnable := WriteDisable
      r_S_llbit_mem2memWb.we := WriteDisable
      r_S_llbit_mem2memWb.llBit := llBitZero
      // 协处理器写
      r_cpu0Write.we := WriteDisable
      r_cpu0Write.addr := NOPRegAddr
      r_cpu0Write.data := ZeroWord
    }
      .elsewhen(io.stall(4) === NoStop){  // 访存阶段（mem)不暂停
      r_S_mem2memWb := io.S_mem2memWb
      r_S_hilo := io.S_hilo_mem2memWb
      r_S_llbit_mem2memWb := io.S_llbit_mem2memWb
      r_cpu0Write <> io.S_cpu0Write_mem2memWb
    }
      .elsewhen(io.stall(4)===Stop && io.stall(5)===NoStop){  // 访存阶段（mem)暂停,回写阶段(memWb)不暂停
        r_S_mem2memWb.wdata := ZeroWord
        r_S_mem2memWb.waddr := NOPRegAddr
        r_S_mem2memWb.we := WriteDisable
        r_S_hilo.hi := ZeroWord
        r_S_hilo.lo := ZeroWord
        r_S_hilo.w_hiloEnable := WriteDisable
        r_S_llbit_mem2memWb.we := WriteDisable
        r_S_llbit_mem2memWb.llBit := llBitZero
        // 协处理器写
        r_cpu0Write.we := WriteDisable
        r_cpu0Write.addr := NOPRegAddr
        r_cpu0Write.data := ZeroWord
      }
      .otherwise{}  // 其余（访存阶段（memWb)暂停,回写阶段也暂停，保持原值
  }
  io.M_memWb2regfile <> areaClk.r_S_mem2memWb
  io.M_hilo_memWb2hiloReg <> areaClk.r_S_hilo
  io.M_memWb2llbitReg <> areaClk.r_S_llbit_mem2memWb
  io.M_cpu0Write_memWb2cpu0Reg <> areaClk.r_cpu0Write
}

class hiloReg extends Component with Global_parameter with Interface_MS{
  val io = new Bundle{
    val clk = in Bool
    val rst = in Bool
    val S_hiloInterface = slave(hiloInterface(RegBus))
    val M_hiloReg = master(hiloRegInterface(RegBus))
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
      val r_hi_i = Reg(io.S_hiloInterface.hi) init (ZeroWord)
      val r_lo_i = Reg(io.S_hiloInterface.lo) init (ZeroWord)
    when(io.S_hiloInterface.w_hiloEnable === WriteEnable){
      r_hi_i := io.S_hiloInterface.hi
      r_lo_i := io.S_hiloInterface.lo
    }
      .otherwise()
  }
  io.M_hiloReg.hi := areaClk.r_hi_i
  io.M_hiloReg.lo := areaClk.r_lo_i
//  io.M_hiloReg.hi := (io.S_hiloInterface.w_hiloEnable === WriteEnable) ? areaClk.r_hi_i | ZeroWord
//  io.M_hiloReg.lo := (io.S_hiloInterface.w_hiloEnable === WriteEnable) ? areaClk.r_lo_i | ZeroWord
}

// 流水线暂停，同时结合了异常控制
class stallCtrl extends Component with Global_parameter with Interface_MS{
  val io = new Bundle{
    val rst = in Bool
    val stallCtrl_id = in Bool  // 来自id的暂停请求`
    val stallCtrl_ex = in Bool  // 来自ex的暂停请求
    val stall = out UInt(stallDir bits)  //  暂停控制信号，1表示暂停。  5-回写 4-访存 3-执行 2-译码 1-取值 0-pc计数(pc)
    // 异常处理相关
    val epc = in UInt(CPU0RegBus bits)
    val exceptType = in UInt(ExceptTypeBus bits)
    val flush = out Bool  // ctrl确认发生异常之信号
    val newPC = out UInt (InstAddrBus bits)  // 重置的新指令地址
  }
  // 防止latch
  io.stall := U(0,stallDir bits)
  io.flush := noException
  io.newPC := ZeroWord

  when(io.rst === RstEnable){
    io.stall := U(0,stallDir bits)
  }
    .elsewhen(io.exceptType =/= ZeroWord){  // 不为ZeroWord说明有异常发生
      io.stall := U(0,stallDir bits)
      io.flush := hasException
      switch(io.exceptType){
        is(U"32'h01"){  // 触发中断 Interrupt
          io.newPC := U"32'h20"
        }
        is(U"32'h08",U"32'h0a",U"32'h0c",U"32'h0d"){ // 触发 系统调用异常、无效指令异常、自陷异常、溢出异常
          io.newPC := U"32'h40"
        }
        is(U"32'h0e"){  // 异常返回eret
          io.newPC := io.epc
        }
      }
    }
    .elsewhen(io.stallCtrl_id === Stop){  // id请求暂停,取值、译码暂停，执行、访存、回写继续
      io.stall := U"6'b000111"
    }
    .elsewhen(io.stallCtrl_ex === Stop){  // ex请求暂停，取值、译码、执行暂停，访存、回写继续
      io.stall := U"6'001111"
    }
    .otherwise()
}

// 除法模块
class div extends Component with Global_parameter with Interface_MS{
  val io = new Bundle{
    val clk = in Bool
    val rst = in Bool
    val annul = in Bool  // 是否取消除法
    val S_divConnect_ex2div = slave(divConnectInterface(RegBus))
    val M_divResult_div2ex = master(divResultInterface(DoubleRegBus))
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )

  val r_tmp_opData1 = UInt(RegBus bits)  // 此为组合逻辑，wire
  val r_tmp_opData2 = UInt(RegBus bits)  // 此为组合逻辑，wire
  when(io.S_divConnect_ex2div.signalDiv === DivIsSignal &&
    io.S_divConnect_ex2div.opData1(RegBus-1) === True) { // 当为有符号计算且被除数为负数
    r_tmp_opData1 := ~io.S_divConnect_ex2div.opData1 + U"32'b1"
  }
    .otherwise {
      r_tmp_opData1 := io.S_divConnect_ex2div.opData1
    }
  when(io.S_divConnect_ex2div.signalDiv === DivIsSignal &&
    io.S_divConnect_ex2div.opData2(RegBus-1) === True) { // 当为有符号计算且除数为负数
    r_tmp_opData2 := ~io.S_divConnect_ex2div.opData2 + U"32'b1"
  }
    .otherwise {
      r_tmp_opData2 := io.S_divConnect_ex2div.opData2
    }

  val areaClk = new ClockingArea(thisClockDomain){
    val r_resultDiv = Reg(io.M_divResult_div2ex.resultDiv) init(U(0,DoubleRegBus bits))
    val r_ready = Reg(io.M_divResult_div2ex.ready) init(DivResultNotReady)
    val r_cnt = Reg(UInt(6 bits)) init(U(0,6 bits))  // 计数次数
    val r_dividend = Reg(UInt(DoubleRegBus+1 bits)) init(U"65'b0")  // 存放运算结果与中间值,比64位多出一位
    val r_divisor = Reg(UInt(RegBus bits)) init(U"32'b0")  // 保存除数
    val r_div_temp = U"1'b0" @@ r_dividend(DoubleRegBus-1 downto RegBus) - U"1'b0" @@ r_divisor  // 首位加0充当符号位，进行计算可通过符号位知道正负
    //    可以使用dividend保存运算结果与中间值（对32bits的除法，初始置入被除数到dividend[32:1]）
    //    此时，第一次取被除数的最高位与除数（divisor）做减法比较
    //    即 assign div_temp = {1'b0,dividend[63:32]} - {1'b0,divisor}（之所以首位加0是充当符号位，进行计算可通过符号位知道正负）
    //    若小于除数，则Dividend整体左移（使得被除数的后一位占据Dividend[32]），用被除数的两位去比较除数，Dividend[0]为0作为商的结果）
    //    若大于除数，则先Dividend整体左移(使得被除数的后一位占据Dividend[32]），Dividend[0]为1作为商的结果
    //    然后取div_temp[31:0]作为Dividend[64:31]（div_temp最高位是符号位，不取，因为原dividend[63:32] >= divisor,
    //    所以div_temp[31]必为0，顶上Dividend的64bit刚好不影响下一clk计算div_temp）
    //    运算32次后，初始末尾的0在Dividend[32] 而Dividend[64:33]保存的是除法运算的余数，Dividend[31:0]保存的是除法运算的商
    val divFSM = new StateMachine{
      val DivFree_state = new State with EntryPoint
      val DivByZero_state = new State
      val DivOn_state = new State
      val DivEnd_state = new State

      DivFree_state  // 初始状态
        .whenIsActive{
          when(io.S_divConnect_ex2div.start === DivStart && io.annul === DivNotAnnul){  // 开始除法计算，且没有外部取消
            when(io.S_divConnect_ex2div.opData2 === ZeroWord){  // 除数为0
              goto(DivByZero_state)  // 状态转移
            }
            .otherwise{  // 除数不为0
              goto(DivOn_state)  // 状态转移
              r_cnt := U(0,6 bits)  // 计数为0（32位除法需32个周期计算）
              r_dividend := ((RegBus downto 1)->r_tmp_opData1,default -> false)  // 初始置入，r_dividend[32:1] 为被除数
              r_divisor := r_tmp_opData2  // 记录除数
              }
            }
          .otherwise{  // 没有起始信号或有外部取消信号
            r_ready := DivResultNotReady
            r_resultDiv := U(0,DoubleRegBus bits)
          }
        }

      DivByZero_state  // 除数为0之状态
        .whenIsActive{
          goto(DivFree_state)
          r_dividend := ZeroWord @@ ZeroWord @@ U"1'b0"
        }

      DivOn_state  // 除数不为0之状态
        .whenIsActive{
          when(io.annul === DivIsAnnul){  // 有外部取消信号
            goto(DivFree_state)
          }
            .otherwise{  // // 无外部取消信号 annul===0
              when(r_cnt === RegBus){  // 已经运行了RegBus（32）个周期，试商法结束
                goto(DivEnd_state)
                r_cnt := U(0,6 bits)
                when((io.S_divConnect_ex2div.opData1(RegBus-1) ^ io.S_divConnect_ex2div.opData2(RegBus-1)) &&
                  io.S_divConnect_ex2div.signalDiv === DivIsSignal){  // 当被除数、除数一正一负、且为有符号除法，则商为负数
                  r_dividend(RegBus-1 downto 0) := ~r_dividend(RegBus-1 downto 0) + 1
                }
                  .otherwise()  // 其余，保持原值
                when(io.S_divConnect_ex2div.opData1(RegBus-1) === True &&
                  io.S_divConnect_ex2div.signalDiv === DivIsSignal){  // 有符号除法，余数的符号跟随被除数
                  //   11 /  2 =  5 ···  1
                  //   11 / -2 = -5 ···  1
                  //  -11 /  2 = -5 ··· -1
                  //  -11 / -2 =  5 ··· -1
                  // 而在初始阶段，被除数、除数均转化为正数进行计算，则其余数必为正数，最终结果的余数需要根据实际情况修正
                  //  关键在于余数的定义，假定a 除以b，商为q，余数为r：
                  //  q = a/b;
                  //  r = a%b;
                  //	则q*b + r == a，因为这是定义余数的关系；如果改变a的正负号，则q的符号也随之改变，但q的绝对值不会变。
                  r_dividend(DoubleRegBus downto RegBus+1) := ~r_dividend(DoubleRegBus downto RegBus+1) + 1
                }
                  .otherwise() // 其余，保持原值
              }
                .otherwise{  // 未达到RegBus（32）个周期，试商法继续
                  r_cnt := r_cnt + 1
                  when(r_div_temp(r_div_temp.getWidth -1)) { // 减法结果为负数（说明当前被减数小于除数）
                    r_dividend := r_dividend |<< 1 // r_dividend左移一位，将被除数纳入比较的位数向后推一位
                  }
                    .otherwise{   // 减法结果为非负数（说明当前被减数大于或等于除数）
                      r_dividend := r_div_temp(RegBus-1 downto 0) @@ r_dividend(RegBus-1 downto 0) @@ U"1'b1"
                    }
                }
            }
        }

      DivEnd_state  // 结束
        .whenIsActive{
          when(io.S_divConnect_ex2div.start === DivStop){
            goto(DivFree_state)
            // 结束状态收到了停止信号
            r_ready := DivResultNotReady
            r_resultDiv := U(0,DoubleRegBus bits)
          }
            .otherwise{
              r_ready := DivResultReady
              r_resultDiv := r_dividend(DoubleRegBus downto RegBus+1) @@ r_dividend(RegBus-1 downto 0)
            }
        }
    }
  }
  io.M_divResult_div2ex.ready := areaClk.r_ready
  io.M_divResult_div2ex.resultDiv := areaClk.r_resultDiv
}

// LLbit链接状态位模块（原子操作的异常标志位）
class llbitReg extends Component with Global_parameter with Interface_MS{
  val io = new Bundle{
    val clk = in Bool
    val rst = in Bool
    val flush = in Bool  // 异常标志位，true为异常发生
    val S_memWb2llbitReg = slave(llbitInterface())
    val llBit = out Bool
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_llBit = Reg(Bool) init(llBitZero)
    when(io.flush === hasException){  // 有异常发生
      r_llBit := llBitZero
    }
      .elsewhen(io.S_memWb2llbitReg.we === WriteEnable){
        r_llBit := io.S_memWb2llbitReg.llBit
      }
  }
  io.llBit := areaClk.r_llBit
}

// 协处理器cpu0模块
class cpu0Reg extends Component with Global_parameter with Interface_MS {
  val io = new Bundle{
    val clk = in Bool
    val rst = in Bool
    val externalInterrupt = in UInt(InterruptBus bits)
    val S_cpu0Write_memWb2cpu0Reg = slave(cpu0WriteInterface(CPU0AddrBus,CPU0RegBus))
    val S_cpu0Read_ex2cpu0Reg = slave(cpu0ReadInterface(CPU0AddrBus,CPU0RegBus))
    val M_cpu0Regout = master(cpu0RegOutInterface(CPU0RegBus))
    // 异常处理（来自mem）
    val S_exception_mem2cpu0Reg = slave(exceptInterface(ExceptTypeBus,InstBus))
  }
  val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
  // 写入cpu0reg
  val areaClk = new ClockingArea(thisClockDomain) {
    val r_cpu0Regout = Reg(io.M_cpu0Regout)
    r_cpu0Regout.status init(U"32'b00010000000000000000000000000000")  // status寄存器的CU为0001，表示协处理器CP0存在
    r_cpu0Regout.count init(ZeroWord)
    r_cpu0Regout.compare init(ZeroWord)
    r_cpu0Regout.cause init(ZeroWord)
    r_cpu0Regout.epc init(ZeroWord)
    r_cpu0Regout.config init(U"32'b00000000000000001000000000000000")  // config寄存器的BE为1，表示Big-Endian；MT为00，表示没有MMU
    r_cpu0Regout.prid init(U"32'b00000000010010000000000100000010")  // //制作商是H，对应的是0x48（ASCII），类型是0x1，基本类型，版本号是1.0
    r_cpu0Regout.timerInterrupt init(InterruptNotAssert)

    r_cpu0Regout.count := r_cpu0Regout.count + 1  // 计数每clk递增
    r_cpu0Regout.cause(15 downto 10) := io.externalInterrupt  // 写入外部中断异常
    when(r_cpu0Regout.count === r_cpu0Regout.compare && r_cpu0Regout.count =/= ZeroWord){
      r_cpu0Regout.timerInterrupt := InterruptAssert
    }  // 计数达到设定值，且计数非0，提起计数（计时）中断
      .otherwise()  // 其余，保持原值
    // mtc0指令的写入
    when(io.S_cpu0Write_memWb2cpu0Reg.we === WriteEnable){
      switch(io.S_cpu0Write_memWb2cpu0Reg.addr){
        is(CP0_REG_STATUS){
          r_cpu0Regout.status := io.S_cpu0Write_memWb2cpu0Reg.data  // 写入status
        }
        is(CP0_REG_COUNT){
          r_cpu0Regout.count := io.S_cpu0Write_memWb2cpu0Reg.data  // 写入计数
        }
        is(CP0_REG_COMPARE){
          r_cpu0Regout.compare := io.S_cpu0Write_memWb2cpu0Reg.data  // 写入截止计数值
          r_cpu0Regout.timerInterrupt := InterruptNotAssert
        }
        is(CP0_REG_CAUSE){  // cause寄存器只有IP[1:0]（软件中断 [9:8]）、IV（中断向量 [23]）、WP（观测挂起 [22]）字段是可写的，外部中断只能由外部中断触发写入
          r_cpu0Regout.cause(9 downto 8) := io.S_cpu0Write_memWb2cpu0Reg.data(9 downto 8)
          r_cpu0Regout.cause(23) := io.S_cpu0Write_memWb2cpu0Reg.data(23)
          r_cpu0Regout.cause(22) := io.S_cpu0Write_memWb2cpu0Reg.data(22)
        }
        is(CP0_REG_EPC){
          r_cpu0Regout.epc := io.S_cpu0Write_memWb2cpu0Reg.data
        }
        is(CP0_REG_CONFIG){
          r_cpu0Regout.config := io.S_cpu0Write_memWb2cpu0Reg.data
        }
        is(CP0_REG_PrId){
          r_cpu0Regout.prid := io.S_cpu0Write_memWb2cpu0Reg.data
        }
        default()  // 其余，保持原值
      }
    }
//     mem模块的当前指令地址、异常类型
//     异常处理步骤如下（需联合mem、cpu0、stallCtrl）
//     1.（mem观测）若EXL为1，表示当前已处于异常处理中，若发生中断，忽略/若发生其他异常，（通过exceptType）记录到ExeCode（cpu0Reg内寄存器）
//       （mem观测）若EXL为0，表示之前并无异常（含中断），若发生异常，（通过exceptType）记录到ExeCode（cpu0Reg内寄存器）
//     2. (cpu0Reg内)观测exceptType，是否为异常
//         -- 为异常，则观测status(1)即EXL区域
//            -- 观测EXL （注）中断由于在mem中已观测EXL并过滤，无需再此观测EXL
//              -- EXL标记已处于异常级，表示当前已处于异常处理中，EXL继续表示为1，ExeCode记录本次异常，EPC不会记录本指令地址
//              -- EXL标记未处于异常级，表示之前无异常，EXL标识表示为1，ExeCode记录本次异常，EPC记录本指令地址
//            --  观测is_inDelayslot
//              -- InDelaySlot，本异常指令处于延迟槽，保存异常地址=地址-4，cause(31)的BD区标志1，表示本异常指令处于分支延迟槽
//              -- NotInDelaySlot,本异常指令非处于延迟槽，保存异常地址=地址，cause(31)的BD区标志0，表示本异常指令不处于分支延迟槽
//     2. (stallCtrl内)观测exceptType，是否为异常
//          -- 为异常，则根据异常类型，发出flush，控制pc跳转到新的指令地址（注意，epc的真正影响只有在ERET异常返回指令中体现）
//             -- 发出flush会使得带时序电路的pc、ifId、idEx、exMem、memWb、LLbit内部寄存器置位
//             -- 则后续异常全部取消，相当于填入空指令
//          -- 非异常，则根据流水线暂停进行

//    注意到，EXL在遇到异常时会置1，并屏蔽掉后续的异常地址，EXL如何刷新？
//	  1. 异常类型为异常返回（ERET）
//	     联系写入cpu0的指令 mtc0 ，可能为：
//	        -- 先利用 mtc0 写入EPC寄存器（异常返回跳转新值）
//	        -- 再触发 ERET（刷新EXL）
//	        -- 则 stallCtrl 收到的 异常地址为mtc0写入的EPC，即下一个clk的pc端会跳转到此EPC地址
//	        -- 而 cpu0Reg的当前current_inst_address及Output的EPC为ERET这条指令的地址
//    2. 直接硬件复位 Rest
    switch(io.S_exception_mem2cpu0Reg.exceptType){
      is(U"32'h00000001"){  // 触发中断 Interrupt
        r_cpu0Regout.cause(6 downto 2) := U"5'b00000"  // cause的ExeCode区域，记载发生的异常事件，此为中断
        r_cpu0Regout.status(1) := IsInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
        when(io.S_exception_mem2cpu0Reg.is_inDelayslot === InDelaySlot){  // 异常指令处于延迟槽
          r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr - 4
          r_cpu0Regout.cause(31) := InDelaySlot  // cause的BD区标志指令处于分支延迟槽
        }
          .otherwise{
            r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr
            r_cpu0Regout.cause(31) := NotInDelaySlot
          }
      }
      is(U"32'h00000008"){  // 系统调用异常syscall
        r_cpu0Regout.cause(6 downto 2) := U"5'h8"  // cause的ExeCode区域，记载发生的异常事件，此为系统调用异常（不管是否处于异常处理状态都记录）
        r_cpu0Regout.status(1) := IsInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
        when(r_cpu0Regout.status(1) === NotInException){  // 非异常处理状态才记录本次异常指令地址
          when(io.S_exception_mem2cpu0Reg.is_inDelayslot === InDelaySlot){  // 异常指令处于延迟槽
            r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr - 4
            r_cpu0Regout.cause(31) := InDelaySlot  // cause的BD区标志指令处于分支延迟槽
          }
            .otherwise{
              r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr
              r_cpu0Regout.cause(31) := NotInDelaySlot
            }
        }
      }
      is(U"32'h0000000a"){
        r_cpu0Regout.cause(6 downto 2) := U"5'ha"  // cause的ExeCode区域，记载发生的异常事件，此为无效指令异常InstInvalid（不管是否处于异常处理状态都记录）
        r_cpu0Regout.status(1) := IsInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
        when(r_cpu0Regout.status(1) === NotInException){  // 非异常处理状态才记录本次异常指令地址
          when(io.S_exception_mem2cpu0Reg.is_inDelayslot === InDelaySlot){  // 异常指令处于延迟槽
            r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr - 4
            r_cpu0Regout.cause(31) := InDelaySlot  // cause的BD区标志指令处于分支延迟槽
          }
            .otherwise{
              r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr
              r_cpu0Regout.cause(31) := NotInDelaySlot
            }
        }
      }
      is(U"32'h0000000d"){
        r_cpu0Regout.cause(6 downto 2) := U"5'hd"  // cause的ExeCode区域，记载发生的异常事件，此为无自陷异常trap（不管是否处于异常处理状态都记录）
        r_cpu0Regout.status(1) := IsInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
        when(r_cpu0Regout.status(1) === NotInException){  // 非异常处理状态才记录本次异常指令地址
          when(io.S_exception_mem2cpu0Reg.is_inDelayslot === InDelaySlot){  // 异常指令处于延迟槽
            r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr - 4
            r_cpu0Regout.cause(31) := InDelaySlot  // cause的BD区标志指令处于分支延迟槽
          }
            .otherwise{
              r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr
              r_cpu0Regout.cause(31) := NotInDelaySlot
            }
        }
      }
      is(U"32'h0000000c"){
        r_cpu0Regout.cause(6 downto 2) := U"5'hc"  // cause的ExeCode区域，记载发生的异常事件，此为溢出异常overflow（不管是否处于异常处理状态都记录）
        r_cpu0Regout.status(1) := IsInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
        when(r_cpu0Regout.status(1) === NotInException){  // 非异常处理状态才记录本次异常指令地址
          when(io.S_exception_mem2cpu0Reg.is_inDelayslot === InDelaySlot){  // 异常指令处于延迟槽
            r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr - 4
            r_cpu0Regout.cause(31) := InDelaySlot  // cause的BD区标志指令处于分支延迟槽
          }
            .otherwise{
              r_cpu0Regout.epc := io.S_exception_mem2cpu0Reg.currentInst_addr
              r_cpu0Regout.cause(31) := NotInDelaySlot
            }
        }
      }
      is(U"32'h0000000e"){
        // 不记录异常地址EPC（由上一clk可能存在的mtf0写入要转跳的epc）
        // 不在EXL区域中记录异常返回事件
        r_cpu0Regout.status(1) := NotInException  // cause的EXL区域，标记已处于异常级（同时禁止中断），等待异常返回ERET才置0
      }
    }
  }
  io.M_cpu0Regout := areaClk.r_cpu0Regout
  // 读取cpu0reg
  when(io.rst === RstEnable){
    io.S_cpu0Read_ex2cpu0Reg.data := ZeroWord
  }
    .otherwise{
      io.S_cpu0Read_ex2cpu0Reg.data := io.S_cpu0Read_ex2cpu0Reg.addr.mux(
        CP0_REG_STATUS -> (areaClk.r_cpu0Regout.status),
        CP0_REG_COUNT -> (areaClk.r_cpu0Regout.count),
        CP0_REG_COMPARE -> (areaClk.r_cpu0Regout.compare),
        CP0_REG_CAUSE -> (areaClk.r_cpu0Regout.cause),
        CP0_REG_EPC -> (areaClk.r_cpu0Regout.epc),
        CP0_REG_CONFIG -> (areaClk.r_cpu0Regout.config),
        CP0_REG_PrId -> (areaClk.r_cpu0Regout.prid),
        default -> (ZeroWord)
      )
    }
}

trait Global_parameter{
  // 全局参数
  val RstEnable: Bool = True
  val RstDisable: Bool = False
  val ZeroWord = U"32'h0"
  val WriteEnable: Bool = True
  val WriteDisable: Bool = False
  val ReadEnable: Bool = True
  val ReadDisable: Bool = False
  val AluOpBus = 8  // 译码阶段输出 aluop_o的宽度
  val AluSelBus = 3 // 译码阶段输出 alusel_o输出
  val InstValid: Bool = False  // 指令有效，则非无效指令，记为False（用于无效指令异常判断）
  val InstInvalid: Bool = True  // 指令无效
  val Stop: Bool = True
  val NoStop: Bool = False
  val InDelaySlot: Bool = True  // 是否为延迟槽
  val NotInDelaySlot: Bool = False
  val Branch: Bool = True  // 分支预测
  val NotBranch: Bool = False
  val InterruptAssert: Bool = True
  val InterruptNotAssert: Bool = False
  val TrapAssert: Bool = True  // 系统调用产生的中断
  val TrapNotAssert: Bool = False
  val True_v: Bool = True  // 用于表示存在异常
  val False_v: Bool = False  // 用于表示无异常
  val ChipEnable: Bool = True  // 芯片使能
  val ChipDisable: Bool = False

  val Reg_31 = U"5'b11111"  // 一般寄存器堆的$31地址

  val stallDir:Int = 6  // 暂停指令的涉及阶段数目

  // 原始指令
  // 逻辑指令
  val EXE_AND =  U"6'b100100"
  val EXE_OR  =  U"6'b100101"
  val EXE_XOR =  U"6'b100110"
  val EXE_NOR =  U"6'b100111"
  val EXE_ANDI = U"6'b001100"
  val EXE_ORI  = U"6'b001101"
  val EXE_XORI = U"6'b001110"
  val EXE_LUI  = U"6'b001111"
  // 移位指令
  val EXE_SLL  = U"6'b000000"
  val EXE_SLLV = U"6'b000100"
  val EXE_SRL  = U"6'b000010"
  val EXE_SRLV = U"6'b000110"
  val EXE_SRA  = U"6'b000011"
  val EXE_SRAV = U"6'b000111"
  // 移动指令
  val EXE_MOVZ = U"6'b001010"
  val EXE_MOVN = U"6'b001011"
  val EXE_MFHI = U"6'b010000"
  val EXE_MTHI = U"6'b010001"
  val EXE_MFLO = U"6'b010010"
  val EXE_MTLO = U"6'b010011"
  // 算数指令
  val EXE_SLT   = U"6'b101010"
  val EXE_SLTU  = U"6'b101011"
  val EXE_SLTI  = U"6'b001010"
  val EXE_SLTIU = U"6'b001011"
  val EXE_ADD   = U"6'b100000"
  val EXE_ADDU  = U"6'b100001"
  val EXE_SUB   = U"6'b100010"
  val EXE_SUBU  = U"6'b100011"
  val EXE_ADDI  = U"6'b001000"
  val EXE_ADDIU = U"6'b001001"
  val EXE_CLZ   = U"6'b100000"
  val EXE_CLO   = U"6'b100001"
  // 乘法指令
  val EXE_MULT  = U"6'b011000"
  val EXE_MULTU = U"6'b011001"
  val EXE_MUL   = U"6'b000010"
  // 乘累加、乘累减指令
  val EXE_MADD  = U"6'b000000"
  val EXE_MADDU = U"6'b000001"
  val EXE_MSUB  = U"6'b000100"
  val EXE_MSUBU = U"6'b000101"
  // 除法指令
  val EXE_DIV  = U"6'b011010"
  val EXE_DIVU = U"6'b011011"
  // 转移/分支指令
  val EXE_J      = U"6'b000010"
  val EXE_JAL    = U"6'b000011"
  val EXE_JALR   = U"6'b001001"
  val EXE_JR     = U"6'b001000"
  val EXE_BEQ    = U"6'b000100"
  val EXE_BGEZ   = U"5'b00001"
  val EXE_BGEZAL = U"5'b10001"
  val EXE_BGTZ   = U"6'b000111"
  val EXE_BLEZ   = U"6'b000110"
  val EXE_BLTZ   = U"5'b00000"
  val EXE_BLTZAL = U"5'b10000"
  val EXE_BNE    = U"6'b000101"
  // 加载/储存指令
  val EXE_LB     = U"6'b100000"
  val EXE_LBU    = U"6'b100100"
  val EXE_LH     = U"6'b100001"
  val EXE_LHU    = U"6'b100101"
  val EXE_LL     = U"6'b110000"
  val EXE_LW     = U"6'b100011"
  val EXE_LWL    = U"6'b100010"
  val EXE_LWR    = U"6'b100110"
  val EXE_SB     = U"6'b101000"
  val EXE_SC     = U"6'b111000"
  val EXE_SH     = U"6'b101001"
  val EXE_SW     = U"6'b101011"
  val EXE_SWL    = U"6'b101010"
  val EXE_SWR    = U"6'b101110"
  // 特殊指令
  val EXE_SYNC = U"6'b001111"
  val EXE_PREF = U"6'b110011"
  val EXE_NOP  = U"6'b000000"
  val SSNOP    = U"32'b00000000000000000000000001000000"
  // 协处理器相关指令
  val COP0    = U"6'b010000"
  val EXE_MTC0 = U"5'b00100"
  val EXE_MFC0 = U"5'b00000"
  // 异常相关指令
  val EXE_SYSCALL       = U"6'b001100"
  val EXE_TEQ           = U"6'b110100"
  val EXE_TEQI          = U"5'b01100"
  val EXE_TGE           = U"6'b110000"
  val EXE_TGEI          = U"5'b01000"
  val EXE_TGEIU         = U"5'b01001"
  val EXE_TGEU          = U"6'b110001"
  val EXE_TLT           = U"6'b110010"
  val EXE_TLTI          = U"5'b01010"
  val EXE_TLTIU         = U"5'b01011"
  val EXE_TLTU          = U"6'b110011"
  val EXE_TNE           = U"6'b110110"
  val EXE_TNEI          = U"5'b01110"
  val EXE_ERET          = U"6'b011000"
//  val EXE_ERET          = U"32'b01000010000000000000000000011000"

  val EXE_SPECIAL_INST  = U"6'b000000"
  val EXE_REGIMM_INST   = U"6'b000001"
  val EXE_SPECIAL2_INST = U"6'b011100"

  // AluOp  ALU操作
  // 逻辑操作
  val EXE_AND_OP  =  U"8'b00100100"
  val EXE_OR_OP   =  U"8'b00100101"
  val EXE_XOR_OP  =  U"8'b00100110"
  val EXE_NOR_OP  =  U"8'b00100111"
  val EXE_ANDI_OP =  U"8'b01011001"  // I指使用立即数Immediate而不是二元操作数
  val EXE_ORI_OP  =  U"8'b01011010"
  val EXE_XORI_OP =  U"8'b01011011"
  val EXE_LUI_OP  =  U"8'b01011100"
  // 移位操作
  val EXE_SLL_OP  =  U"8'b01111100"
  val EXE_SLLV_OP =  U"8'b00000100"
  val EXE_SRL_OP  =  U"8'b00000010"
  val EXE_SRLV_OP =  U"8'b00000110"
  val EXE_SRA_OP  =  U"8'b00000011"
  val EXE_SRAV_OP =  U"8'b00000111"
  // 移动操作
  val EXE_MOVZ_OP =  U"8'b00001010"
  val EXE_MOVN_OP =  U"8'b00001011"
  val EXE_MFHI_OP =  U"8'b00010000"
  val EXE_MTHI_OP =  U"8'b00010001"
  val EXE_MFLO_OP =  U"8'b00010010"
  val EXE_MTLO_OP =  U"8'b00010011"
  // 算术操作
  val EXE_SLT_OP   = U"8'b00101010"
  val EXE_SLTU_OP  = U"8'b00101011"
  val EXE_SLTI_OP  = U"8'b01010111"
  val EXE_SLTIU_OP = U"8'b01011000"
  val EXE_ADD_OP   = U"8'b00100000"
  val EXE_ADDU_OP  = U"8'b00100001"
  val EXE_SUB_OP   = U"8'b00100010"
  val EXE_SUBU_OP  = U"8'b00100011"
  val EXE_ADDI_OP  = U"8'b01010101"
  val EXE_ADDIU_OP = U"8'b01010110"
  val EXE_CLZ_OP   = U"8'b10110000"
  val EXE_CLO_OP   = U"8'b10110001"
  // 乘法操作
  val EXE_MULT_OP  = U"8'b00011000"
  val EXE_MULTU_OP = U"8'b00011001"
  val EXE_MUL_OP   = U"8'b10101001"
  // 乘累加、乘累减操作
  val EXE_MADD_OP  = U"8'b10100110"
  val EXE_MADDU_OP = U"8'b10101000"
  val EXE_MSUB_OP  = U"8'b10101010"
  val EXE_MSUBU_OP = U"8'b10101011"
  // 除法操作
  val EXE_DIV_OP   = U"8'b00011010"
  val EXE_DIVU_OP  = U"8'b00011011"
  // 转移/分支操作
  val EXE_J_OP       = U"8'b01001111"
  val EXE_JAL_OP     = U"8'b01010000"
  val EXE_JALR_OP    = U"8'b00001001"
  val EXE_JR_OP      = U"8'b00001000"
  val EXE_BEQ_OP     = U"8'b01010001"
  val EXE_BGEZ_OP    = U"8'b01000001"
  val EXE_BGEZAL_OP  = U"8'b01001011"
  val EXE_BGTZ_OP    = U"8'b01010100"
  val EXE_BLEZ_OP    = U"8'b01010011"
  val EXE_BLTZ_OP    = U"8'b01000000"
  val EXE_BLTZAL_OP  = U"8'b01001010"
  val EXE_BNE_OP     = U"8'b01010010"
  // 加载/储存操作
  val EXE_LB_OP      = U"8'b11100000"
  val EXE_LBU_OP     = U"8'b11100100"
  val EXE_LH_OP      = U"8'b11100001"
  val EXE_LHU_OP     = U"8'b11100101"
  val EXE_LL_OP      = U"8'b11110000"
  val EXE_LW_OP      = U"8'b11100011"
  val EXE_LWL_OP     = U"8'b11100010"
  val EXE_LWR_OP     = U"8'b11100110"
  val EXE_PREF_OP    = U"8'b11110011"
  val EXE_SB_OP      = U"8'b11101000"
  val EXE_SC_OP      = U"8'b11111000"
  val EXE_SH_OP      = U"8'b11101001"
  val EXE_SW_OP      = U"8'b11101011"
  val EXE_SWL_OP     = U"8'b11101010"
  val EXE_SWR_OP     = U"8'b11101110"
  val EXE_SYNC_OP    = U"8'b00001111"
  // 协处理器相关操作
  val EXE_MFC0_OP    = U"8'b01011101"
  val EXE_MTC0_OP    = U"8'b01100000"
  // 异常相关操作
  val EXE_SYSCALL_OP  = U"8'b00001100"
  val EXE_TEQ_OP      = U"8'b00110100"
  val EXE_TEQI_OP     = U"8'b01001000"
  val EXE_TGE_OP      = U"8'b00110000"
  val EXE_TGEI_OP     = U"8'b01000100"
  val EXE_TGEIU_OP    = U"8'b01000101"
  val EXE_TGEU_OP     = U"8'b00110001"
  val EXE_TLT_OP      = U"8'b00110010"
  val EXE_TLTI_OP     = U"8'b01000110"
  val EXE_TLTIU_OP    = U"8'b01000111"
  val EXE_TLTU_OP     = U"8'b00110011"
  val EXE_TNE_OP      = U"8'b00110110"
  val EXE_TNEI_OP     = U"8'b01001001"
  val EXE_ERET_OP     = U"8'b01101011"
  // 空
  val EXE_NOP_OP = U"8'b00000000"

  //AluSel
  val EXE_RES_LOGIC           = U"3'b001"  // 逻辑操作
  val EXE_RES_SHIFT           = U"3'b010"  // 移位操作
  val EXE_RES_MOVE            = U"3'b011"  // 移动操作
  val EXE_RES_ARITHMETIC      = U"3'b100"  // 算术操作
  val EXE_RES_MUL             = U"3'b101"  // 乘法操作
  val EXE_RES_JUMP_BRANCH     = U"3'b110" // 转移/分支操作
  val EXE_RES_LOAD_STORE      = U"3'b111"  // 加载/储存操作
  val EXE_RES_NOP             = U"3'b000"

  // 除法相关
  val DivResultReady:Bool = True
  val DivResultNotReady: Bool = False
  val DivStart: Bool = True
  val DivStop: Bool = False
  val DivIsAnnul:Bool = True
  val DivNotAnnul:Bool = False
  val DivIsSignal:Bool = True
  val DivNotSignal:Bool = False

  //指令存储器inst_rom
  val InstAddrBus = 32  // ROM地址总线宽度
  val InstBus = 32  // ROM数据总线宽度
//  val InstMemNum = 131071  // ROM实际大小：128KB
//  val InstMemNumLog2 = 17  // ROM实际使用的地址宽度,即 2^17=131071，PC计数为32bits，实际上对于ROM储存区只用到了17bits即可
  val InstMemNum = 65536  // ROM实际大小：64KB
  val InstMemNumLog2 = 16  // ROM实际使用的地址宽度,即 2^16=65536，PC计数为32bits，实际上对于ROM储存区只用到了16bits即可

  // cpu通用寄存器regfile
  val RegAddrBus = 5  // Regfile模块地址线宽度
  val RegBus = 32  // Regfile模块数据线宽度
  val RegWidth = 32  // 通用寄存器宽度
  val DoubleRegWidth = 64  // 双倍的通用寄存器宽度
  val DoubleRegBus = 64 // 双倍的通用寄存器数据线宽度
  val cntWidth = 2 // 流水线暂停相关的计数长度
  val RegNum = 32  // 通用寄存器数量
  val RegNumLog2 = 5  // 通用寄存器使用的地址位数
  val NOPRegAddr = U"5'b00000"

  // 数据储存器（访存涉及的内存RAM）
  val DataAddrBus   = 32
  val DataBus       = 32
  val DataMemNum    = 65536  // // 单个RAM实际大小：64KB
  val DataMemNumLog2  = 16  // 单个RAM实际使用的地址宽度,即 2^16=65536
  val ByteWidth       = 8
  val MemSelBus = 4 // mem的字节选择宽度（0000-1111）
  val MemSelZero = U"4'b0000"  // 内存字节的初始化选择为第0位
  val MemSelOne = U"4'b1111"  // 内存字节的初始化选择为无效的全1
  val Memsel_1 = U"4'b1000"  // 内存返回字的第一个字节（首字节）
  val Memsel_2 = U"4'b0100"
  val Memsel_3 = U"4'b0010"
  val Memsel_4 = U"4'b0001"
  val Memsel_1H = U"4'b1100"  // 第一个半字
  val Memsel_2H = U"4'b0011"
  val Memsel_maskL1 = U"4'b0111"  // SWL 一个字节空悬的的掩码
  val Memsel_maskL2 = U"4'b0011"
  val Memsel_maskL3 = U"4'b0001"
  val Memsel_maskR1 = U"4'b1110"  // SWR 一个字节空悬的的掩码
  val Memsel_maskR2 = U"4'b1100"
  val Memsel_maskR3 = U"4'b1000"

  // llBit 链接状态位相关
  val llBitOne = True
  val llBitZero = False

  // CP0寄存器地址
  val CP0_REG_COUNT     = U"5'b01001"        //可读写
  val CP0_REG_COMPARE   = U"5'b01011"      //可读写
  val CP0_REG_STATUS    = U"5'b01100"       //可读写
  val CP0_REG_CAUSE     = U"5'b01101"        //只读
  val CP0_REG_EPC       = U"5'b01110"          //可读写
  val CP0_REG_PrId      = U"5'b01111"         //只读
  val CP0_REG_CONFIG    = U"5'b10000"       //只读
  val CPU0AddrBus:Int  =  5  // cpu0的地址宽度
  val CPU0RegBus:Int   = 32  // cpu0内部寄存器宽度
  val InterruptBus:Int = 6  // 输入外界中断的宽度

  // 异常处理相关
  val ExceptTypeBus   = 32  // 异常类型的宽度
  val hasException = True  // 有异常发生
  val noException = False  // 无异常
  val IsInException = True  // 处于异常处理状态
  val NotInException = False  // 不处于异常处理状态
  val AllowException = True  // 允许中断
  val NotAllowException = False  // 不允许中断

  // Configure the clock domain
  val ClockConfig_rstH = ClockDomainConfig(
      clockEdge        = RISING,
      resetKind        = SYNC,
      resetActiveLevel = HIGH  // RstEnable为1
    )
  val ClockConfig_rstL = ClockDomainConfig(
    clockEdge        = RISING,
    resetKind        = SYNC,
    resetActiveLevel = LOW  // RstEnable为0
  )
}

// Master & Slave接口
trait Interface_MS extends Global_parameter {
  case class ifId2idInterface(pcWidth:Int, instWidth:Int) extends Bundle with IMasterSlave{
    val pc =  UInt(pcWidth bits)
    val inst =  UInt(instWidth bits)
    override def asMaster(): Unit = {
      out(pc, inst)
    }
  }
  case class id2regfileInterface(raddr1Width:Int, rdata1Width:Int, raddr2Width:Int, rdata2Width:Int) extends Bundle with IMasterSlave{
    val re1 = Bool
    val re2 = Bool
    val raddr1 = UInt(raddr1Width bits)
    val rdata1 = UInt(rdata1Width bits)
    val raddr2 = UInt(raddr2Width bits)
    val rdata2 = UInt(rdata2Width bits)
    override def asMaster(): Unit = {
      in(rdata1)
      in(rdata2)
      out(raddr1)
      out(raddr2)
      out(re1)
      out(re2)
    }
  }
  case class id2idExInterface(aluopBusWidth:Int, aluselBusWidth:Int, reg1Width:Int, reg2Width:Int, wdWidth:Int, instWidth:Int) extends Bundle with IMasterSlave{
    val aluop = UInt(aluopBusWidth bits)
    val alusel = UInt(aluselBusWidth bits)
    val reg1 = UInt(reg1Width bits)
    val reg2 = UInt(reg2Width bits)
    val waddr = UInt(wdWidth bits)  // 要写入的目标地址
    val we = Bool
    val inst = UInt(instWidth bits)  // 指令码
    override def asMaster(): Unit = {
      out(aluop)
      out(alusel)
      out(reg1)
      out(reg2)
      out(waddr)
      out(we)
      out(inst)
    }
  }
  case class idEx2exInterface(aluopBusWidth:Int, aluselBusWidth:Int, reg1Width:Int, reg2Width:Int, wdWidth:Int, instWidth:Int) extends Bundle with IMasterSlave{
    val aluop = UInt(aluopBusWidth bits)
    val alusel = UInt(aluselBusWidth bits)
    val reg1 = UInt(reg1Width bits)
    val reg2 = UInt(reg2Width bits)
    val waddr = UInt(wdWidth bits)
    val we = Bool
    val inst = UInt(instWidth bits)
    override def asMaster(): Unit = {
      out(aluop)
      out(alusel)
      out(reg1)
      out(reg2)
      out(waddr)
      out(we)
      out(inst)
    }
  }
  // 使用wInterface贯穿 EX-->EX/MEM-->MEM-->MEM/WB-->regFile过程
  case class wInterface(wdWidth:Int,wdataWidth:Int)extends Bundle with IMasterSlave {
    val wdata = UInt(wdataWidth bits)
    val waddr = UInt(wdWidth bits)
    val we = Bool
    override def asMaster(): Unit = {
      out(wdata)
      out(waddr)
      out(we)
    }
  }
  // 使用hiloReg贯穿特殊寄存器hi、lo的数据传递过程
  case class hiloRegInterface(dataWidth:Int) extends Bundle with IMasterSlave{
    val hi = UInt(dataWidth bits)
    val lo = UInt(dataWidth bits)
    override def asMaster(): Unit = {
      out(hi)
      out(lo)
    }
  }
  // 使用hiloInterface贯穿特殊寄存器hi、lo、允许写入hi/lo标志位的数据传递过程
  case class hiloInterface(dataWidth:Int) extends Bundle with IMasterSlave{
    val hi = UInt(dataWidth bits)
    val lo = UInt(dataWidth bits)
    val w_hiloEnable = Bool  // 允许写入hi/lo标志位
    override def asMaster(): Unit = {
      out(hi)
      out(lo)
      out(w_hiloEnable)
    }
  }

  // 使用stallHILO以描述乘累加、乘累减操作下，hi、lo特殊寄存器的临时存放(EX与EXMEM的交流）
  case class stallHiLoInterface(dataWidth:Int, cntWidth:Int) extends Bundle with IMasterSlave{
    val hilo_temp = UInt(dataWidth bits)
    val cnt = UInt(cntWidth bits)
    override def asMaster(): Unit = {
      out(hilo_temp)
      out(cnt)
    }
  }

  // 使用div_result描述除法计算结果
  case class divResultInterface(dataWidth:Int) extends Bundle with IMasterSlave{
    val resultDiv = UInt(dataWidth bits)
    val ready = Bool
    override def asMaster(): Unit = {
      out(resultDiv)
      out(ready)
    }
  }

  // 使用divConnect表示ex与div模块间的数据传递
  case class divConnectInterface(dataWidth:Int) extends Bundle with IMasterSlave{
    val start = Bool
    val opData1 = UInt(dataWidth bits)  // 被除数
    val opData2 = UInt(dataWidth bits)  // 除数
    val signalDiv = Bool  // 是否为有符号除法
    override def asMaster(): Unit = {
      out(start)
      out(opData1)
      out(opData2)
      out(signalDiv)
    }
  }

  // 使用branchInterface传递id译码步骤与取值步骤-判断是否为转移/分支
  case class branchDetermineInterface(addrWidth:Int) extends Bundle with IMasterSlave{
    val brach_flag = Bool
    val brach_targetAddress = UInt(addrWidth bits)  // 取值的目标地址
    override def asMaster(): Unit = {
      out(brach_flag)
      out(brach_targetAddress)
    }
  }

  // 关于转移/分支指令的id与idEx的交互（延迟槽相关）
  case class branchInterce(addrWidth:Int) extends Bundle with IMasterSlave{
    val next_inst_inDelayslot = Bool  // 下一条指令位于延迟槽
    val is_inDelayslot = Bool // 告诉后面的模块本条指令是否在延迟槽(来自idEx，依靠前一时刻的next_inst_inDelayslot获得)
    val check_inDelayslot = Bool  // 检查本条指令是否在延迟槽(送往idEx/为后续ex执行步骤待用)
    val link_addr = UInt(addrWidth bits)  // 分支指令bltzal、bgezal相关 保存转移指令后的第二条指令地址（即延迟槽后的指令，可能是为for等做准备） 在ex步骤中保存于寄存器$31
    override def asMaster(): Unit = {
      out(next_inst_inDelayslot)
      in(check_inDelayslot)
      out(is_inDelayslot)
      out(link_addr)
    }
  }

  case class branchLinkInterface(addrWidth:Int) extends Bundle with IMasterSlave{
    val is_inDelayslot = Bool  // 留待异常指令使用
    val link_addr = UInt(addrWidth bits)
    override def asMaster(): Unit = {
      out(is_inDelayslot)
      out(link_addr)
    }
  }

  // 加载/储存指令相关数据流动接口,贯穿ex、exMem、mem模块
  case class load_storeInterface(aluopBusWidth:Int,addrWidth:Int,regWidth:Int) extends Bundle with IMasterSlave{
    val aluop = UInt(aluopBusWidth bits)  // 运算类型
    val memAddr = UInt(addrWidth bits)  // 访存模块要访问的地址
    val memReg = UInt(addrWidth bits)  // 通用寄存器rt数据
    override def asMaster(): Unit = {
      out(aluop)
      out(memAddr)
      out(memReg)
    }
  }

  // 访存区 内存接口
  case class ramInterface(selWidth:Int,addrWidth:Int,dataWidth:Int) extends Bundle with IMasterSlave{
    val ramEn = Bool  // 内存使能
    val we = Bool  // 内存可写
    val sel = UInt(selWidth bits)  // // 选择内存对应字起始端地址的内部字节（一个字有4个字节，每个字节8bit）,与ram的总线形式相关
    // （在预先设计中，ram会忽略mem输出的32bit addr的后两位，依据sel进行补全）
    val addr = UInt(addrWidth bits)
    val dataWrite = UInt(dataWidth bits)  // 向内存写入的数据
    val dataRead = UInt(dataWidth bits)  // 从内存读取的数据
    override def asMaster(): Unit = {
      out(ramEn)
      out(we)
      out(sel)
      out(addr)
      out(dataWrite)
      in(dataRead)
    }
  }

  // LLbit链接访问位的数据接口（ll、sc等）在mem、memWb、llibit_reg间的传递(包括memWb向mem的llbit相关数据前推)
  case class llbitInterface() extends Bundle with IMasterSlave{
    val llBit = Bool
    val we = Bool
    override def asMaster(): Unit = {
      out(llBit)
      out(we)
    }
  }

  // cpu0的写入数据传播
  case class cpu0WriteInterface(addrWidth:Int,dataWidth:Int) extends Bundle with IMasterSlave{
    val addr = UInt(addrWidth bits)
    val data = UInt(dataWidth bits)
    val we = Bool
    override def asMaster(): Unit = {
      out(addr)
      out(data)
      out(we)
    }
  }

  // ex向cpu0的读取通道
  case class cpu0ReadInterface(addrWidth:Int,dataWidth:Int) extends Bundle with IMasterSlave {
    val addr = UInt(addrWidth bits)
    val data = UInt(dataWidth bits)
    override def asMaster(): Unit = {
      out(addr)
      in(data)
    }
  }

  // cpu0的对外输出各寄存器值
  case class cpu0RegOutInterface(dataWidth:Int) extends Bundle with IMasterSlave {
    val status = UInt(dataWidth bits)  // 状态寄存器，控制处理器操作模式、中断使能、诊断状态
    val count = UInt(dataWidth bits)  // 计数
    val compare = UInt(dataWidth bits)  // 用于与count比对，完成定时中断
    val cause = UInt(dataWidth bits)  // 记录外部异常发生，控制软件中断请求
    val epc = UInt(dataWidth bits)  // Exception Program Counter 储存异常返回地址
    val config = UInt(dataWidth bits)  // 处理器相关配置及功能信息
    val prid = UInt(dataWidth bits)  // Processor Identifier 处理标志寄存器，保存版本信息等
    val timerInterrupt = Bool  // 定时中断
    override def asMaster(): Unit = {
      out(status)
      out(count)
      out(compare)
      out(cause)
      out(epc)
      out(config)
      out(prid)
      out(timerInterrupt)
    }
  }

  // 异常类型及异常发生地址的传递
  case class exceptTypeAddrInterface(exceptBus:Int,instWidth:Int) extends Bundle with IMasterSlave {
    val exceptType = UInt(exceptBus bits)
    val currentInst_addr = UInt(instWidth bits)
    override def asMaster(): Unit = {
      out(exceptType)
      out(currentInst_addr)
    }
  }

  // 包含延迟槽标志位的异常类型及异常发生地址的传递
  case class exceptInterface(exceptBus:Int,instWidth:Int) extends Bundle with IMasterSlave {
    val exceptType = UInt(exceptBus bits)
    val currentInst_addr = UInt(instWidth bits)
    val is_inDelayslot = Bool
    override def asMaster(): Unit = {
      out(exceptType)
      out(currentInst_addr)
      out(is_inDelayslot)
    }
  }

  // CPU0向访存区传递的寄存器值，以辅助判断异常触发
  case class exceptCPU0RegoutInterface(dataWidth:Int) extends Bundle with IMasterSlave {
    val status = UInt(dataWidth bits)
    val cause = UInt(dataWidth bits)
    val epc = UInt(dataWidth bits)
    override def asMaster(): Unit = {
      out(status)
      out(cause)
      out(epc)
    }
  }
}

class CPU_ISA extends  Component with Global_parameter with Interface_MS {
  val io = new Bundle {
    val clk = in Bool
    val rst = in Bool
    val rom_data_i = in UInt (InstBus bits)
    val rom_addr_o = out UInt (InstAddrBus bits)
    val rom_ce_o = out Bool
    val M_cpu2ram = master(ramInterface(MemSelBus, DataAddrBus, DataBus))
    // 协处理器
    val externalIntr = in UInt (InterruptBus bits)
    val timerIntr = out Bool
  }
  val pc_Inst = new pcReg
  val ifId_Inst = new ifId
  val id_Inst = new id
  val regFile_Inst = new regFile
  val idEx_Inst = new idEx
  val ex_Inst = new ex
  val exMem_Inst = new exMem
  val mem_Inst = new mem
  val memWb_Inst = new memWb
  val hiloReg_Inst = new hiloReg
  val stallCtrl_Inst = new stallCtrl
  val div_Inst = new div
  val llbitReg_Inst = new llbitReg
  val cpu0Reg_Inst = new cpu0Reg

  io.rom_ce_o := pc_Inst.io.ce
  io.rom_addr_o := pc_Inst.io.pc

  io.M_cpu2ram connect mem_Inst.io.M_mem2ram
  io.timerIntr <> cpu0Reg_Inst.io.M_cpu0Regout.timerInterrupt

  pc_Inst.io.clk := io.clk
  pc_Inst.io.rst := io.rst
  pc_Inst.io.stall <> stallCtrl_Inst.io.stall // 执行流水线暂停
  pc_Inst.io.S_branch_id2pcReg connect id_Inst.io.M_branch_id2pcReg // 转移/分支指令下的指令影响
  pc_Inst.io.newPC <> stallCtrl_Inst.io.newPC // 异常指令，置入新指令地址
  pc_Inst.io.flush <> stallCtrl_Inst.io.flush

  ifId_Inst.io.clk := io.clk
  ifId_Inst.io.rst := io.rst
  ifId_Inst.io.if_pc := pc_Inst.io.pc
  ifId_Inst.io.if_inst := io.rom_data_i
  ifId_Inst.io.stall <> stallCtrl_Inst.io.stall // 执行流水线暂停
  ifId_Inst.io.flush <> stallCtrl_Inst.io.flush // 执行异常处理

  id_Inst.io.rst := io.rst
  id_Inst.io.S_ifId2id connect ifId_Inst.io.M_ifId2id
  id_Inst.io.S_PullForward_ex2id connect ex_Inst.io.M_ex2exMem
  id_Inst.io.S_PullForward_mem2id connect mem_Inst.io.M_mem2memWb
  id_Inst.io.stallCtrl <> stallCtrl_Inst.io.stallCtrl_id // 请求流水线暂停
  id_Inst.io.aluop_ex2id <> ex_Inst.io.aluop_ex2id // id感知上一周期的指令类型（load相关）

  idEx_Inst.io.clk := io.clk
  idEx_Inst.io.rst := io.rst
  idEx_Inst.io.S_id2idEx connect id_Inst.io.M_id2idEx
  idEx_Inst.io.stall <> stallCtrl_Inst.io.stall // 执行流水线暂停
  idEx_Inst.io.S_branch_id2idEx connect id_Inst.io.M_branch_id2idEx // 转移/分支指令相关
  idEx_Inst.io.flush <> stallCtrl_Inst.io.flush // 执行异常处理
  idEx_Inst.io.S_except_id2idEx connect id_Inst.io.M_except_id2idEx // 异常判断

  ex_Inst.io.rst := io.rst
  ex_Inst.io.S_idEx2ex connect idEx_Inst.io.M_idEx2ex
  ex_Inst.io.S_hiloData_hiloReg2ex connect hiloReg_Inst.io.M_hiloReg // 特殊寄存器hi、lo数据前推
  ex_Inst.io.S_hilo_mem2ex connect mem_Inst.io.M_hilo_mem2memWb // 特殊寄存器hi、lo数据前推
  ex_Inst.io.S_hilo_memWb2ex connect memWb_Inst.io.M_hilo_memWb2hiloReg // 特殊寄存器hi、lo数据前推
  ex_Inst.io.stallCtrl <> stallCtrl_Inst.io.stallCtrl_ex // 请求流水线暂停
  ex_Inst.io.S_tmpStall_mem_ex2ex connect exMem_Inst.io.M_tmpStall_mem_ex2ex // 流水线暂停(多周期操作缓存）
  ex_Inst.io.S_divResult_div2ex connect div_Inst.io.M_divResult_div2ex // 与除法相关
  ex_Inst.io.S_branchLink_idEx2ex connect idEx_Inst.io.M_branchLink_idEx2ex // 转移/分支指令相关
  ex_Inst.io.S_PullForword_cpu0Write_memWb2ex connect memWb_Inst.io.M_cpu0Write_memWb2cpu0Reg // 协处理器写数据前推
  ex_Inst.io.S_PullForward_cpu0Write_mem2ex connect mem_Inst.io.M_cpu0Write_mem2memWb // 协处理器写数据前推
  ex_Inst.io.S_except_idEx2ex connect idEx_Inst.io.M_except_idEx2ex // 异常判断

  div_Inst.io.clk := io.clk
  div_Inst.io.rst := io.rst
  div_Inst.io.S_divConnect_ex2div connect ex_Inst.io.M_divConnect_ex2div // 除法相关
  div_Inst.io.annul := DivNotAnnul

  exMem_Inst.io.clk := io.clk
  exMem_Inst.io.rst := io.rst
  exMem_Inst.io.S_ex2exMem connect ex_Inst.io.M_ex2exMem
  exMem_Inst.io.S_hilo_ex2exMem connect ex_Inst.io.M_hilo_ex2exMem // 特殊寄存器连接
  exMem_Inst.io.S_tmpStall_ex2mem_ex connect ex_Inst.io.M_tmpStall_ex2mem_ex // 流水线暂停
  exMem_Inst.io.stall <> stallCtrl_Inst.io.stall // 执行流水线暂停
  exMem_Inst.io.S_loadstore_ex2exMem connect ex_Inst.io.M_loadstore_ex2exMem // 访存RAM相关
  exMem_Inst.io.S_cpu0Write_ex2exMem connect ex_Inst.io.M_cpu0Write_ex2exMem // 协处理器写
  exMem_Inst.io.flush <> stallCtrl_Inst.io.flush // 执行异常处理
  exMem_Inst.io.S_exception_ex2exMem connect ex_Inst.io.M_exception_ex2exMem // 异常判断

  mem_Inst.io.rst := io.rst
  mem_Inst.io.S_exMem2mem connect exMem_Inst.io.M_exMem2mem
  mem_Inst.io.S_hilo_exMem2mem connect exMem_Inst.io.M_hilo_exMem2mem // 特殊寄存器连接
  mem_Inst.io.S_loadstore_exMem2mem connect exMem_Inst.io.M_loadstore_exMem2mem // 访存RAM相关
  mem_Inst.io.llbit <> llbitReg_Inst.io.llBit // 链接状态位
  mem_Inst.io.S_PullForward_memWb2mem connect memWb_Inst.io.M_memWb2llbitReg // llbit数据前推
  mem_Inst.io.S_cpu0Write_exMem2mem connect exMem_Inst.io.M_cpu0Write_exMem2mem // 协处理器写
  mem_Inst.io.S_exception_exMem2mem connect exMem_Inst.io.M_exception_exMem2mem // 异常判断
  mem_Inst.io.S_exceptCPU0RegOut_cpu0Reg2mem.status <> cpu0Reg_Inst.io.M_cpu0Regout.status // 读取cpu0寄存器
  mem_Inst.io.S_exceptCPU0RegOut_cpu0Reg2mem.cause <> cpu0Reg_Inst.io.M_cpu0Regout.cause // 读取cpu0寄存器
  mem_Inst.io.S_exceptCPU0RegOut_cpu0Reg2mem.epc <> cpu0Reg_Inst.io.M_cpu0Regout.epc // 读取cpu0寄存器
  mem_Inst.io.S_PullForward_cpu0Write_memWb2mem connect memWb_Inst.io.M_cpu0Write_memWb2cpu0Reg // cpu0写的数据前推

  memWb_Inst.io.clk := io.clk
  memWb_Inst.io.rst := io.rst
  memWb_Inst.io.S_mem2memWb connect mem_Inst.io.M_mem2memWb
  memWb_Inst.io.S_hilo_mem2memWb connect mem_Inst.io.M_hilo_mem2memWb // 特殊寄存器连接
  memWb_Inst.io.stall <> stallCtrl_Inst.io.stall // 执行流水线暂停
  memWb_Inst.io.S_llbit_mem2memWb connect mem_Inst.io.M_llbit_mem2memWb // 链接状态位
  memWb_Inst.io.S_cpu0Write_mem2memWb connect mem_Inst.io.M_cpu0Write_mem2memWb // 协处理器写
  memWb_Inst.io.flush <> stallCtrl_Inst.io.flush // 执行异常处理

  hiloReg_Inst.io.clk := io.clk
  hiloReg_Inst.io.rst := io.rst
  hiloReg_Inst.io.S_hiloInterface connect memWb_Inst.io.M_hilo_memWb2hiloReg // 特殊寄存器连接

  regFile_Inst.io.clk := io.clk
  regFile_Inst.io.rst := io.rst
  regFile_Inst.io.S_id2regfile connect id_Inst.io.M_id2regfile
  regFile_Inst.io.S_memWb2regfile connect memWb_Inst.io.M_memWb2regfile // 一般寄存器数据前推

  stallCtrl_Inst.io.rst := io.rst
  stallCtrl_Inst.io.epc <> mem_Inst.io.epc // 上一异常地址（异常返回 ERET使用）
  stallCtrl_Inst.io.exceptType <> mem_Inst.io.M_exception_mem2cpu0Reg.exceptType // 异常类型


  llbitReg_Inst.io.clk := io.clk
  llbitReg_Inst.io.rst := io.rst
  llbitReg_Inst.io.S_memWb2llbitReg connect memWb_Inst.io.M_memWb2llbitReg
  llbitReg_Inst.io.flush <> stallCtrl_Inst.io.flush // 执行异常处理

  cpu0Reg_Inst.io.clk := io.clk
  cpu0Reg_Inst.io.rst := io.rst
  cpu0Reg_Inst.io.S_cpu0Read_ex2cpu0Reg connect ex_Inst.io.M_cpu0Read_ex2cpu0Reg
  cpu0Reg_Inst.io.S_cpu0Write_memWb2cpu0Reg connect memWb_Inst.io.M_cpu0Write_memWb2cpu0Reg
  cpu0Reg_Inst.io.externalInterrupt <> io.externalIntr
  cpu0Reg_Inst.io.S_exception_mem2cpu0Reg connect mem_Inst.io.M_exception_mem2cpu0Reg // 异常类型判断
}

class ROM extends  Component with Global_parameter{
  val io = new Bundle{
    val ce = in Bool
    val pc_addr = in UInt(InstAddrBus bits)
    val inst = out UInt(InstBus bits)
  }

  val init_array = new Array[UInt](InstMemNum)
  // ORI指令的测试
//  init_array(0)=U"32'h34011100"
//  init_array(1)=U"32'h34020020"
//  init_array(2)=U"32'h3403ff00"
//  init_array(3)=U"32'h3404ffff"
  // 数据前推的测试
//  init_array(0)=U"32'h34011100"
//  init_array(1)=U"32'h34210020"
//  init_array(2)=U"32'h34214400"
//  init_array(3)=U"32'h34210044"

  // 逻辑测试
  init_array(0) = U"32'h3c010101"
  init_array(1) = U"32'h34210101"
  init_array(2) = U"32'h34221100"
  init_array(3) = U"32'h00220825"
  init_array(4) = U"32'h302300fe"
  init_array(5) = U"32'h00610824"
  init_array(6) = U"32'h3824ff00"
  init_array(7) = U"32'h00810826"
  init_array(8) = U"32'h00810827"

  for(i <- 9 until InstMemNum) {init_array(i)=U"32'h00000000"}

//  val mem_rom = Mem(UInt(InstBus bits),InstMemNum)
  val mem_rom = Mem(UInt(InstBus bits),initialContent = init_array)
  when( io.ce === ChipDisable){
    io.inst := ZeroWord
  }.otherwise{
    io.inst := mem_rom(io.pc_addr(InstMemNumLog2+1 downto 2))  // 所以取pc_addr具有32bits，而ROM储存区条目只有InstMemNum（2^17=131071)
                                                                // 所以取pc_addr[16:0]，又因为pc_addr每一clk加2^2=4，相当于低端的2位没有作用
                                                               // 或者说，等同于pc_addr右移2bit，故而取pc_addr[18:2]即可反映Ori指令下的地址
  }
}

class RAM extends Component with Global_parameter with Interface_MS {
  val io = new Bundle{
    val clk = in Bool
    val S_mem2ram = slave(ramInterface(MemSelBus,DataAddrBus,DataBus))
  }

  val ram_0 = Mem(UInt(ByteWidth bits),DataMemNum)  // 每个ram模块内含8bit*DataMemNum的数据，则四个ram并行共有32bit数据
  val ram_1 = Mem(UInt(ByteWidth bits),DataMemNum)
  val ram_2 = Mem(UInt(ByteWidth bits),DataMemNum)
  val ram_3 = Mem(UInt(ByteWidth bits),DataMemNum)

  // 读内存操作,mem访存请求读取内存时，内存一次提供字，由mem决定取字的字节/半字/全字
  when(io.S_mem2ram.ramEn === ChipEnable){
    when(io.S_mem2ram.we === WriteDisable){
      // 取对应地址（去掉后两位，即对齐4字节，共取16bit地址）
      io.S_mem2ram.dataRead := ram_3(io.S_mem2ram.addr(DataMemNumLog2+1 downto 2)) @@
        ram_2(io.S_mem2ram.addr(DataMemNumLog2+1 downto 2)) @@
        ram_1(io.S_mem2ram.addr(DataMemNumLog2+1 downto 2)) @@
        ram_0(io.S_mem2ram.addr(DataMemNumLog2+1 downto 2))
      }
      .otherwise{
        io.S_mem2ram.dataRead := ZeroWord
      }
    }
    .otherwise{
      io.S_mem2ram.dataRead := ZeroWord  // 防止latch
    }

  // 写操作：时序逻辑电路, rst==RstDisable触发
  val wClockDomain = ClockDomain(
    clock = io.clk,
    reset = null,
    config = ClockConfig_rstH
  )

  val r_ramAddr = io.S_mem2ram.addr(DataMemNumLog2+1 downto 2)  // 舍去原始地址后两位，对齐4的倍数
  val r_ram0En = io.S_mem2ram.sel(0)
  val r_ram1En = io.S_mem2ram.sel(1)
  val r_ram2En = io.S_mem2ram.sel(2)
  val r_ram3En = io.S_mem2ram.sel(3)
  val r_data0 = io.S_mem2ram.dataWrite(7 downto 0)
  val r_data1 = io.S_mem2ram.dataWrite(15 downto 8)
  val r_data2 = io.S_mem2ram.dataWrite(23 downto 16)
  val r_data3 = io.S_mem2ram.dataWrite(31 downto 24)

  val areaClk = new ClockingArea(wClockDomain) {
    ram_0.write(
      address = r_ramAddr,
      data = r_data0,
      enable = r_ram0En && (io.S_mem2ram.ramEn === ChipEnable) && (io.S_mem2ram.we === WriteEnable)
    )
    ram_1.write(
      address = r_ramAddr,
      data = r_data1,
      enable = r_ram1En && (io.S_mem2ram.ramEn === ChipEnable) && (io.S_mem2ram.we === WriteEnable)
    )
    ram_2.write(
      address = r_ramAddr,
      data = r_data2,
      enable = r_ram2En && (io.S_mem2ram.ramEn === ChipEnable) && (io.S_mem2ram.we === WriteEnable)
    )
    ram_3.write(
      address = r_ramAddr,
      data = r_data3,
      enable = r_ram3En && (io.S_mem2ram.ramEn === ChipEnable) && (io.S_mem2ram.we === WriteEnable)
    )
  }
}

class socp extends Component with Global_parameter {
  val io = new Bundle{
    val clk = in Bool
    val rst = in Bool
  }
  val rom = new ROM
  val cpu_Inst = new CPU_ISA
  val ram = new RAM

  cpu_Inst.io.rom_ce_o <> rom.io.ce
  cpu_Inst.io.rom_addr_o <> rom.io.pc_addr
  cpu_Inst.io.rom_data_i <> rom.io.inst
  cpu_Inst.io.clk <> io.clk
  cpu_Inst.io.rst <> io.rst
  cpu_Inst.io.M_cpu2ram connect ram.io.S_mem2ram
  cpu_Inst.io.externalIntr(0) := cpu_Inst.io.timerIntr
  cpu_Inst.io.externalIntr(5 downto 1) := U"5'b00000"
  ram.io.clk <> io.clk
}

//Generate the MyTopLevel's Verilog
object MyOriISA {
  def main(args: Array[String]) {
    SpinalVerilog(new socp)
  }
}
object MyOriISA_Config extends SpinalConfig(defaultConfigForClockDomains = ClockDomainConfig(resetKind = SYNC))
//Generate the MyTopLevel's Verilog using the above custom configuration.
object MyOriISAVerilog {
  def main(args: Array[String]) {
    MyOriISA_Config
      .generateVerilog(new socp)
  }
}

object DutTest{
  def main(args: Array[String]): Unit = {
    val ClockConfig_rstL = ClockDomainConfig(
      clockEdge        = RISING,
      resetKind        = SYNC,
      resetActiveLevel = LOW  // RstEnable为1
    )
    SimConfig
      .withWave
      .withConfig(SpinalConfig(
        defaultClockDomainFrequency = FixedFrequency(50 MHz)
        ,defaultConfigForClockDomains = ClockConfig_rstL
      ))
//      .allOptimisation
      .workspacePath("~/sim_tmp")
      .compile(new socp)
      .doSim { dut =>
        // Simulation code here
//        ClockDomain(dut.io.clk,dut.io.rst).forkStimulus(10)
        dut.io.clk#=false
        dut.io.rst#=true
        sleep(10)
        dut.io.clk#=true
        sleep(10)
        dut.io.clk#=false
        dut.io.rst#=false
        sleep(10)
        dut.io.clk#=true
        for(i <- 0 until 100) {
                  dut.io.clk#=false
                  sleep(10)
                  dut.io.clk#=true
                  sleep(10)
        }
      }
  }
}

