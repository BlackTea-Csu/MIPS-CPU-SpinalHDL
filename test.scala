package mylib
import spinal.{core, lib}
import spinal.core._
import spinal.core.internals.Operator
import spinal.lib.{MS, _}

import scala.language.postfixOps
import scala.util.Random
import spinal.lib.com.uart._
import spinal.lib.com.spi._
import BundleImplicit._
import spinal.lib.fsm._
import spinal.sim._
import spinal.core.sim._

import java.awt.{Graphics, Panel}
import scala.math._

// 图像读取
import java.awt.image.BufferedImage
import java.io.File
import javax.imageio.ImageIO


case class KernelConfig(lineNum : Int, // 限定的缓冲区行数(即为卷积核行数），需为奇数
                        lineDepth : Int, // 缓冲区的行缓冲区内像素数
                        pixelSize : Int, // 像素的位数
                        kernelRowNum : Int, // 卷积核的列数，需为奇数
                        ImagePixelNum : Int = 1254  // 图像的总像素点个数
                       )
// 以Sobel算子为例，需要从原图像矩阵中取 3*3 的窗口作卷积

//
case class ConvolutionArea(cfg:KernelConfig) extends Component {
  val io = new Bundle {
    val data_pixel = slave Flow (UInt(cfg.pixelSize bits))
    val kernelArea = master Flow (Vec(Vec(UInt(cfg.pixelSize bits), cfg.lineNum), cfg.kernelRowNum))
  }

  val r_row = Vec(UInt(cfg.pixelSize bits), cfg.lineNum) // 矩阵的列，由行缓冲的输出构成
  // 共有lineNum个行缓冲，形成卷积核的行
  val buffer_line = History(io.data_pixel.payload, cfg.lineNum * cfg.lineDepth + 1, init = U(0, cfg.pixelSize bits))
  when(io.data_pixel.valid && ~clockDomain.isResetActive) {
    // 当输入数据有效且非reset时才置入
    for (index <- 0 until cfg.lineNum) {
      r_row(index) := buffer_line((index + 1) * cfg.lineDepth) // 取第 lineDepth 个延时（即io.data_pixel.payload的后接lineDepth个移位寄存器）开始的输出
    }
  }
    .otherwise {
      for (index <- 0 until cfg.lineNum) {
        r_row(index) := U(0, cfg.pixelSize bits)
      }
    }
  val r_kernelArea = History(r_row, cfg.kernelRowNum,init = Vec(U(0,cfg.pixelSize bits), cfg.lineNum)) // 卷积核由kernelRowNum列的列（列由行缓冲的输出形成）构成，只需要缓冲cfg.kernelRowNum-1个即可
  //  延时t1-将第一个像素送往缓冲区中间行的最后一位
  //  延时t2-将第一个像素送往卷积核中心
  val t1 = (cfg.lineNum + 1) / 2 * cfg.lineDepth
  val t2 = (cfg.kernelRowNum + 1) / 2 - 1
  val delay = t1 + 1 + t2 + 1
  val r_valid = History(io.data_pixel.valid, delay,init = False)  // 实际延时
  io.kernelArea.payload := r_kernelArea
  io.kernelArea.valid := clockDomain.isResetActive ? False | r_valid.vec.last
}

// Sobel边缘检测
case class Sobel_edgeDetection(cfg:KernelConfig) extends  Component{
  val io = new Bundle{
    val kernelArea = slave Flow(Vec(Vec(UInt(cfg.pixelSize bits), 3), 3))  // 接收的图像卷积区域，大小为 3行*3列
    val SobelThreshold = in UInt(cfg.pixelSize bits)  // 输入的灰度阈值
    val edgePixel = master Flow(Bool)
    val gradImg = master Flow(UInt(cfg.pixelSize bits))
  }
  val area =(clockDomain.isResetActive || ~io.kernelArea.valid) ? Vec(Vec(U(0,cfg.pixelSize bits),3),3) | (io.kernelArea.payload)
//  sobel卷积计算
  val Gx = (area(0)(0).expand.asSInt -^ area(2)(0).expand.asSInt) +^
    (( area(0)(1).expand.asSInt -^ area(2)(1).expand.asSInt)<<1) +^
    (area(0)(2).expand.asSInt  -^ area(2)(2).expand.asSInt)
  val Gy = (area(0)(2).expand.asSInt -^ area(0)(0).expand.asSInt) +^
    ((area(1)(2).expand.asSInt -^ area(1)(0).expand.asSInt)<<1) +^
    (area(2)(2).expand.asSInt -^ area(2)(0).expand.asSInt)
  val sobelAdd = (Gx.abs +|  Gy.abs).fixTo(cfg.pixelSize-1 downto 0,RoundType.ROUNDUP)  // 位对齐，取到原有的pixelSize位
  val sobelEdge = (sobelAdd > io.SobelThreshold) ? True | False
  val r_sobelEdge = RegNext(sobelEdge).init(False)  // 缓冲1个clk
  val r_sobelAdd = RegNext(sobelAdd).init(U(0, cfg.pixelSize bits))
  io.edgePixel.valid := RegNext(io.kernelArea.valid).init(False)
  io.edgePixel.payload := io.edgePixel.valid ? r_sobelEdge | False
  io.gradImg.valid := RegNext(io.kernelArea.valid).init(False)
  io.gradImg.payload := io.edgePixel.valid ? r_sobelAdd | U(0, cfg.pixelSize bits)
}

case class ImageRam(cfg:KernelConfig) extends Component {
  val io = new Bundle{
    val edgePixel = slave Flow(Bool)
  }
  val ramImage = Mem(UInt(cfg.pixelSize bits),cfg.ImagePixelNum)  // 保存图片(0,1)值的区域
  val r_count =  Reg(UInt((log((cfg.ImagePixelNum+1)*2) / log(2)).toInt bits))
    .init(U(0, cfg.ImagePixelNum bits).resized)  // 计数器，记录当前收到的像素个数
  when(io.edgePixel.valid){
    r_count := r_count + 1
  }
    .otherwise{
      r_count := (default -> false)
    }
  val r_pixel = Reg(UInt(cfg.pixelSize bits)).init(U(0,cfg.pixelSize bits))
  when(io.edgePixel.valid){
    when(io.edgePixel.payload){
      r_pixel := (default -> true)
    }
      .otherwise {
        r_pixel := (default -> false)
      }
  }
    .otherwise{
      r_pixel := (default -> false)
    }
  val r_valid = RegNext(io.edgePixel.valid).init(False)
  ramImage.write(
    address = r_count-1,
    data = r_pixel,
    enable = r_valid && (r_count <= cfg.ImagePixelNum)
  )
}


case class VirtualRam(cfg:KernelConfig) extends Component {
  val io = new Bundle{
    val data_pixel = master Flow(UInt(cfg.pixelSize bits))
  }
  val r_valid = Reg(Bool).init(False)
//  // 测试行
  val init_array = new Array[UInt](cfg.lineDepth) // 每行像素数目
  for (i <- 0 until cfg.lineDepth) {
    init_array(i) = U(i+1, cfg.pixelSize bits)
    val a = U(i+1, cfg.pixelSize bits).toString()
    println(Console.GREEN+s"init_array  " + Console.YELLOW + s"$i" + Console.GREEN+s" is "+ Console.YELLOW + s"$a")
  }
  val mem_rom = Mem(UInt(cfg.pixelSize bits), initialContent = init_array)

  // 历遍生成的行数据
  val r_pc = Reg(UInt((log(cfg.lineDepth*2) / log(2)).toInt bits)) init (U(0, cfg.lineDepth bits).resized)
  when(r_pc >= cfg.lineDepth-1 || clockDomain.isResetActive) {
    r_pc := 0
  }
    .otherwise {
      r_pc := r_pc + 1
    }

  val r_payload = RegNext(mem_rom(r_pc.resized)) init(U(0,cfg.pixelSize bits).resized)

//  val r_pc = Reg(UInt((log((cfg.ImagePixelNum+1)*2) / log(2)).toInt bits)) init (U(0, cfg.ImagePixelNum bits).resized)
//  when(r_pc === cfg.ImagePixelNum){
//    r_pc := U(cfg.ImagePixelNum,r_pc.getWidth bits)
//  }
//    .otherwise{
//      r_pc := r_pc + 1
//    }
//
//  val r_payload = Reg(mem_rom(r_pc.resized)) init(U(0,cfg.pixelSize bits).resized)
//  when(r_pc === cfg.ImagePixelNum){
//    r_payload := (default -> false)
//  }
//    .otherwise{
//      r_payload := mem_rom(r_pc.resized)
//    }
//
//  val r_valid = Reg(Bool) init(False)
//  when(r_pc === cfg.ImagePixelNum){
//    r_payload := False
//  }
//    .otherwise{
//      r_payload := True
//    }

  io.data_pixel.payload :=  clockDomain.isResetActive ?  U(0, cfg.pixelSize bits) | r_payload
  io.data_pixel.valid := clockDomain.isResetActive ? False | True
}

case class SobelProcess() extends  Component{
  // 读取数据，形成灰度图并保留像素矩阵
  val img: BufferedImage = ImageIO.read(new File("D:\\vivado_project\\ImageProcess\\hello.jpeg"))
  val grayImage = new BufferedImage(img.getWidth, img.getHeight, img.getType)  // 灰度图
  val matrixImg = Array.ofDim[Int](img.getWidth(), img.getHeight())  // 图像矩阵
  var i: Int = 0;
  for (i <- 0 to (img.getWidth() - 1); j <- 0 to (img.getHeight() - 1)) {
    val rgb = img.getRGB(i, j)
    println(s"rgb is : " + rgb)
    val r = (rgb >> 16) & 0xff;
    val g = (rgb >> 8) & 0xff;
    val b = rgb & 0xff;
    val gray = (r + g + b) / 3;
    val gray_pixel = (255 & 0xff) << 24 | (gray & 0xff) << 16 | (gray & 0xff) << 8 | gray & 0xff;
    println(s"gray_pixel is : " + gray_pixel)
    grayImage.setRGB(i,j,gray_pixel)
    matrixImg(i)(j)=grayImage.getRGB(i,j)
  }
  ImageIO.write(grayImage,"JPEG", new File("D:\\vivado_project\\ImageProcess\\gray_pixel.jpg"));
  println(s"matrixImg is : " + matrixImg.size + s" " + matrixImg(0).size)
  println(s"img Width is : " + img.getWidth)
  println(s"img Height is : " + img.getHeight)
  println(s"grayImage Width is : " + grayImage.getWidth)
  println(s"grayImage Height is : " + grayImage.getHeight)


  // 使用Sobel算子进行卷积计算
  val config = new KernelConfig(3,img.getWidth(),24,3)
  val io = new Bundle {
    val data = in UInt (config.pixelSize bits)  // 外来输入的像素
    val valid = in Bool  // 允许执行
    val gradOut = master Flow(UInt(config.pixelSize bits))
  }

  val convolutionArea = new ConvolutionArea(config)
  val sobel_edgeDetection = new Sobel_edgeDetection(config)

  convolutionArea.io.data_pixel.payload := io.data  // 输入原始像素数据
  convolutionArea.io.data_pixel.valid := io.valid
  sobel_edgeDetection.io.kernelArea <-< convolutionArea.io.kernelArea  // 输入卷积区域，进行卷积计算
  sobel_edgeDetection.io.SobelThreshold := U(0,config.pixelSize bits)
  io.gradOut <> sobel_edgeDetection.io.gradImg

  def init={
    clockDomain.forkStimulus(10)
//    SimTimeout(100000)
    clockDomain.assertReset()
    io.data #= 0
    io.valid #= false
    clockDomain.waitRisingEdge(5)
    clockDomain.deassertReset()
    clockDomain.waitRisingEdge(10)
  }

  def test={
    var gradArray = new Array[Int](img.getWidth()*img.getHeight())
    var count:Int = 0
    for(i <- 0 to (matrixImg(0).size-1);j <- 0 to (matrixImg.size- 1)){
      val pixelInt:Int = matrixImg(j)(i)
      val pixelFit:Int = (pixelInt<<8)>>>8
      val pixel:Int = pixelFit + 0xff000000
      println(s"matrixImg is on : " + j + s" " + i + s" value is " + pixelInt + " Fit: "+ pixelFit+ " raw: "+ pixel)
      io.data #= pixelFit
      io.valid #= true
      if (io.gradOut.valid.toBoolean == true){
        gradArray(count) = (io.gradOut.payload.toInt) + + 0xff000000
        println(s"gradArray is on : " + count + s" with value " + gradArray(count) + s" size: " )
        count = count + 1
      }
      clockDomain.waitRisingEdge()
    }
    io.valid #= false
    io.data #= 0
    println(s"在这里！ "+ io.gradOut.payload.toInt)
    while(io.gradOut.valid.toBoolean == true){
      gradArray(count) = (io.gradOut.payload.toInt) + + 0xff000000
      println(s"gradArray is on : " + count + s" with value " + gradArray(count))
      count = count + 1
      clockDomain.waitRisingEdge()
    }
    clockDomain.waitRisingEdge(10)

    // 写入图像
    val sobelImage = new BufferedImage(img.getWidth, img.getHeight, img.getType)  // 卷积后的梯度图
    for (i <- 0 to (sobelImage.getHeight() - 1); j <- 0 to (sobelImage.getWidth() - 1)) {
      val index = (i*sobelImage.getWidth()+j)
      sobelImage.setRGB(j,i,gradArray(index))
    }
    ImageIO.write(sobelImage,"JPEG", new File("D:\\vivado_project\\ImageProcess\\sobelImage.jpg"));
  }
}


case class VisionProcess() extends  Component{

  val config = new KernelConfig(3,3,32,3)
  val io = new Bundle{
    val kernel_out = out (Vec(Vec(UInt(config.pixelSize bits), config.lineNum),config.kernelRowNum))
    val kernel_valid = out Bool
  }
  val ram_Inst = VirtualRam(config)
  val kernel = ConvolutionArea(config)
  kernel.io.data_pixel <-< ram_Inst.io.data_pixel
  io.kernel_out := kernel.io.kernelArea.payload
  io.kernel_valid := kernel.io.kernelArea.valid
}


//Generate the MyTopLevel's Verilog
object imageProcess {
  def main(args: Array[String]) {
    SpinalVerilog(VisionProcess())
  }
}

object clockApp extends App{
  SpinalConfig(
    defaultConfigForClockDomains = ClockDomainConfig(resetKind = SYNC,resetActiveLevel = HIGH)
  ).generateVerilog(VisionProcess())
}
//object MyImageProcess_Config extends SpinalConfig(defaultConfigForClockDomains = ClockDomainConfig(resetKind = SYNC))
//Generate the MyTopLevel's Verilog using the above custom configuration.
object MyImageProcess {
  def main(args: Array[String]) {
    MyOriISA_Config
      .generateVerilog(VisionProcess())
  }
}


object dutSim {

  val simConfig = SpinalConfig(defaultClockDomainFrequency = FixedFrequency(50 MHz),
    defaultConfigForClockDomains = ClockDomainConfig(resetKind = SYNC))
  def main(args: Array[String]): Unit = {
    SimConfig
//      .withWave
      .withConfig(simConfig)
      .allOptimisation
      .compile(SobelProcess())
      .doSim{dut =>
        dut.init
        dut.test
      }
  }

}


