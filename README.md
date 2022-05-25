# -CPU-SpinalHDL-
使用SpinalHDL实现《自己动手写CPU》的内容（仅1-12章内容）

本人在实现时，没能较好的理解SpinalHDL的时钟域含义，存在多处重复定义相同时钟域：  
val thisClockDomain = ClockDomain(
    clock = io.clk,
    reset = io.rst,
    config = ClockConfig_rstH
  )
其实可以同归于sysclkArea，也就不必要后续各模块的rst等信号控制如此啰嗦。

此外，当时本人没有很好的理解SpinalHDL结合verilator的使用，其实相关的汇编指令完全可以作为外部文档，利用Scala直接读取，完成仿真工作；本人当时是基于转译的Verilog实现仿真的，比较麻烦。后续在整理下基于SpinalHDL实现的数字图像处理-连通域分割的实现，里面实现了Scala读取图片-模拟外来输入流仿真。

本人在实现过程中，经过指令仿真，发现《自己动手写CPU》书中存在一些疏漏，后续有时间再整理补充。

SpinalHDL是个很好的框架，非常佩服Charles Papon的能力。感谢玉骐（微信号：FPGA_geek）对SpianlHDL的布道，实现过程中的模块间connect功能实现参考了他的代码。
