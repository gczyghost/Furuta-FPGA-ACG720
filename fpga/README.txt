B114 FPGA 工程

B114 发布固件 UserCode 0x0000FF61；二进制在赛事提交 RAR 包，本源码仓库不含 firmware 目录。
SHA256：54153da7bfbd20d139bf763f19d23c8e5c841010fcbdc318234314dd141dd1e5

工程包含原B114实际编译的40个SystemVerilog文件、CST/SDC各1份、7份被引用ROM，以及直接构建入口build.tcl。rtl/及variants/为源码，constraints/为约束，rom/为ROM内容。没有测试、实验分支或额外准备脚本。

构建环境
Windows x64，Gowin V1.9.12.04（原发布工具build89079）及有效许可证。
器件：GW5AT-LV138PG484AC1/I0，device_version B。
顶层：variants/furuta_enabled.sv中的furuta_enabled。
引脚约束：constraints/acg720_adc_din.cst。
时序约束：constraints/acg720_enabled_din10.sdc。
无额外用户宏或外部include。Gowin器件原语由工具提供，不需要另加仿真库。
控制时钟25MHz，外设50MHz，协议时间基准50MHz。

构建方法（PowerShell）
将整个目录放到任意可写路径，保持目录结构。修改下面两处路径后执行：
& 'D:/Software/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe' 'D:/contest/fpga-clean/build.tcl'

build.tcl按自身位置定位源码，不依赖启动命令的当前目录；按原文件顺序执行综合和布局布线。工具选项保持SystemVerilog2017、dsp_style=dsp、rom_style=block_rom、global_freq=50、route_option=2、correct_hold_violation=1，并输出SDF和Verilog网表。具体选项均在build.tcl末尾。
构建工作区为本目录的build/，项目产物位于build/gowin/furuta_enabled/impl/。ROM由Tcl复制到工具工作目录的rom/，供源码相对路径读取，请不要单独移动ROM或更改脚本执行顺序。
若build/已存在，入口会停止；再次构建请使用不含旧build/的独立工程副本，避免覆盖结果。本源码仓库不附预编译位流。

源码与发布位流的关系
核心源码、约束和保留ROM逐字节来自已成功构建的B114冻结输入；仅将入口改为相对目录定位，并去除4份未被源码引用的ROM副本。本目录已经过输入完整性及Tcl路径/文件复制检查，没有为本次整理再次综合或布局布线，不宣称新重建通过或与发布位流逐位一致。新构建仍需检查工具退出、资源、时序与器件身份。
此目录不包含下载脚本。B114为易失SRAM固件；实机下载、标定与运动需要按设备使用流程另行处理。
