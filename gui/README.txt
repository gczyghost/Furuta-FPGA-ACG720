GUI95 源码（B114 配套）
Windows x64 / Python 3.12.10 x64。在 source 中：
  py -3.12 -m venv .venv
  .venv\Scripts\python.exe -m pip install -r requirements.txt
  .venv\Scripts\python.exe launch_gui.py
构建：
  .venv\Scripts\python.exe -m PyInstaller --clean --noconfirm FurutaFPGA.spec
输出 source/dist/FurutaFPGA，分发时保留完整目录及本目录 third-party-licenses。
source/tools/operation_log.py 是异步日志的真实运行依赖。
本仓库仅含源码，不含 EXE 及运行库二进制；赛事提交 RAR 包包含配套程序。
源码只保留正常启动路径；本次静态复核不等同于新打包或实机通过。
第三方许可见 third-party-licenses；本项目自有内容暂未授予开源许可证。
