# Build on Windows x64: python -m PyInstaller --clean --noconfirm FurutaFPGA.spec
from pathlib import Path
root = Path(SPECPATH)
a = Analysis([str(root/'launch_gui.py')], pathex=[str(root/'host'), str(root)],
             binaries=[], datas=[], hiddenimports=['tools.operation_log', 'serial.tools.list_ports_windows'],
             hookspath=[], hooksconfig={}, runtime_hooks=[], excludes=[], noarchive=False)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name='FurutaFPGA',
          debug=False, bootloader_ignore_signals=False, strip=False, upx=False, console=False)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name='FurutaFPGA')
