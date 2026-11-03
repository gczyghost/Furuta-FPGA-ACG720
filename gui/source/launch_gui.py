"""Start the B114 graphical host; connection requires an explicit user action."""
from pathlib import Path
import sys

def main():
    root = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent
    if not getattr(sys, 'frozen', False):
        sys.path.insert(0, str(root/'host'))
        sys.path.insert(0, str(root))
    data = root/'user-data'
    sys.argv[1:1] = ['--manual-control', '--records', str(data/'records'), '--settings', str(data/'ui-preferences.json')]
    from pc_host.fpga_app import main as gui_main
    return gui_main()

if __name__ == '__main__':
    raise SystemExit(main())
