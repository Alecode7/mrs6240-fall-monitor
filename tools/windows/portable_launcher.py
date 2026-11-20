"""Entry point shared by the two Windows portable executables.

Self-tests deliberately never open a serial port or start SPI collection.
"""
from __future__ import annotations

import ctypes
import json
import os
import platform
import sys
import traceback
from pathlib import Path

if not getattr(sys, "frozen", False):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fall_detection.runtime_paths import application_directory, default_hif_dll_path


def _write_report(path: Path, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")


def self_test(path: Path) -> int:
    result = {"passed": False, "frozen": bool(getattr(sys, "frozen", False)),
              "application_directory": str(application_directory()),
              "python": platform.python_version(), "architecture": platform.machine(),
              "opened_hardware": False, "checks": {}}
    try:
        import tkinter
        import serial
        import matplotlib
        import numpy
        from fall_detection.device_source import HifDevicePointCloudSource
        from fall_detection.serial_event import parse_board_status

        result["checks"]["python_modules"] = True
        result["checks"]["tk_version"] = tkinter.TkVersion
        result["checks"]["matplotlib_version"] = matplotlib.__version__
        result["checks"]["numpy_version"] = numpy.__version__
        dll = default_hif_dll_path()
        with os.add_dll_directory(str(dll.parent)):
            native_library = ctypes.WinDLL(str(dll.with_name("CH347DLLA64.dll")))
            result["checks"]["ch347_native_dll"] = bool(native_library._handle)
            source = HifDevicePointCloudSource()
            source._create_api()
            result["checks"]["hif_dotnet_api"] = True
        sample = "FALL frame=1 state=0 track=1 target=1 x=0 y=0 z=100 h=60 pts=20 mot=5 v=7"
        result["checks"]["board_status_parser"] = parse_board_status(sample) is not None
        result["passed"] = all(result["checks"].get(key) for key in
                               ("python_modules", "ch347_native_dll", "hif_dotnet_api", "board_status_parser"))
    except Exception:
        result["error"] = traceback.format_exc()
    _write_report(path, result)
    return 0 if result["passed"] else 1


def launch() -> int:
    app_dir = application_directory()
    os.environ["MPLCONFIGDIR"] = str(app_dir / "logs" / ".matplotlib")
    # The frozen application ships pythonnet for the Windows .NET Framework.
    os.environ["PYTHONNET_RUNTIME"] = "netfx"
    if "--self-test" in sys.argv:
        index = sys.argv.index("--self-test")
        path = Path(sys.argv[index + 1]) if index + 1 < len(sys.argv) else app_dir / "logs" / "self-test.json"
        return self_test(path)

    smoke_path = None
    if "--smoke-test" in sys.argv:
        index = sys.argv.index("--smoke-test")
        smoke_path = Path(sys.argv[index + 1])
        del sys.argv[index:index + 2]
    if not any(arg == "--ui-mode" or arg.startswith("--ui-mode=") for arg in sys.argv[1:]):
        mode = "engineering" if "engineering" in Path(sys.executable).stem.lower() else "monitor"
        sys.argv.extend(["--ui-mode", mode])

    import tkinter as tk
    from fall_detection import serial_event_live_visualizer as app

    original_tk = tk.Tk
    errors = []

    def create_root(*args, **kwargs):
        root = original_tk(*args, **kwargs)

        def callback_error(exc_type, value, tb):
            detail = "".join(traceback.format_exception(exc_type, value, tb))
            errors.append(detail)
            _write_report(app_dir / "logs" / "last-error.json", {"error": detail})
            if smoke_path:
                root.after(0, finish)
            else:
                from tkinter import messagebox
                messagebox.showerror("程序运行错误", "请查看程序文件夹 logs/last-error.json。\n" + str(value), parent=root)

        def finish():
            if smoke_path:
                _write_report(smoke_path, {"passed": not errors, "title": root.title(),
                                          "width": root.winfo_width(), "height": root.winfo_height(),
                                          "application_directory": str(app_dir), "errors": errors,
                                          "opened_hardware": False})
            close_command = root.protocol("WM_DELETE_WINDOW")
            if close_command:
                root.tk.call(close_command)
            else:
                root.destroy()

        root.report_callback_exception = callback_error
        if smoke_path:
            root.after(2500, finish)
        return root

    tk.Tk = create_root
    try:
        app.main()
    finally:
        tk.Tk = original_tk
    return 1 if errors else 0


if __name__ == "__main__":
    try:
        raise SystemExit(launch())
    except Exception:
        detail = traceback.format_exc()
        error_file = application_directory() / "logs" / "startup-error.json"
        try:
            _write_report(error_file, {"error": detail})
        except OSError:
            import tempfile
            error_file = Path(tempfile.gettempdir()) / "mrs6240-startup-error.json"
            _write_report(error_file, {"error": detail})
        ctypes.windll.user32.MessageBoxW(None, "启动失败，请查看：\n" + str(error_file), "MRS6240", 0x10)
        raise SystemExit(1)
