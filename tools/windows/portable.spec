# Local build only: vendor DLLs and portable output stay excluded from Git.
import os
from pathlib import Path
from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs

root = Path(SPECPATH).resolve().parents[1]
hif = Path(os.environ.get("MRS6240_HIF_DLL", str(root / "SDK/Software_Kit/03_Tool/HifMsgDataCollectionLib/HifMsgDataCollectionLib.dll")))
native = Path(os.environ.get("MRS6240_CH347_DLL", str(Path(os.environ["WINDIR"]) / "System32/CH347DLLA64.dll")))
for dependency in (hif, native):
    if not dependency.is_file():
        raise FileNotFoundError(f"Required hardware dependency is missing: {dependency}")

a = Analysis(
    [str(root / "tools/windows/portable_launcher.py")],
    pathex=[str(root)],
    binaries=[(str(native), "hardware"), *collect_dynamic_libs("clr_loader")],
    datas=[(str(hif), "hardware"), *collect_data_files("pythonnet"), *collect_data_files("clr_loader")],
    hiddenimports=["clr", "pythonnet", "clr_loader", "serial.tools.list_ports", "winsound"],
    excludes=["pytest", "scipy", "pandas", "cv2", "IPython"],
    hooksconfig={"matplotlib": {"backends": ["TkAgg"]}},
    noarchive=False,
)
pyz = PYZ(a.pure)
monitor = EXE(pyz, a.scripts, [], exclude_binaries=True, name="MRS6240Monitor",
              debug=False, strip=False, upx=False, console=False)
engineering = EXE(pyz, a.scripts, [], exclude_binaries=True, name="MRS6240Engineering",
                  debug=False, strip=False, upx=False, console=False)
coll = COLLECT(monitor, engineering, a.binaries, a.datas,
               strip=False, upx=False, name="MRS6240-Fall-Monitor")
