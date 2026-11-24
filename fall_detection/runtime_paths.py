"""区分打包后的只读资源路径与程序可写的数据路径。"""
from __future__ import annotations

import sys
from pathlib import Path


def application_directory() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]


def default_log_directory() -> Path:
    return application_directory() / "logs" / "realtime_fall"


def default_hif_dll_path() -> Path:
    if getattr(sys, "frozen", False):
        # PyInstaller 的文件夹模式将资源放在此 Python 包所在的目录旁。
        return Path(__file__).resolve().parents[1] / "hardware" / "HifMsgDataCollectionLib.dll"
    return (
        application_directory() / "SDK" / "Software_Kit" / "03_Tool"
        / "HifMsgDataCollectionLib" / "HifMsgDataCollectionLib.dll"
    )
