"""Separate bundled read-only resources from user-writable portable data."""
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
        # PyInstaller's onedir bundle puts resources beside this package.
        return Path(__file__).resolve().parents[1] / "hardware" / "HifMsgDataCollectionLib.dll"
    return (
        application_directory() / "SDK" / "Software_Kit" / "03_Tool"
        / "HifMsgDataCollectionLib" / "HifMsgDataCollectionLib.dll"
    )
