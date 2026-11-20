from pathlib import Path
from unittest.mock import patch

from fall_detection import runtime_paths


def test_source_paths_stay_in_existing_project_layout():
    with patch.object(runtime_paths.sys, "frozen", False, create=True):
        root = Path(runtime_paths.__file__).resolve().parents[1]
        assert runtime_paths.default_log_directory() == root / "logs" / "realtime_fall"
        assert runtime_paths.default_hif_dll_path() == root / "SDK" / "Software_Kit" / "03_Tool" / "HifMsgDataCollectionLib" / "HifMsgDataCollectionLib.dll"


def test_frozen_resources_and_writable_data_follow_different_roots(tmp_path):
    bundle = tmp_path / "复制后的程序" / "_internal"
    executable = tmp_path / "复制后的程序" / "MRS6240Monitor.exe"
    with patch.object(runtime_paths.sys, "frozen", True, create=True), \
         patch.object(runtime_paths.sys, "executable", str(executable)), \
         patch.object(runtime_paths, "__file__", str(bundle / "fall_detection" / "runtime_paths.py")):
        assert runtime_paths.application_directory() == executable.parent
        assert runtime_paths.default_log_directory() == executable.parent / "logs" / "realtime_fall"
        assert runtime_paths.default_hif_dll_path() == bundle / "hardware" / "HifMsgDataCollectionLib.dll"
