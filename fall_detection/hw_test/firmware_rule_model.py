from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import os
import subprocess
import sys
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence


HW_TEST_DIR = Path(__file__).resolve().parent
FALL_DETECTION_DIR = HW_TEST_DIR.parent
MRS6240_ROOT = FALL_DETECTION_DIR.parent
FIRMWARE_DIR = (
    MRS6240_ROOT
    / "SDK"
    / "Software_Kit"
    / "02_SDK"
    / "psdf_sdk"
    / "project"
    / "mmwave"
    / "mmwave_application"
    / "Falldown"
)
FIRMWARE_SRC_DIR = FIRMWARE_DIR / "src"
RADAR_FRAMEWORK_INC_DIR = (
    MRS6240_ROOT
    / "SDK"
    / "Software_Kit"
    / "02_SDK"
    / "psdf_sdk"
    / "project"
    / "mmwave"
    / "common"
    / "radar_framework"
    / "inc"
)
BUILD_DIR = HW_TEST_DIR / "build"
RESULTS_DIR = HW_TEST_DIR / "results"
DLL_PATH = BUILD_DIR / "firmware_rule_harness.dll"
DEFAULT_MANIFEST = FALL_DETECTION_DIR / "pc_ml" / "configs" / "163cm_30deg_v2.json"
DEFAULT_EVENT_POST_DURATION_MS = 3_000
MAX_EVENT_BATCH = 2

if str(MRS6240_ROOT) not in sys.path:
    sys.path.insert(0, str(MRS6240_ROOT))

from fall_detection.datadealer_loader import (  # noqa: E402
    CombinedFrame,
    load_datadealer_recording,
)
from fall_detection.pc_ml.config import load_manifest  # noqa: E402
from fall_detection.pc_ml.dataset import load_labeled_marks  # noqa: E402


EVENT_STATE_NAMES = {
    0: "none",
    1: "suspected",
    2: "confirmed",
    3: "recovered",
}
EVENT_REASON_NAMES = {
    0: "none",
    1: "terminal_persisted",
    2: "low_still",
    3: "low_active",
    4: "suspected_expired",
    5: "stood_up",
}
TRACK_STATE_NAMES = {
    0: "lost",
    1: "tracking",
    2: "static_lock",
    3: "hold",
    4: "reacquire",
}


class NativeInputPoint(ctypes.Structure):
    _fields_ = (
        ("x_m", ctypes.c_float),
        ("y_m", ctypes.c_float),
        ("z_m", ctypes.c_float),
        ("velocity_m_s", ctypes.c_float),
        ("snr_db", ctypes.c_float),
        ("is_motion", ctypes.c_int32),
    )


class NativeFrameResult(ctypes.Structure):
    _fields_ = (
        ("has_target", ctypes.c_int32),
        ("track_state", ctypes.c_int32),
        ("center_x_cm", ctypes.c_int32),
        ("center_y_cm", ctypes.c_int32),
        ("center_z_cm", ctypes.c_int32),
        ("height_cm", ctypes.c_int32),
        ("point_count", ctypes.c_int32),
        ("motion_count", ctypes.c_int32),
        ("max_abs_velocity_cm_s", ctypes.c_int32),
        ("frame_index", ctypes.c_uint32),
        ("timestamp_ms", ctypes.c_uint32),
        ("accepted_point_count", ctypes.c_uint32),
        ("overflow_count", ctypes.c_uint32),
        ("cluster_count", ctypes.c_uint32),
        ("event_count", ctypes.c_uint32),
        ("event_state", ctypes.c_int32 * MAX_EVENT_BATCH),
        ("event_reason", ctypes.c_int32 * MAX_EVENT_BATCH),
        ("event_drop_cm", ctypes.c_int32 * MAX_EVENT_BATCH),
        ("event_center_z_cm", ctypes.c_int32 * MAX_EVENT_BATCH),
        ("event_frame_index", ctypes.c_uint32 * MAX_EVENT_BATCH),
        ("event_timestamp_ms", ctypes.c_uint32 * MAX_EVENT_BATCH),
    )


class NativeConfigSnapshot(ctypes.Structure):
    _fields_ = tuple(
        (name, ctypes.c_int32)
        for name in (
            "frame_period_ms",
            "mount_height_cm",
            "mount_pitch_deg",
            "roi_x_min_cm",
            "roi_x_max_cm",
            "roi_y_min_cm",
            "roi_y_max_cm",
            "roi_z_min_cm",
            "roi_z_max_cm",
            "min_snr_db",
            "cluster_radius_cm",
            "min_cluster_points",
            "max_cluster_width_cm",
            "max_cluster_depth_cm",
            "track_gate_cm",
            "hold_ms",
            "switch_confirm_frames",
            "drop_window_ms",
            "confirm_window_ms",
            "min_drop_cm",
            "motion_velocity_threshold_cm_s",
            "motion_point_threshold",
            "candidate_low_height_cm",
            "terminal_height_cm",
            "suspect_low_duration_ms",
            "suspect_min_fresh_samples",
            "suspect_low_ratio_percent",
            "confirm_low_duration_ms",
            "confirm_min_fresh_samples",
            "confirm_low_ratio_percent",
            "confirm_min_horizontal_displacement_cm",
            "fresh_min_point_count",
            "still_velocity_cm_s",
            "still_motion_points_max",
            "rearm_height_cm",
            "rearm_duration_ms",
        )
    )


@dataclass(frozen=True)
class RadarFrameReference:
    source_frame_index: int
    relative_ms: int
    motion_count: int
    presence_count: int


class NativeFirmwareRule:
    def __init__(self, dll_path: Path) -> None:
        self.dll = ctypes.WinDLL(str(dll_path))
        self.dll.hw_fall_reset.argtypes = ()
        self.dll.hw_fall_reset.restype = None
        self.dll.hw_fall_get_config.argtypes = (
            ctypes.POINTER(NativeConfigSnapshot),
        )
        self.dll.hw_fall_get_config.restype = None
        self.dll.hw_fall_process_frame.argtypes = (
            ctypes.c_uint32,
            ctypes.POINTER(NativeInputPoint),
            ctypes.c_uint32,
            ctypes.POINTER(NativeFrameResult),
        )
        self.dll.hw_fall_process_frame.restype = ctypes.c_int32

    def reset(self) -> None:
        self.dll.hw_fall_reset()

    def config(self) -> dict[str, int]:
        snapshot = NativeConfigSnapshot()
        self.dll.hw_fall_get_config(ctypes.byref(snapshot))
        return {
            name: int(getattr(snapshot, name))
            for name, _ctype in snapshot._fields_
        }

    def process_frame(
        self,
        frame_index: int,
        frame: CombinedFrame,
    ) -> NativeFrameResult:
        points = frame.motion_points + frame.presence_points
        native_array = (NativeInputPoint * len(points))(
            *(
                NativeInputPoint(
                    point.x_m,
                    point.y_m,
                    point.z_m,
                    point.velocity_m_s,
                    point.snr_db,
                    int(point.kind == "motion"),
                )
                for point in points
            )
        )
        output = NativeFrameResult()
        pointer = native_array if points else None
        status = self.dll.hw_fall_process_frame(
            frame_index,
            pointer,
            len(points),
            ctypes.byref(output),
        )
        if status != 0:
            raise RuntimeError(f"native firmware rule returned {status}")
        return output


def score_confirmed_events(
    *,
    confirmed_timestamps_ms: Iterable[int],
    marks: Iterable[object],
    event_post_duration_ms: int,
) -> dict[str, object]:
    if event_post_duration_ms < 0:
        raise ValueError("event_post_duration_ms must be non-negative")
    confirmations = tuple(int(value) for value in confirmed_timestamps_ms)
    normalized_marks = tuple(_normalize_mark(mark) for mark in marks)
    positive_marks = tuple(mark for mark in normalized_marks if mark["label"] == 1)
    negative_marks = tuple(mark for mark in normalized_marks if mark["label"] == 0)

    detected_positive_events = sum(
        any(
            mark["timestamp_ms"]
            <= timestamp_ms
            <= mark["timestamp_ms"] + event_post_duration_ms
            for timestamp_ms in confirmations
        )
        for mark in positive_marks
    )
    false_by_label: Counter[str] = Counter()
    false_confirmed_count = 0
    for timestamp_ms in confirmations:
        if any(
            mark["timestamp_ms"]
            <= timestamp_ms
            <= mark["timestamp_ms"] + event_post_duration_ms
            for mark in positive_marks
        ):
            continue
        false_confirmed_count += 1
        matched_negative = next(
            (
                mark
                for mark in negative_marks
                if mark["timestamp_ms"]
                <= timestamp_ms
                <= mark["timestamp_ms"] + event_post_duration_ms
            ),
            None,
        )
        false_by_label[
            matched_negative["name"] if matched_negative else "background"
        ] += 1

    positive_event_count = len(positive_marks)
    event_recall = (
        detected_positive_events / positive_event_count
        if positive_event_count
        else 0.0
    )
    precision_denominator = detected_positive_events + false_confirmed_count
    event_precision = (
        detected_positive_events / precision_denominator
        if precision_denominator
        else 0.0
    )
    return {
        "confirmed_count": len(confirmations),
        "positive_event_count": positive_event_count,
        "detected_positive_events": detected_positive_events,
        "false_confirmed_count": false_confirmed_count,
        "event_recall": event_recall,
        "event_precision": event_precision,
        "false_by_label": dict(sorted(false_by_label.items())),
    }


def build_native_harness() -> Path:
    if os.name != "nt":
        raise RuntimeError("the firmware host harness currently requires Windows/MSVC")
    vcvars = _find_vcvars64()
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    sources = (
        HW_TEST_DIR / "firmware_rule_harness.c",
        FIRMWARE_SRC_DIR / "fall_pipeline.c",
        FIRMWARE_SRC_DIR / "fall_rule.c",
    )
    compile_command = " ".join(
        (
            "cl /nologo /std:c11 /utf-8 /O2 /LD /W4",
            "/D CONFIG_BOARD_MRS6240_P2512_CPUF=1",
            f'/I"{FIRMWARE_SRC_DIR}"',
            f'/I"{RADAR_FRAMEWORK_INC_DIR}"',
            f'/Fe:"{DLL_PATH}"',
            *(f'"{source}"' for source in sources),
        )
    )
    build_script = BUILD_DIR / "build_firmware_rule_harness.cmd"
    build_script.write_text(
        "@echo off\n"
        f'call "{vcvars}" >nul\n'
        "if errorlevel 1 exit /b %errorlevel%\n"
        f"{compile_command}\n",
        encoding="ascii",
    )
    result = subprocess.run(
        ["cmd.exe", "/d", "/c", str(build_script)],
        cwd=BUILD_DIR,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0 or not DLL_PATH.is_file():
        raise RuntimeError(
            "native firmware harness build failed\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return DLL_PATH


def run_session(
    native: NativeFirmwareRule,
    *,
    session_name: str,
    role: str,
    data_root: Path,
    event_post_duration_ms: int,
) -> dict[str, object]:
    session_dir = data_root / session_name
    recording = load_datadealer_recording(session_dir, stem=session_name)
    radar_frames = _load_radar_frame_references(session_dir / "radar_events.jsonl")
    references_by_relative_ms = {
        reference.relative_ms: reference for reference in radar_frames
    }
    missing_timestamps = sorted(
        frame.timestamp_ms
        for frame in recording.frames
        if frame.timestamp_ms not in references_by_relative_ms
    )
    if missing_timestamps:
        raise ValueError(
            f"{session_name}: {len(missing_timestamps)} point-cloud frames have no "
            "matching radar_events.jsonl record"
        )

    native.reset()
    confirmed_timestamps: list[int] = []
    firmware_events: list[dict[str, object]] = []
    track_state_counts: Counter[str] = Counter()
    input_point_count = 0
    accepted_point_count = 0
    target_frame_count = 0
    final_overflow_count = 0

    for frame in recording.frames:
        reference = references_by_relative_ms[frame.timestamp_ms]
        if reference.motion_count != frame.motion_count or (
            reference.presence_count != frame.presence_count
        ):
            raise ValueError(
                f"{session_name}: point count mismatch at {frame.timestamp_ms} ms"
            )
        output = native.process_frame(reference.source_frame_index, frame)
        input_point_count += frame.motion_count + frame.presence_count
        accepted_point_count += int(output.accepted_point_count)
        final_overflow_count = int(output.overflow_count)
        track_state_counts[TRACK_STATE_NAMES.get(output.track_state, "unknown")] += 1
        target_frame_count += int(output.has_target != 0)
        for event_index in range(int(output.event_count)):
            state = int(output.event_state[event_index])
            event_record = {
                "state": EVENT_STATE_NAMES.get(state, f"unknown_{state}"),
                "reason": EVENT_REASON_NAMES.get(
                    int(output.event_reason[event_index]),
                    f"unknown_{int(output.event_reason[event_index])}",
                ),
                "recording_timestamp_ms": frame.timestamp_ms,
                "source_frame_index": int(output.event_frame_index[event_index]),
                "firmware_timestamp_ms": int(output.event_timestamp_ms[event_index]),
                "drop_cm": int(output.event_drop_cm[event_index]),
                "center_z_cm": int(output.event_center_z_cm[event_index]),
            }
            firmware_events.append(event_record)
            if state == 2:
                confirmed_timestamps.append(frame.timestamp_ms)

    marks = load_labeled_marks(session_dir / "operator_marks.csv")
    score = score_confirmed_events(
        confirmed_timestamps_ms=confirmed_timestamps,
        marks=marks,
        event_post_duration_ms=event_post_duration_ms,
    )
    return {
        "session_name": session_name,
        "role": role,
        "duration_s": recording.duration_s,
        "frame_count": len(recording.frames),
        "radar_event_count": len(radar_frames),
        "input_point_count": input_point_count,
        "accepted_point_count": accepted_point_count,
        "point_overflow_count": final_overflow_count,
        "target_frame_count": target_frame_count,
        "target_frame_ratio": (
            target_frame_count / len(recording.frames)
            if recording.frames
            else 0.0
        ),
        "track_state_counts": dict(sorted(track_state_counts.items())),
        "event_score": score,
        "firmware_events": firmware_events,
    }


def run_evaluation(
    *,
    manifest_path: Path,
    output_dir: Path,
    selected_sessions: Sequence[str] = (),
) -> dict[str, object]:
    manifest = load_manifest(manifest_path)
    session_specs = tuple(
        session
        for session in manifest.sessions
        if session.name.startswith("v2-")
        and (not selected_sessions or session.name in selected_sessions)
    )
    if not session_specs:
        raise ValueError("no V2 train/validation sessions selected")

    dll_path = build_native_harness()
    native = NativeFirmwareRule(dll_path)
    config = native.config()
    session_results = []
    all_confirmations: list[int] = []
    all_marks: list[dict[str, object]] = []

    for session_index, session in enumerate(session_specs):
        _validate_mount_metadata(
            manifest.data_root / session.name / "session.json",
            config,
        )
        result = run_session(
            native,
            session_name=session.name,
            role=session.role,
            data_root=manifest.data_root,
            event_post_duration_ms=manifest.window.event_post_duration_ms,
        )
        session_results.append(result)
        offset_ms = session_index * 1_000_000_000
        all_confirmations.extend(
            offset_ms + int(event["recording_timestamp_ms"])
            for event in result["firmware_events"]
            if event["state"] == "confirmed"
        )
        all_marks.extend(
            {
                "name": mark.name,
                "label": mark.label,
                "timestamp_ms": offset_ms + mark.timestamp_ms,
            }
            for mark in load_labeled_marks(
                manifest.data_root / session.name / "operator_marks.csv"
            )
        )

    aggregate = score_confirmed_events(
        confirmed_timestamps_ms=all_confirmations,
        marks=all_marks,
        event_post_duration_ms=manifest.window.event_post_duration_ms,
    )
    role_scores = {
        role: _aggregate_session_results(
            tuple(result for result in session_results if result["role"] == role)
        )
        for role in sorted({str(result["role"]) for result in session_results})
    }
    report = {
        "schema_version": 1,
        "model_family": "firmware_rule_alg3",
        "manifest": str(manifest_path.resolve()),
        "event_post_duration_ms": manifest.window.event_post_duration_ms,
        "native_dll": str(dll_path.resolve()),
        "firmware_sources": _firmware_source_hashes(),
        "firmware_config": config,
        "sessions": session_results,
        "aggregate": aggregate,
        "by_role": role_scores,
        "limitations": [
            "Replays captured Cartesian point-cloud reports, not ADC/datacube input.",
            "Recorded point coordinates and velocity/SNR values are quantized report values.",
            "All current V2 sessions are P01 in room-a; this is not a blind test.",
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "firmware_rule_v2_report.json"
    csv_path = output_dir / "firmware_rule_v2_sessions.csv"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_session_csv(session_results, csv_path)
    print(
        json.dumps(
            {
                "report": str(report_path.resolve()),
                "sessions_csv": str(csv_path.resolve()),
                "aggregate": aggregate,
                "by_role": role_scores,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return report


def _aggregate_session_results(
    sessions: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    positive_event_count = sum(
        int(session["event_score"]["positive_event_count"])
        for session in sessions
    )
    detected_positive_events = sum(
        int(session["event_score"]["detected_positive_events"])
        for session in sessions
    )
    confirmed_count = sum(
        int(session["event_score"]["confirmed_count"])
        for session in sessions
    )
    false_confirmed_count = sum(
        int(session["event_score"]["false_confirmed_count"])
        for session in sessions
    )
    false_by_label: Counter[str] = Counter()
    for session in sessions:
        false_by_label.update(session["event_score"]["false_by_label"])
    precision_denominator = detected_positive_events + false_confirmed_count
    return {
        "session_count": len(sessions),
        "confirmed_count": confirmed_count,
        "positive_event_count": positive_event_count,
        "detected_positive_events": detected_positive_events,
        "false_confirmed_count": false_confirmed_count,
        "event_recall": (
            detected_positive_events / positive_event_count
            if positive_event_count
            else 0.0
        ),
        "event_precision": (
            detected_positive_events / precision_denominator
            if precision_denominator
            else 0.0
        ),
        "false_by_label": dict(sorted(false_by_label.items())),
    }


def _normalize_mark(mark: object) -> dict[str, object]:
    if isinstance(mark, Mapping):
        return {
            "name": str(mark["name"]),
            "label": int(mark["label"]),
            "timestamp_ms": int(mark["timestamp_ms"]),
        }
    return {
        "name": str(getattr(mark, "name")),
        "label": int(getattr(mark, "label")),
        "timestamp_ms": int(getattr(mark, "timestamp_ms")),
    }


def _load_radar_frame_references(path: Path) -> tuple[RadarFrameReference, ...]:
    references = []
    seen_relative_ms: set[int] = set()
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            raw = json.loads(line)
            reference = RadarFrameReference(
                source_frame_index=int(raw["frame_index"]),
                relative_ms=int(raw["relative_ms"]),
                motion_count=int(raw["motion_count"]),
                presence_count=int(raw["presence_count"]),
            )
            if reference.relative_ms in seen_relative_ms:
                raise ValueError(
                    f"duplicate radar relative_ms at {path}:{line_number}"
                )
            seen_relative_ms.add(reference.relative_ms)
            references.append(reference)
    if not references:
        raise ValueError(f"no radar events in {path}")
    return tuple(references)


def _validate_mount_metadata(
    session_path: Path,
    config: Mapping[str, int],
) -> None:
    raw = json.loads(session_path.read_text(encoding="utf-8"))
    metadata = raw.get("metadata", {})
    mount_height_cm = round(float(metadata["mount_height_m"]) * 100.0)
    pitch_deg = round(float(metadata["pitch_deg"]))
    if mount_height_cm != config["mount_height_cm"] or (
        pitch_deg != config["mount_pitch_deg"]
    ):
        raise ValueError(
            f"{session_path.parent.name}: capture mount {mount_height_cm} cm / "
            f"{pitch_deg} deg does not match firmware "
            f"{config['mount_height_cm']} cm / {config['mount_pitch_deg']} deg"
        )


def _find_vcvars64() -> Path:
    program_files_x86 = Path(
        os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
    )
    candidates = sorted(
        (
            program_files_x86
            / "Microsoft Visual Studio"
        ).glob("*/*/VC/Auxiliary/Build/vcvars64.bat"),
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError("Visual Studio vcvars64.bat was not found")
    return candidates[0]


def _firmware_source_hashes() -> dict[str, str]:
    files = (
        FIRMWARE_SRC_DIR / "fall_pipeline.c",
        FIRMWARE_SRC_DIR / "fall_pipeline.h",
        FIRMWARE_SRC_DIR / "fall_rule.c",
        FIRMWARE_SRC_DIR / "fall_rule.h",
        FIRMWARE_SRC_DIR / "prj_config.h",
        HW_TEST_DIR / "firmware_rule_harness.c",
    )
    return {
        str(path.relative_to(MRS6240_ROOT)).replace("\\", "/"): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in files
    }


def _write_session_csv(
    sessions: Sequence[Mapping[str, object]],
    path: Path,
) -> None:
    columns = (
        "session_name",
        "role",
        "frame_count",
        "target_frame_ratio",
        "positive_event_count",
        "detected_positive_events",
        "confirmed_count",
        "false_confirmed_count",
        "event_recall",
        "event_precision",
        "false_by_label",
    )
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for session in sessions:
            score = session["event_score"]
            writer.writerow(
                {
                    "session_name": session["session_name"],
                    "role": session["role"],
                    "frame_count": session["frame_count"],
                    "target_frame_ratio": f"{float(session['target_frame_ratio']):.9f}",
                    "positive_event_count": score["positive_event_count"],
                    "detected_positive_events": score["detected_positive_events"],
                    "confirmed_count": score["confirmed_count"],
                    "false_confirmed_count": score["false_confirmed_count"],
                    "event_recall": f"{float(score['event_recall']):.9f}",
                    "event_precision": f"{float(score['event_precision']):.9f}",
                    "false_by_label": json.dumps(
                        score["false_by_label"],
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                }
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay captured V2 point clouds through the exact firmware C rules",
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=RESULTS_DIR)
    parser.add_argument(
        "--session",
        action="append",
        default=[],
        help="limit replay to one or more V2 session names",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_evaluation(
        manifest_path=args.manifest,
        output_dir=args.output_dir,
        selected_sessions=tuple(args.session),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
