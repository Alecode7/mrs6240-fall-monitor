from __future__ import annotations

import csv
import json
import re
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from pathlib import Path
from typing import TextIO

from fall_detection.datadealer_loader import CombinedFrame, Point3D


POINT_CLOUD_SUFFIXES = {
    "motion": "_motion_point_cloud.csv",
    "presence": "_presence_point_cloud.csv",
}
SYNC_FRAME_PERIOD_MS = 50
MARK_COLUMNS = (
    "mark",
    "monotonic_ns",
    "wall_time",
    "note",
    "radar_frame_index",
    "radar_timestamp_ms",
    "capture_wall_time",
    "source_radar_frame_index",
    "source_timestamp_ms",
    "source_timestamp_text",
    "sync_frame_index",
    "camera_frame_index",
    "camera_relative_ms",
    "camera_video_time_ms",
    "camera_wall_time",
    "camera_offset_ms",
    "camera_time_delta_ms",
)


def wall_time_now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def default_session_name() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def sanitize_session_name(name: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z._-]+", "_", name.strip())
    return cleaned.strip("._-") or default_session_name()


@dataclass(frozen=True)
class AnnotationEventReference:
    capture_wall_time: str = ""
    source_radar_frame_index: int | None = None
    source_timestamp_ms: int | None = None
    source_timestamp_text: str = ""
    sync_frame_index: int | None = None
    camera_frame_index: int | None = None
    camera_relative_ms: int | None = None
    camera_video_time_ms: int | None = None
    camera_wall_time: str = ""
    camera_offset_ms: int = 0
    camera_time_delta_ms: int | None = None


@dataclass(frozen=True)
class SessionTimeline:
    session_dir: Path
    started_wall_time: str
    radar_events: tuple[dict[str, object], ...]
    camera_rows: tuple[dict[str, str], ...]

    def resolve(
        self,
        *,
        replay_frame_index: int,
        capture_relative_ms: int,
        camera_offset_ms: int = 0,
    ) -> AnnotationEventReference:
        radar_event = _matching_radar_event(
            self.radar_events,
            replay_frame_index=replay_frame_index,
            capture_relative_ms=capture_relative_ms,
        )
        camera_row = _nearest_camera_row(
            self.camera_rows,
            target_ms=capture_relative_ms + camera_offset_ms,
        )
        capture_wall_time = _text_value(radar_event, "wall_time")
        if not capture_wall_time:
            capture_wall_time = _capture_wall_time(
                self.started_wall_time,
                capture_relative_ms,
            )
        camera_relative_ms = _optional_int(camera_row.get("relative_ms")) if camera_row else None
        return AnnotationEventReference(
            capture_wall_time=capture_wall_time,
            source_radar_frame_index=_mapping_optional_int(radar_event, "frame_index"),
            source_timestamp_ms=_mapping_optional_int(radar_event, "source_timestamp_ms"),
            source_timestamp_text=_text_value(radar_event, "source_timestamp_text"),
            sync_frame_index=_mapping_optional_int(radar_event, "sync_frame_index"),
            camera_frame_index=_optional_int(camera_row.get("camera_frame_index")) if camera_row else None,
            camera_relative_ms=camera_relative_ms,
            camera_video_time_ms=_optional_int(camera_row.get("video_time_ms")) if camera_row else None,
            camera_wall_time=str(camera_row.get("wall_time", "") or "").strip() if camera_row else "",
            camera_offset_ms=int(camera_offset_ms),
            camera_time_delta_ms=(
                camera_relative_ms - int(capture_relative_ms)
                if camera_relative_ms is not None
                else None
            ),
        )


def load_session_timeline(session_dir: str | Path) -> SessionTimeline:
    directory = Path(session_dir)
    metadata_path = directory / "session.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    radar_events = []
    radar_path = directory / "radar_events.jsonl"
    if radar_path.exists():
        with radar_path.open("r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    radar_events.append(json.loads(line))
    camera_rows = []
    camera_path = directory / "camera_frames.csv"
    if camera_path.exists():
        with camera_path.open("r", encoding="utf-8-sig", newline="") as stream:
            camera_rows.extend(csv.DictReader(stream))
    return SessionTimeline(
        session_dir=directory,
        started_wall_time=str(metadata.get("started_wall_time", "") or "").strip(),
        radar_events=tuple(radar_events),
        camera_rows=tuple(camera_rows),
    )


@dataclass
class SessionRecorder:
    data_root: Path
    session_name: str
    session_dir: Path
    started_monotonic_ns: int
    started_wall_time: str
    metadata: dict = field(default_factory=dict)
    _motion_file: TextIO | None = field(default=None, init=False, repr=False)
    _presence_file: TextIO | None = field(default=None, init=False, repr=False)
    _motion_writer: csv.writer | None = field(default=None, init=False, repr=False)
    _presence_writer: csv.writer | None = field(default=None, init=False, repr=False)
    _camera_file: TextIO | None = field(default=None, init=False, repr=False)
    _camera_writer: csv.DictWriter | None = field(default=None, init=False, repr=False)
    _radar_events_file: TextIO | None = field(default=None, init=False, repr=False)

    @classmethod
    def create(cls, data_root: str | Path, *, session_name: str | None = None, metadata: dict | None = None) -> "SessionRecorder":
        root = Path(data_root)
        root.mkdir(parents=True, exist_ok=True)
        base_name = sanitize_session_name(session_name or default_session_name())
        session_dir = _unique_session_dir(root, base_name)
        session_dir.mkdir(parents=True, exist_ok=False)
        started_monotonic_ns = time.monotonic_ns()
        started_wall_time = wall_time_now()
        recorder = cls(
            data_root=root,
            session_name=session_dir.name,
            session_dir=session_dir,
            started_monotonic_ns=started_monotonic_ns,
            started_wall_time=started_wall_time,
            metadata=dict(metadata or {}),
        )
        recorder._write_session_metadata()
        return recorder

    @property
    def motion_path(self) -> Path:
        return self.session_dir / f"{self.session_name}{POINT_CLOUD_SUFFIXES['motion']}"

    @property
    def presence_path(self) -> Path:
        return self.session_dir / f"{self.session_name}{POINT_CLOUD_SUFFIXES['presence']}"

    @property
    def camera_video_path(self) -> Path:
        return self.session_dir / "camera.mp4"

    def relative_ms(self, monotonic_ns: int | None = None) -> int:
        now_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        return max(int((now_ns - self.started_monotonic_ns) / 1_000_000), 0)

    def write_radar_frame(self, frame: CombinedFrame, *, monotonic_ns: int | None = None, wall_time: str | None = None) -> None:
        event_monotonic_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        event_wall_time = wall_time or wall_time_now()
        relative_ms = self.relative_ms(event_monotonic_ns)
        self._ensure_pointcloud_writers()
        assert self._motion_writer is not None
        assert self._presence_writer is not None
        self._motion_writer.writerow(_pointcloud_row(event_wall_time, relative_ms, frame.motion_points))
        self._presence_writer.writerow(_pointcloud_row(event_wall_time, relative_ms, frame.presence_points))
        self._write_radar_event(frame, event_monotonic_ns=event_monotonic_ns, event_wall_time=event_wall_time, relative_ms=relative_ms)

    def write_camera_frame(self, *, frame_index: int, monotonic_ns: int | None = None, wall_time: str | None = None, video_time_ms: int | None = None) -> None:
        event_monotonic_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        event_wall_time = wall_time or wall_time_now()
        relative_ms = self.relative_ms(event_monotonic_ns)
        self._ensure_camera_writer()
        assert self._camera_writer is not None
        self._camera_writer.writerow(
            {
                "camera_frame_index": int(frame_index),
                "sync_frame_index": sync_frame_index(relative_ms),
                "monotonic_ns": int(event_monotonic_ns),
                "wall_time": event_wall_time,
                "relative_ms": int(relative_ms),
                "video_time_ms": int(relative_ms if video_time_ms is None else video_time_ms),
            }
        )

    def write_operator_mark(
        self,
        mark: str,
        *,
        monotonic_ns: int | None = None,
        wall_time: str | None = None,
        note: str = "",
        radar_frame_index: int | None = None,
        radar_timestamp_ms: int | None = None,
        event_reference: AnnotationEventReference | None = None,
    ) -> None:
        append_operator_mark(
            self.session_dir,
            mark,
            monotonic_ns=time.monotonic_ns() if monotonic_ns is None else monotonic_ns,
            wall_time=wall_time or wall_time_now(),
            note=note,
            radar_frame_index=radar_frame_index,
            radar_timestamp_ms=radar_timestamp_ms,
            event_reference=event_reference,
        )

    def close(self) -> None:
        for handle in (self._motion_file, self._presence_file, self._camera_file, self._radar_events_file):
            if handle is not None:
                handle.close()
        self._motion_file = None
        self._presence_file = None
        self._camera_file = None
        self._radar_events_file = None
        self._motion_writer = None
        self._presence_writer = None
        self._camera_writer = None

    def _write_session_metadata(self) -> None:
        payload = {
            "session_name": self.session_name,
            "started_wall_time": self.started_wall_time,
            "started_monotonic_ns": self.started_monotonic_ns,
            "metadata": self.metadata,
            "files": {
                "motion_point_cloud": self.motion_path.name,
                "presence_point_cloud": self.presence_path.name,
                "camera_video": self.camera_video_path.name,
                "camera_frames": "camera_frames.csv",
                "operator_marks": "operator_marks.csv",
                "radar_events": "radar_events.jsonl",
            },
            "sync": {
                "sync_frame_period_ms": SYNC_FRAME_PERIOD_MS,
                "sync_frame_index": "round(relative_ms / sync_frame_period_ms)",
            },
        }
        (self.session_dir / "session.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _ensure_pointcloud_writers(self) -> None:
        if self._motion_writer is None:
            self._motion_file = self.motion_path.open("a", encoding="utf-8", newline="")
            self._motion_writer = csv.writer(self._motion_file)
        if self._presence_writer is None:
            self._presence_file = self.presence_path.open("a", encoding="utf-8", newline="")
            self._presence_writer = csv.writer(self._presence_file)

    def _ensure_camera_writer(self) -> None:
        if self._camera_writer is not None:
            return
        self._camera_file = (self.session_dir / "camera_frames.csv").open("a", encoding="utf-8", newline="")
        self._camera_writer = csv.DictWriter(
            self._camera_file,
            fieldnames=("camera_frame_index", "sync_frame_index", "monotonic_ns", "wall_time", "relative_ms", "video_time_ms"),
        )
        if self._camera_file.tell() == 0:
            self._camera_writer.writeheader()

    def _write_radar_event(self, frame: CombinedFrame, *, event_monotonic_ns: int, event_wall_time: str, relative_ms: int) -> None:
        if self._radar_events_file is None:
            self._radar_events_file = (self.session_dir / "radar_events.jsonl").open("a", encoding="utf-8")
        event = {
            "frame_index": frame.frame_index,
            "sync_frame_index": sync_frame_index(relative_ms),
            "source_timestamp_ms": frame.timestamp_ms,
            "source_timestamp_text": frame.timestamp_text,
            "monotonic_ns": int(event_monotonic_ns),
            "wall_time": event_wall_time,
            "relative_ms": int(relative_ms),
            "motion_count": frame.motion_count,
            "presence_count": frame.presence_count,
        }
        self._radar_events_file.write(json.dumps(event, ensure_ascii=False) + "\n")


def sync_frame_index(relative_ms: int) -> int:
    return max((int(relative_ms) + SYNC_FRAME_PERIOD_MS // 2) // SYNC_FRAME_PERIOD_MS, 0)


def append_operator_mark(
    session_dir: str | Path,
    mark: str,
    *,
    monotonic_ns: int,
    wall_time: str,
    note: str = "",
    radar_frame_index: int | None = None,
    radar_timestamp_ms: int | None = None,
    event_reference: AnnotationEventReference | None = None,
) -> None:
    path = Path(session_dir) / "operator_marks.csv"
    fieldnames = _ensure_mark_columns(path)
    exists = path.exists() and path.stat().st_size > 0
    reference_fields = _event_reference_fields(event_reference)
    with path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        if not exists:
            writer.writeheader()
        row = {
            "mark": mark,
            "monotonic_ns": int(monotonic_ns),
            "wall_time": wall_time,
            "note": note,
            "radar_frame_index": "" if radar_frame_index is None else int(radar_frame_index),
            "radar_timestamp_ms": "" if radar_timestamp_ms is None else int(radar_timestamp_ms),
        }
        row.update(reference_fields)
        writer.writerow(row)


def backfill_operator_mark_references(session_dir: str | Path) -> int:
    directory = Path(session_dir)
    path = directory / "operator_marks.csv"
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        original_fieldnames = tuple(reader.fieldnames or ())
        rows = list(reader)
    timeline = load_session_timeline(directory)
    updated = 0
    for row in rows:
        replay_frame_index = _optional_int(row.get("radar_frame_index"))
        capture_relative_ms = _optional_int(row.get("radar_timestamp_ms"))
        if replay_frame_index is None or capture_relative_ms is None:
            continue
        camera_offset_ms = _optional_int(row.get("camera_offset_ms")) or 0
        reference = timeline.resolve(
            replay_frame_index=replay_frame_index,
            capture_relative_ms=capture_relative_ms,
            camera_offset_ms=camera_offset_ms,
        )
        changed = False
        for key, value in _event_reference_fields(reference).items():
            if str(row.get(key, "") or "").strip() or value in (None, ""):
                continue
            row[key] = str(value)
            changed = True
        if changed:
            updated += 1
    fieldnames = _merged_mark_columns(original_fieldnames)
    _rewrite_mark_rows(path, fieldnames, rows)
    return updated


def _event_reference_fields(reference: AnnotationEventReference | None) -> dict[str, object]:
    reference = reference or AnnotationEventReference()
    return {
        "capture_wall_time": reference.capture_wall_time,
        "source_radar_frame_index": _csv_optional(reference.source_radar_frame_index),
        "source_timestamp_ms": _csv_optional(reference.source_timestamp_ms),
        "source_timestamp_text": reference.source_timestamp_text,
        "sync_frame_index": _csv_optional(reference.sync_frame_index),
        "camera_frame_index": _csv_optional(reference.camera_frame_index),
        "camera_relative_ms": _csv_optional(reference.camera_relative_ms),
        "camera_video_time_ms": _csv_optional(reference.camera_video_time_ms),
        "camera_wall_time": reference.camera_wall_time,
        "camera_offset_ms": int(reference.camera_offset_ms),
        "camera_time_delta_ms": _csv_optional(reference.camera_time_delta_ms),
    }


def _ensure_mark_columns(path: Path) -> tuple[str, ...]:
    if not path.exists() or path.stat().st_size == 0:
        return MARK_COLUMNS
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        original_fieldnames = tuple(reader.fieldnames or ())
        rows = list(reader)
    fieldnames = _merged_mark_columns(original_fieldnames)
    if fieldnames != original_fieldnames:
        _rewrite_mark_rows(path, fieldnames, rows)
    return fieldnames


def _merged_mark_columns(original_fieldnames: tuple[str, ...]) -> tuple[str, ...]:
    extras = tuple(name for name in original_fieldnames if name and name not in MARK_COLUMNS)
    return MARK_COLUMNS + extras


def _rewrite_mark_rows(path: Path, fieldnames: tuple[str, ...], rows: list[dict[str, str]]) -> None:
    temporary_path = path.with_name(path.name + ".tmp")
    with temporary_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary_path.replace(path)


def _matching_radar_event(
    events: tuple[dict[str, object], ...],
    *,
    replay_frame_index: int,
    capture_relative_ms: int,
) -> dict[str, object]:
    if 0 <= replay_frame_index < len(events):
        candidate = events[replay_frame_index]
        if _mapping_optional_int(candidate, "relative_ms") == int(capture_relative_ms):
            return candidate
    if not events:
        return {}
    return min(
        events,
        key=lambda event: abs(
            (_mapping_optional_int(event, "relative_ms") or 0) - int(capture_relative_ms)
        ),
    )


def _nearest_camera_row(
    camera_rows: tuple[dict[str, str], ...],
    *,
    target_ms: int,
) -> dict[str, str] | None:
    if not camera_rows:
        return None
    return min(
        camera_rows,
        key=lambda row: abs((_optional_int(row.get("relative_ms")) or 0) - int(target_ms)),
    )


def _capture_wall_time(started_wall_time: str, capture_relative_ms: int) -> str:
    if not started_wall_time:
        return ""
    try:
        event_time = datetime.fromisoformat(started_wall_time) + timedelta(milliseconds=int(capture_relative_ms))
    except ValueError:
        return ""
    return event_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _mapping_optional_int(mapping: dict[str, object], key: str) -> int | None:
    return _optional_int(mapping.get(key))


def _optional_int(value: object) -> int | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return int(float(text))
    except ValueError:
        return None


def _text_value(mapping: dict[str, object], key: str) -> str:
    return str(mapping.get(key, "") or "").strip()


def _csv_optional(value: int | None) -> int | str:
    return "" if value is None else int(value)


def _pointcloud_row(timestamp_text: str, timestamp_ms: int, points: tuple[Point3D, ...]) -> list[float | int | str]:
    return (
        [timestamp_text, int(timestamp_ms)]
        + [point.x_m for point in points]
        + [point.y_m for point in points]
        + [point.z_m for point in points]
        + [point.velocity_m_s for point in points]
        + [point.snr_db for point in points]
    )


def _unique_session_dir(root: Path, base_name: str) -> Path:
    candidate = root / base_name
    if not candidate.exists():
        return candidate
    for index in range(2, 1000):
        candidate = root / f"{base_name}_{index:03d}"
        if not candidate.exists():
            return candidate
    raise FileExistsError(f"too many duplicate session directories for {base_name}")
