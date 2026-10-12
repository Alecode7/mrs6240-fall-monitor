from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class OperatorMark:
    mark: str
    radar_frame_index: int
    radar_timestamp_ms: int | None = None
    monotonic_ns: int | None = None
    wall_time: str = ""
    note: str = ""
    capture_wall_time: str = ""
    source_radar_frame_index: int | None = None
    source_timestamp_ms: int | None = None
    source_timestamp_text: str = ""
    sync_frame_index: int | None = None
    camera_frame_index: int | None = None
    camera_relative_ms: int | None = None
    camera_video_time_ms: int | None = None
    camera_wall_time: str = ""
    camera_offset_ms: int | None = None
    camera_time_delta_ms: int | None = None


def load_operator_marks(path: str | Path) -> tuple[OperatorMark, ...]:
    csv_path = Path(path)
    if not csv_path.exists():
        return ()

    marks_by_identity: dict[tuple[int, str], OperatorMark] = {}
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            mark = str(row.get("mark", "") or "").strip()
            frame_index = _optional_int(row.get("radar_frame_index"))
            if not mark or frame_index is None:
                continue
            identity = (frame_index, mark)
            marks_by_identity.setdefault(
                identity,
                OperatorMark(
                    mark=mark,
                    radar_frame_index=frame_index,
                    radar_timestamp_ms=_optional_int(row.get("radar_timestamp_ms")),
                    monotonic_ns=_optional_int(row.get("monotonic_ns")),
                    wall_time=str(row.get("wall_time", "") or "").strip(),
                    note=str(row.get("note", "") or "").strip(),
                    capture_wall_time=str(row.get("capture_wall_time", "") or "").strip(),
                    source_radar_frame_index=_optional_int(row.get("source_radar_frame_index")),
                    source_timestamp_ms=_optional_int(row.get("source_timestamp_ms")),
                    source_timestamp_text=str(row.get("source_timestamp_text", "") or "").strip(),
                    sync_frame_index=_optional_int(row.get("sync_frame_index")),
                    camera_frame_index=_optional_int(row.get("camera_frame_index")),
                    camera_relative_ms=_optional_int(row.get("camera_relative_ms")),
                    camera_video_time_ms=_optional_int(row.get("camera_video_time_ms")),
                    camera_wall_time=str(row.get("camera_wall_time", "") or "").strip(),
                    camera_offset_ms=_optional_int(row.get("camera_offset_ms")),
                    camera_time_delta_ms=_optional_int(row.get("camera_time_delta_ms")),
                ),
            )

    return tuple(
        sorted(
            marks_by_identity.values(),
            key=lambda item: (item.radar_frame_index, item.mark),
        )
    )


def _optional_int(value: object) -> int | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return int(float(text))
    except ValueError:
        return None
