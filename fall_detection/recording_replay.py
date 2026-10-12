from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Queue

from .datadealer_loader import CombinedFrame, load_datadealer_recording
from .live_source import LatestFrameBuffer
from .serial_event import SerialRecord, parse_board_status, parse_fall_event


_LOG_LINE_PATTERN = re.compile(r"^\[(?P<timestamp>[^\]]+)\]\s*(?P<body>.*)$")
_TIMESTAMP_FORMATS = (
    "%Y-%m-%d %H:%M:%S.%f",
    "%Y-%m-%d %H:%M:%S",
    "%Y-%m-%d-%H:%M:%S.%f",
    "%Y-%m-%d-%H:%M:%S",
)


@dataclass(frozen=True)
class TimedSerialRecord:
    relative_ms: int
    record: SerialRecord


@dataclass(frozen=True)
class RecordedSession:
    session_dir: Path
    stem: str
    frames: tuple[CombinedFrame, ...]
    serial_records: tuple[TimedSerialRecord, ...]
    duration_ms: int
    serial_log_available: bool


@dataclass(frozen=True)
class ReplayItem:
    relative_ms: int
    sequence: int
    frame: CombinedFrame | None = None
    serial_record: SerialRecord | None = None


def load_recorded_session(session_dir: str | Path) -> RecordedSession:
    """Load one capture-station session without opening hardware."""

    directory = Path(session_dir).expanduser()
    if not directory.is_dir():
        raise FileNotFoundError(f"recorded session directory not found: {directory}")

    metadata = _read_session_metadata(directory)
    stem = str(metadata.get("session_name") or directory.name)
    try:
        recording = load_datadealer_recording(directory, stem=stem)
    except FileNotFoundError:
        if stem == directory.name:
            raise
        stem = directory.name
        recording = load_datadealer_recording(directory, stem=stem)

    serial_path = directory / "serial_control.log"
    serial_records = (
        parse_serial_log(
            serial_path,
            started_wall_time=_metadata_datetime(metadata.get("started_wall_time")),
        )
        if serial_path.exists()
        else ()
    )
    duration_ms = max(
        [0]
        + [int(frame.timestamp_ms) for frame in recording.frames]
        + [int(item.relative_ms) for item in serial_records]
    )
    return RecordedSession(
        session_dir=directory,
        stem=stem,
        frames=recording.frames,
        serial_records=serial_records,
        duration_ms=duration_ms,
        serial_log_available=serial_path.exists(),
    )


def parse_serial_log(
    path: str | Path,
    *,
    started_wall_time: datetime | None = None,
) -> tuple[TimedSerialRecord, ...]:
    """Parse and time-align a capture-station ``serial_control.log``."""

    log_path = Path(path)
    rows: list[tuple[datetime, SerialRecord]] = []
    with log_path.open("r", encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            parsed = _parse_log_row(raw_line)
            if parsed is None:
                continue
            rows.append(parsed)

    if not rows:
        return ()
    rows = _limit_to_recording_interval(rows)
    if not rows:
        return ()
    base_time = started_wall_time or rows[0][0]
    return tuple(
        TimedSerialRecord(
            relative_ms=max(int((timestamp - base_time).total_seconds() * 1000), 0),
            record=record,
        )
        for timestamp, record in rows
    )


def replay_items(session: RecordedSession) -> tuple[ReplayItem, ...]:
    """Return a stable time-ordered view of both capture streams."""

    items: list[ReplayItem] = []
    sequence = 0
    for frame in session.frames:
        items.append(ReplayItem(relative_ms=max(int(frame.timestamp_ms), 0), sequence=sequence, frame=frame))
        sequence += 1
    for timed_record in session.serial_records:
        items.append(
            ReplayItem(
                relative_ms=max(int(timed_record.relative_ms), 0),
                sequence=sequence,
                serial_record=timed_record.record,
            )
        )
        sequence += 1
    return tuple(sorted(items, key=lambda item: (item.relative_ms, item.sequence)))


class RecordedSessionReplay:
    """Publish one recorded session to the same queues used by the live UI."""

    def __init__(self, session: RecordedSession, *, speed: float = 1.0, loop: bool = False):
        if float(speed) <= 0:
            raise ValueError("replay speed must be greater than zero")
        self.session = session
        self.speed = float(speed)
        self.loop = bool(loop)
        self.items = replay_items(session)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, point_buffer: LatestFrameBuffer, serial_records: Queue[SerialRecord]) -> None:
        if self.is_running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(point_buffer, serial_records),
            name="RecordedSessionReplay",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout_s: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout_s)

    def _run(self, point_buffer: LatestFrameBuffer, serial_records: Queue[SerialRecord]) -> None:
        if not self.items:
            return
        while not self._stop_event.is_set():
            cycle_start = time.monotonic()
            for item in self.items:
                due_at = cycle_start + item.relative_ms / 1000.0 / self.speed
                if not self._wait_until(due_at):
                    return
                if item.frame is not None:
                    point_buffer.publish(item.frame)
                elif item.serial_record is not None:
                    serial_records.put(item.serial_record)
            if not self.loop:
                return

    def _wait_until(self, due_at: float) -> bool:
        while not self._stop_event.is_set():
            remaining = due_at - time.monotonic()
            if remaining <= 0:
                return True
            self._stop_event.wait(min(remaining, 0.05))
        return False


def _parse_log_row(raw_line: str) -> tuple[datetime, SerialRecord] | None:
    match = _LOG_LINE_PATTERN.match(raw_line.strip("\r\n"))
    if match is None:
        return None
    timestamp = _parse_datetime(match.group("timestamp"))
    if timestamp is None:
        return None
    direction, line = _split_direction(match.group("body"))
    return timestamp, SerialRecord(
        direction=direction,
        line=line,
        received_at=timestamp,
        event=parse_fall_event(line, received_at=timestamp),
        status=parse_board_status(line, received_at=timestamp),
    )


def _split_direction(body: str) -> tuple[str, str]:
    for prefix, direction in (
        ("收 <-", "rx"),
        ("收←", "rx"),
        ("发 ->", "tx"),
        ("发→", "tx"),
    ):
        if body.startswith(prefix):
            return direction, body[len(prefix) :].lstrip()
    return "rx", body.strip()


def _limit_to_recording_interval(rows: list[tuple[datetime, SerialRecord]]) -> list[tuple[datetime, SerialRecord]]:
    start = next(
        (timestamp for timestamp, record in rows if "recording started" in record.line.lower()),
        None,
    )
    if start is None:
        return rows
    stop = next(
        (
            timestamp
            for timestamp, record in rows
            if timestamp >= start and "recording stopped" in record.line.lower()
        ),
        None,
    )
    return [
        (timestamp, record)
        for timestamp, record in rows
        if timestamp >= start and (stop is None or timestamp <= stop)
    ]


def _read_session_metadata(directory: Path) -> dict:
    path = directory / "session.json"
    if not path.exists():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _metadata_datetime(value: object) -> datetime | None:
    if value is None:
        return None
    return _parse_datetime(str(value))


def _parse_datetime(value: str) -> datetime | None:
    text = str(value).strip()
    for fmt in _TIMESTAMP_FORMATS:
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            continue
    return None
