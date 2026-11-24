from __future__ import annotations

import re
import threading
from dataclasses import dataclass
from datetime import datetime
from queue import Queue
from typing import Literal, Mapping


FALL_EVENT_PREFIX = "FALL_EVENT"
FALL_EVENT_STATES = frozenset({"suspected", "confirmed", "recovered"})
_FIELD_PATTERN = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[^\s]+)")


@dataclass(frozen=True)
class FallSerialEvent:
    state: str
    fields: Mapping[str, str]
    raw_line: str
    received_at: datetime

    @property
    def identity(self) -> tuple[str, str, str, str]:
        return (
            self.state,
            self.fields.get("frame", ""),
            self.fields.get("t", ""),
            self.raw_line if not self.fields.get("frame") and not self.fields.get("t") else "",
        )


@dataclass(frozen=True)
class BoardFrameStatus:
    frame_index: int
    state: int
    track: int | None
    has_target: bool
    x_cm: int | None
    y_cm: int | None
    z_cm: int | None
    height_cm: int | None
    point_count: int | None
    motion_count: int | None
    velocity_cm_s: int | None
    raw_count: int | None
    clusters: int | None
    fresh: int | None
    low: int | None
    overflow: int | None
    raw_line: str
    received_at: datetime


@dataclass(frozen=True)
class SerialRecord:
    direction: Literal["rx", "tx", "error"]
    line: str
    received_at: datetime
    event: FallSerialEvent | None = None
    status: BoardFrameStatus | None = None


@dataclass(frozen=True)
class SerialDiagnostics:
    """Thread-safe snapshot of control-line parsing quality."""

    received_lines: int = 0
    status_frames: int = 0
    fall_events: int = 0
    other_lines: int = 0
    decode_replacement_lines: int = 0
    last_line_at: datetime | None = None

    @property
    def status_ratio(self) -> float | None:
        if self.received_lines == 0:
            return None
        return self.status_frames / self.received_lines


def parse_fall_event(line: str, *, received_at: datetime | None = None) -> FallSerialEvent | None:
    raw_text = str(line).strip().strip("\x00")
    text = _protocol_payload(raw_text, FALL_EVENT_PREFIX)
    if text is None:
        return None
    fields = {match.group("key"): match.group("value") for match in _FIELD_PATTERN.finditer(text)}
    state = fields.get("state", "")
    if state not in FALL_EVENT_STATES:
        return None
    return FallSerialEvent(
        state=state,
        fields=fields,
        raw_line=text,
        received_at=received_at or datetime.now(),
    )


def parse_board_status(line: str, *, received_at: datetime | None = None) -> BoardFrameStatus | None:
    raw_text = str(line).strip().strip("\x00")
    text = _protocol_payload(raw_text, "FALL ")
    if text is None:
        return None
    fields = {match.group("key"): match.group("value") for match in _FIELD_PATTERN.finditer(text)}
    if "frame" not in fields or "state" not in fields:
        return None
    try:
        frame_index = int(fields["frame"])
        state = int(fields["state"])
    except ValueError:
        return None
    target = _optional_int(fields, "target", 0) or 0
    return BoardFrameStatus(
        frame_index=frame_index,
        state=state,
        track=_optional_int(fields, "track"),
        has_target=bool(target),
        x_cm=_optional_int(fields, "x"),
        y_cm=_optional_int(fields, "y"),
        z_cm=_optional_int(fields, "z"),
        height_cm=_optional_int(fields, "h"),
        point_count=_optional_int(fields, "pts"),
        motion_count=_optional_int(fields, "mot"),
        velocity_cm_s=_optional_int(fields, "v"),
        raw_count=_optional_int(fields, "raw"),
        clusters=_optional_int(fields, "clusters"),
        fresh=_optional_int(fields, "fresh"),
        low=_optional_int(fields, "low"),
        overflow=_optional_int(fields, "overflow"),
        raw_line=text,
        received_at=received_at or datetime.now(),
    )


def command_bytes(command: str) -> bytes:
    text = str(command).strip()
    if not text:
        return b""
    return (text + "\r\n").encode("utf-8")


def _protocol_payload(text: str, prefix: str) -> str | None:
    start = text.find(prefix)
    if start < 0:
        return None
    if start > 0 and not text[start - 1].isspace():
        return None
    return text[start:]


def _optional_int(fields: Mapping[str, str], key: str, default: int | None = None) -> int | None:
    value = fields.get(key)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


class SerialLogReader:
    """Read a control UART on a worker thread and emit immutable records."""

    def __init__(self, records: Queue[SerialRecord] | None = None, *, serial_factory=None):
        self.records = records or Queue()
        self.serial_factory = serial_factory
        self.serial = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._diagnostics_lock = threading.Lock()
        self._received_lines = 0
        self._status_frames = 0
        self._fall_events = 0
        self._other_lines = 0
        self._decode_replacement_lines = 0
        self._last_line_at: datetime | None = None

    @property
    def diagnostics(self) -> SerialDiagnostics:
        with self._diagnostics_lock:
            return SerialDiagnostics(
                received_lines=self._received_lines,
                status_frames=self._status_frames,
                fall_events=self._fall_events,
                other_lines=self._other_lines,
                decode_replacement_lines=self._decode_replacement_lines,
                last_line_at=self._last_line_at,
            )

    @property
    def is_connected(self) -> bool:
        return self.serial is not None and bool(getattr(self.serial, "is_open", False))

    def connect(self, port: str, baud_rate: int) -> None:
        if self.is_connected:
            return
        if self.serial is not None:
            self.stop()
        factory = self.serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("pyserial is required for the control serial port") from exc
            factory = serial.Serial
        self.serial = factory(port=port, baudrate=int(baud_rate), timeout=0.1)
        self._reset_diagnostics()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, name="SerialLogReader", daemon=True)
        self._thread.start()

    def send_line(self, command: str) -> None:
        payload = command_bytes(command)
        if not payload:
            return
        if not self.is_connected:
            raise RuntimeError("control serial port is not connected")
        with self._write_lock:
            self.serial.write(payload)
        self.records.put(SerialRecord("tx", payload.decode("utf-8").rstrip("\r\n"), datetime.now()))

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._thread = None
        serial_port = self.serial
        self.serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def _read_loop(self) -> None:
        while not self._stop_event.is_set() and self.serial is not None:
            try:
                raw_line = self.serial.readline()
            except Exception as exc:
                self.records.put(SerialRecord("error", f"serial read error: {exc}", datetime.now()))
                self._close_from_reader()
                return
            if not raw_line:
                continue
            line, has_decode_replacement = _decode_serial_line_with_quality(raw_line)
            received_at = datetime.now()
            event = parse_fall_event(line, received_at=received_at)
            status = parse_board_status(line, received_at=received_at)
            self.records.put(SerialRecord("rx", line, received_at, event, status))
            if line:
                self._record_received_line(event, status, has_decode_replacement, received_at)

    def _reset_diagnostics(self) -> None:
        with self._diagnostics_lock:
            self._received_lines = 0
            self._status_frames = 0
            self._fall_events = 0
            self._other_lines = 0
            self._decode_replacement_lines = 0
            self._last_line_at = None

    def _record_received_line(
        self,
        event: FallSerialEvent | None,
        status: BoardFrameStatus | None,
        has_decode_replacement: bool,
        received_at: datetime,
    ) -> None:
        with self._diagnostics_lock:
            self._received_lines += 1
            if event is not None:
                self._fall_events += 1
            elif status is not None:
                self._status_frames += 1
            else:
                self._other_lines += 1
            if has_decode_replacement:
                self._decode_replacement_lines += 1
            self._last_line_at = received_at

    def _close_from_reader(self) -> None:
        """Release a failed port without joining the current reader thread."""
        serial_port = self.serial
        self.serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass


def _decode_serial_line(raw: bytes) -> str:
    return _decode_serial_line_with_quality(raw)[0]


def _decode_serial_line_with_quality(raw: bytes) -> tuple[str, bool]:
    for encoding in ("utf-8", "gbk", "ascii"):
        try:
            return raw.decode(encoding).strip("\x00\r\n"), False
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace").strip("\x00\r\n"), True
