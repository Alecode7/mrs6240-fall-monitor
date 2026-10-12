from __future__ import annotations

import threading
from typing import Callable


def list_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


class SerialController:
    def __init__(self, *, on_line: Callable[[str], None] | None = None):
        self.on_line = on_line
        self.serial = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def is_connected(self) -> bool:
        return self.serial is not None and self.serial.is_open

    def connect(self, port: str, baud_rate: int) -> None:
        if self.is_connected:
            return
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for the control serial port. Install it with: uv add pyserial") from exc
        self.serial = serial.Serial(port=port, baudrate=int(baud_rate), timeout=0.1)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, name="SerialControllerReader", daemon=True)
        self._thread.start()

    def send_line(self, command: str) -> None:
        if not self.is_connected:
            raise RuntimeError("control serial port is not connected")
        text = command.rstrip("\r\n") + "\r\n"
        self.serial.write(text.encode("utf-8"))

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        if self.serial is not None:
            self.serial.close()
        self.serial = None

    def _read_loop(self) -> None:
        while not self._stop_event.is_set() and self.serial is not None:
            try:
                line = self.serial.readline()
            except Exception as exc:
                self._emit(f"[serial read error] {exc}")
                break
            if line:
                self._emit(_decode_serial_line(line))

    def _emit(self, line: str) -> None:
        if self.on_line is not None:
            self.on_line(line)


def _decode_serial_line(raw: bytes) -> str:
    for encoding in ("utf-8", "gbk", "ascii"):
        try:
            return raw.decode(encoding).strip("\x00\r\n")
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace").strip("\x00\r\n")
