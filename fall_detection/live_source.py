from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Protocol

from .datadealer_loader import CombinedFrame


class LiveSource(Protocol):
    @property
    def is_running(self) -> bool:
        ...

    def start(self, buffer: "LatestFrameBuffer") -> None:
        ...

    def stop(self) -> None:
        ...

    def join(self, timeout_s: float | None = None) -> None:
        ...


class LatestFrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._pending: deque[CombinedFrame] = deque()
        self.published_count = 0
        self.consumed_count = 0

    def publish(self, frame: CombinedFrame) -> None:
        with self._lock:
            self._pending.append(frame)
            self.published_count += 1

    def take_latest(self) -> CombinedFrame | None:
        with self._lock:
            if not self._pending:
                return None
            latest = self._pending[-1]
            self._pending.clear()
            self.consumed_count += 1
            return latest

    def take_pending(self) -> tuple[CombinedFrame, ...]:
        with self._lock:
            if not self._pending:
                return ()
            pending = tuple(self._pending)
            self._pending.clear()
            self.consumed_count += len(pending)
            return pending


@dataclass(frozen=True)
class LiveSourceStats:
    published_count: int
    consumed_count: int
    is_running: bool


class ReplayLiveSource:
    def __init__(
        self,
        frames: Iterable[CombinedFrame],
        *,
        interval_s: float = 0.05,
        loop: bool = False,
    ):
        self.frames = tuple(frames)
        self.interval_s = max(interval_s, 0.0)
        self.loop = loop
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, buffer: LatestFrameBuffer) -> None:
        if self.is_running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, args=(buffer,), name="ReplayLiveSource", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout_s: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout_s)

    def _run(self, buffer: LatestFrameBuffer) -> None:
        while not self._stop_event.is_set():
            for frame in self.frames:
                if self._stop_event.is_set():
                    break
                buffer.publish(frame)
                if self.interval_s > 0:
                    time.sleep(self.interval_s)
            if not self.loop:
                break


def live_source_stats(source: LiveSource, buffer: LatestFrameBuffer) -> LiveSourceStats:
    return LiveSourceStats(
        published_count=buffer.published_count,
        consumed_count=buffer.consumed_count,
        is_running=source.is_running,
    )
