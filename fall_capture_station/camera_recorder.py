from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path

from .session_recorder import wall_time_now


@dataclass(frozen=True)
class CameraFrame:
    frame_index: int
    monotonic_ns: int
    wall_time: str
    video_time_ms: int
    image_bgr: object


class CameraRecorder:
    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index
        self.cv2 = None
        self.capture = None
        self.writer = None
        self.output_path: Path | None = None
        self.frame_index = 0
        self.recording_started_ns: int | None = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._opening = False
        self._opened = False
        self._latest_frame: CameraFrame | None = None
        self._last_error: str | None = None

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._opened

    @property
    def is_opening(self) -> bool:
        with self._lock:
            return self._opening

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self.output_path is not None

    def open(self) -> None:
        with self._lock:
            if self._opened or self._opening:
                return
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._last_error = None
            self._latest_frame = None
            self._opening = True
            self.frame_index = 0
            camera_index = int(self.camera_index)
            thread = threading.Thread(
                target=self._capture_loop,
                args=(camera_index,),
                name=f"CameraRecorder-{camera_index}",
                daemon=True,
            )
            self._thread = thread
            thread.start()

    def start_recording(self, output_path: str | Path) -> None:
        self.open()
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self.output_path = path
            self.frame_index = 0
            self.recording_started_ns = time.monotonic_ns()
            self._latest_frame = None

    def stop_recording(self) -> None:
        with self._lock:
            self.output_path = None
            self.recording_started_ns = None

    def read_frame(self) -> CameraFrame | None:
        with self._lock:
            frame = self._latest_frame
            self._latest_frame = None
            return frame

    def take_error(self) -> str | None:
        with self._lock:
            error = self._last_error
            self._last_error = None
            return error

    def close(self) -> None:
        self.stop_recording()
        self._stop_event.set()
        with self._lock:
            capture = self.capture
            thread = self._thread
        if capture is not None:
            capture.release()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._latest_frame = None
            self._opening = False
            self._opened = False
            self.capture = None
            self.cv2 = None
            if self._thread is not None and not self._thread.is_alive():
                self._thread = None

    def _capture_loop(self, camera_index: int) -> None:
        capture = None
        writer = None
        writer_path: Path | None = None
        try:
            cv2 = _import_cv2()
            capture = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
            if not capture.isOpened():
                capture.release()
                raise RuntimeError(f"failed to open camera index {camera_index}")
            with self._lock:
                self.cv2 = cv2
                self.capture = capture
                self._opening = False
                self._opened = True

            while not self._stop_event.is_set():
                ok, image_bgr = capture.read()
                if self._stop_event.is_set():
                    break
                if not ok:
                    if not capture.isOpened():
                        raise RuntimeError(f"camera index {camera_index} stopped")
                    self._stop_event.wait(0.01)
                    continue

                monotonic_ns = time.monotonic_ns()
                wall_time = wall_time_now()
                with self._lock:
                    frame_index = self.frame_index
                    self.frame_index += 1
                    recording_started_ns = self.recording_started_ns
                    output_path = self.output_path
                    video_time_ms = (
                        int((monotonic_ns - recording_started_ns) / 1_000_000)
                        if recording_started_ns is not None
                        else 0
                    )
                    self._latest_frame = CameraFrame(
                        frame_index=frame_index,
                        monotonic_ns=monotonic_ns,
                        wall_time=wall_time,
                        video_time_ms=video_time_ms,
                        image_bgr=image_bgr,
                    )

                if output_path != writer_path:
                    if writer is not None:
                        writer.release()
                    writer = None
                    writer_path = None
                    with self._lock:
                        self.writer = None
                    if output_path is not None:
                        writer = self._create_writer(
                            cv2,
                            capture,
                            output_path,
                            image_bgr,
                        )
                        writer_path = output_path
                        with self._lock:
                            self.writer = writer
                if writer is not None:
                    writer.write(image_bgr)
        except Exception as error:
            if not self._stop_event.is_set():
                with self._lock:
                    self._last_error = str(error)
        finally:
            if writer is not None:
                writer.release()
            if capture is not None:
                capture.release()
            with self._lock:
                self.writer = None
                self.capture = None
                self.cv2 = None
                self._opening = False
                self._opened = False
                if self._thread is threading.current_thread():
                    self._thread = None

    @staticmethod
    def _create_writer(cv2, capture, output_path: Path, image_bgr):
        height, width = image_bgr.shape[:2]
        fps = _camera_fps(capture) or 30.0
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            str(output_path),
            fourcc,
            fps,
            (int(width), int(height)),
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f"failed to create camera video: {output_path}")
        return writer


def frame_to_photoimage(image_bgr, *, max_width: int = 760, max_height: int = 520):
    cv2 = _import_cv2()
    import tkinter as tk

    height, width = image_bgr.shape[:2]
    scale = min(max_width / max(width, 1), max_height / max(height, 1), 1.0)
    if scale < 1.0:
        image_bgr = cv2.resize(image_bgr, (int(width * scale), int(height * scale)))
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    ppm = b"P6\n%d %d\n255\n" % (width, height) + rgb.tobytes()
    return tk.PhotoImage(data=ppm, format="PPM")


def _import_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("opencv-python is required for camera capture. Install it with: uv add opencv-python") from exc
    return cv2


def _camera_fps(capture) -> float:
    if capture is None:
        return 0.0
    fps = float(capture.get(5))
    if fps <= 1.0 or fps > 240.0:
        return 0.0
    return fps
