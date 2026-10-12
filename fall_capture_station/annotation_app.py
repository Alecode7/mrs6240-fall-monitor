from __future__ import annotations

import csv
import json
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_capture_station.camera_recorder import frame_to_photoimage
    from fall_capture_station.session_recorder import (
        AnnotationEventReference,
        SessionTimeline,
        append_operator_mark,
        backfill_operator_mark_references,
        load_session_timeline,
        wall_time_now,
    )
    from fall_capture_station.v2_protocol import (
        ANNOTATION_CONTEXTS,
        CaptureProfileError,
        format_annotation_note,
    )
else:
    from .camera_recorder import frame_to_photoimage
    from .session_recorder import (
        AnnotationEventReference,
        SessionTimeline,
        append_operator_mark,
        backfill_operator_mark_references,
        load_session_timeline,
        wall_time_now,
    )
    from .v2_protocol import (
        ANNOTATION_CONTEXTS,
        CaptureProfileError,
        format_annotation_note,
    )

from fall_detection.datadealer_loader import Point3D, load_datadealer_recording
from fall_detection.plot_fonts import (
    CHINESE_FONT_CANDIDATES as PLOT_CHINESE_FONT_CANDIDATES,
    configure_chinese_matplotlib_font as _configure_chinese_matplotlib_font,
)
from fall_detection.tracking import TrackerConfig
from fall_detection.visualization_model import VisualizationFrame, build_visualization_frames


DEFAULT_DATA_ROOT = Path(__file__).resolve().parent / "data"
ANNOTATION_MARKS = ("无人", "有人", "蹲下", "弯腰", "坐下", "跌倒")
MARK_BUTTON_COLUMNS = 4
CHINESE_FONT_CANDIDATES = PLOT_CHINESE_FONT_CANDIDATES
DEFAULT_PLAYBACK_FPS = 5
PLAYBACK_FPS_OPTIONS = (1, 2, 5, 10, 20, 35, 60)


@dataclass(frozen=True)
class MarkButtonPosition:
    mark: str
    row: int
    column: int


@dataclass(frozen=True)
class PointGroup:
    label: str
    points: tuple[Point3D, ...]
    color: str
    size: int
    alpha: float


def _annotation_mark_positions() -> tuple[MarkButtonPosition, ...]:
    return tuple(
        MarkButtonPosition(mark, index // MARK_BUTTON_COLUMNS, index % MARK_BUTTON_COLUMNS)
        for index, mark in enumerate(ANNOTATION_MARKS)
    )


def _point_groups(points: tuple[Point3D, ...] | list[Point3D]) -> list[PointGroup]:
    presence_points = tuple(point for point in points if point.kind != "motion")
    motion_points = tuple(point for point in points if point.kind == "motion")
    groups: list[PointGroup] = []
    if presence_points:
        groups.append(PointGroup("存在点", presence_points, "#38bdf8", 11, 0.45))
    if motion_points:
        groups.append(PointGroup("运动点", motion_points, "#f97316", 18, 0.92))
    return groups


def _playback_interval_ms(fps: int) -> int:
    if int(fps) <= 0:
        raise ValueError("fps must be positive")
    return max(1, round(1000 / int(fps)))


def _next_playback_index(current_index: int, frame_count: int) -> int | None:
    if frame_count <= 0 or current_index >= frame_count - 1:
        return None
    return max(0, current_index + 1)


class AnnotationApp:
    def __init__(self, root: tk.Tk, *, data_root: Path = DEFAULT_DATA_ROOT):
        self.root = root
        self.data_root = data_root
        self.session_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="未加载")
        self.camera_offset_var = tk.IntVar(value=0)
        self.playback_fps_var = tk.IntVar(value=DEFAULT_PLAYBACK_FPS)
        self.context_var = tk.StringVar(value="plain")
        self.frames: list[VisualizationFrame] = []
        self.camera_rows: list[dict[str, str]] = []
        self.session_timeline: SessionTimeline | None = None
        self.video_capture = None
        self.video_photo = None
        self.current_index = 0
        self.playing = False
        self.playback_after_id: str | None = None
        self._updating_scale = False
        self.session_dir: Path | None = None
        self.session_name = ""
        self._build_ui()
        self._refresh_sessions()

    def _build_ui(self) -> None:
        self.root.title("MRS6240 采集数据标注上位机")
        self.root.geometry("1320x820")
        self.root.minsize(1080, 720)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=(10, 8))
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        top.columnconfigure(6, weight=1)
        ttk.Label(top, text="采集会话").grid(row=0, column=0, padx=(0, 6))
        self.session_combo = ttk.Combobox(top, textvariable=self.session_var, width=32, state="readonly")
        self.session_combo.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(top, text="刷新", command=self._refresh_sessions).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(top, text="加载", command=self.load_session).grid(row=0, column=3, padx=(0, 12))
        ttk.Label(top, text="上下文").grid(row=0, column=4, padx=(0, 6))
        context_combo = ttk.Combobox(
            top,
            textvariable=self.context_var,
            values=ANNOTATION_CONTEXTS,
            width=14,
            state="readonly",
        )
        context_combo.grid(row=0, column=5, padx=(0, 12))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=6, sticky="w")

        nav = ttk.Frame(top)
        nav.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        nav.columnconfigure(3, weight=1)
        ttk.Button(nav, text="上一帧", command=self.previous_frame).grid(row=0, column=0, padx=(0, 6))
        self.play_button = ttk.Button(nav, text="播放", command=self.toggle_playback, width=8)
        self.play_button.grid(row=0, column=1, padx=(0, 6))
        ttk.Button(nav, text="下一帧", command=self.next_frame).grid(row=0, column=2, padx=(0, 10))
        self.frame_scale = ttk.Scale(nav, from_=0, to=0, orient="horizontal", command=self._on_slider)
        self.frame_scale.grid(row=0, column=3, sticky="ew", padx=(0, 10))
        ttk.Label(nav, text="摄像头偏移ms").grid(row=0, column=4, padx=(0, 4))
        offset_spin = ttk.Spinbox(nav, from_=-5000, to=5000, increment=50, textvariable=self.camera_offset_var, width=7, command=self.draw_current_frame)
        offset_spin.grid(row=0, column=5, padx=(0, 4))
        offset_spin.bind("<Return>", lambda _event: self.draw_current_frame())
        offset_spin.bind("<FocusOut>", lambda _event: self.draw_current_frame())
        ttk.Button(nav, text="-50", command=lambda: self._adjust_camera_offset(-50)).grid(row=0, column=6, padx=(0, 4))
        ttk.Button(nav, text="+50", command=lambda: self._adjust_camera_offset(50)).grid(row=0, column=7, padx=(0, 10))
        ttk.Label(nav, text="播放FPS").grid(row=0, column=8, padx=(0, 4))
        playback_fps_combo = ttk.Combobox(
            nav,
            textvariable=self.playback_fps_var,
            values=PLAYBACK_FPS_OPTIONS,
            width=4,
            state="readonly",
        )
        playback_fps_combo.grid(row=0, column=9)
        playback_fps_combo.bind("<<ComboboxSelected>>", self._on_playback_speed_changed)
        self.root.bind("<space>", self._on_space_key)

        marks_bar = ttk.Frame(top)
        marks_bar.grid(row=2, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        for column in range(MARK_BUTTON_COLUMNS):
            marks_bar.columnconfigure(column, weight=1, uniform="marks")
        for position in _annotation_mark_positions():
            ttk.Button(marks_bar, text=position.mark, command=lambda item=position.mark: self.write_mark(item)).grid(
                row=position.row,
                column=position.column,
                sticky="ew",
                padx=(0, 6),
                pady=(0 if position.row == 0 else 6, 0),
            )

        left = ttk.Frame(self.root, padding=(10, 0, 6, 10))
        left.grid(row=1, column=0, sticky="nsew")
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        ttk.Label(left, text="同步摄像头画面").grid(row=0, column=0, sticky="w")
        self.video_label = ttk.Label(left, background="#0b0f14", anchor="center")
        self.video_label.grid(row=1, column=0, sticky="nsew", pady=(6, 0))

        right = ttk.Frame(self.root, padding=(6, 0, 10, 10))
        right.grid(row=1, column=1, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        _configure_chinese_matplotlib_font()
        self.figure = Figure(figsize=(6, 6), dpi=100, facecolor="#0b0f14")
        self.ax_xy = self.figure.add_subplot(311)
        self.ax_xz = self.figure.add_subplot(312)
        self.ax_yz = self.figure.add_subplot(313)
        self.figure_canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.figure_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _refresh_sessions(self) -> None:
        sessions = [path.name for path in sorted(self.data_root.iterdir()) if (path / "session.json").exists()] if self.data_root.exists() else []
        self.session_combo["values"] = sessions
        if sessions and not self.session_var.get():
            self.session_var.set(sessions[0])

    def load_session(self) -> None:
        self.pause_playback()
        name = self.session_var.get()
        if not name:
            return
        self.session_dir = self.data_root / name
        try:
            metadata = json.loads((self.session_dir / "session.json").read_text(encoding="utf-8"))
            self.session_name = str(metadata["session_name"])
            recording = load_datadealer_recording(self.session_dir, stem=self.session_name)
            self.frames = build_visualization_frames(recording.frames, TrackerConfig())
            backfilled_marks = backfill_operator_mark_references(self.session_dir)
            self.session_timeline = load_session_timeline(self.session_dir)
            self.camera_rows = list(self.session_timeline.camera_rows)
            self._open_video(self.session_dir / "camera.mp4")
        except Exception as exc:
            messagebox.showerror("加载失败", str(exc))
            return
        self.current_index = 0
        self.frame_scale.configure(from_=0, to=max(len(self.frames) - 1, 0))
        self._set_scale_value(0)
        self.status_var.set(
            f"{self.session_name}  雷达帧={len(self.frames)}  摄像头帧={len(self.camera_rows)}"
            f"  补全标注={backfilled_marks}"
        )
        self.draw_current_frame()

    def previous_frame(self) -> None:
        if not self.frames:
            return
        self.pause_playback()
        self._show_frame_index(self.current_index - 1)

    def next_frame(self) -> None:
        if not self.frames:
            return
        self.pause_playback()
        self._show_frame_index(self.current_index + 1)

    def _on_slider(self, value: str) -> None:
        if not self.frames or self._updating_scale:
            return
        self.pause_playback()
        self.current_index = min(max(int(float(value)), 0), len(self.frames) - 1)
        self.draw_current_frame()

    def _set_scale_value(self, value: int) -> None:
        self._updating_scale = True
        try:
            self.frame_scale.set(value)
        finally:
            self._updating_scale = False

    def _show_frame_index(self, index: int) -> None:
        if not self.frames:
            return
        self.current_index = min(max(int(index), 0), len(self.frames) - 1)
        self._set_scale_value(self.current_index)
        self.draw_current_frame()

    def toggle_playback(self) -> None:
        if self.playing:
            self.pause_playback()
        else:
            self.start_playback()

    def start_playback(self) -> None:
        if not self.frames:
            return
        if self.current_index >= len(self.frames) - 1:
            self._show_frame_index(0)
        self.playing = True
        self.play_button.configure(text="暂停")
        self.draw_current_frame()
        self._schedule_playback_tick()

    def pause_playback(self) -> None:
        self.playing = False
        if self.playback_after_id is not None:
            try:
                self.root.after_cancel(self.playback_after_id)
            except tk.TclError:
                pass
            self.playback_after_id = None
        if hasattr(self, "play_button"):
            self.play_button.configure(text="播放")

    def _schedule_playback_tick(self) -> None:
        if not self.playing:
            return
        interval_ms = _playback_interval_ms(int(self.playback_fps_var.get()))
        self.playback_after_id = self.root.after(interval_ms, self._playback_tick)

    def _playback_tick(self) -> None:
        self.playback_after_id = None
        if not self.playing:
            return
        next_index = _next_playback_index(self.current_index, len(self.frames))
        if next_index is None:
            self.pause_playback()
            return
        self._show_frame_index(next_index)
        if self.current_index >= len(self.frames) - 1:
            self.pause_playback()
            self.status_var.set(f"{self.session_name}  播放完成  帧 {self.current_index + 1}/{len(self.frames)}")
            return
        self._schedule_playback_tick()

    def _on_playback_speed_changed(self, _event=None) -> None:
        if not self.playing:
            return
        if self.playback_after_id is not None:
            try:
                self.root.after_cancel(self.playback_after_id)
            except tk.TclError:
                pass
            self.playback_after_id = None
        self._schedule_playback_tick()

    def _on_space_key(self, _event=None):
        self.toggle_playback()
        return "break"

    def _adjust_camera_offset(self, delta_ms: int) -> None:
        self.camera_offset_var.set(int(self.camera_offset_var.get()) + int(delta_ms))
        self.draw_current_frame()

    def draw_current_frame(self) -> None:
        if not self.frames:
            return
        view = self.frames[self.current_index]
        self._draw_camera(view.frame.timestamp_ms)
        self._draw_pointcloud(view)
        reference = self._event_reference(view.frame)
        self.status_var.set(
            f"{self.session_name}  帧 {self.current_index + 1}/{len(self.frames)}  "
            f"t={view.frame.timestamp_ms}ms  {_format_event_reference(reference)}  "
            f"点={len(view.filtered_points)}"
            f"{'  播放中 ' + str(self.playback_fps_var.get()) + 'FPS' if self.playing else ''}"
        )

    def write_mark(self, mark: str) -> None:
        if self.session_dir is None or not self.frames:
            return
        try:
            note = format_annotation_note(self.context_var.get())
        except CaptureProfileError as error:
            messagebox.showerror("标注上下文无效", str(error))
            return
        self.pause_playback()
        frame = self.frames[self.current_index].frame
        reference = self._event_reference(frame)
        append_operator_mark(
            self.session_dir,
            mark,
            monotonic_ns=time.monotonic_ns(),
            wall_time=wall_time_now(),
            note=note,
            radar_frame_index=frame.frame_index,
            radar_timestamp_ms=frame.timestamp_ms,
            event_reference=reference,
        )
        self.status_var.set(
            f"已标注 {mark}  上下文={self.context_var.get()}  "
            f"回放帧={frame.frame_index} t={frame.timestamp_ms}ms  "
            f"{_format_event_reference(reference)}"
        )

    def _event_reference(self, frame) -> AnnotationEventReference:
        if self.session_timeline is None:
            return AnnotationEventReference()
        return self.session_timeline.resolve(
            replay_frame_index=frame.frame_index,
            capture_relative_ms=frame.timestamp_ms,
            camera_offset_ms=int(self.camera_offset_var.get()),
        )

    def _draw_camera(self, timestamp_ms: int) -> None:
        if self.video_capture is None or not self.camera_rows:
            self.video_label.configure(text="没有 camera.mp4 或 camera_frames.csv", foreground="#e5e7eb")
            return
        row = _nearest_camera_row(self.camera_rows, radar_timestamp_ms=timestamp_ms, camera_offset_ms=int(self.camera_offset_var.get()))
        try:
            frame_index = int(row["camera_frame_index"])
            self.video_capture.set(1, frame_index)
            ok, image_bgr = self.video_capture.read()
            if not ok:
                return
            self.video_photo = frame_to_photoimage(image_bgr, max_width=640, max_height=560)
            self.video_label.configure(image=self.video_photo, text="")
        except Exception as exc:
            self.video_label.configure(text=f"视频显示失败: {exc}", foreground="#dc2626")

    def _draw_pointcloud(self, view: VisualizationFrame) -> None:
        for axis in (self.ax_xy, self.ax_xz, self.ax_yz):
            axis.clear()
            axis.set_facecolor("#0b0f14")
            axis.grid(True, color="#1e2833", linewidth=0.6)
            axis.tick_params(colors="#94a3b8", labelsize=7)
            for spine in axis.spines.values():
                spine.set_color("#334155")
        point_groups = _point_groups(view.filtered_points)
        self._scatter_groups(self.ax_xy, point_groups, "x_m", "y_m", "XY")
        self._scatter_groups(self.ax_xz, point_groups, "x_m", "z_m", "XZ")
        self._scatter_groups(self.ax_yz, point_groups, "y_m", "z_m", "YZ")
        if view.target.has_target:
            self.ax_xy.scatter([view.target.center_x_m], [view.target.center_y_m], c=["#ff4d4d"], s=50, marker="x")
            self.ax_xz.scatter([view.target.center_x_m], [view.target.center_z_m], c=["#ff4d4d"], s=50, marker="x")
            self.ax_yz.scatter([view.target.center_y_m], [view.target.center_z_m], c=["#ff4d4d"], s=50, marker="x")
        self.ax_xy.set_xlabel("X left + (m)", color="#cbd5e1", fontsize=8)
        self.ax_xy.set_ylabel("Y forward (m)", color="#cbd5e1", fontsize=8)
        self.ax_xz.set_xlabel("X left + (m)", color="#cbd5e1", fontsize=8)
        self.ax_xz.set_ylabel("Z height (m)", color="#cbd5e1", fontsize=8)
        self.ax_yz.set_xlabel("Y forward (m)", color="#cbd5e1", fontsize=8)
        self.ax_yz.set_ylabel("Z height (m)", color="#cbd5e1", fontsize=8)
        self.figure.subplots_adjust(left=0.10, right=0.98, top=0.96, bottom=0.08, hspace=0.42)
        self.figure_canvas.draw_idle()

    def _scatter_groups(self, axis, point_groups: list[PointGroup], x_attr: str, y_attr: str, title: str) -> None:
        axis.set_title(title, color="#cbd5e1", fontsize=9)
        for group in point_groups:
            xs = [getattr(point, x_attr) for point in group.points]
            ys = [getattr(point, y_attr) for point in group.points]
            if xs:
                axis.scatter(xs, ys, c=[group.color], s=group.size, alpha=group.alpha, label=group.label)
        if point_groups:
            legend = axis.legend(loc="upper right", fontsize=7, frameon=True)
            legend.get_frame().set_facecolor("#0b0f14")
            legend.get_frame().set_edgecolor("#334155")
            for text in legend.get_texts():
                text.set_color("#cbd5e1")

    def _open_video(self, path: Path) -> None:
        if self.video_capture is not None:
            self.video_capture.release()
            self.video_capture = None
        if not path.exists():
            return
        try:
            import cv2
        except ImportError:
            return
        capture = cv2.VideoCapture(str(path))
        if capture.isOpened():
            self.video_capture = capture

    def close(self) -> None:
        self.pause_playback()
        if self.video_capture is not None:
            self.video_capture.release()
        self.root.destroy()


def _load_camera_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _nearest_camera_row(camera_rows, *, radar_timestamp_ms: int, camera_offset_ms: int = 0):
    target_ms = int(radar_timestamp_ms) + int(camera_offset_ms)
    return min(camera_rows, key=lambda item: abs(_camera_row_time_ms(item) - target_ms))


def _camera_row_time_ms(row: dict[str, str]) -> int:
    value = row.get("relative_ms") or row.get("video_time_ms") or 0
    return int(float(value))


def _format_event_reference(reference: AnnotationEventReference) -> str:
    capture_time = reference.capture_wall_time or "--"
    source_frame = (
        str(reference.source_radar_frame_index)
        if reference.source_radar_frame_index is not None
        else "--"
    )
    camera_frame = (
        str(reference.camera_frame_index)
        if reference.camera_frame_index is not None
        else "--"
    )
    camera_delta = (
        f"{reference.camera_time_delta_ms:+d}ms"
        if reference.camera_time_delta_ms is not None
        else "--"
    )
    return (
        f"录制时刻={capture_time}  源帧={source_frame}  "
        f"摄像头帧={camera_frame} 差={camera_delta}"
    )


def main() -> None:
    root = tk.Tk()
    app = AnnotationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    main()
