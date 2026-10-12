from __future__ import annotations

import argparse
import sys
import tkinter as tk
import time
from dataclasses import dataclass, replace
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any, Callable

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_detection.coordinate_transform import (
        GroundPlane,
        calibrate_ground_plane,
        transform_frame_to_world,
    )
    from fall_detection.datadealer_loader import Point3D, iter_sample_stems, load_datadealer_recording
    from fall_detection.operator_marks import OperatorMark, load_operator_marks
    from fall_detection.plot_fonts import configure_chinese_matplotlib_font
    from fall_detection.rule_detector import FallEvent, FallRuleConfig, FallState, detect_falls
    from fall_detection.tracking import TargetState, TrackerConfig
    from fall_detection.visualization_model import (
        ClusterView,
        VisualizationFrame,
        bounds_from_config,
        build_visualization_frames,
        format_debug_candidate_label,
        format_target_diagnosis as _model_format_target_diagnosis,
        project_xy,
    )
else:
    from .coordinate_transform import GroundPlane, calibrate_ground_plane, transform_frame_to_world
    from .datadealer_loader import Point3D, iter_sample_stems, load_datadealer_recording
    from .operator_marks import OperatorMark, load_operator_marks
    from .plot_fonts import configure_chinese_matplotlib_font
    from .rule_detector import FallEvent, FallRuleConfig, FallState, detect_falls
    from .tracking import TargetState, TrackerConfig
    from .visualization_model import (
        ClusterView,
        VisualizationFrame,
        bounds_from_config,
        build_visualization_frames,
        format_debug_candidate_label,
        format_target_diagnosis as _model_format_target_diagnosis,
        project_xy,
    )


configure_chinese_matplotlib_font()


@dataclass(frozen=True)
class DisplayClusterBox:
    cluster: ClusterView
    color: str
    label: str
    line_width: float


@dataclass(frozen=True)
class FrameAlert:
    jump_m: float | None = None
    fall_states: tuple[FallState, ...] = ()

    @property
    def has_alert(self) -> bool:
        return self.jump_m is not None or bool(self.fall_states)

    @property
    def summary(self) -> str:
        parts = []
        if self.jump_m is not None:
            parts.append(f"跳变 {self.jump_m:.2f}m")
        parts.extend(_fall_state_label(state) for state in self.fall_states)
        return " / ".join(parts) if parts else "正常"


def _format_source_frame_label(frame: Any) -> str:
    timestamp_text = str(getattr(frame, "timestamp_text", "") or "").strip()
    if timestamp_text.lower().startswith("frame "):
        return f"源帧 {timestamp_text.split(None, 1)[1]}"
    if timestamp_text:
        return f"源 {timestamp_text}"
    frame_index = getattr(frame, "frame_index", None)
    if frame_index is None:
        return "源帧 --"
    return f"源帧 {frame_index}"


def _rotate_frames_for_visualization(frames: Any, yz_rotation_deg: float) -> tuple[Any, ...]:
    transformed, _ = _prepare_frames_for_visualization(
        frames,
        yz_rotation_deg=yz_rotation_deg,
    )
    return transformed


def _prepare_frames_for_visualization(
    frames: Any,
    *,
    yz_rotation_deg: float = 0.0,
    mount_height_m: float = 0.0,
    ground_calibration_frames: int = 0,
    ground_band_m: float = 0.25,
    min_snr_db: float = 10.0,
) -> tuple[tuple[Any, ...], GroundPlane | None]:
    source_frames = tuple(frames)
    ground_plane = None
    if ground_calibration_frames > 0:
        ground_plane = calibrate_ground_plane(
            source_frames,
            yz_rotation_deg=yz_rotation_deg,
            mount_height_m=mount_height_m,
            frame_count=ground_calibration_frames,
            ground_band_m=ground_band_m,
            min_snr_db=min_snr_db,
        )
    transformed = tuple(
        transform_frame_to_world(
            frame,
            yz_rotation_deg=yz_rotation_deg,
            mount_height_m=mount_height_m,
            ground_plane=ground_plane,
        )
        for frame in source_frames
    )
    return transformed, ground_plane


def _format_ground_plane_status(ground_plane: GroundPlane | None) -> str:
    if ground_plane is None:
        return "地面: 未标定"
    return (
        f"地面: 点={ground_plane.point_count} RMSE={ground_plane.rmse_m:.3f}m "
        f"坡度x={ground_plane.x_slope:.3f} y={ground_plane.y_slope:.3f}"
    )


def _replace_tracker_z_roi(config: TrackerConfig, *, z_min_m: float, z_max_m: float) -> TrackerConfig:
    if z_min_m >= z_max_m:
        raise ValueError("Z 下限必须小于 Z 上限")
    return replace(config, z_min_m=float(z_min_m), z_max_m=float(z_max_m))


def _format_operator_mark_status(
    operator_marks: tuple[OperatorMark, ...] | list[OperatorMark],
    *,
    frame_index: int,
) -> str:
    labels = [mark.mark for mark in operator_marks if mark.radar_frame_index == frame_index]
    return f"人工标注: {' / '.join(labels)}" if labels else "人工标注: 无"


def _load_operator_marks_for_recording(recording: Any) -> tuple[OperatorMark, ...]:
    return load_operator_marks(Path(recording.motion_path).parent / "operator_marks.csv")


def _operator_mark_color(mark: OperatorMark) -> str:
    if "跌倒" in mark.mark:
        return "#ef4444"
    if "蹲" in mark.mark:
        return "#f59e0b"
    if "站" in mark.mark:
        return "#22c55e"
    return "#22d3ee"


@dataclass(frozen=True)
class TimelineSeries:
    frame_indices: tuple[int, ...]
    height_z_m: tuple[float | None, ...]
    motion_counts: tuple[int, ...]
    max_velocity_m_s: tuple[float, ...]
    alert_indices: tuple[int, ...]
    operator_marks: tuple[OperatorMark, ...] = ()


def _build_frame_alerts(
    views: list[VisualizationFrame] | tuple[VisualizationFrame, ...],
    fall_events: list[FallEvent] | tuple[FallEvent, ...],
    *,
    jump_threshold_m: float = 0.8,
) -> tuple[FrameAlert, ...]:
    fall_states_by_frame: dict[int, list[FallState]] = {}
    for event in fall_events:
        fall_states_by_frame.setdefault(event.frame_index, []).append(event.state)

    alerts: list[FrameAlert] = []
    previous = None
    for view in views:
        jump_m = None
        if previous is not None:
            distance_m = _target_xy_jump_m(previous.target, view.target)
            if distance_m is not None and distance_m >= jump_threshold_m:
                jump_m = distance_m
        if _target_has_xy(view.target):
            previous = view
        alerts.append(
            FrameAlert(
                jump_m=jump_m,
                fall_states=tuple(fall_states_by_frame.get(view.frame.frame_index, ())),
            )
        )
    return tuple(alerts)


def _build_timeline_series(
    views: list[VisualizationFrame] | tuple[VisualizationFrame, ...],
    alerts: tuple[FrameAlert, ...] | list[FrameAlert],
    *,
    operator_marks: tuple[OperatorMark, ...] | list[OperatorMark] = (),
) -> TimelineSeries:
    frame_indices = tuple(view.frame.frame_index for view in views)
    available_frames = set(frame_indices)
    return TimelineSeries(
        frame_indices=frame_indices,
        height_z_m=tuple(view.target.center_z_m if view.target.has_target else None for view in views),
        motion_counts=tuple(view.target.motion_count if view.target.has_target else 0 for view in views),
        max_velocity_m_s=tuple(view.target.max_abs_velocity_m_s if view.target.has_target else 0.0 for view in views),
        alert_indices=tuple(index for index, alert in enumerate(alerts) if alert.has_alert),
        operator_marks=tuple(
            mark
            for mark in operator_marks
            if mark.radar_frame_index in available_frames
        ),
    )


def _next_alert_index(alerts: tuple[FrameAlert, ...] | list[FrameAlert], *, current_index: int, direction: int) -> int | None:
    if not alerts or not any(alert.has_alert for alert in alerts):
        return None
    step = 1 if direction >= 0 else -1
    count = len(alerts)
    for offset in range(1, count + 1):
        index = (current_index + step * offset) % count
        if alerts[index].has_alert:
            return index
    return None


def _display_cluster_boxes(view: VisualizationFrame) -> tuple[DisplayClusterBox, ...]:
    boxes: list[DisplayClusterBox] = []
    for cluster in view.clusters:
        if not cluster.is_target:
            boxes.append(_cluster_box_display(cluster))
    for cluster in view.clusters:
        if cluster.is_target:
            boxes.append(_cluster_box_display(cluster))
    if view.target_cluster is not None and all(cluster is not view.target_cluster for cluster in view.clusters):
        boxes.append(_cluster_box_display(view.target_cluster))
    return tuple(boxes)


def _cluster_box_display(cluster: ClusterView) -> DisplayClusterBox:
    if cluster.is_target:
        return DisplayClusterBox(cluster=cluster, color="#f6d365", label="目标", line_width=2.2)
    return DisplayClusterBox(cluster=cluster, color="#7aa2f7", label="聚类", line_width=1.2)


def _format_target_diagnosis(view: VisualizationFrame) -> str:
    return _model_format_target_diagnosis(view)


def _target_state_suffix(target) -> str:
    state = getattr(target, "state", None)
    if state == TargetState.TRACKING:
        return ""
    if state == TargetState.STATIC_LOCK:
        return " 静态锁定"
    if state == TargetState.HOLD:
        return " 保持"
    if state == TargetState.REACQUIRE:
        return " 重捕获"
    if not getattr(target, "held", False):
        return ""
    if getattr(target, "point_count", 0) > 0:
        return " 静态锁定"
    return " 保持"


def _target_has_xy(target) -> bool:
    return bool(getattr(target, "has_target", False)) and target.center_x_m is not None and target.center_y_m is not None


def _target_xy_jump_m(previous, current) -> float | None:
    if not _target_has_xy(previous) or not _target_has_xy(current):
        return None
    dx = current.center_x_m - previous.center_x_m
    dy = current.center_y_m - previous.center_y_m
    return (dx * dx + dy * dy) ** 0.5


def _fall_state_label(state: FallState) -> str:
    if state == FallState.SUSPECTED:
        return "疑似跌倒"
    if state == FallState.CONFIRMED:
        return "确认跌倒"
    if state == FallState.RECOVERED:
        return "跌倒恢复"
    return str(state)


def _fall_popup_message(alert: FrameAlert, *, frame_index: int | None, timestamp_ms: int | None) -> str | None:
    if FallState.CONFIRMED in alert.fall_states:
        title = "确认跌倒"
        action = "请立即查看现场"
    elif FallState.SUSPECTED in alert.fall_states:
        title = "疑似跌倒"
        action = "检测到快速高度下降"
    else:
        return None

    lines = [title, action]
    details = []
    if frame_index is not None:
        details.append(f"帧 {frame_index}")
    if timestamp_ms is not None:
        details.append(f"t={timestamp_ms}")
    if details:
        lines.append("  ".join(details))
    if alert.summary:
        lines.append(alert.summary)
    return "\n".join(lines)


class FallAlertPopup:
    HOLD_MS = 1000

    def __init__(
        self,
        root: tk.Tk,
        *,
        window_factory: Callable[[Any], Any] | None = None,
        label_factory: Callable[..., Any] | None = None,
    ):
        self.root = root
        self.window_factory = window_factory or tk.Toplevel
        self.label_factory = label_factory or tk.Label
        self.window = None
        self.label = None
        self._hide_after_id: str | None = None
        self._hide_generation = 0
        self._fallback_visible_until_s: float | None = None
        self._last_message: str | None = None

    def sync(self, alert: FrameAlert, *, frame_index: int | None, timestamp_ms: int | None) -> None:
        message = _fall_popup_message(alert, frame_index=frame_index, timestamp_ms=timestamp_ms)
        if message is None:
            if self._should_hold():
                self._show_message(self._last_message or "", lift=False)
                return
            self.hide()
            return
        self._last_message = message
        self._show_message(message, lift=True)
        self._schedule_hide()

    def hide(self) -> None:
        if self.window is not None:
            self.window.withdraw()
        self._cancel_scheduled_hide()
        self._fallback_visible_until_s = None
        self._last_message = None

    def destroy(self) -> None:
        if self.window is not None:
            self.window.destroy()
            self.window = None
            self.label = None
            self._cancel_scheduled_hide()
            self._fallback_visible_until_s = None
            self._last_message = None

    def _should_hold(self) -> bool:
        if self._last_message is None:
            return False
        if self._hide_after_id is not None:
            return True
        if self._fallback_visible_until_s is None:
            return False
        return time.monotonic() <= self._fallback_visible_until_s

    def _show_message(self, message: str, *, lift: bool) -> None:
        self._ensure_window()
        self.label.configure(text=message)
        self.window.deiconify()
        if lift:
            self.window.lift()

    def _schedule_hide(self) -> None:
        self._cancel_scheduled_hide()
        self._hide_generation += 1
        generation = self._hide_generation
        after = getattr(self.root, "after", None)
        if callable(after):
            self._hide_after_id = after(self.HOLD_MS, lambda: self._hide_from_timer(generation))
            self._fallback_visible_until_s = None
            return
        self._hide_after_id = None
        self._fallback_visible_until_s = time.monotonic() + self.HOLD_MS / 1000.0

    def _cancel_scheduled_hide(self) -> None:
        self._hide_generation += 1
        if self._hide_after_id is None:
            return
        after_cancel = getattr(self.root, "after_cancel", None)
        if callable(after_cancel):
            try:
                after_cancel(self._hide_after_id)
            except Exception:
                pass
        self._hide_after_id = None

    def _hide_from_timer(self, generation: int) -> None:
        if generation != self._hide_generation:
            return
        self._hide_after_id = None
        self.hide()

    def _ensure_window(self) -> None:
        if self.window is not None:
            return
        window = self.window_factory(self.root)
        window.title("跌倒告警")
        window.configure(bg="#7f1d1d")
        window.geometry("1240x980+24+24")
        window.resizable(False, False)
        try:
            window.attributes("-topmost", True)
        except Exception:
            pass
        window.protocol("WM_DELETE_WINDOW", self.hide)
        label = self.label_factory(
            window,
            text="",
            bg="#7f1d1d",
            fg="#fee2e2",
            font=("Microsoft YaHei UI", 17, "bold"),
            justify="center",
        )
        label.pack(fill="both", expand=True, padx=14, pady=12)
        self.window = window
        self.label = label


class DatadealerVisualizerApp:
    def __init__(
        self,
        root: tk.Tk,
        *,
        data_dir: Path,
        stem: str | None = None,
        config: TrackerConfig | None = None,
        fall_config: FallRuleConfig | None = None,
        yz_rotation_deg: float = 0.0,
        mount_height_m: float = 0.0,
        ground_calibration_frames: int = 0,
        ground_band_m: float = 0.25,
    ):
        self.root = root
        self.data_dir = data_dir
        self.config = config or TrackerConfig()
        self.fall_config = fall_config or FallRuleConfig()
        self.yz_rotation_deg = float(yz_rotation_deg)
        self.mount_height_m = float(mount_height_m)
        self.ground_calibration_frames = max(int(ground_calibration_frames), 0)
        self.ground_band_m = float(ground_band_m)
        self.ground_plane: GroundPlane | None = None
        self.bounds = bounds_from_config(self.config)
        self.frames: list[VisualizationFrame] = []
        self.operator_marks: tuple[OperatorMark, ...] = ()
        self.alerts: tuple[FrameAlert, ...] = ()
        self.timeline_series = TimelineSeries((), (), (), (), ())
        self.current_index = 0
        self.is_playing = False
        self.play_after_id: str | None = None

        self.stem_var = tk.StringVar(value=stem or "")
        self.status_var = tk.StringVar(value="未加载")
        self.target_var = tk.StringVar(value="目标: 无")
        self.diagnosis_var = tk.StringVar(value="诊断: 无")
        self.alert_var = tk.StringVar(value="异常: 无")
        self.manual_mark_var = tk.StringVar(value="人工标注: 无")
        self.play_text = tk.StringVar(value="播放")
        self.speed_var = tk.IntVar(value=1)
        self.frame_var = tk.IntVar(value=0)
        self.yz_rotation_var = tk.DoubleVar(value=self.yz_rotation_deg)
        self.mount_height_var = tk.DoubleVar(value=self.mount_height_m)
        self.z_min_var = tk.DoubleVar(value=self.config.z_min_m)
        self.z_max_var = tk.DoubleVar(value=self.config.z_max_m)
        self.ground_calibration_frames_var = tk.IntVar(value=self.ground_calibration_frames)
        self.ground_band_var = tk.DoubleVar(value=self.ground_band_m)
        self.ground_status_var = tk.StringVar(value="地面: 未标定")
        self._last_3d_elev = 24.0
        self._last_3d_azim = -58.0
        self._is_3d_interacting = False
        self._suppress_slider_callback = False
        self.fall_alert_popup: FallAlertPopup | None = None

        self._build_ui()
        self.fall_alert_popup = FallAlertPopup(self.root)
        self._load_stem_list()
        if not self.stem_var.get() and self.stem_combo["values"]:
            self.stem_var.set(str(self.stem_combo["values"][0]))
        self.load_selected_recording()

    def _build_ui(self) -> None:
        self.root.title("MRS6240 跌倒检测点云聚类可视化")
        self.root.geometry("1280x760")
        self.root.minsize(980, 660)

        top = ttk.Frame(self.root, padding=(10, 8))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(4, weight=1)
        top.columnconfigure(9, weight=1)

        ttk.Label(top, text="采集文件").grid(row=0, column=0, padx=(0, 6), sticky="w")
        self.stem_combo = ttk.Combobox(top, textvariable=self.stem_var, width=28, state="readonly")
        self.stem_combo.grid(row=0, column=1, padx=(0, 8), sticky="w")
        ttk.Button(top, text="加载", command=self.load_selected_recording).grid(row=0, column=2, padx=(0, 8))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=3, padx=(0, 12), sticky="w")
        ttk.Label(top, textvariable=self.target_var).grid(row=0, column=4, sticky="w")

        if self._supports_world_calibration_controls():
            ttk.Label(top, text="俯角deg").grid(row=1, column=0, pady=(8, 0), sticky="w")
            ttk.Spinbox(
                top,
                from_=-90,
                to=90,
                increment=1,
                width=7,
                textvariable=self.yz_rotation_var,
            ).grid(row=1, column=1, pady=(8, 0), padx=(0, 8), sticky="w")
            ttk.Label(top, text="安装高度m").grid(row=1, column=2, pady=(8, 0), sticky="w")
            ttk.Spinbox(
                top,
                from_=0,
                to=5,
                increment=0.05,
                width=7,
                textvariable=self.mount_height_var,
            ).grid(row=1, column=3, pady=(8, 0), padx=(0, 8), sticky="w")
            ttk.Label(top, text="标定帧数").grid(row=1, column=4, pady=(8, 0), sticky="e")
            ttk.Spinbox(
                top,
                from_=0,
                to=2000,
                increment=20,
                width=7,
                textvariable=self.ground_calibration_frames_var,
            ).grid(row=1, column=5, pady=(8, 0), padx=(4, 8), sticky="w")
            ttk.Label(top, text="地面带+/-m").grid(row=1, column=6, pady=(8, 0), sticky="w")
            ttk.Spinbox(
                top,
                from_=0.05,
                to=1.0,
                increment=0.05,
                width=7,
                textvariable=self.ground_band_var,
            ).grid(row=1, column=7, pady=(8, 0), padx=(0, 8), sticky="w")
            ttk.Button(top, text="应用/标定", command=self._apply_world_coordinate_settings).grid(
                row=1,
                column=8,
                pady=(8, 0),
                padx=(0, 10),
                sticky="w",
            )
            ttk.Label(top, textvariable=self.ground_status_var).grid(row=1, column=9, pady=(8, 0), sticky="w")
            ttk.Label(top, text="Z下限m").grid(row=2, column=0, pady=(8, 0), sticky="w")
            ttk.Spinbox(
                top,
                from_=-3,
                to=5,
                increment=0.05,
                width=7,
                textvariable=self.z_min_var,
            ).grid(row=2, column=1, pady=(8, 0), padx=(0, 8), sticky="w")
            ttk.Label(top, text="Z上限m").grid(row=2, column=2, pady=(8, 0), sticky="w")
            ttk.Spinbox(
                top,
                from_=-3,
                to=5,
                increment=0.05,
                width=7,
                textvariable=self.z_max_var,
            ).grid(row=2, column=3, pady=(8, 0), padx=(0, 8), sticky="w")

        plot_area = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        plot_area.grid(row=1, column=0, sticky="nsew")
        plot_area.columnconfigure(0, weight=1, uniform="plot")
        plot_area.columnconfigure(1, weight=1, uniform="plot")
        plot_area.rowconfigure(1, weight=1)
        plot_area.rowconfigure(2, weight=0)
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        ttk.Label(plot_area, text="XY 2D 俯视图  X: 雷达左侧为正 / Y: 雷达前方").grid(row=0, column=0, sticky="w")
        right_title = "3D 坐标轴视图  鼠标左键拖动旋转 / Z: 现实高度" if self._uses_3d_view() else "XZ / YZ 2D 侧视图  Z: 现实高度"
        ttk.Label(plot_area, text=right_title).grid(row=0, column=1, sticky="w")

        self.xy_canvas = tk.Canvas(plot_area, bg="#0b0f14", highlightthickness=1, highlightbackground="#29323d")
        self.xy_canvas.grid(row=1, column=0, padx=(0, 6), pady=(6, 0), sticky="nsew")
        self.figure = Figure(figsize=(5.4, 4.2), dpi=100, facecolor="#0b0f14")
        if self._uses_3d_view():
            self.ax3d = self.figure.add_subplot(111, projection="3d")
        else:
            self.ax_xz = self.figure.add_subplot(211)
            self.ax_yz = self.figure.add_subplot(212)
        self.figure_canvas = FigureCanvasTkAgg(self.figure, master=plot_area)
        self.figure_canvas.get_tk_widget().grid(row=1, column=1, padx=(6, 0), pady=(6, 0), sticky="nsew")
        if self._uses_3d_view():
            self.figure_canvas.mpl_connect("button_press_event", self._on_3d_mouse_press)
            self.figure_canvas.mpl_connect("button_release_event", self._on_3d_mouse_release)

        self.timeline_figure = Figure(figsize=(10.8, 1.45), dpi=100, facecolor="#0b0f14")
        self.timeline_ax_height = self.timeline_figure.add_subplot(311)
        self.timeline_ax_motion = self.timeline_figure.add_subplot(312, sharex=self.timeline_ax_height)
        self.timeline_ax_velocity = self.timeline_figure.add_subplot(313, sharex=self.timeline_ax_height)
        self.timeline_axes = (self.timeline_ax_height, self.timeline_ax_motion, self.timeline_ax_velocity)
        self.timeline_canvas = FigureCanvasTkAgg(self.timeline_figure, master=plot_area)
        self.timeline_canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, pady=(8, 0), sticky="ew")

        controls = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(4, weight=1)

        ttk.Button(controls, text="上一帧", command=self.previous_frame).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(controls, textvariable=self.play_text, command=self.toggle_play).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(controls, text="下一帧", command=self.next_frame).grid(row=0, column=2, padx=(0, 10))
        ttk.Button(controls, text="上一异常", command=self.previous_alert).grid(row=0, column=3, padx=(0, 6))
        ttk.Button(controls, text="下一异常", command=self.next_alert).grid(row=0, column=4, padx=(0, 14))
        ttk.Label(controls, text="速度").grid(row=0, column=5, sticky="e")
        ttk.Scale(controls, from_=1, to=5, variable=self.speed_var, orient="horizontal", length=110).grid(
            row=0, column=6, sticky="w", padx=(6, 16)
        )
        self.frame_scale = ttk.Scale(controls, from_=0, to=0, orient="horizontal", command=self._on_slider)
        self.frame_scale.grid(row=0, column=7, sticky="ew")
        controls.columnconfigure(7, weight=1)

        legend = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        legend.grid(row=3, column=0, sticky="ew")
        legend.columnconfigure(0, weight=1)
        ttk.Label(
            legend,
            text="颜色: 灰=背景/小聚类  蓝点/蓝框=普通聚类  橙=运动点  黄框=人体目标聚类  红十字=目标中心",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(legend, textvariable=self.diagnosis_var, foreground="#d97706").grid(row=1, column=0, sticky="w", pady=(3, 0))
        ttk.Label(legend, textvariable=self.alert_var, foreground="#dc2626").grid(row=2, column=0, sticky="w", pady=(3, 0))
        ttk.Label(legend, textvariable=self.manual_mark_var, foreground="#0891b2").grid(row=3, column=0, sticky="w", pady=(3, 0))

        self.root.bind("<space>", lambda _event: self.toggle_play())
        self.root.bind("<Left>", lambda _event: self.previous_frame())
        self.root.bind("<Right>", lambda _event: self.next_frame())
        self.xy_canvas.bind("<Configure>", lambda _event: self.draw_current_frame())

    def _uses_3d_view(self) -> bool:
        return False

    def _supports_world_calibration_controls(self) -> bool:
        return True

    def _load_stem_list(self) -> None:
        stems = list(iter_sample_stems(self.data_dir)) if self.data_dir.exists() else []
        self.stem_combo["values"] = stems

    def load_selected_recording(self) -> None:
        stem = self.stem_var.get() or None
        try:
            recording = load_datadealer_recording(self.data_dir, stem=stem)
            operator_marks = _load_operator_marks_for_recording(recording)
            transformed_frames, ground_plane = _prepare_frames_for_visualization(
                recording.frames,
                yz_rotation_deg=self.yz_rotation_deg,
                mount_height_m=self.mount_height_m,
                ground_calibration_frames=self.ground_calibration_frames,
                ground_band_m=self.ground_band_m,
                min_snr_db=self.config.min_snr_db,
            )
        except Exception as exc:
            self.frames = []
            self.operator_marks = ()
            self.alerts = ()
            self.timeline_series = TimelineSeries((), (), (), (), ())
            self.status_var.set("加载失败")
            self.ground_status_var.set("地面: 标定失败")
            self.target_var.set("目标: 无")
            self.diagnosis_var.set("诊断: 无")
            self.alert_var.set("异常: 无")
            self.manual_mark_var.set("人工标注: 无")
            self._clear_canvases()
            messagebox.showerror("加载失败", str(exc))
            return

        self.ground_plane = ground_plane
        self.operator_marks = operator_marks
        self.frames = build_visualization_frames(transformed_frames, self.config)
        fall_events = detect_falls([view.target for view in self.frames], self.fall_config)
        self.alerts = _build_frame_alerts(self.frames, fall_events)
        self.timeline_series = _build_timeline_series(
            self.frames,
            self.alerts,
            operator_marks=self.operator_marks,
        )
        self.current_index = 0
        max_index = max(len(self.frames) - 1, 0)
        self.frame_scale.configure(from_=0, to=max_index)
        self._set_frame_scale(0)
        self.status_var.set(
            f"{recording.stem}  帧={len(self.frames)}  时长={recording.duration_s:.1f}s  "
            f"角度={self.yz_rotation_deg:g}deg  高度={self.mount_height_m:g}m  "
            f"Z={self.config.z_min_m:g}..{self.config.z_max_m:g}m  标注={len(self.operator_marks)}"
        )
        self.ground_status_var.set(_format_ground_plane_status(self.ground_plane))
        self.draw_current_frame()

    def _apply_world_coordinate_settings(self) -> None:
        try:
            yz_rotation_deg = float(self.yz_rotation_var.get())
            mount_height_m = float(self.mount_height_var.get())
            z_min_m = float(self.z_min_var.get())
            z_max_m = float(self.z_max_var.get())
            ground_calibration_frames = int(self.ground_calibration_frames_var.get())
            ground_band_m = float(self.ground_band_var.get())
            if mount_height_m < 0:
                raise ValueError("安装高度不能为负数")
            if ground_calibration_frames < 0:
                raise ValueError("标定帧数不能为负数")
            if ground_band_m <= 0:
                raise ValueError("地面带宽必须大于 0")
            updated_config = _replace_tracker_z_roi(
                self.config,
                z_min_m=z_min_m,
                z_max_m=z_max_m,
            )
        except (tk.TclError, TypeError, ValueError) as exc:
            messagebox.showerror("坐标参数无效", str(exc))
            return

        self.yz_rotation_deg = yz_rotation_deg
        self.mount_height_m = mount_height_m
        self.config = updated_config
        self.bounds = bounds_from_config(self.config)
        self.ground_calibration_frames = ground_calibration_frames
        self.ground_band_m = ground_band_m
        self.load_selected_recording()

    def toggle_play(self) -> None:
        self.is_playing = not self.is_playing
        self.play_text.set("暂停" if self.is_playing else "播放")
        if self.is_playing:
            self._schedule_next()
        elif self.play_after_id is not None:
            self.root.after_cancel(self.play_after_id)
            self.play_after_id = None

    def previous_frame(self) -> None:
        if not self.frames:
            return
        self.current_index = max(self.current_index - 1, 0)
        self._set_frame_scale(self.current_index)
        self.draw_current_frame()

    def next_frame(self) -> None:
        if not self.frames:
            return
        self.current_index = min(self.current_index + 1, len(self.frames) - 1)
        self._set_frame_scale(self.current_index)
        self.draw_current_frame()

    def previous_alert(self) -> None:
        self._go_to_alert(direction=-1)

    def next_alert(self) -> None:
        self._go_to_alert(direction=1)

    def _go_to_alert(self, *, direction: int) -> None:
        index = _next_alert_index(self.alerts, current_index=self.current_index, direction=direction)
        if index is None:
            return
        self.current_index = index
        self._set_frame_scale(self.current_index)
        self.draw_current_frame()

    def _on_slider(self, value: str) -> None:
        if getattr(self, "_suppress_slider_callback", False):
            return
        if not self.frames:
            return
        self.current_index = min(max(int(float(value)), 0), len(self.frames) - 1)
        self.draw_current_frame()

    def _set_frame_scale(self, value: int) -> None:
        self._suppress_slider_callback = True
        try:
            self.frame_scale.set(value)
        finally:
            self._suppress_slider_callback = False

    def _schedule_next(self) -> None:
        if not self.is_playing:
            return
        if not self.frames:
            self.toggle_play()
            return
        if self._is_3d_interacting:
            delay_ms = max(20, int(160 / max(self.speed_var.get(), 1)))
            self.play_after_id = self.root.after(delay_ms, self._schedule_next)
            return
        if self.current_index >= len(self.frames) - 1:
            self.current_index = 0
        else:
            self.current_index += 1
        self._set_frame_scale(self.current_index)
        self.draw_current_frame()
        delay_ms = max(20, int(160 / max(self.speed_var.get(), 1)))
        self.play_after_id = self.root.after(delay_ms, self._schedule_next)

    def _on_3d_mouse_press(self, event) -> None:
        if hasattr(self, "ax3d") and event.inaxes is self.ax3d:
            self._is_3d_interacting = True

    def _on_3d_mouse_release(self, _event) -> None:
        if self._is_3d_interacting:
            self._capture_3d_view()
        self._is_3d_interacting = False

    def _capture_3d_view(self) -> None:
        if hasattr(self, "ax3d") and hasattr(self.ax3d, "elev") and hasattr(self.ax3d, "azim"):
            self._last_3d_elev = float(self.ax3d.elev)
            self._last_3d_azim = float(self.ax3d.azim)

    def draw_current_frame(self) -> None:
        if not self.frames:
            self._clear_canvases()
            return
        view = self.frames[self.current_index]
        self._update_text(view)
        self._draw_xy(view)
        self._draw_3d(view)
        self._draw_timeline()
        self._sync_fall_alert_popup(view)

    def _update_text(self, view: VisualizationFrame) -> None:
        target = view.target
        status = (
            f"帧 {self.current_index + 1}/{len(self.frames)}  "
            f"{_format_source_frame_label(view.frame)}  "
            f"t={view.frame.timestamp_ms}  "
            f"聚类={len(view.clusters)}  "
            f"原始点={view.frame.presence_count + view.frame.motion_count}  "
            f"过滤点={len(view.filtered_points)}"
        )
        self.status_var.set(status)
        self.diagnosis_var.set(_format_target_diagnosis(view))
        alert = self._current_alert()
        self.alert_var.set(f"异常: {alert.summary}" if alert.has_alert else "异常: 无")
        self.manual_mark_var.set(
            _format_operator_mark_status(
                self.operator_marks,
                frame_index=view.frame.frame_index,
            )
        )
        if not target.has_target:
            if getattr(target, "state", None) == TargetState.REACQUIRE:
                self.target_var.set("目标: 重捕获中")
            else:
                self.target_var.set("目标: 无")
            return
        state_suffix = _target_state_suffix(target)
        self.target_var.set(
            "目标: "
            f"x={target.center_x_m:.2f}m  y={target.center_y_m:.2f}m  z={target.center_z_m:.2f}m  "
            f"h={target.height_m:.2f}m  点={target.point_count}  运动={target.motion_count}{state_suffix}"
        )

    def _draw_xy(self, view: VisualizationFrame) -> None:
        canvas = self.xy_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 200)
        height = max(canvas.winfo_height(), 200)
        self._draw_xy_grid(canvas, width, height)

        for point in view.filtered_points:
            self._draw_xy_point(canvas, point, width, height, fill="#4c5663", radius=2)
        for cluster in view.clusters:
            self._draw_xy_cluster(canvas, cluster, width, height)
        for box in _display_cluster_boxes(view):
            self._draw_xy_cluster_box(canvas, box.cluster, width, height, fill=box.color, line_width=box.line_width, label=box.label)
        self._draw_xy_alert_badge(canvas, width)
        self._draw_xy_tracking_debug(canvas, view, width, height)
        if view.target.has_target:
            self._draw_xy_target_center(canvas, view.target.center_x_m, view.target.center_y_m, width, height)

    def _draw_3d(self, view: VisualizationFrame) -> None:
        if hasattr(self, "ax_xz") and hasattr(self, "ax_yz") and not hasattr(self, "ax3d"):
            self._draw_side_projection_views(view)
            return

        self._capture_3d_view()

        self.ax3d.clear()
        self._setup_3d_axes()
        alert = self._current_alert()
        if alert.has_alert:
            self.ax3d.set_title(alert.summary, color="#f87171", fontsize=10, pad=12)
        self._scatter_3d_points(view.filtered_points, color="#4c5663", size=8, alpha=0.45)

        for cluster in view.clusters:
            presence = [point for point in cluster.points if point.kind != "motion"]
            motion = [point for point in cluster.points if point.kind == "motion"]
            base_color = "#f59e0b" if cluster.is_target else "#58a6ff"
            self._scatter_3d_points(presence, color=base_color, size=22 if cluster.is_target else 16, alpha=0.9)
            self._scatter_3d_points(motion, color="#f97316", size=32 if cluster.is_target else 24, alpha=0.95)

        for box in _display_cluster_boxes(view):
            self._draw_3d_cluster_box(box.cluster, color=box.color, line_width=box.line_width, label=box.label)
        if view.target.has_target and view.target.center_x_m is not None and view.target.center_y_m is not None and view.target.center_z_m is not None:
            self.ax3d.scatter(
                [view.target.center_x_m],
                [view.target.center_y_m],
                [view.target.center_z_m],
                c=["#ff4d4d"],
                s=56,
                marker="x",
                depthshade=False,
            )

        self.figure_canvas.draw_idle()

    def _draw_side_projection_views(self, view: VisualizationFrame) -> None:
        self.ax_xz.clear()
        self.ax_yz.clear()
        self._setup_projection_axis(
            self.ax_xz,
            title="XZ 侧视图",
            xlabel="X left + (m)",
            xlim=(self.bounds.x_min_m, self.bounds.x_max_m),
        )
        self._setup_projection_axis(
            self.ax_yz,
            title="YZ 侧视图",
            xlabel="Y forward (m)",
            xlim=(self.bounds.y_min_m, self.bounds.y_max_m),
        )

        alert = self._current_alert()
        if alert.has_alert:
            self.ax_xz.set_title(f"XZ 侧视图  {alert.summary}", color="#f87171", fontsize=10, pad=7)

        self._scatter_projection_points(self.ax_xz, view.filtered_points, horizontal="x", color="#4c5663", size=8, alpha=0.45)
        self._scatter_projection_points(self.ax_yz, view.filtered_points, horizontal="y", color="#4c5663", size=8, alpha=0.45)

        for cluster in view.clusters:
            presence = [point for point in cluster.points if point.kind != "motion"]
            motion = [point for point in cluster.points if point.kind == "motion"]
            base_color = "#f59e0b" if cluster.is_target else "#58a6ff"
            self._scatter_projection_points(self.ax_xz, presence, horizontal="x", color=base_color, size=22 if cluster.is_target else 16, alpha=0.9)
            self._scatter_projection_points(self.ax_yz, presence, horizontal="y", color=base_color, size=22 if cluster.is_target else 16, alpha=0.9)
            self._scatter_projection_points(self.ax_xz, motion, horizontal="x", color="#f97316", size=32 if cluster.is_target else 24, alpha=0.95)
            self._scatter_projection_points(self.ax_yz, motion, horizontal="y", color="#f97316", size=32 if cluster.is_target else 24, alpha=0.95)

        for box in _display_cluster_boxes(view):
            self._draw_projection_cluster_box(
                self.ax_xz,
                box.cluster,
                horizontal="x",
                color=box.color,
                line_width=box.line_width,
                label=box.label,
            )
            self._draw_projection_cluster_box(
                self.ax_yz,
                box.cluster,
                horizontal="y",
                color=box.color,
                line_width=box.line_width,
                label=box.label,
            )

        target = view.target
        if target.has_target and target.center_x_m is not None and target.center_y_m is not None and target.center_z_m is not None:
            self.ax_xz.scatter([target.center_x_m], [target.center_z_m], c=["#ff4d4d"], s=56, marker="x")
            self.ax_yz.scatter([target.center_y_m], [target.center_z_m], c=["#ff4d4d"], s=56, marker="x")

        self.figure.subplots_adjust(left=0.10, right=0.97, top=0.94, bottom=0.12, hspace=0.42)
        self.figure_canvas.draw_idle()

    def _draw_timeline(self) -> None:
        axes = self.timeline_axes
        for axis in axes:
            axis.clear()
            self._setup_timeline_axis(axis)

        series = self.timeline_series
        if not series.frame_indices:
            self.timeline_canvas.draw_idle()
            return

        x_values = series.frame_indices
        current_frame = x_values[self.current_index] if 0 <= self.current_index < len(x_values) else x_values[0]
        self.timeline_ax_height.plot(x_values, series.height_z_m, color="#38bdf8", linewidth=1.4)
        self.timeline_ax_motion.plot(x_values, series.motion_counts, color="#f97316", linewidth=1.2)
        self.timeline_ax_velocity.plot(x_values, series.max_velocity_m_s, color="#a78bfa", linewidth=1.2)

        self.timeline_ax_height.set_ylabel("Z m", color="#cbd5e1", rotation=0, labelpad=22, fontsize=8)
        self.timeline_ax_motion.set_ylabel("Motion", color="#cbd5e1", rotation=0, labelpad=22, fontsize=8)
        self.timeline_ax_velocity.set_ylabel("Vel", color="#cbd5e1", rotation=0, labelpad=22, fontsize=8)
        self.timeline_ax_velocity.set_xlabel("Frame", color="#cbd5e1", fontsize=8)

        for index in series.alert_indices:
            if 0 <= index < len(x_values):
                alert_frame = x_values[index]
                for axis in axes:
                    axis.axvline(alert_frame, color="#ef4444", linewidth=1.0, alpha=0.85)
        mark_counts_by_frame: dict[int, int] = {}
        for mark in series.operator_marks:
            color = _operator_mark_color(mark)
            for axis in axes:
                axis.axvline(
                    mark.radar_frame_index,
                    color=color,
                    linewidth=0.9,
                    linestyle="--",
                    alpha=0.8,
                )
            same_frame_index = mark_counts_by_frame.get(mark.radar_frame_index, 0)
            mark_counts_by_frame[mark.radar_frame_index] = same_frame_index + 1
            self.timeline_ax_height.text(
                mark.radar_frame_index,
                max(0.12, 0.96 - same_frame_index * 0.22),
                mark.mark,
                transform=self.timeline_ax_height.get_xaxis_transform(),
                color=color,
                fontsize=7,
                rotation=90,
                ha="right",
                va="top",
            )
        for axis in axes:
            axis.axvline(current_frame, color="#f8fafc", linewidth=1.0, alpha=0.9)
            axis.margins(x=0.01)

        self.timeline_figure.subplots_adjust(left=0.06, right=0.99, top=0.96, bottom=0.18, hspace=0.12)
        self.timeline_canvas.draw_idle()

    def _setup_timeline_axis(self, axis) -> None:
        axis.set_facecolor("#0b0f14")
        axis.grid(True, color="#1e2833", linewidth=0.6)
        axis.tick_params(colors="#94a3b8", labelsize=7)
        for spine in axis.spines.values():
            spine.set_color("#334155")

    def _setup_3d_axes(self) -> None:
        self.ax3d.set_facecolor("#0b0f14")
        self.ax3d.set_xlim(self.bounds.x_min_m, self.bounds.x_max_m)
        self.ax3d.set_ylim(self.bounds.y_min_m, self.bounds.y_max_m)
        self.ax3d.set_zlim(self.bounds.z_min_m, self.bounds.z_max_m)
        self.ax3d.set_xlabel("X left + (m)", color="#cbd5e1", labelpad=8)
        self.ax3d.set_ylabel("Y forward (m)", color="#cbd5e1", labelpad=8)
        self.ax3d.set_zlabel("Z height (m)", color="#cbd5e1", labelpad=8)
        self.ax3d.tick_params(colors="#94a3b8", labelsize=8)
        self.ax3d.view_init(elev=self._last_3d_elev, azim=self._last_3d_azim)
        self.ax3d.grid(True)
        try:
            self.ax3d.set_box_aspect(
                (
                    self.bounds.x_max_m - self.bounds.x_min_m,
                    self.bounds.y_max_m - self.bounds.y_min_m,
                    self.bounds.z_max_m - self.bounds.z_min_m,
                )
            )
        except Exception:
            pass
        for axis in (self.ax3d.xaxis, self.ax3d.yaxis, self.ax3d.zaxis):
            axis._axinfo.get("grid", {})["color"] = "#243244"
            axis._axinfo.get("tick", {})["color"] = "#94a3b8"
            axis._axinfo.get("axisline", {})["color"] = "#64748b"
            axis.pane.set_facecolor("#101820")
            axis.pane.set_edgecolor("#334155")

    def _setup_projection_axis(self, axis, *, title: str, xlabel: str, xlim: tuple[float, float]) -> None:
        axis.set_facecolor("#0b0f14")
        axis.set_title(title, color="#cbd5e1", fontsize=10, pad=7)
        axis.set_xlim(*xlim)
        axis.set_ylim(self.bounds.z_min_m, self.bounds.z_max_m)
        axis.set_xlabel(xlabel, color="#cbd5e1", fontsize=8)
        axis.set_ylabel("Z height (m)", color="#cbd5e1", fontsize=8)
        axis.grid(True, color="#1e2833", linewidth=0.6)
        axis.axhline(0.0, color="#22c55e", linewidth=1.0, linestyle="--", alpha=0.8)
        axis.tick_params(colors="#94a3b8", labelsize=7)
        for spine in axis.spines.values():
            spine.set_color("#334155")

    def _scatter_projection_points(
        self,
        axis,
        points: tuple[Point3D, ...] | list[Point3D],
        *,
        horizontal: str,
        color: str,
        size: int,
        alpha: float,
    ) -> None:
        if not points:
            return
        if horizontal == "x":
            horizontal_values = [point.x_m for point in points]
        else:
            horizontal_values = [point.y_m for point in points]
        axis.scatter(
            horizontal_values,
            [point.z_m for point in points],
            c=[color],
            s=size,
            alpha=alpha,
        )

    def _draw_projection_cluster_box(
        self,
        axis,
        cluster: ClusterView,
        *,
        horizontal: str,
        color: str,
        line_width: float,
        label: str,
    ) -> None:
        if horizontal == "x":
            low = cluster.x_min_m
            high = cluster.x_max_m
            text_x = cluster.x_min_m
        else:
            low = cluster.y_min_m
            high = cluster.y_max_m
            text_x = cluster.y_min_m
        z_low = cluster.z_min_m
        z_high = cluster.z_max_m
        axis.plot(
            [low, high, high, low, low],
            [z_low, z_low, z_high, z_high, z_low],
            color=color,
            linewidth=line_width,
        )
        axis.text(
            text_x,
            z_high,
            f"{label} {cluster.point_count}/{cluster.motion_count}",
            color=color,
            fontsize=8,
            va="bottom",
        )

    def _scatter_3d_points(self, points: tuple[Point3D, ...] | list[Point3D], *, color: str, size: int, alpha: float) -> None:
        if not points:
            return
        self.ax3d.scatter(
            [point.x_m for point in points],
            [point.y_m for point in points],
            [point.z_m for point in points],
            c=[color],
            s=size,
            alpha=alpha,
            depthshade=False,
        )

    def _draw_3d_cluster_box(self, cluster: ClusterView, *, color: str, line_width: float, label: str) -> None:
        corners = {
            (x, y, z): (x, y, z)
            for x in (cluster.x_min_m, cluster.x_max_m)
            for y in (cluster.y_min_m, cluster.y_max_m)
            for z in (cluster.z_min_m, cluster.z_max_m)
        }
        edges = [
            ((cluster.x_min_m, cluster.y_min_m, cluster.z_min_m), (cluster.x_max_m, cluster.y_min_m, cluster.z_min_m)),
            ((cluster.x_min_m, cluster.y_max_m, cluster.z_min_m), (cluster.x_max_m, cluster.y_max_m, cluster.z_min_m)),
            ((cluster.x_min_m, cluster.y_min_m, cluster.z_max_m), (cluster.x_max_m, cluster.y_min_m, cluster.z_max_m)),
            ((cluster.x_min_m, cluster.y_max_m, cluster.z_max_m), (cluster.x_max_m, cluster.y_max_m, cluster.z_max_m)),
            ((cluster.x_min_m, cluster.y_min_m, cluster.z_min_m), (cluster.x_min_m, cluster.y_max_m, cluster.z_min_m)),
            ((cluster.x_max_m, cluster.y_min_m, cluster.z_min_m), (cluster.x_max_m, cluster.y_max_m, cluster.z_min_m)),
            ((cluster.x_min_m, cluster.y_min_m, cluster.z_max_m), (cluster.x_min_m, cluster.y_max_m, cluster.z_max_m)),
            ((cluster.x_max_m, cluster.y_min_m, cluster.z_max_m), (cluster.x_max_m, cluster.y_max_m, cluster.z_max_m)),
            ((cluster.x_min_m, cluster.y_min_m, cluster.z_min_m), (cluster.x_min_m, cluster.y_min_m, cluster.z_max_m)),
            ((cluster.x_max_m, cluster.y_min_m, cluster.z_min_m), (cluster.x_max_m, cluster.y_min_m, cluster.z_max_m)),
            ((cluster.x_min_m, cluster.y_max_m, cluster.z_min_m), (cluster.x_min_m, cluster.y_max_m, cluster.z_max_m)),
            ((cluster.x_max_m, cluster.y_max_m, cluster.z_min_m), (cluster.x_max_m, cluster.y_max_m, cluster.z_max_m)),
        ]
        for start, end in edges:
            sx, sy, sz = corners[start]
            ex, ey, ez = corners[end]
            self.ax3d.plot([sx, ex], [sy, ey], [sz, ez], color=color, linewidth=line_width)
        self.ax3d.text(
            cluster.center_x_m,
            cluster.center_y_m,
            cluster.z_max_m,
            f"{label} {cluster.point_count}/{cluster.motion_count}",
            color=color,
            fontsize=8,
        )

    def _draw_xy_grid(self, canvas: tk.Canvas, width: int, height: int) -> None:
        for x in _meter_ticks(self.bounds.x_min_m, self.bounds.x_max_m):
            p1 = Point3D(x, self.bounds.y_min_m, 0, 0, 0, "grid")
            p2 = Point3D(x, self.bounds.y_max_m, 0, 0, 0, "grid")
            x1, y1 = project_xy(p1, self.bounds, width_px=width, height_px=height)
            x2, y2 = project_xy(p2, self.bounds, width_px=width, height_px=height)
            canvas.create_line(x1, y1, x2, y2, fill="#1e2833")
            canvas.create_text(x1, height - 10, text=f"{x:g}m", fill="#74808f", font=("Microsoft YaHei UI", 8))
        for y in _meter_ticks(self.bounds.y_min_m, self.bounds.y_max_m):
            p1 = Point3D(self.bounds.x_min_m, y, 0, 0, 0, "grid")
            p2 = Point3D(self.bounds.x_max_m, y, 0, 0, 0, "grid")
            x1, y1 = project_xy(p1, self.bounds, width_px=width, height_px=height)
            x2, y2 = project_xy(p2, self.bounds, width_px=width, height_px=height)
            canvas.create_line(x1, y1, x2, y2, fill="#1e2833")
            canvas.create_text(16, y1, text=f"{y:g}m", fill="#74808f", font=("Microsoft YaHei UI", 8))
        canvas.create_text(width / 2, height - 26, text="雷达位置 / X 左侧为正", fill="#9aa7b6", font=("Microsoft YaHei UI", 9))
        canvas.create_text(width - 52, 18, text="Y 前方", fill="#9aa7b6", font=("Microsoft YaHei UI", 9))

    def _draw_xy_cluster(self, canvas: tk.Canvas, cluster: ClusterView, width: int, height: int) -> None:
        fill = "#f59e0b" if cluster.is_target else "#58a6ff"
        radius = 4 if cluster.is_target else 3
        for point in cluster.points:
            point_fill = "#f97316" if point.kind == "motion" else fill
            self._draw_xy_point(canvas, point, width, height, fill=point_fill, radius=radius)

    def _draw_xy_point(self, canvas: tk.Canvas, point: Point3D, width: int, height: int, *, fill: str, radius: int) -> None:
        x, y = project_xy(point, self.bounds, width_px=width, height_px=height)
        canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=fill, outline="")

    def _draw_xy_cluster_box(
        self,
        canvas: tk.Canvas,
        cluster: ClusterView,
        width: int,
        height: int,
        *,
        fill: str,
        line_width: float,
        label: str,
    ) -> None:
        p1 = Point3D(cluster.x_min_m, cluster.y_min_m, 0, 0, 0, "box")
        p2 = Point3D(cluster.x_max_m, cluster.y_max_m, 0, 0, 0, "box")
        x1, y1 = project_xy(p1, self.bounds, width_px=width, height_px=height)
        x2, y2 = project_xy(p2, self.bounds, width_px=width, height_px=height)
        canvas.create_rectangle(x1, y2, x2, y1, outline=fill, width=line_width)
        canvas.create_text(
            x1 + 4,
            y2 - 12,
            anchor="w",
            text=f"{label} 点={cluster.point_count} 动={cluster.motion_count}",
            fill=fill,
            font=("Microsoft YaHei UI", 9, "bold"),
        )

    def _draw_xy_tracking_debug(self, canvas: tk.Canvas, view: VisualizationFrame, width: int, height: int) -> None:
        if view.previous_target_x_m is not None and view.previous_target_y_m is not None:
            previous = Point3D(view.previous_target_x_m, view.previous_target_y_m, 0, 0, 0, "previous")
            cx, cy = project_xy(previous, self.bounds, width_px=width, height_px=height)
            radius_px = self._meters_to_xy_pixels(view.track_gate_m, width, height)
            canvas.create_oval(
                cx - radius_px,
                cy - radius_px,
                cx + radius_px,
                cy + radius_px,
                outline="#a78bfa",
                width=1,
                dash=(4, 3),
            )
            canvas.create_line(cx - 5, cy, cx + 5, cy, fill="#c084fc", width=1)
            canvas.create_line(cx, cy - 5, cx, cy + 5, fill="#c084fc", width=1)
            canvas.create_text(cx + 8, cy - 8, anchor="sw", text="上一帧", fill="#c084fc", font=("Microsoft YaHei UI", 8))
            if view.target.has_target and view.target.center_x_m is not None and view.target.center_y_m is not None:
                tx, ty = project_xy(
                    Point3D(view.target.center_x_m, view.target.center_y_m, 0, 0, 0, "target"),
                    self.bounds,
                    width_px=width,
                    height_px=height,
                )
                canvas.create_line(cx, cy, tx, ty, fill="#f6d365", width=1, dash=(2, 2))

        for candidate in view.debug_candidates:
            x, y = project_xy(
                Point3D(candidate.center_x_m, candidate.center_y_m, 0, 0, 0, "candidate"),
                self.bounds,
                width_px=width,
                height_px=height,
            )
            fill = "#f6d365" if candidate.selected else "#94a3b8" if candidate.within_gate else "#64748b"
            canvas.create_text(
                x + 6,
                y + 8,
                anchor="nw",
                text=format_debug_candidate_label(candidate),
                fill=fill,
                font=("Consolas", 8),
            )

    def _draw_xy_alert_badge(self, canvas: tk.Canvas, width: int) -> None:
        alert = self._current_alert()
        if not alert.has_alert:
            return
        text = alert.summary
        x1, y1, x2, y2 = 12, 12, min(width - 12, 220), 42
        canvas.create_rectangle(x1, y1, x2, y2, fill="#3f1111", outline="#ef4444", width=1)
        canvas.create_text(x1 + 10, y1 + 15, anchor="w", text=text, fill="#fecaca", font=("Microsoft YaHei UI", 9, "bold"))

    def _meters_to_xy_pixels(self, meters: float, width: int, height: int) -> float:
        plot_w = max(width - 48, 1)
        plot_h = max(height - 48, 1)
        px_per_x_meter = plot_w / max(self.bounds.x_max_m - self.bounds.x_min_m, 0.001)
        px_per_y_meter = plot_h / max(self.bounds.y_max_m - self.bounds.y_min_m, 0.001)
        return meters * min(px_per_x_meter, px_per_y_meter)

    def _draw_xy_target_center(self, canvas: tk.Canvas, x_m: float | None, y_m: float | None, width: int, height: int) -> None:
        if x_m is None or y_m is None:
            return
        x, y = project_xy(Point3D(x_m, y_m, 0, 0, 0, "target"), self.bounds, width_px=width, height_px=height)
        self._draw_cross(canvas, x, y, fill="#ff4d4d")

    def _draw_cross(self, canvas: tk.Canvas, x: float, y: float, *, fill: str) -> None:
        canvas.create_line(x - 7, y, x + 7, y, fill=fill, width=2)
        canvas.create_line(x, y - 7, x, y + 7, fill=fill, width=2)

    def _clear_canvases(self) -> None:
        self._hide_fall_alert_popup()
        self.xy_canvas.delete("all")
        if hasattr(self, "ax3d"):
            self.ax3d.clear()
            self._setup_3d_axes()
            self.figure_canvas.draw_idle()
        elif hasattr(self, "ax_xz") and hasattr(self, "ax_yz"):
            self.ax_xz.clear()
            self.ax_yz.clear()
            self._setup_projection_axis(
                self.ax_xz,
                title="XZ 侧视图",
                xlabel="X left + (m)",
                xlim=(self.bounds.x_min_m, self.bounds.x_max_m),
            )
            self._setup_projection_axis(
                self.ax_yz,
                title="YZ 侧视图",
                xlabel="Y forward (m)",
                xlim=(self.bounds.y_min_m, self.bounds.y_max_m),
            )
            self.figure.subplots_adjust(left=0.10, right=0.97, top=0.94, bottom=0.12, hspace=0.42)
            self.figure_canvas.draw_idle()
        if hasattr(self, "timeline_axes"):
            for axis in self.timeline_axes:
                axis.clear()
                self._setup_timeline_axis(axis)
            self.timeline_canvas.draw_idle()

    def _current_alert(self) -> FrameAlert:
        if 0 <= self.current_index < len(self.alerts):
            return self.alerts[self.current_index]
        return FrameAlert()

    def _sync_fall_alert_popup(self, view: VisualizationFrame) -> None:
        if self.fall_alert_popup is None:
            return
        self.fall_alert_popup.sync(
            self._current_alert(),
            frame_index=view.frame.frame_index,
            timestamp_ms=view.frame.timestamp_ms,
        )

    def _hide_fall_alert_popup(self) -> None:
        if self.fall_alert_popup is not None:
            self.fall_alert_popup.hide()


def _meter_ticks(low: float, high: float) -> list[float]:
    start = int(low)
    end = int(high)
    ticks = [float(value) for value in range(start, end + 1)]
    return [tick for tick in ticks if low <= tick <= high]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize MRS6240 datadealer point-cloud clusters.")
    parser.add_argument("--data-dir", default="datadealer/data", help="datadealer data directory or SDK Software_Kit/05_Data directory")
    parser.add_argument("--stem", default=None, help="recording stem, for example 2026_07_03_111005")
    parser.add_argument("--min-snr", type=float, default=15.0, help="minimum SNR used before clustering")
    parser.add_argument("--z-min", type=float, default=TrackerConfig.z_min_m, help="minimum world-coordinate height ROI in meters")
    parser.add_argument("--z-max", type=float, default=TrackerConfig.z_max_m, help="maximum world-coordinate height ROI in meters")
    parser.add_argument("--cluster-radius", type=float, default=0.45, help="cluster radius in meters")
    parser.add_argument("--min-cluster-points", type=int, default=3, help="minimum points per cluster")
    parser.add_argument("--max-cluster-width", type=float, default=0.5, help="maximum target cluster X width in meters")
    parser.add_argument("--max-cluster-depth", type=float, default=0.5, help="maximum target cluster Y depth in meters")
    parser.add_argument("--track-gate", type=float, default=0.5, help="maximum XY target jump while locked, in meters")
    parser.add_argument("--reacquire-gate", type=float, default=1.0, help="kept for old commands; current tracker uses switch-confirm-frames")
    parser.add_argument("--switch-confirm-frames", type=int, default=5, help="consecutive far-candidate frames required before target switching")
    parser.add_argument("--hold-ms", type=int, default=3000, help="milliseconds to hold the last target after local tracking is lost")
    parser.add_argument("--fall-suspect-duration-ms", type=int, default=FallRuleConfig.suspect_low_duration_ms, help="terminal-low duration required before a suspected fall")
    parser.add_argument("--fall-suspect-low-ratio", type=float, default=FallRuleConfig.suspect_low_ratio, help="required terminal-low ratio for a suspected fall, from 0 to 1")
    parser.add_argument("--fall-confirm-min-horizontal-displacement", type=float, default=FallRuleConfig.confirm_min_horizontal_displacement_m, help="minimum candidate XY travel before confirmed fall; 0 disables this deployment-specific gate")
    parser.add_argument("--yz-rotation-deg", type=float, default=0.0, help="rotate points in the Y-Z plane before tracking; 0=old/front placement, 55=55 degree downward mount")
    parser.add_argument("--mount-height-m", type=float, default=0.0, help="radar installation height above the floor, in meters")
    parser.add_argument("--ground-calibration-frames", type=int, default=0, help="fit and flatten the ground plane from the first N frames; 0 disables calibration")
    parser.add_argument("--ground-band-m", type=float, default=0.25, help="half-width around expected Z=0 used to select ground calibration points")
    return parser.parse_args()


def tracker_config_from_args(args: argparse.Namespace) -> TrackerConfig:
    if args.z_min >= args.z_max:
        raise ValueError("--z-min must be less than --z-max")
    return TrackerConfig(
        z_min_m=args.z_min,
        z_max_m=args.z_max,
        min_snr_db=args.min_snr,
        cluster_radius_m=args.cluster_radius,
        min_cluster_points=args.min_cluster_points,
        max_cluster_width_x_m=args.max_cluster_width,
        max_cluster_depth_y_m=args.max_cluster_depth,
        track_gate_m=args.track_gate,
        reacquire_gate_m=args.reacquire_gate,
        switch_confirm_frames=args.switch_confirm_frames,
        hold_ms=args.hold_ms,
    )


def fall_rule_config_from_args(args: argparse.Namespace) -> FallRuleConfig:
    if args.fall_suspect_duration_ms < 0:
        raise ValueError("--fall-suspect-duration-ms must not be negative")
    if not 0.0 <= args.fall_suspect_low_ratio <= 1.0:
        raise ValueError("--fall-suspect-low-ratio must be within 0..1")
    if args.fall_confirm_min_horizontal_displacement < 0.0:
        raise ValueError("--fall-confirm-min-horizontal-displacement must not be negative")
    return FallRuleConfig(
        suspect_low_duration_ms=args.fall_suspect_duration_ms,
        suspect_low_ratio=args.fall_suspect_low_ratio,
        confirm_min_horizontal_displacement_m=args.fall_confirm_min_horizontal_displacement,
    )


def main() -> None:
    args = parse_args()
    config = tracker_config_from_args(args)
    fall_config = fall_rule_config_from_args(args)
    root = tk.Tk()
    DatadealerVisualizerApp(
        root,
        data_dir=Path(args.data_dir),
        stem=args.stem,
        config=config,
        fall_config=fall_config,
        yz_rotation_deg=args.yz_rotation_deg,
        mount_height_m=args.mount_height_m,
        ground_calibration_frames=args.ground_calibration_frames,
        ground_band_m=args.ground_band_m,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
