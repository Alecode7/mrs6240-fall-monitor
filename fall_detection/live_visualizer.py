from __future__ import annotations

import argparse
import sys
from pathlib import Path
from tkinter import messagebox

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_detection.datadealer_loader import CombinedFrame, load_datadealer_recording
    from fall_detection.device_source import HifDeviceConfig, HifDevicePointCloudSource
    from fall_detection.live_processing import LiveFrameWindow
    from fall_detection.live_source import LatestFrameBuffer, LiveSource, ReplayLiveSource, live_source_stats
    from fall_detection.rule_detector import FallRuleConfig, FallState
    from fall_detection.tracking import TrackerConfig
    from fall_detection.visualize_datadealer import DatadealerVisualizerApp, TimelineSeries, _display_cluster_boxes, _format_source_frame_label
else:
    from .datadealer_loader import CombinedFrame, load_datadealer_recording
    from .device_source import HifDeviceConfig, HifDevicePointCloudSource
    from .live_processing import LiveFrameWindow
    from .live_source import LatestFrameBuffer, LiveSource, ReplayLiveSource, live_source_stats
    from .rule_detector import FallRuleConfig, FallState
    from .tracking import TrackerConfig
    from .visualize_datadealer import DatadealerVisualizerApp, TimelineSeries, _display_cluster_boxes, _format_source_frame_label

import tkinter as tk


def _format_frame_frequency(views, *, current_index: int) -> str:
    if current_index <= 0 or current_index >= len(views):
        return "频率 --Hz"
    current = views[current_index]
    previous = views[current_index - 1]
    current_ms = getattr(getattr(current, "frame", None), "timestamp_ms", None)
    previous_ms = getattr(getattr(previous, "frame", None), "timestamp_ms", None)
    try:
        delta_ms = float(current_ms) - float(previous_ms)
    except (TypeError, ValueError):
        return "频率 --Hz"
    if delta_ms <= 0:
        return "频率 --Hz"
    return f"频率 {1000.0 / delta_ms:.1f}Hz"


class LiveReplayVisualizerApp(DatadealerVisualizerApp):
    def _uses_3d_view(self) -> bool:
        return True

    def _supports_world_calibration_controls(self) -> bool:
        return False

    def __init__(
        self,
        root: tk.Tk,
        *,
        data_dir: Path,
        stem: str | None = None,
        config: TrackerConfig | None = None,
        interval_ms: int = 50,
        poll_ms: int = 5,
        history_frames: int = 600,
        draw_3d_every: int = 1,
        draw_timeline_every: int = 10,
        loop: bool = True,
        autostart: bool = True,
        source: str = "replay",
        device_config: HifDeviceConfig | None = None,
        fall_config: FallRuleConfig | None = None,
        yz_rotation_deg: float = 0.0,
        mount_height_m: float = 0.0,
    ):
        self.interval_ms = max(int(interval_ms), 1)
        self.poll_ms = max(int(poll_ms), 1)
        self.history_frames = max(int(history_frames), 1)
        self.draw_3d_every = max(int(draw_3d_every), 1)
        self.draw_timeline_every = max(int(draw_timeline_every), 1)
        self._live_draw_count = 0
        self._live_scale_to: int | None = None
        self._live_3d_ready = False
        self._live_3d_box_artists = []
        self.loop = loop
        self.autostart = autostart
        self.source_mode = source
        self.device_config = device_config or HifDeviceConfig()
        self.fall_config = fall_config or FallRuleConfig()
        self.yz_rotation_deg = float(yz_rotation_deg)
        self.mount_height_m = float(mount_height_m)
        self.live_buffer: LatestFrameBuffer | None = None
        self.live_source: LiveSource | None = None
        self.live_window: LiveFrameWindow | None = None
        self.replay_frames: tuple[CombinedFrame, ...] = ()
        self.source_frame_count = 0
        self._loaded_live_stem = ""
        super().__init__(
            root,
            data_dir=data_dir,
            stem=stem,
            config=config,
            yz_rotation_deg=yz_rotation_deg,
            mount_height_m=mount_height_m,
        )
        self.root.title("MRS6240 实时点云聚类可视化")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        if self.autostart and self.live_source is not None:
            self.toggle_play()

    def draw_current_frame(self) -> None:
        if not self.frames:
            self._clear_canvases()
            return
        view = self.frames[self.current_index]
        self._update_text(view)
        self._draw_xy(view)
        if not self.is_playing or self._live_draw_count % self.draw_3d_every == 0:
            self._draw_3d(view)
        if not self.is_playing or self._live_draw_count % self.draw_timeline_every == 0:
            self._draw_timeline()
        self._sync_fall_alert_popup(view)
        if self.is_playing:
            self._live_draw_count += 1

    def _draw_3d(self, view) -> None:
        if self.is_playing:
            self._draw_live_3d(view)
            return
        self._reset_live_3d_cache()
        super()._draw_3d(view)

    def _draw_live_3d(self, view) -> None:
        self._capture_3d_view()
        if not getattr(self, "_live_3d_ready", False):
            self.ax3d.clear()
            self._setup_3d_axes()
            self._live_3d_filtered = self.ax3d.scatter([], [], [], c=["#4c5663"], s=[], alpha=0.45, depthshade=False)
            self._live_3d_presence = self.ax3d.scatter([], [], [], c=[], s=[], alpha=0.9, depthshade=False)
            self._live_3d_motion = self.ax3d.scatter([], [], [], c=[], s=[], alpha=0.95, depthshade=False)
            self._live_3d_target = self.ax3d.scatter([], [], [], c=["#ff4d4d"], s=[], marker="x", depthshade=False)
            self._live_3d_box_artists = []
            self._live_3d_ready = True

        self._set_scatter_points(self._live_3d_filtered, view.filtered_points, colors=["#4c5663"] * len(view.filtered_points), sizes=[8] * len(view.filtered_points))

        presence_points = []
        presence_colors = []
        presence_sizes = []
        motion_points = []
        motion_sizes = []
        for cluster in view.clusters:
            base_color = "#f59e0b" if cluster.is_target else "#58a6ff"
            for point in cluster.points:
                if point.kind == "motion":
                    motion_points.append(point)
                    motion_sizes.append(32 if cluster.is_target else 24)
                else:
                    presence_points.append(point)
                    presence_colors.append(base_color)
                    presence_sizes.append(22 if cluster.is_target else 16)
        self._set_scatter_points(self._live_3d_presence, presence_points, colors=presence_colors, sizes=presence_sizes)
        self._set_scatter_points(self._live_3d_motion, motion_points, colors=["#f97316"] * len(motion_points), sizes=motion_sizes)

        if view.target.has_target and view.target.center_x_m is not None and view.target.center_y_m is not None and view.target.center_z_m is not None:
            self._set_raw_scatter(self._live_3d_target, [view.target.center_x_m], [view.target.center_y_m], [view.target.center_z_m], sizes=[56])
        else:
            self._set_raw_scatter(self._live_3d_target, [], [], [], sizes=[])

        self._clear_live_3d_boxes()
        for box in _display_cluster_boxes(view):
            self._live_3d_box_artists.extend(
                self._draw_live_3d_cluster_box(box.cluster, color=box.color, line_width=box.line_width, label=box.label)
            )

        alert = self._current_alert()
        if alert.has_alert:
            self.ax3d.set_title(alert.summary, color="#f87171", fontsize=10, pad=12)
        else:
            self.ax3d.set_title("")
        self.figure_canvas.draw_idle()

    def _set_scatter_points(self, artist, points, *, colors, sizes) -> None:
        xs = [point.x_m for point in points]
        ys = [point.y_m for point in points]
        zs = [point.z_m for point in points]
        self._set_raw_scatter(artist, xs, ys, zs, colors=colors, sizes=sizes)

    def _set_raw_scatter(self, artist, xs, ys, zs, *, colors=None, sizes) -> None:
        artist._offsets3d = (
            np.asarray(xs, dtype=float),
            np.asarray(ys, dtype=float),
            np.asarray(zs, dtype=float),
        )
        artist.set_sizes(np.asarray(sizes, dtype=float))
        if colors:
            artist.set_color(colors)

    def _clear_live_3d_boxes(self) -> None:
        for artist in getattr(self, "_live_3d_box_artists", []):
            try:
                artist.remove()
            except Exception:
                pass
        self._live_3d_box_artists = []

    def _draw_live_3d_cluster_box(self, cluster, *, color: str, line_width: float, label: str) -> list:
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
        artists = []
        for start, end in edges:
            sx, sy, sz = corners[start]
            ex, ey, ez = corners[end]
            artists.extend(self.ax3d.plot([sx, ex], [sy, ey], [sz, ez], color=color, linewidth=line_width))
        artists.append(
            self.ax3d.text(
                cluster.center_x_m,
                cluster.center_y_m,
                cluster.z_max_m,
                f"{label} {cluster.point_count}/{cluster.motion_count}",
                color=color,
                fontsize=8,
            )
        )
        return artists

    def _reset_live_3d_cache(self) -> None:
        self._live_3d_ready = False
        self._live_3d_box_artists = []

    def _clear_canvases(self) -> None:
        self._reset_live_3d_cache()
        super()._clear_canvases()

    def load_selected_recording(self) -> None:
        self._stop_live_source()
        if self.source_mode == "device":
            self._prepare_device_source()
            return
        stem = self.stem_var.get() or None
        try:
            recording = load_datadealer_recording(self.data_dir, stem=stem)
        except Exception as exc:
            self.replay_frames = ()
            self.frames = []
            self.alerts = ()
            self.timeline_series = TimelineSeries((), (), (), (), ())
            self.status_var.set("加载失败")
            self.target_var.set("目标: 无")
            self.diagnosis_var.set("诊断: 无")
            self.alert_var.set("异常: 无")
            self.play_text.set("开始实时")
            self._clear_canvases()
            messagebox.showerror("加载失败", str(exc))
            return

        self.replay_frames = recording.frames
        self.source_frame_count = len(recording.frames)
        self._loaded_live_stem = recording.stem
        self.live_buffer = LatestFrameBuffer()
        self.live_window = self._new_live_window()
        self.live_source = self._new_live_source()
        self.frames = []
        self.alerts = ()
        self.timeline_series = TimelineSeries((), (), (), (), ())
        self.current_index = 0
        self._live_draw_count = 0
        self._live_scale_to = 0
        self.is_playing = False
        self.play_text.set("开始实时")
        self.frame_scale.configure(from_=0, to=0)
        self._set_frame_scale(0)
        self.status_var.set(
            f"{recording.stem}  实时回放待开始  源帧={self.source_frame_count}  "
            f"间隔={self.interval_ms}ms  历史={self.history_frames}"
        )
        self.target_var.set("目标: 无")
        self.diagnosis_var.set("诊断: 无")
        self.alert_var.set("异常: 无")
        self._clear_canvases()

    def _prepare_device_source(self) -> None:
        self.replay_frames = ()
        self.source_frame_count = 0
        self._loaded_live_stem = f"device:{self.device_config.transport}"
        self.live_buffer = LatestFrameBuffer()
        self.live_window = self._new_live_window()
        self.live_source = self._new_live_source()
        self.frames = []
        self.alerts = ()
        self.timeline_series = TimelineSeries((), (), (), (), ())
        self.current_index = 0
        self._live_draw_count = 0
        self._live_scale_to = 0
        self.is_playing = False
        self.play_text.set("连接设备")
        self.frame_scale.configure(from_=0, to=0)
        self._set_frame_scale(0)
        try:
            self.stem_combo.configure(state="disabled")
        except Exception:
            pass
        device_label = (
            f"SPI index={self.device_config.spi_index} freqMode={self.device_config.spi_freq_mode}"
            if self.device_config.transport == "spi"
            else f"UART {self.device_config.uart_com} @ {self.device_config.uart_baud_rate}"
        )
        self.status_var.set(f"设备源待连接  {device_label}  历史={self.history_frames}")
        self.target_var.set("目标: 无")
        self.diagnosis_var.set("诊断: 无")
        self.alert_var.set("异常: 无")
        self._clear_canvases()

    def toggle_play(self) -> None:
        if self.live_buffer is None or self.live_window is None:
            return
        self.is_playing = not self.is_playing
        self.play_text.set("暂停实时" if self.is_playing else "继续实时")
        if self.is_playing:
            if self.live_source is None or not self.live_source.is_running:
                self.live_source = self._new_live_source()
                try:
                    self.live_source.start(self.live_buffer)
                except Exception as exc:
                    self.is_playing = False
                    self.play_text.set("连接设备" if self.source_mode == "device" else "开始实时")
                    messagebox.showerror("实时源启动失败", str(exc))
                    return
            self._schedule_live_poll()
        elif self.play_after_id is not None:
            self.root.after_cancel(self.play_after_id)
            self.play_after_id = None

    def previous_frame(self) -> None:
        self._pause_live_view()
        super().previous_frame()

    def next_frame(self) -> None:
        self._pause_live_view()
        super().next_frame()

    def close(self) -> None:
        self._stop_live_source()
        self.root.destroy()

    def _new_live_source(self) -> LiveSource:
        if self.source_mode == "device":
            return HifDevicePointCloudSource(config=self.device_config)
        return ReplayLiveSource(
            self.replay_frames,
            interval_s=self.interval_ms / 1000.0,
            loop=self.loop,
        )

    def _new_live_window(self) -> LiveFrameWindow:
        return LiveFrameWindow(
            self.config,
            max_history=self.history_frames,
            fall_config=self.fall_config,
            yz_rotation_deg=self.yz_rotation_deg,
            mount_height_m=self.mount_height_m,
        )

    def _schedule_live_poll(self) -> None:
        if not self.is_playing:
            return
        if self.play_after_id is not None:
            self.root.after_cancel(self.play_after_id)
        self.play_after_id = self.root.after(self.poll_ms, self._poll_live_frame)

    def _poll_live_frame(self) -> None:
        self.play_after_id = None
        if not self.is_playing:
            return
        pending_frames = self.live_buffer.take_pending() if self.live_buffer is not None else ()
        if pending_frames and self.live_window is not None:
            refresh_timeline = self._live_draw_count % self.draw_timeline_every == 0
            snapshot = None
            pending_fall_popup = None
            last_pending_index = len(pending_frames) - 1
            for pending_index, frame in enumerate(pending_frames):
                snapshot = self.live_window.push_frame(
                    frame,
                    build_timeline=refresh_timeline and pending_index == last_pending_index,
                )
                if snapshot.alerts:
                    latest_alert = snapshot.alerts[-1]
                    if any(
                        state in (FallState.SUSPECTED, FallState.CONFIRMED)
                        for state in latest_alert.fall_states
                    ):
                        pending_fall_popup = (latest_alert, snapshot.views[-1].frame)
            assert snapshot is not None
            self.frames = snapshot.views
            self.alerts = snapshot.alerts
            self.timeline_series = snapshot.timeline
            self.current_index = max(len(self.frames) - 1, 0)
            if getattr(self, "_live_scale_to", None) != self.current_index:
                self.frame_scale.configure(from_=0, to=self.current_index)
                self._live_scale_to = self.current_index
            self._set_frame_scale(self.current_index)
            self.draw_current_frame()
            if pending_fall_popup is not None and self.fall_alert_popup is not None:
                popup_alert, popup_frame = pending_fall_popup
                if popup_frame.frame_index != self.frames[-1].frame.frame_index:
                    self.fall_alert_popup.sync(
                        popup_alert,
                        frame_index=popup_frame.frame_index,
                        timestamp_ms=popup_frame.timestamp_ms,
                    )
        elif self._source_finished():
            self.is_playing = False
            self.play_text.set("重新开始")
            self._update_finished_status()
            return
        self._schedule_live_poll()

    def _pause_live_view(self) -> None:
        if not self.is_playing:
            return
        self.is_playing = False
        self.play_text.set("继续实时")
        if self.play_after_id is not None:
            self.root.after_cancel(self.play_after_id)
            self.play_after_id = None

    def _stop_live_source(self) -> None:
        if getattr(self, "play_after_id", None) is not None:
            self.root.after_cancel(self.play_after_id)
            self.play_after_id = None
        if self.live_source is not None:
            self.live_source.stop()
            self.live_source.join(timeout_s=1.0)
        self.is_playing = False

    def _source_finished(self) -> bool:
        return (
            self.source_mode == "replay"
            and not self.loop
            and self.live_source is not None
            and not self.live_source.is_running
            and self.live_buffer is not None
            and self.live_buffer.published_count > 0
        )

    def _update_finished_status(self) -> None:
        if self.live_source is None or self.live_buffer is None:
            return
        stats = live_source_stats(self.live_source, self.live_buffer)
        self.status_var.set(
            f"{self._loaded_live_stem}  实时回放结束  "
            f"源帧={self.source_frame_count}  发布={stats.published_count}  使用={stats.consumed_count}"
        )

    def _update_text(self, view) -> None:
        super()._update_text(view)
        if self.live_source is None or self.live_buffer is None:
            return
        stats = live_source_stats(self.live_source, self.live_buffer)
        source_detail = f"源帧={self.source_frame_count}" if self.source_mode == "replay" else "设备源"
        frequency_text = _format_frame_frequency(self.frames, current_index=self.current_index)
        self.status_var.set(
            f"实时 {self._loaded_live_stem}  "
            f"窗口帧 {self.current_index + 1}/{len(self.frames)}  "
            f"{_format_source_frame_label(view.frame)}  "
            f"{source_detail}  发布={stats.published_count}  使用={stats.consumed_count}  "
            f"{frequency_text}  "
            f"聚类={len(view.clusters)}  过滤点={len(view.filtered_points)}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live visualize MRS6240 point-cloud clusters.")
    parser.add_argument("--source", choices=("replay", "device"), default="replay", help="live source: replay CSV files or connect device")
    parser.add_argument("--data-dir", default="datadealer/data", help="datadealer data directory or SDK Software_Kit/05_Data directory")
    parser.add_argument("--stem", default=None, help="recording stem, for example 2026_07_03_111005")
    parser.add_argument("--min-snr", type=float, default=15.0, help="minimum SNR used before clustering")
    parser.add_argument("--x-min", type=float, default=TrackerConfig.x_min_m, help="minimum X ROI in meters; radar-left is positive")
    parser.add_argument("--x-max", type=float, default=TrackerConfig.x_max_m, help="maximum X ROI in meters; radar-left is positive")
    parser.add_argument("--y-min", type=float, default=TrackerConfig.y_min_m, help="minimum forward-distance ROI in meters")
    parser.add_argument("--y-max", type=float, default=TrackerConfig.y_max_m, help="maximum forward-distance ROI in meters")
    parser.add_argument("--z-min", type=float, default=TrackerConfig.z_min_m, help="minimum height ROI in meters")
    parser.add_argument("--z-max", type=float, default=TrackerConfig.z_max_m, help="maximum height ROI in meters")
    parser.add_argument("--yz-rotation-deg", type=float, default=0.0, help="rotate points in the Y-Z plane before tracking; 0=old/front placement, 45=45 degree downward mount")
    parser.add_argument("--mount-height-m", type=float, default=0.0, help="radar installation height above the floor, in meters")
    parser.add_argument("--cluster-radius", type=float, default=0.45, help="cluster radius in meters")
    parser.add_argument("--min-cluster-points", type=int, default=3, help="minimum points per cluster")
    parser.add_argument("--max-cluster-width", type=float, default=0.5, help="maximum target cluster X width in meters")
    parser.add_argument("--max-cluster-depth", type=float, default=0.5, help="maximum target cluster Y depth in meters")
    parser.add_argument("--track-gate", type=float, default=0.5, help="maximum XY target jump while locked, in meters")
    parser.add_argument("--reacquire-gate", type=float, default=1.0, help="kept for old commands; current tracker uses switch-confirm-frames")
    parser.add_argument("--switch-confirm-frames", type=int, default=5, help="consecutive far-candidate frames required before target switching")
    parser.add_argument("--hold-ms", type=int, default=3000, help="milliseconds to hold the last target after local tracking is lost")
    parser.add_argument("--fall-min-drop-height", type=float, default=FallRuleConfig.min_drop_height_m, help="minimum target height drop before creating an internal fall candidate, in meters")
    parser.add_argument("--fall-terminal-height", type=float, default=FallRuleConfig.terminal_height_m, help="maximum world-coordinate target height for a fall terminal state, in meters")
    parser.add_argument("--fall-low-duration-ms", type=int, default=FallRuleConfig.low_still_duration_ms, help="milliseconds terminal-low posture must cover before confirmed fall")
    parser.add_argument("--fall-confirm-window-ms", type=int, default=FallRuleConfig.confirm_window_ms, help="maximum milliseconds from internal candidate to confirmed fall")
    parser.add_argument("--interval-ms", type=int, default=50, help="replay publish interval in milliseconds")
    parser.add_argument("--poll-ms", type=int, default=5, help="GUI latest-frame polling interval in milliseconds")
    parser.add_argument("--history-frames", type=int, default=600, help="recent processed frames kept for timeline and review")
    parser.add_argument("--draw-3d-every", type=int, default=1, help="refresh 3D view once per N live frames")
    parser.add_argument("--draw-timeline-every", type=int, default=10, help="refresh timeline once per N live frames")
    parser.add_argument("--no-loop", action="store_true", help="play the recording once instead of looping")
    parser.add_argument("--no-autostart", action="store_true", help="open the window without starting replay")
    parser.add_argument("--device-transport", choices=("spi", "uart"), default="spi", help="device transport used when --source device")
    parser.add_argument("--spi-index", type=int, default=0, help="CH347 SPI device index used when --source device")
    parser.add_argument("--spi-freq-mode", type=int, default=0, help="SPI frequency mode: 0=60MHz, 1=30MHz, 2=15MHz, 3=7.5MHz")
    parser.add_argument("--uart-com", default="COM12", help="UART COM port used when --source device --device-transport uart")
    parser.add_argument("--uart-baud", default="1000000", help="UART baud rate used when --source device --device-transport uart")
    parser.add_argument("--dll-path", default=None, help="optional HifMsgDataCollectionLib.dll path")
    return parser.parse_args()


def tracker_config_from_args(args: argparse.Namespace) -> TrackerConfig:
    return TrackerConfig(
        x_min_m=args.x_min,
        x_max_m=args.x_max,
        y_min_m=args.y_min,
        y_max_m=args.y_max,
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
    return FallRuleConfig(
        min_drop_height_m=args.fall_min_drop_height,
        terminal_height_m=args.fall_terminal_height,
        low_still_duration_ms=args.fall_low_duration_ms,
        confirm_window_ms=args.fall_confirm_window_ms,
    )


def main() -> None:
    args = parse_args()
    config = tracker_config_from_args(args)
    fall_config = fall_rule_config_from_args(args)
    root = tk.Tk()
    device_config = HifDeviceConfig(
        transport=args.device_transport,
        spi_index=args.spi_index,
        spi_freq_mode=args.spi_freq_mode,
        uart_com=args.uart_com,
        uart_baud_rate=args.uart_baud,
        dll_path=args.dll_path,
    )
    LiveReplayVisualizerApp(
        root,
        data_dir=Path(args.data_dir),
        stem=args.stem,
        config=config,
        interval_ms=args.interval_ms,
        poll_ms=args.poll_ms,
        history_frames=args.history_frames,
        draw_3d_every=args.draw_3d_every,
        draw_timeline_every=args.draw_timeline_every,
        loop=not args.no_loop,
        autostart=not args.no_autostart,
        source=args.source,
        device_config=device_config,
        fall_config=fall_config,
        yz_rotation_deg=args.yz_rotation_deg,
        mount_height_m=args.mount_height_m,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
