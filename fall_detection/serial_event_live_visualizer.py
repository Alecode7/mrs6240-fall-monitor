from __future__ import annotations

import argparse
import json
import re
import time
import sys
import threading
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from tkinter import messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_detection.datadealer_loader import Point3D
    from fall_detection.device_source import HifDeviceConfig, HifDevicePointCloudSource
    from fall_detection.live_source import LatestFrameBuffer, live_source_stats
    from fall_detection.plot_fonts import configure_chinese_matplotlib_font
    from fall_detection.coordinate_transform import transform_frame_to_world
    from fall_detection.raw_visualization import RawFrameHistory, RawFrameView, RawTimeline, raw_view_from_frame
    from fall_detection.recording_replay import RecordedSessionReplay, load_recorded_session
    from fall_detection.serial_event import BoardFrameStatus, FallSerialEvent, SerialLogReader, SerialRecord
else:
    from .datadealer_loader import Point3D
    from .device_source import HifDeviceConfig, HifDevicePointCloudSource
    from .live_source import LatestFrameBuffer, live_source_stats
    from .plot_fonts import configure_chinese_matplotlib_font
    from .coordinate_transform import transform_frame_to_world
    from .raw_visualization import RawFrameHistory, RawFrameView, RawTimeline, raw_view_from_frame
    from .recording_replay import RecordedSessionReplay, load_recorded_session
    from .serial_event import BoardFrameStatus, FallSerialEvent, SerialLogReader, SerialRecord


configure_chinese_matplotlib_font()

DEFAULT_SESSION_LOG_DIR = Path(__file__).resolve().parents[1] / "logs" / "realtime_fall"
RADAR_START_COMMAND = "mmwc start"
RADAR_STOP_COMMAND = "mmwc stop"
FALL_CONFIG_COMMAND = "fall_config"
SERIAL_RECONNECT_DELAY_MS = 3000
POINT_RECONNECT_DELAY_MS = 3000
POINT_DATA_STALE_AFTER_S = 3.0
BOARD_STATUS_STALE_AFTER_S = 3.0
TARGET_ASSOCIATION_RADIUS_M = 0.60
TARGET_ASSOCIATION_ALIGNED_M = 0.35
EVIDENCE_POST_EVENT_MS = 5000
EVIDENCE_HISTORY_FRAME_LIMIT = 240
ALARM_SOUND_INTERVAL_MS = 1200
DEFAULT_POINT_YZ_ROTATION_DEG = 90.0
DEFAULT_POINT_MOUNT_HEIGHT_M = 3.1
_PROTOCOL_FIELD_PATTERN = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[^\s]+)")
_HEIGHT_REPLY_PATTERN = re.compile(r"\bFALL_HEIGHT\b(?:\s+(?:loaded|default)=|\s+set=|\s+)(?P<height>\d+)cm\b")


@dataclass(frozen=True)
class BoardDeploymentConfig:
    mount_height_cm: int
    pitch_deg: int
    roi_x_min_cm: int
    roi_x_max_cm: int
    roi_y_min_cm: int
    roi_y_max_cm: int
    roi_z_min_cm: int
    roi_z_max_cm: int


@dataclass(frozen=True)
class DeploymentReadiness:
    level: str
    title: str
    detail: str


EXPECTED_DEPLOYMENT_CONFIG = BoardDeploymentConfig(
    mount_height_cm=310,
    pitch_deg=90,
    roi_x_min_cm=-250,
    roi_x_max_cm=250,
    roi_y_min_cm=-250,
    roi_y_max_cm=250,
    roi_z_min_cm=0,
    roi_z_max_cm=220,
)


def parse_board_deployment_config(line: str) -> BoardDeploymentConfig | None:
    """Parse a FALL_CFG line emitted at startup or in response to fall_config."""
    text = str(line).strip().strip("\x00")
    start = text.find("FALL_CFG")
    if start < 0:
        return None
    fields = {match.group("key"): match.group("value") for match in _PROTOCOL_FIELD_PATTERN.finditer(text[start:])}
    try:
        roi_x_min, roi_x_max = (int(value) for value in fields["roi_x"].split("..", 1))
        roi_y_min, roi_y_max = (int(value) for value in fields["roi_y"].split("..", 1))
        roi_z_min, roi_z_max = (int(value) for value in fields["roi_z"].split("..", 1))
        return BoardDeploymentConfig(
            mount_height_cm=int(fields["mount_h"]),
            pitch_deg=int(fields["pitch"]),
            roi_x_min_cm=roi_x_min,
            roi_x_max_cm=roi_x_max,
            roi_y_min_cm=roi_y_min,
            roi_y_max_cm=roi_y_max,
            roi_z_min_cm=roi_z_min,
            roi_z_max_cm=roi_z_max,
        )
    except (KeyError, TypeError, ValueError):
        return None


def parse_board_mount_height(line: str) -> int | None:
    """Parse FALL_HEIGHT startup, set, or query responses."""
    match = _HEIGHT_REPLY_PATTERN.search(str(line).strip().strip("\x00"))
    return int(match.group("height")) if match is not None else None


def deployment_readiness(
    *,
    source_mode: str,
    monitoring_requested: bool,
    serial_connected: bool,
    point_health: str,
    board_health: str,
    board_config: BoardDeploymentConfig | None,
    expected_mount_height_cm: int | None,
    overflow_delta: int = 0,
) -> DeploymentReadiness:
    """Collapse transport, board telemetry, and configuration into one deployable status."""
    if source_mode != "device":
        return DeploymentReadiness("info", "回放模式", "回放数据不参与现场部署自检")
    if not monitoring_requested:
        return DeploymentReadiness("pending", "等待连接", "连接控制串口和点云后开始部署自检")
    failures: list[str] = []
    if not serial_connected:
        failures.append("控制串口未连接")
    elif board_health in {"error", "stale"}:
        failures.append("板端状态帧异常")
    if point_health in {"disconnected", "error", "stale", "stopped"}:
        failures.append("点云数据异常")
    if overflow_delta > 0:
        failures.append(f"overflow 增长 +{overflow_delta}")
    if failures:
        return DeploymentReadiness("error", "数据异常", "；".join(failures))

    pending: list[str] = []
    if board_health != "healthy":
        pending.append("等待板端 FALL 状态帧")
    if point_health != "healthy":
        pending.append("等待点云帧")
    if board_config is None:
        pending.append("等待 FALL_CFG 配置回显")
    else:
        expected = EXPECTED_DEPLOYMENT_CONFIG
        mismatches: list[str] = []
        if board_config.pitch_deg != expected.pitch_deg:
            mismatches.append(f"俯角 {board_config.pitch_deg}°")
        if (board_config.roi_x_min_cm, board_config.roi_x_max_cm) != (expected.roi_x_min_cm, expected.roi_x_max_cm):
            mismatches.append("ROI X")
        if (board_config.roi_y_min_cm, board_config.roi_y_max_cm) != (expected.roi_y_min_cm, expected.roi_y_max_cm):
            mismatches.append("ROI Y")
        if (board_config.roi_z_min_cm, board_config.roi_z_max_cm) != (expected.roi_z_min_cm, expected.roi_z_max_cm):
            mismatches.append("ROI Z")
        if expected_mount_height_cm is not None and board_config.mount_height_cm != expected_mount_height_cm:
            mismatches.append(f"高度 {board_config.mount_height_cm}cm")
        if mismatches:
            return DeploymentReadiness("pending", "配置不一致", "、".join(mismatches))
    if pending:
        return DeploymentReadiness("pending", "配置待核验", "；".join(pending))
    return DeploymentReadiness("ready", "可监护", "串口、点云、板端状态与部署配置已核验")


def monitor_connection_summary(*, serial_connected: bool, point_health: str, board_health: str) -> str:
    """Return a concise transport summary for the operator-facing monitor UI."""
    serial_text = "已连接" if serial_connected else "未连接"
    point_text = {"healthy": "正常", "waiting": "等待数据", "stale": "超时", "error": "异常"}.get(
        point_health, "未连接"
    )
    board_text = {"healthy": "正常", "waiting": "等待状态", "stale": "超时", "error": "异常"}.get(
        board_health, "未连接"
    )
    return f"控制口：{serial_text}    点云：{point_text}    板端状态：{board_text}"


@dataclass(frozen=True)
class PlotBounds:
    x_min_m: float = -2.5
    x_max_m: float = 2.5
    y_min_m: float = -2.5
    y_max_m: float = 2.5
    z_min_m: float = -1.0
    z_max_m: float = 2.5


@dataclass(frozen=True)
class BoardTargetAssociation:
    candidate_count: int
    nearest_distance_m: float | None
    centroid_x_m: float | None
    centroid_y_m: float | None
    centroid_distance_m: float | None

    @property
    def is_aligned(self) -> bool:
        return self.candidate_count > 0 and self.centroid_distance_m is not None and self.centroid_distance_m <= TARGET_ASSOCIATION_ALIGNED_M


def board_target_association(
    view: RawFrameView | None,
    status: BoardFrameStatus | None,
    *,
    radius_m: float = TARGET_ASSOCIATION_RADIUS_M,
) -> BoardTargetAssociation | None:
    if view is None or status is None or not status.has_target or status.x_cm is None or status.y_cm is None:
        return None
    target_x = status.x_cm / 100.0
    target_y = status.y_cm / 100.0
    distances = tuple(
        ((point.x_m - target_x) ** 2 + (point.y_m - target_y) ** 2) ** 0.5
        for point in view.all_points
    )
    if not distances:
        return BoardTargetAssociation(0, None, None, None, None)
    nearby = tuple(point for point, distance in zip(view.all_points, distances) if distance <= radius_m)
    if not nearby:
        return BoardTargetAssociation(0, min(distances), None, None, None)
    centroid_x = sum(point.x_m for point in nearby) / len(nearby)
    centroid_y = sum(point.y_m for point in nearby) / len(nearby)
    centroid_distance = ((centroid_x - target_x) ** 2 + (centroid_y - target_y) ** 2) ** 0.5
    return BoardTargetAssociation(
        len(nearby),
        min(distances),
        centroid_x,
        centroid_y,
        centroid_distance,
    )


def board_target_association_text(association: BoardTargetAssociation | None) -> str:
    if association is None:
        return "关联=无板端目标"
    if association.candidate_count == 0:
        return "关联=点云未覆盖"
    state = "对齐" if association.is_aligned else "偏离"
    return f"关联={state} {association.candidate_count}点 偏差={association.centroid_distance_m:.2f}m"


def fall_alert_colors(state: str) -> tuple[str, str]:
    """Return background and foreground colors for a board fall event."""
    if state == "suspected":
        return "#1d4ed8", "#dbeafe"
    if state == "recovered":
        return "#14532d", "#dcfce7"
    return "#7f1d1d", "#fee2e2"


def create_replay_source(session_dir: str | Path, *, speed: float = 1.0, loop: bool = False) -> RecordedSessionReplay:
    return RecordedSessionReplay(load_recorded_session(session_dir), speed=speed, loop=loop)


def board_overflow_delta(previous: BoardFrameStatus | None, current: BoardFrameStatus | None) -> int:
    if previous is None or current is None or previous.overflow is None or current.overflow is None:
        return 0
    if current.overflow < previous.overflow:
        return 0
    return current.overflow - previous.overflow


class SessionLogWriter:
    """Persist one realtime monitor run without affecting the serial reader thread."""

    def __init__(self, directory: str | Path, *, source_mode: str):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        started_at = datetime.now()
        self.path = self.directory / f"mrs6240_realtime_{started_at:%Y%m%d_%H%M%S}.log"
        self._stream = self.path.open("a", encoding="utf-8", newline="\n")
        self.write_note(f"会话开始 source={source_mode}")

    def write_records(self, records) -> None:
        for record in records:
            self._stream.write(SerialEventLiveVisualizerApp._format_serial_record(record))
        self._stream.flush()

    def write_note(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._stream.write(f"[{timestamp}] 系统 {message}\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream.closed:
            return
        self.write_note("会话结束")
        self._stream.close()


class BoardEventState:
    def __init__(self, *, max_events: int = 200):
        self._seen: set[tuple[str, str, str, str]] = set()
        self._events: list[FallSerialEvent] = []
        self.max_events = max(int(max_events), 1)
        self.active = False
        self.last_event: FallSerialEvent | None = None

    @property
    def events(self) -> tuple[FallSerialEvent, ...]:
        return tuple(self._events)

    def accept(self, event: FallSerialEvent | None) -> bool:
        if event is None or event.identity in self._seen:
            return False
        self._seen.add(event.identity)
        self._events.append(event)
        if len(self._events) > self.max_events:
            self._events.pop(0)
        self.last_event = event
        if event.state in {"suspected", "confirmed"}:
            self.active = True
        elif event.state == "recovered":
            self.active = False
        return True


class FallAlarmSound:
    """Repeat a rising and falling electronic siren while a fall remains confirmed."""

    def __init__(
        self,
        root: tk.Misc,
        *,
        interval_ms: int = ALARM_SOUND_INTERVAL_MS,
        sound_player=None,
        play_in_background: bool = True,
    ):
        self.root = root
        self.interval_ms = max(int(interval_ms), 100)
        self.sound_player = sound_player or self._play_windows_siren
        self.play_in_background = bool(play_in_background)
        self.active = False
        self._after_id: str | None = None

    @staticmethod
    def _play_windows_siren() -> None:
        try:
            import winsound

            for frequency_hz in (720, 860, 1000, 1140, 1000, 860):
                winsound.Beep(frequency_hz, 90)
        except (ImportError, RuntimeError):
            pass

    def start(self) -> bool:
        if self.active:
            return False
        self.active = True
        self._play_once()
        return True

    def stop(self) -> bool:
        was_active = self.active or self._after_id is not None
        self.active = False
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        return was_active

    def _play_once(self) -> None:
        self._after_id = None
        if not self.active:
            return
        if self.play_in_background:
            threading.Thread(target=self._play_sound_safely, name="FallAlarmSound", daemon=True).start()
        else:
            self._play_sound_safely()
        self._after_id = self.root.after(self.interval_ms, self._play_once)

    def _play_sound_safely(self) -> None:
        try:
            self.sound_player()
        except Exception:
            pass


class FallAlertPopup:
    def __init__(self, root: tk.Misc):
        self.root = root
        self.window: tk.Toplevel | None = None
        self.label: tk.Label | None = None
        self._after_id: str | None = None
        self.confirmed_active = False

    def show(self, event: FallSerialEvent) -> None:
        background, foreground = fall_alert_colors(event.state)
        if self.window is None or not self.window.winfo_exists():
            self.window = tk.Toplevel(self.root)
            self.window.title("跌倒告警")
            self.window.geometry("1240x980")
            self.window.configure(bg=background)
            self.window.protocol("WM_DELETE_WINDOW", self._on_close_requested)
            self.label = tk.Label(
                self.window,
                text="",
                bg=background,
                fg=foreground,
                font=("Microsoft YaHei UI", 30, "bold"),
                justify="center",
            )
            self.label.pack(fill="both", expand=True, padx=24, pady=24)
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        assert self.window is not None and self.label is not None
        detail = "  ".join(f"{key}={value}" for key, value in event.fields.items() if key != "state")
        title = "疑似跌倒" if event.state == "suspected" else "确认跌倒"
        if event.state == "confirmed":
            self.confirmed_active = True
        self.window.configure(bg=background)
        self.label.configure(bg=background, fg=foreground)
        self.label.configure(text=f"{title}\n\n{detail or event.raw_line}")
        self.window.deiconify()
        self.window.lift()
        self.window.attributes("-topmost", True)
        if event.state == "suspected":
            self._after_id = self.root.after(1000, self.hide)

    def sync_board_state(self, state: int) -> None:
        if state != 2 and self.confirmed_active:
            self.confirmed_active = False
            self.hide()

    def _on_close_requested(self) -> None:
        if self.confirmed_active and self.window is not None and self.window.winfo_exists():
            self.window.deiconify()
            self.window.lift()
            return
        self.hide()

    def hide(self) -> None:
        if self.confirmed_active:
            return
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        if self.window is not None and self.window.winfo_exists():
            self.window.withdraw()

    def close(self) -> None:
        self.confirmed_active = False
        self.hide()
        if self.window is not None and self.window.winfo_exists():
            self.window.destroy()
        self.window = None
        self.label = None


class SerialEventLiveVisualizerApp:
    def __init__(
        self,
        root: tk.Tk,
        *,
        device_config: HifDeviceConfig,
        source_mode: str = "device",
        session_dir: str | Path | None = None,
        replay_speed: float = 1.0,
        replay_loop: bool = False,
        serial_port: str = "COM3",
        serial_baud: int = 115200,
        poll_ms: int = 20,
        history_frames: int = 600,
        draw_timeline_every: int = 10,
        bounds: PlotBounds | None = None,
        point_yz_rotation_deg: float = DEFAULT_POINT_YZ_ROTATION_DEG,
        point_mount_height_m: float = DEFAULT_POINT_MOUNT_HEIGHT_M,
        session_log_dir: str | Path = DEFAULT_SESSION_LOG_DIR,
        ui_mode: str = "engineering",
    ):
        self.root = root
        if ui_mode not in {"engineering", "monitor"}:
            raise ValueError(f"unsupported ui mode: {ui_mode}")
        if ui_mode == "monitor" and source_mode != "device":
            raise ValueError("monitor UI only supports device mode")
        self.ui_mode = ui_mode
        self.root.title("MRS6240 跌倒监护" if ui_mode == "monitor" else "MRS6240 实时点云 / 串口事件监视器")
        self.root.geometry("1000x660" if ui_mode == "monitor" else "1280x940")
        if ui_mode == "monitor":
            self.root.minsize(760, 520)
        else:
            self.root.minsize(1000, 760)
        self.device_config = device_config
        if source_mode not in {"device", "replay"}:
            raise ValueError(f"unsupported source mode: {source_mode}")
        self.source_mode = source_mode
        self.session_dir = Path(session_dir).expanduser() if session_dir else None
        self.replay_speed = float(replay_speed)
        self.replay_loop = bool(replay_loop)
        self.poll_ms = max(int(poll_ms), 5)
        self.draw_timeline_every = max(int(draw_timeline_every), 1)
        self.bounds = bounds or PlotBounds()
        self.point_yz_rotation_deg = float(point_yz_rotation_deg)
        self.point_mount_height_m = float(point_mount_height_m)
        self.session_log = SessionLogWriter(session_log_dir, source_mode=self.source_mode)
        self.evidence_directory = self.session_log.directory / "evidence"
        self.evidence_directory.mkdir(parents=True, exist_ok=True)
        self.point_buffer = LatestFrameBuffer()
        self.point_source: HifDevicePointCloudSource | RecordedSessionReplay | None = None
        self.replay_session = None
        self.point_poll_after_id: str | None = None
        self.point_reconnect_after_id: str | None = None
        self.serial_poll_after_id: str | None = None
        self.serial_reconnect_after_id: str | None = None
        self.event_evidence_after_ids: set[str] = set()
        self.point_source_started_at_monotonic: float | None = None
        self.point_last_frame_at_monotonic: float | None = None
        self.point_health_state = "disconnected"
        self.deployment_check_requested = False
        self.history = RawFrameHistory(max_frames=history_frames)
        self.current_view: RawFrameView | None = None
        self.serial_records: Queue[SerialRecord] = Queue()
        self.serial_reader = SerialLogReader(self.serial_records)
        self.board_events = BoardEventState()
        self.board_statuses: deque[BoardFrameStatus] = deque(maxlen=max(int(history_frames), 1))
        self.latest_board_status: BoardFrameStatus | None = None
        self.latest_board_config: BoardDeploymentConfig | None = None
        self.latest_board_height_cm: int | None = None
        self.latest_overflow_delta = 0
        self.board_status_last_at_monotonic: float | None = None
        self.serial_connected_at_monotonic: float | None = None
        self.board_link_health_state = "disconnected"
        self.alert_popup = FallAlertPopup(root)
        self.alarm_sound = FallAlarmSound(root)
        self._point_draw_count = 0
        self._timeline_dirty = True
        self._xz_presence_artist = None
        self._xz_motion_artist = None
        self._xz_target_artist = None
        self._xz_target_text = None
        self._yz_presence_artist = None
        self._yz_motion_artist = None
        self._yz_target_artist = None
        self._yz_target_text = None
        self._closed = False

        self.point_status_var = tk.StringVar(value="点云未连接")
        self.point_health_var = tk.StringVar(value="数据：未连接")
        self.point_metrics_var = tk.StringVar(value="帧 --  点 --  频率 --Hz")
        self.deployment_status_var = tk.StringVar(value="部署自检：等待连接")
        self.board_status_var = tk.StringVar(value="板端状态: 等待 FALL 日志")
        self.event_status_var = tk.StringVar(value="当前状态：等待板端状态")
        self.last_event_var = tk.StringVar(value="最近事件：无")
        self.spi_index_var = tk.IntVar(value=int(self.device_config.spi_index))
        self.spi_freq_mode_var = tk.IntVar(value=int(self.device_config.spi_freq_mode))
        self.serial_status_var = tk.StringVar(value="串口未连接")
        self.session_log_var = tk.StringVar(value=f"自动日志: {self.session_log.path}")
        self.serial_port_var = tk.StringVar(value=serial_port)
        self.serial_baud_var = tk.IntVar(value=int(serial_baud))
        self.serial_auto_reconnect_var = tk.BooleanVar(value=True)
        self.point_auto_reconnect_var = tk.BooleanVar(value=True)
        self.audio_alarm_enabled_var = tk.BooleanVar(value=True)
        self.mount_height_cm_var = tk.StringVar(value="310")
        self.command_var = tk.StringVar()
        self._build_ui()
        self._write_session_note("自动启动固件模式：上电后直接连接，不发送 mmwc start")
        self._configure_source_ui()
        if self.ui_mode == "engineering":
            self._setup_axes()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._schedule_serial_poll()

    def _configure_source_ui(self) -> None:
        if self.source_mode == "replay":
            self._set_point_connect_button_text("开始回放")
            self.point_status_var.set("录制回放未开始")
            self._set_serial_connect_button_text("回放日志随点云")
            self.serial_connect_button.configure(state="disabled")
            self.radar_start_button.configure(state="disabled")
            self.radar_stop_button.configure(state="disabled")
            self.serial_status_var.set("回放模式：串口日志随点云回放")

    def _build_ui(self) -> None:
        if self.ui_mode == "monitor":
            self._build_monitor_ui()
            return
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True)
        self.point_tab = ttk.Frame(self.notebook, padding=8)
        self.serial_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(self.point_tab, text="实时点云")
        self.notebook.add(self.serial_tab, text="串口控制台")
        self._build_point_tab()
        self._build_serial_tab()
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed, add="+")

    def _build_monitor_ui(self) -> None:
        self.root.configure(bg="#111827")
        monitor = tk.Frame(self.root, bg="#111827", padx=28, pady=24)
        monitor.pack(fill="both", expand=True)
        monitor.columnconfigure(0, weight=1)
        monitor.rowconfigure(4, weight=1)

        header = tk.Frame(monitor, bg="#111827")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 18))
        header.columnconfigure(0, weight=1)
        tk.Label(
            header,
            text="MRS6240 跌倒监护",
            bg="#111827",
            fg="#f8fafc",
            font=("Microsoft YaHei UI", 24, "bold"),
            anchor="w",
        ).grid(row=0, column=0, sticky="w")
        self.point_connect_button = ttk.Button(header, text="开始监护", command=self.toggle_monitoring)
        self.point_connect_button.grid(row=0, column=1, padx=(12, 8))
        self.serial_connect_button = self.point_connect_button
        ttk.Checkbutton(
            header,
            text="声音告警",
            variable=self.audio_alarm_enabled_var,
            command=self._on_audio_alarm_changed,
        ).grid(row=0, column=2, sticky="e")

        self.deployment_status_label = tk.Label(
            monitor,
            textvariable=self.deployment_status_var,
            bg="#713f12",
            fg="#fef3c7",
            anchor="w",
            padx=22,
            pady=14,
            font=("Microsoft YaHei UI", 15, "bold"),
        )
        self.deployment_status_label.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        self.event_status_label = tk.Label(
            monitor,
            textvariable=self.event_status_var,
            bg="#17212b",
            fg="#cbd5e1",
            anchor="w",
            padx=22,
            pady=30,
            font=("Microsoft YaHei UI", 24, "bold"),
            wraplength=900,
            justify="left",
        )
        self.event_status_label.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        self.last_event_label = tk.Label(
            monitor,
            textvariable=self.last_event_var,
            bg="#17212b",
            fg="#94a3b8",
            anchor="w",
            padx=22,
            pady=0,
            font=("Microsoft YaHei UI", 11),
        )
        self.last_event_label.grid(row=3, column=0, sticky="ew")

        status_area = tk.Frame(monitor, bg="#111827")
        status_area.grid(row=4, column=0, sticky="nsew", pady=(22, 0))
        status_area.columnconfigure(0, weight=1)
        self.monitor_connection_var = tk.StringVar(value="控制口：未连接    点云：未连接    板端状态：未连接")
        self.monitor_config_var = tk.StringVar(value="安装配置：等待板端读取")
        tk.Label(
            status_area,
            textvariable=self.monitor_connection_var,
            bg="#111827",
            fg="#cbd5e1",
            anchor="w",
            font=("Microsoft YaHei UI", 14),
        ).grid(row=0, column=0, sticky="ew", pady=(0, 16))
        tk.Label(
            status_area,
            textvariable=self.monitor_config_var,
            bg="#111827",
            fg="#94a3b8",
            anchor="w",
            font=("Microsoft YaHei UI", 13),
        ).grid(row=1, column=0, sticky="ew")
        tk.Label(
            status_area,
            textvariable=self.session_log_var,
            bg="#111827",
            fg="#64748b",
            anchor="w",
            font=("Microsoft YaHei UI", 9),
        ).grid(row=2, column=0, sticky="ew", pady=(22, 0))

    def toggle_monitoring(self) -> None:
        if self.serial_reader.is_connected or (self.point_source is not None and self.point_source.is_running):
            self._stop_point_source()
            self._disconnect_serial()
            self._reset_board_state()
            return
        self.connect_device()

    def _build_point_tab(self) -> None:
        self.point_tab.columnconfigure(0, weight=1)
        self.point_tab.rowconfigure(2, weight=1)

        controls = ttk.Frame(self.point_tab)
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        controls.columnconfigure(5, weight=1)
        ttk.Button(controls, text="一键直连", command=self.connect_device).grid(row=0, column=0, padx=(0, 6))
        self.point_connect_button = ttk.Button(controls, text="连接点云", command=self.toggle_point_source)
        self.point_connect_button.grid(row=0, column=1, padx=(0, 6))
        ttk.Checkbutton(
            controls,
            text="点云自动重连",
            variable=self.point_auto_reconnect_var,
            command=self._on_point_auto_reconnect_changed,
        ).grid(row=0, column=2, padx=(0, 12))
        ttk.Checkbutton(
            controls,
            text="声音告警",
            variable=self.audio_alarm_enabled_var,
            command=self._on_audio_alarm_changed,
        ).grid(row=0, column=6, sticky="e")
        ttk.Label(controls, textvariable=self.point_status_var).grid(row=0, column=3, padx=(0, 12), sticky="w")
        ttk.Label(controls, textvariable=self.point_health_var).grid(row=0, column=4, padx=(0, 12), sticky="w")
        ttk.Label(controls, textvariable=self.point_metrics_var).grid(row=0, column=5, sticky="w")
        ttk.Label(controls, text="SPI index").grid(row=1, column=0, sticky="e", pady=(4, 0))
        ttk.Spinbox(controls, from_=0, to=8, width=5, textvariable=self.spi_index_var).grid(
            row=1, column=1, padx=(4, 8), pady=(4, 0), sticky="w"
        )
        ttk.Label(controls, text="freqMode").grid(row=1, column=2, sticky="e", pady=(4, 0))
        ttk.Spinbox(controls, from_=0, to=3, width=5, textvariable=self.spi_freq_mode_var).grid(
            row=1, column=3, padx=(4, 12), pady=(4, 0), sticky="w"
        )
        status_bar = tk.Frame(self.point_tab, bg="#17212b", highlightthickness=1, highlightbackground="#334155")
        status_bar.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self.deployment_status_label = tk.Label(
            status_bar,
            textvariable=self.deployment_status_var,
            bg="#713f12",
            fg="#fef3c7",
            anchor="w",
            padx=14,
            pady=7,
            font=("Microsoft YaHei UI", 12, "bold"),
        )
        self.deployment_status_label.pack(fill="x")
        self.event_status_label = tk.Label(
            status_bar,
            textvariable=self.event_status_var,
            bg="#17212b",
            fg="#cbd5e1",
            anchor="w",
            padx=14,
            pady=7,
            font=("Microsoft YaHei UI", 14, "bold"),
        )
        self.event_status_label.pack(fill="x")
        self.last_event_label = tk.Label(
            status_bar,
            textvariable=self.last_event_var,
            bg="#17212b",
            fg="#94a3b8",
            anchor="w",
            padx=14,
            pady=0,
            font=("Microsoft YaHei UI", 9),
        )
        self.last_event_label.pack(fill="x", pady=(0, 6))

        plot_area = ttk.Frame(self.point_tab)
        plot_area.grid(row=2, column=0, sticky="nsew")
        plot_area.columnconfigure(0, weight=1, uniform="plot")
        plot_area.columnconfigure(1, weight=1, uniform="plot")
        plot_area.rowconfigure(1, weight=1)
        plot_area.rowconfigure(2, weight=0)

        ttk.Label(plot_area, text="XY 2D 俯视图（天花板世界坐标）  X/Y：水平位置").grid(row=0, column=0, sticky="w")
        ttk.Label(plot_area, text="XZ / YZ 2D 侧视图（世界坐标）  Z：离地高度").grid(row=0, column=1, sticky="w")
        self.xy_canvas = tk.Canvas(plot_area, bg="#0b0f14", highlightthickness=1, highlightbackground="#29323d")
        self.xy_canvas.grid(row=1, column=0, padx=(0, 6), pady=(6, 0), sticky="nsew")
        self.projection_figure = Figure(figsize=(5.4, 4.2), dpi=100, facecolor="#0b0f14")
        self.xz_axis = self.projection_figure.add_subplot(211)
        self.yz_axis = self.projection_figure.add_subplot(212, sharey=self.xz_axis)
        self.projection_canvas = FigureCanvasTkAgg(self.projection_figure, master=plot_area)
        self.projection_canvas.get_tk_widget().grid(row=1, column=1, padx=(6, 0), pady=(6, 0), sticky="nsew")
        self.xy_canvas.bind("<Configure>", lambda _event: self._draw_xy())

        self.timeline_figure = Figure(figsize=(10.8, 2.8), dpi=100, facecolor="#0b0f14")
        self.timeline_ax_height = self.timeline_figure.add_subplot(311)
        self.timeline_ax_motion = self.timeline_figure.add_subplot(312, sharex=self.timeline_ax_height)
        self.timeline_ax_velocity = self.timeline_figure.add_subplot(313, sharex=self.timeline_ax_height)
        self.timeline_axes = (self.timeline_ax_height, self.timeline_ax_motion, self.timeline_ax_velocity)
        self.timeline_canvas = FigureCanvasTkAgg(self.timeline_figure, master=plot_area)
        self.timeline_canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, pady=(8, 0), sticky="ew")

        info = ttk.Frame(self.point_tab)
        info.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        info.columnconfigure(1, weight=1)
        ttk.Label(info, text="蓝=存在点  橙=运动点  世界Z=P95  黄/绿=板端目标遥测").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(info, textvariable=self.board_status_var).grid(row=0, column=1, sticky="e")

    def _build_serial_tab(self) -> None:
        self.serial_tab.columnconfigure(0, weight=1)
        self.serial_tab.rowconfigure(2, weight=1)
        controls = ttk.Frame(self.serial_tab)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(8, weight=1)
        ttk.Label(controls, text="串口").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.serial_port_var, width=10).grid(row=0, column=1, padx=(0, 8))
        ttk.Label(controls, text="波特率").grid(row=0, column=2, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.serial_baud_var, width=10).grid(row=0, column=3, padx=(0, 8))
        self.serial_connect_button = ttk.Button(controls, text="连接串口", command=self.toggle_serial)
        self.serial_connect_button.grid(row=0, column=4, padx=(0, 8))
        ttk.Button(controls, text="清空窗口", command=self.clear_serial_log).grid(row=0, column=5, padx=(0, 8))
        ttk.Checkbutton(
            controls,
            text="自动重连",
            variable=self.serial_auto_reconnect_var,
            command=self._on_auto_reconnect_changed,
        ).grid(row=0, column=6, padx=(0, 8))
        ttk.Label(controls, textvariable=self.serial_status_var).grid(row=0, column=8, sticky="e")
        ttk.Label(self.serial_tab, textvariable=self.session_log_var, foreground="#64748b").grid(
            row=3, column=0, sticky="w", pady=(6, 0)
        )

        command_row = ttk.Frame(self.serial_tab)
        command_row.grid(row=1, column=0, sticky="ew", pady=(8, 8))
        command_row.columnconfigure(0, weight=1)
        command_entry = ttk.Entry(command_row, textvariable=self.command_var)
        command_entry.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        command_entry.bind("<Return>", lambda _event: self.send_command())
        self.radar_start_button = ttk.Button(
            command_row,
            text="启动雷达（调试）",
            command=lambda: self.send_radar_command(RADAR_START_COMMAND),
        )
        self.radar_start_button.grid(row=0, column=1, padx=(0, 6))
        self.radar_stop_button = ttk.Button(
            command_row,
            text="停止雷达",
            command=lambda: self.send_radar_command(RADAR_STOP_COMMAND),
        )
        self.radar_stop_button.grid(row=0, column=2, padx=(0, 6))
        ttk.Button(command_row, text="发送命令", command=self.send_command).grid(row=0, column=3)
        ttk.Label(command_row, text="安装高度(cm)").grid(row=0, column=4, padx=(12, 4))
        ttk.Entry(command_row, textvariable=self.mount_height_cm_var, width=7).grid(row=0, column=5, padx=(0, 6))
        ttk.Button(command_row, text="设置高度", command=self.send_mount_height).grid(row=0, column=6)
        ttk.Button(command_row, text="读取高度", command=self.request_mount_height).grid(row=0, column=7, padx=(6, 0))

        ttk.Button(command_row, text="读取配置", command=self.request_deployment_configuration).grid(
            row=0, column=8, padx=(6, 0)
        )

        log_frame = ttk.Frame(self.serial_tab)
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.serial_log_text = tk.Text(
            log_frame,
            wrap="none",
            state="disabled",
            bg="#0b0f14",
            fg="#d8dee9",
            insertbackground="#ffffff",
            font=("Consolas", 10),
        )
        self.serial_log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.serial_log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.serial_log_text.configure(yscrollcommand=scrollbar.set)

    def _setup_axes(self) -> None:
        self._ensure_projection_artists()
        for axis in self.timeline_axes:
            self._setup_timeline_axis(axis)
        self.projection_canvas.draw_idle()
        self.timeline_canvas.draw_idle()

    def _ensure_projection_artists(self) -> None:
        artist = getattr(self, "_xz_presence_artist", None)
        if artist is not None and artist.axes is self.xz_axis:
            return
        self._setup_projection_axis(
            self.xz_axis,
            title="XZ 侧视图",
            x_label="X left + (m)",
            x_limits=(self.bounds.x_min_m, self.bounds.x_max_m),
        )
        self._setup_projection_axis(
            self.yz_axis,
            title="YZ 侧视图",
            x_label="Y world (m)",
            x_limits=(self.bounds.y_min_m, self.bounds.y_max_m),
        )
        self._xz_presence_artist = self.xz_axis.scatter([], [], color="#58a6ff", s=[], alpha=0.9)
        self._xz_motion_artist = self.xz_axis.scatter([], [], color="#f97316", s=[], alpha=0.95)
        self._xz_target_artist = self.xz_axis.scatter([], [], color="#facc15", s=[], marker="x")
        self._xz_target_text = self.xz_axis.text(
            0.0, 0.0, "", color="#facc15", fontsize=8, visible=False
        )
        self._yz_presence_artist = self.yz_axis.scatter([], [], color="#58a6ff", s=[], alpha=0.9)
        self._yz_motion_artist = self.yz_axis.scatter([], [], color="#f97316", s=[], alpha=0.95)
        self._yz_target_artist = self.yz_axis.scatter([], [], color="#facc15", s=[], marker="x")
        self._yz_target_text = self.yz_axis.text(
            0.0, 0.0, "", color="#facc15", fontsize=8, visible=False
        )
        self.projection_figure.subplots_adjust(left=0.14, right=0.98, top=0.93, bottom=0.12, hspace=0.48)

    def _setup_projection_axis(self, axis, *, title: str, x_label: str, x_limits: tuple[float, float]) -> None:
        axis.set_facecolor("#0b0f14")
        axis.set_xlim(*x_limits)
        axis.set_ylim(self.bounds.z_min_m, self.bounds.z_max_m)
        axis.set_title(title, color="#cbd5e1", fontsize=9, pad=5)
        axis.set_xlabel(x_label, color="#cbd5e1", fontsize=8, labelpad=3)
        axis.set_ylabel("Z height (m)", color="#cbd5e1", fontsize=8, labelpad=4)
        axis.tick_params(colors="#94a3b8", labelsize=7)
        axis.grid(True, color="#1e2833", linewidth=0.6)
        axis.axhline(0.0, color="#22c55e", linewidth=1.0, linestyle="--", alpha=0.9)
        for spine in axis.spines.values():
            spine.set_color("#334155")

    def _setup_timeline_axis(self, axis) -> None:
        axis.set_facecolor("#0b0f14")
        axis.grid(True, color="#1e2833", linewidth=0.6)
        axis.tick_params(colors="#94a3b8", labelsize=7)
        for spine in axis.spines.values():
            spine.set_color("#334155")

    def toggle_point_source(self) -> None:
        if self.point_source is not None and self.point_source.is_running:
            self._stop_point_source()
            return
        self._reset_point_view()
        self._reset_board_state()
        if self.source_mode == "replay":
            self._start_replay_source()
            return

        self._connect_point_source(show_error=True)

    def _connect_point_source(self, *, show_error: bool) -> bool:
        self.deployment_check_requested = True
        if self.point_source is not None and self.point_source.is_running:
            return True
        config = HifDeviceConfig(
            transport=self.device_config.transport,
            spi_index=int(self.spi_index_var.get()),
            spi_freq_mode=int(self.spi_freq_mode_var.get()),
            uart_com=self.device_config.uart_com,
            uart_baud_rate=self.device_config.uart_baud_rate,
            dll_path=self.device_config.dll_path,
        )
        source = HifDevicePointCloudSource(config=config)
        self.point_source = source
        try:
            source.start(self.point_buffer)
        except Exception as exc:
            try:
                source.stop()
                source.join(timeout_s=1.0)
            except Exception:
                pass
            self.point_source = None
            self.point_source_started_at_monotonic = None
            self.point_last_frame_at_monotonic = None
            self.point_health_state = "error"
            self.point_status_var.set("点云连接失败")
            self.point_health_var.set("数据：连接失败")
            self._set_point_connect_button_text("连接点云")
            self._write_session_note(f"点云连接失败: {exc}")
            self._schedule_point_reconnect()
            if show_error:
                messagebox.showerror("点云连接失败", str(exc), parent=self.root)
            return False
        self._cancel_point_reconnect()
        self.point_source_started_at_monotonic = time.monotonic()
        self.point_last_frame_at_monotonic = None
        self.point_health_state = "waiting"
        self.point_status_var.set("点云已连接")
        self.point_health_var.set("数据：等待首帧")
        self._set_point_connect_button_text("断开点云")
        self._write_session_note(
            f"点云已连接 SPI index={config.spi_index} freqMode={config.spi_freq_mode}"
        )
        self._schedule_point_poll()
        return True

    def connect_device(self) -> None:
        """Connect the already-running radar without issuing a start command."""
        if self.source_mode != "device":
            self.point_status_var.set("回放模式不连接设备")
            return
        self.deployment_check_requested = True
        serial_connected = self._connect_serial(show_error=False)
        point_connected = self._connect_point_source(show_error=False)
        if serial_connected and point_connected:
            self._write_session_note("一键直连成功：未发送 mmwc start")
        else:
            self._write_session_note("一键直连未完成，等待设备连接或自动重连")

    def _start_replay_source(self) -> None:
        if self.session_dir is None:
            self.point_status_var.set("缺少录制目录")
            messagebox.showerror("录制回放失败", "replay 模式需要 --session-dir。", parent=self.root)
            return
        try:
            self.point_source = create_replay_source(
                self.session_dir,
                speed=self.replay_speed,
                loop=self.replay_loop,
            )
            self.replay_session = self.point_source.session
            self.point_source.start(self.point_buffer, self.serial_records)
        except Exception as exc:
            self.replay_session = None
            self.point_source = None
            self.point_status_var.set("录制回放失败")
            self._set_point_connect_button_text("开始回放")
            messagebox.showerror("录制回放失败", str(exc), parent=self.root)
            return

        self.point_status_var.set(f"回放中 {self.replay_session.stem}")
        self.point_health_state = "replay"
        self.point_health_var.set("数据：回放中")
        self._set_point_connect_button_text("停止回放")
        if self.replay_session.serial_log_available:
            self.serial_status_var.set("回放日志：已加载")
        else:
            self.serial_status_var.set("回放日志：文件不存在，仅显示点云")
        self._schedule_point_poll()

    def _reset_point_view(self) -> None:
        self.history.clear()
        self.current_view = None
        self.point_buffer.take_pending()
        self._point_draw_count = 0
        self._timeline_dirty = True

    def _reset_board_state(self) -> None:
        if self.source_mode == "replay":
            while True:
                try:
                    self.serial_records.get_nowait()
                except Empty:
                    break
        self.board_events = BoardEventState()
        self._stop_alarm_sound("状态已重置")
        self.board_statuses.clear()
        self.latest_board_status = None
        self.board_status_last_at_monotonic = None
        if self.source_mode == "device" and self.serial_reader.is_connected:
            self.serial_connected_at_monotonic = time.monotonic()
            self.board_link_health_state = "waiting"
            self.serial_status_var.set("串口已连接，等待板端状态")
        else:
            self.serial_connected_at_monotonic = None
            self.board_link_health_state = "disconnected"
            self.serial_status_var.set("串口未连接" if self.source_mode == "device" else "回放日志未开始")
        self.board_status_var.set("板端状态: 等待 FALL 日志")
        self._set_event_status("当前状态：等待板端状态", foreground="#cbd5e1", background="#17212b")
        self._set_last_event("最近事件：无")

    def _set_event_status(self, text: str, *, foreground: str, background: str) -> None:
        self.event_status_var.set(text)
        label = getattr(self, "event_status_label", None)
        if label is not None:
            label.configure(fg=foreground, bg=background)
        last_label = getattr(self, "last_event_label", None)
        if last_label is not None:
            last_label.configure(bg=background)

    def _set_last_event(self, text: str) -> None:
        variable = getattr(self, "last_event_var", None)
        if variable is not None:
            variable.set(text)

    def _stop_point_source(self) -> None:
        self._cancel_point_reconnect()
        if self.point_poll_after_id is not None:
            self.root.after_cancel(self.point_poll_after_id)
            self.point_poll_after_id = None
        if self.point_source is not None:
            self.point_source.stop()
            self.point_source.join(timeout_s=1.0)
        self.point_source = None
        self.point_source_started_at_monotonic = None
        self.point_last_frame_at_monotonic = None
        self.point_health_state = "disconnected"
        if self.source_mode == "replay":
            self.point_status_var.set("录制回放已停止")
            self.point_health_var.set("数据：回放已停止")
            self._set_point_connect_button_text("开始回放")
            self.serial_status_var.set("回放日志未开始")
        else:
            self.point_status_var.set("点云未连接")
            self.point_health_var.set("数据：未连接")
            self._set_point_connect_button_text("连接点云")

    def _set_point_connect_button_text(self, text: str) -> None:
        button = getattr(self, "point_connect_button", None)
        if button is None:
            return
        if getattr(self, "ui_mode", "engineering") == "monitor":
            text = {
                "连接点云": "开始监护",
                "断开点云": "停止监护",
                "重新连接点云": "重新连接监护",
            }.get(text, text)
        button.configure(text=text)

    def _set_serial_connect_button_text(self, text: str) -> None:
        button = getattr(self, "serial_connect_button", None)
        if button is None:
            return
        if getattr(self, "ui_mode", "engineering") == "monitor":
            text = {"连接串口": "开始监护", "断开串口": "停止监护", "重新连接": "重新连接监护"}.get(text, text)
        button.configure(text=text)

    def _schedule_point_poll(self) -> None:
        if self._closed:
            return
        if self.point_poll_after_id is not None:
            self.root.after_cancel(self.point_poll_after_id)
        self.point_poll_after_id = self.root.after(self.poll_ms, self._poll_point_frames)

    def _on_point_auto_reconnect_changed(self) -> None:
        if self.point_auto_reconnect_var.get():
            if self.point_health_state in {"error", "stale", "stopped"}:
                self._schedule_point_reconnect()
        else:
            self._cancel_point_reconnect()

    def _schedule_point_reconnect(self) -> None:
        if (
            self._closed
            or self.source_mode != "device"
            or not self.point_auto_reconnect_var.get()
            or self.point_reconnect_after_id is not None
        ):
            return
        self.point_reconnect_after_id = self.root.after(
            POINT_RECONNECT_DELAY_MS,
            self._retry_point_connection,
        )

    def _retry_point_connection(self) -> None:
        self.point_reconnect_after_id = None
        if self._closed or self.source_mode != "device" or not self.point_auto_reconnect_var.get():
            return
        source = self.point_source
        if source is not None:
            try:
                source.stop()
                source.join(timeout_s=1.0)
            except Exception:
                pass
        self.point_source = None
        self.point_source_started_at_monotonic = None
        self.point_last_frame_at_monotonic = None
        self.point_health_state = "reconnecting"
        self.point_status_var.set("正在自动重连点云")
        self.point_health_var.set("数据：等待重连")
        self._set_point_connect_button_text("重新连接点云")
        self._write_session_note("正在自动重连点云")
        self._connect_point_source(show_error=False)

    def _cancel_point_reconnect(self) -> None:
        if self.point_reconnect_after_id is None:
            return
        try:
            self.root.after_cancel(self.point_reconnect_after_id)
        except tk.TclError:
            pass
        self.point_reconnect_after_id = None

    def _update_point_health(self) -> str:
        source = self.point_source
        if source is None:
            return self.point_health_state
        if self.source_mode == "replay":
            self.point_health_var.set("数据：回放中")
            return "replay"
        now = time.monotonic()
        if not source.is_running:
            return self._set_point_health_state(
                "stopped",
                point_status="点云读取已停止",
                point_health="数据：读取任务停止，等待重连",
            )
        reference = self.point_last_frame_at_monotonic or self.point_source_started_at_monotonic or now
        age_s = max(now - reference, 0.0)
        if age_s >= POINT_DATA_STALE_AFTER_S:
            return self._set_point_health_state(
                "stale",
                point_status="点云数据超时",
                point_health=f"数据：{age_s:.1f}s 无新帧，等待重连",
            )
        if self.point_last_frame_at_monotonic is None:
            return self._set_point_health_state(
                "waiting",
                point_status="点云已连接",
                point_health=f"数据：等待首帧 {age_s:.1f}s",
            )
        return self._set_point_health_state(
            "healthy",
            point_status="点云已连接",
            point_health=f"数据：正常，{age_s:.1f}s 前更新",
        )

    def _set_point_health_state(self, state: str, *, point_status: str, point_health: str) -> str:
        previous = self.point_health_state
        self.point_health_state = state
        self.point_status_var.set(point_status)
        self.point_health_var.set(point_health)
        if state == "stale" and previous != "stale":
            self._write_session_note(point_health)
        elif state == "stopped" and previous != "stopped":
            self._write_session_note("点云读取任务已停止，等待自动重连")
        elif state == "healthy" and previous in {"stale", "stopped", "error", "reconnecting"}:
            self._write_session_note("点云数据已恢复")
        return state

    def _poll_point_frames(self) -> None:
        self.point_poll_after_id = None
        if self._closed or self.point_source is None:
            return
        pending = self.point_buffer.take_pending()
        if pending:
            self.point_last_frame_at_monotonic = time.monotonic()
            for frame in pending:
                display_frame = transform_frame_to_world(
                    frame,
                    yz_rotation_deg=getattr(self, "point_yz_rotation_deg", 0.0),
                    mount_height_m=getattr(self, "point_mount_height_m", 0.0),
                )
                self.history.append(raw_view_from_frame(display_frame))
            self.current_view = self.history.views[-1]
        point_health = self._update_point_health()
        if pending:
            if self._point_tab_is_visible():
                self._draw_current_frame()
            else:
                self._update_point_status()
        else:
            self._update_point_status()
        if self.point_source is not None and self.point_source.is_running:
            if point_health == "stale":
                self._schedule_point_reconnect()
            self._schedule_point_poll()
        elif self.source_mode == "replay" and self.point_source is not None:
            self.point_status_var.set("录制回放已完成")
            self.point_health_var.set("数据：回放完成")
            self._set_point_connect_button_text("重新回放")
            self._update_point_status()
        elif self.source_mode == "device":
            self._update_point_health()
            self._schedule_point_reconnect()

    def _draw_current_frame(self, *, force_timeline: bool = False) -> None:
        self._draw_xy()
        self._draw_projections()
        self._point_draw_count += 1
        if force_timeline or self._timeline_dirty or self._point_draw_count % self.draw_timeline_every == 0:
            self._draw_timeline()
            self._timeline_dirty = False
        self._update_point_status()

    def _point_tab_is_visible(self) -> bool:
        notebook = getattr(self, "notebook", None)
        point_tab = getattr(self, "point_tab", None)
        if notebook is None or point_tab is None:
            return False
        try:
            return notebook.select() == str(point_tab)
        except tk.TclError:
            return False

    def _on_tab_changed(self, _event=None) -> None:
        if self._point_tab_is_visible() and (self.current_view is not None or self.board_statuses):
            self._draw_current_frame(force_timeline=True)

    def _update_point_status(self) -> None:
        if self.current_view is None:
            self.point_metrics_var.set("帧 --  点 --  频率 --Hz")
            return
        views = self.history.views
        frequency = "--"
        if len(views) >= 2:
            delta_ms = views[-1].frame.timestamp_ms - views[-2].frame.timestamp_ms
            if delta_ms > 0:
                frequency = f"{1000.0 / delta_ms:.1f}"
        stats = live_source_stats(self.point_source, self.point_buffer) if self.point_source is not None else None
        transfer = f"发布={stats.published_count} 使用={stats.consumed_count}" if stats else ""
        self.point_metrics_var.set(
            f"帧 {self.current_view.frame.frame_index}  点 {self.current_view.total_count} "
            f"存在 {self.current_view.presence_count} 运动 {self.current_view.motion_count} "
            f"频率 {frequency}Hz  {transfer}"
        )

    def _draw_xy(self) -> None:
        canvas = self.xy_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 220)
        height = max(canvas.winfo_height(), 220)
        self._draw_xy_grid(canvas, width, height)
        view = self.current_view
        if view is None:
            canvas.create_text(width / 2, height / 2, text="等待点云", fill="#94a3b8", font=("Microsoft YaHei UI", 14))
            return
        for point in view.presence_points:
            self._draw_xy_point(canvas, point, width, height, fill="#58a6ff", radius=3)
        for point in view.motion_points:
            self._draw_xy_point(canvas, point, width, height, fill="#f97316", radius=4)
        self._draw_board_target_xy(canvas, width, height)
        if self.board_events.active:
            state = self.board_events.last_event.state if self.board_events.last_event is not None else "confirmed"
            background, foreground = fall_alert_colors(state)
            canvas.create_rectangle(12, 12, min(width - 12, 300), 44, fill=background, outline=foreground)
            canvas.create_text(22, 28, anchor="w", text=self.event_status_var.get(), fill=foreground, font=("Microsoft YaHei UI", 9, "bold"))

    def _draw_board_target_xy(self, canvas: tk.Canvas, width: int, height: int) -> None:
        status = self.latest_board_status
        if status is None or not status.has_target or status.x_cm is None or status.y_cm is None:
            return
        target_x = status.x_cm / 100.0
        target_y = status.y_cm / 100.0
        x, y = self._project_xy(target_x, target_y, width, height)
        association = board_target_association(self.current_view, status)
        color = "#22c55e" if association is not None and association.is_aligned else "#facc15"
        x_scale = (width - 58.0) / max(self.bounds.x_max_m - self.bounds.x_min_m, 0.001)
        y_scale = (height - 42.0) / max(self.bounds.y_max_m - self.bounds.y_min_m, 0.001)
        canvas.create_oval(
            x - TARGET_ASSOCIATION_RADIUS_M * x_scale,
            y - TARGET_ASSOCIATION_RADIUS_M * y_scale,
            x + TARGET_ASSOCIATION_RADIUS_M * x_scale,
            y + TARGET_ASSOCIATION_RADIUS_M * y_scale,
            outline=color,
            width=1,
            dash=(3, 3),
        )
        canvas.create_oval(x - 7, y - 7, x + 7, y + 7, outline=color, width=2)
        canvas.create_line(x - 10, y, x + 10, y, fill=color, width=2)
        canvas.create_line(x, y - 10, x, y + 10, fill=color, width=2)
        canvas.create_text(
            x + 9,
            y - 9,
            anchor="sw",
            text=f"板端目标\n{board_target_association_text(association)}",
            fill=color,
            font=("Microsoft YaHei UI", 8, "bold"),
        )

    def _draw_xy_grid(self, canvas: tk.Canvas, width: int, height: int) -> None:
        for x in self._ticks(self.bounds.x_min_m, self.bounds.x_max_m):
            x1, _ = self._project_xy(x, self.bounds.y_min_m, width, height)
            x2, _ = self._project_xy(x, self.bounds.y_max_m, width, height)
            canvas.create_line(x1, 0, x2, height, fill="#1e2833")
            canvas.create_text(x1, height - 10, text=f"{x:g}m", fill="#74808f", font=("Microsoft YaHei UI", 8))
        for y in self._ticks(self.bounds.y_min_m, self.bounds.y_max_m):
            _, y1 = self._project_xy(self.bounds.x_min_m, y, width, height)
            _, y2 = self._project_xy(self.bounds.x_max_m, y, width, height)
            canvas.create_line(0, y1, width, y2, fill="#1e2833")
            canvas.create_text(18, y1, text=f"{y:g}m", fill="#74808f", font=("Microsoft YaHei UI", 8))
        canvas.create_text(width / 2, height - 26, text="雷达位置 / X 左侧为正", fill="#9aa7b6", font=("Microsoft YaHei UI", 9))
        canvas.create_text(width - 52, 18, text="Y 正方向", fill="#9aa7b6", font=("Microsoft YaHei UI", 9))

    def _draw_xy_point(self, canvas: tk.Canvas, point: Point3D, width: int, height: int, *, fill: str, radius: int) -> None:
        x, y = self._project_xy(point.x_m, point.y_m, width, height)
        canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=fill, outline="")

    def _project_xy(self, x: float, y: float, width: int, height: int) -> tuple[float, float]:
        left, right, top, bottom = 42.0, max(width - 16.0, 43.0), 12.0, max(height - 30.0, 31.0)
        px = left + (x - self.bounds.x_min_m) / max(self.bounds.x_max_m - self.bounds.x_min_m, 0.001) * (right - left)
        py = bottom - (y - self.bounds.y_min_m) / max(self.bounds.y_max_m - self.bounds.y_min_m, 0.001) * (bottom - top)
        return px, py

    @staticmethod
    def _ticks(low: float, high: float) -> list[float]:
        start = int(low)
        end = int(high)
        return [float(value) for value in range(start, end + 1) if low <= value <= high]

    def _draw_projections(self) -> None:
        self._ensure_projection_artists()
        view = self.current_view
        presence_points = view.presence_points if view is not None else ()
        motion_points = view.motion_points if view is not None else ()
        self._set_projection_points(
            self._xz_presence_artist,
            self._yz_presence_artist,
            presence_points,
            size=18,
        )
        self._set_projection_points(
            self._xz_motion_artist,
            self._yz_motion_artist,
            motion_points,
            size=28,
        )

        status = self.latest_board_status
        if status is not None and status.has_target and status.x_cm is not None and status.y_cm is not None and status.z_cm is not None:
            target_x = status.x_cm / 100.0
            target_y = status.y_cm / 100.0
            target_z = status.z_cm / 100.0
            self._set_2d_offsets(self._xz_target_artist, ((target_x, target_z),), size=70)
            self._set_2d_offsets(self._yz_target_artist, ((target_y, target_z),), size=70)
            self._set_target_text(self._xz_target_text, (target_x, target_z))
            self._set_target_text(self._yz_target_text, (target_y, target_z))
        else:
            self._set_2d_offsets(self._xz_target_artist, (), size=70)
            self._set_2d_offsets(self._yz_target_artist, (), size=70)
            self._hide_target_text(self._xz_target_text)
            self._hide_target_text(self._yz_target_text)
        self.projection_canvas.draw_idle()

    def _set_projection_points(self, xz_artist, yz_artist, points: tuple[Point3D, ...], *, size: int) -> None:
        self._set_2d_offsets(
            xz_artist,
            tuple((point.x_m, point.z_m) for point in points),
            size=size,
        )
        self._set_2d_offsets(
            yz_artist,
            tuple((point.y_m, point.z_m) for point in points),
            size=size,
        )

    @staticmethod
    def _set_2d_offsets(artist, coordinates: tuple[tuple[float, float], ...], *, size: int) -> None:
        artist.set_offsets(coordinates if coordinates else ((0.0, 0.0),))
        artist.set_sizes([size] * len(coordinates))
        artist.set_visible(bool(coordinates))

    @staticmethod
    def _set_target_text(text_artist, position: tuple[float, float]) -> None:
        text_artist.set_position(position)
        text_artist.set_text(" 板端目标")
        text_artist.set_visible(True)

    @staticmethod
    def _hide_target_text(text_artist) -> None:
        text_artist.set_text("")
        text_artist.set_visible(False)

    def _draw_timeline(self) -> None:
        series = self.history.timeline()
        board_statuses = tuple(self.board_statuses)
        for axis in self.timeline_axes:
            axis.clear()
            self._setup_timeline_axis(axis)
        if not series.frame_indices and not board_statuses:
            self.timeline_canvas.draw_idle()
            return

        x = series.frame_indices
        if x:
            self.timeline_ax_height.plot(
                x,
                [value if value is not None else float("nan") for value in series.p95_z_m],
                color="#38bdf8",
                linewidth=1.0,
                alpha=0.65,
                label="Raw Z P95",
            )
            self.timeline_ax_motion.plot(x, series.motion_counts, color="#f97316", linewidth=1.0, alpha=0.55, label="Raw motion")
            self.timeline_ax_velocity.plot(x, series.max_velocity_m_s, color="#a78bfa", linewidth=1.0, alpha=0.55, label="Raw max vel")

        if board_statuses:
            board_x = [status.frame_index for status in board_statuses]
            board_z = [status.z_cm / 100.0 if status.has_target and status.z_cm is not None else float("nan") for status in board_statuses]
            board_h = [status.height_cm / 100.0 if status.has_target and status.height_cm is not None else float("nan") for status in board_statuses]
            board_pts = [status.point_count if status.point_count is not None else float("nan") for status in board_statuses]
            board_mot = [status.motion_count if status.motion_count is not None else float("nan") for status in board_statuses]
            board_vel = [status.velocity_cm_s / 100.0 if status.velocity_cm_s is not None else float("nan") for status in board_statuses]
            self.timeline_ax_height.plot(board_x, board_z, color="#facc15", linewidth=1.6, label="Board Z")
            self.timeline_ax_height.plot(board_x, board_h, color="#22c55e", linewidth=1.1, linestyle="--", label="Board H")
            self.timeline_ax_motion.plot(board_x, board_pts, color="#60a5fa", linewidth=1.5, label="Board pts")
            self.timeline_ax_motion.plot(board_x, board_mot, color="#fb923c", linewidth=1.2, linestyle="--", label="Board mot")
            self.timeline_ax_velocity.plot(board_x, board_vel, color="#fef08a", linewidth=1.6, label="Board v")

        self.timeline_ax_height.set_ylabel("Z/H (m)", color="#cbd5e1", rotation=0, labelpad=8, fontsize=9)
        self.timeline_ax_motion.set_ylabel("Pts/Mot", color="#cbd5e1", rotation=0, labelpad=8, fontsize=9)
        self.timeline_ax_velocity.set_ylabel("V (m/s)", color="#cbd5e1", rotation=0, labelpad=8, fontsize=9)
        self.timeline_ax_velocity.set_xlabel("Frame", color="#cbd5e1", fontsize=9)

        for event in self.board_events.events:
            try:
                event_frame = int(event.fields.get("frame", ""))
            except ValueError:
                continue
            if event.state == "suspected":
                color = "#60a5fa"
            elif event.state == "confirmed":
                color = "#ef4444"
            else:
                color = "#22c55e"
            for axis in self.timeline_axes:
                axis.axvline(event_frame, color=color, linewidth=1.0, alpha=0.85)

        current_candidates = [*x, *(status.frame_index for status in board_statuses)]
        current = self.current_view.frame.frame_index if self.current_view is not None else current_candidates[-1]
        for axis in self.timeline_axes:
            axis.axvline(current, color="#f8fafc", linewidth=1.0, alpha=0.9)
            axis.margins(x=0.01)
            handles, labels = axis.get_legend_handles_labels()
            if handles:
                axis.legend(handles, labels, loc="upper right", fontsize=7, ncol=3, framealpha=0.35)
        self.timeline_figure.subplots_adjust(left=0.13, right=0.99, top=0.96, bottom=0.16, hspace=0.2)
        self.timeline_canvas.draw_idle()

    def toggle_serial(self) -> None:
        if self.source_mode == "replay":
            self.serial_status_var.set("回放模式不连接串口，日志由录制源提供")
            return
        if self.serial_reader.is_connected:
            self._disconnect_serial()
            return
        self._connect_serial(show_error=True)

    def _connect_serial(self, *, show_error: bool) -> bool:
        self.deployment_check_requested = True
        try:
            self.serial_reader.connect(self.serial_port_var.get().strip(), int(self.serial_baud_var.get()))
        except Exception as exc:
            self.serial_status_var.set("串口连接失败，等待自动重连")
            self._set_serial_connect_button_text("重新连接")
            self._write_session_note(f"串口连接失败: {exc}")
            self._schedule_serial_reconnect()
            if show_error:
                messagebox.showerror("串口连接失败", str(exc), parent=self.root)
            return False
        self._cancel_serial_reconnect()
        self.serial_connected_at_monotonic = time.monotonic()
        self.board_status_last_at_monotonic = None
        self.board_link_health_state = "waiting"
        self.latest_board_config = None
        self.latest_board_height_cm = None
        self.latest_overflow_delta = 0
        monitor_config_var = getattr(self, "monitor_config_var", None)
        if monitor_config_var is not None:
            monitor_config_var.set("安装配置：等待板端读取")
        self.serial_status_var.set(f"串口已连接 {self.serial_port_var.get().strip()} @ {self.serial_baud_var.get()}")
        self._set_serial_connect_button_text("断开串口")
        self._write_session_note(
            f"串口已连接 {self.serial_port_var.get().strip()} @ {self.serial_baud_var.get()}"
        )
        self.request_deployment_configuration(automatic=True)
        return True

    def _disconnect_serial(self) -> None:
        self._cancel_serial_reconnect()
        self.serial_reader.stop()
        self.serial_connected_at_monotonic = None
        self.board_status_last_at_monotonic = None
        self.board_link_health_state = "disconnected"
        self.serial_status_var.set("串口未连接")
        self._set_serial_connect_button_text("连接串口")
        self._write_session_note("串口已手动断开")

    def _on_auto_reconnect_changed(self) -> None:
        if self.serial_auto_reconnect_var.get():
            self._schedule_serial_reconnect()
        else:
            self._cancel_serial_reconnect()

    def _on_audio_alarm_changed(self) -> None:
        if not self.audio_alarm_enabled_var.get():
            self._stop_alarm_sound("已手动关闭")
            return
        last_event = self.board_events.last_event
        if self.source_mode == "device" and self.board_events.active and last_event is not None and last_event.state == "confirmed":
            self._start_alarm_sound()

    def _schedule_serial_reconnect(self) -> None:
        if (
            self._closed
            or self.source_mode != "device"
            or not self.serial_auto_reconnect_var.get()
            or self.serial_reader.is_connected
            or self.serial_reconnect_after_id is not None
        ):
            return
        self.serial_reconnect_after_id = self.root.after(
            SERIAL_RECONNECT_DELAY_MS,
            self._retry_serial_connection,
        )

    def _retry_serial_connection(self) -> None:
        self.serial_reconnect_after_id = None
        if self._closed or self.serial_reader.is_connected:
            return
        self.serial_status_var.set("正在自动重连串口")
        self._connect_serial(show_error=False)

    def _cancel_serial_reconnect(self) -> None:
        if self.serial_reconnect_after_id is None:
            return
        try:
            self.root.after_cancel(self.serial_reconnect_after_id)
        except tk.TclError:
            pass
        self.serial_reconnect_after_id = None

    def send_radar_command(self, command: str) -> None:
        self._send_serial_command(command)

    def send_mount_height(self) -> None:
        try:
            height_cm = int(self.mount_height_cm_var.get().strip())
        except ValueError:
            messagebox.showwarning("安装高度无效", "请输入整数厘米值，例如 310。", parent=self.root)
            return
        if not 100 <= height_cm <= 500:
            messagebox.showwarning("安装高度无效", "安装高度范围为 100~500 cm。", parent=self.root)
            return
        self._send_serial_command(f"fall_height {height_cm}")

    def request_mount_height(self) -> None:
        self._send_serial_command("fall_height")

    def request_deployment_configuration(self, *, automatic: bool = False) -> None:
        if self._send_serial_command(FALL_CONFIG_COMMAND) and automatic:
            self._write_session_note("已自动请求板端部署配置")

    def send_command(self) -> None:
        command = self.command_var.get().strip()
        if command and self._send_serial_command(command):
            self.command_var.set("")

    def _send_serial_command(self, command: str) -> bool:
        if self.source_mode == "replay":
            self.serial_status_var.set("回放模式不发送命令")
            return False
        try:
            self.serial_reader.send_line(command)
        except Exception as exc:
            self.serial_status_var.set(f"发送失败: {exc}")
            return False
        self.serial_status_var.set(f"已发送: {command}")
        return True

    def clear_serial_log(self) -> None:
        self.serial_log_text.configure(state="normal")
        self.serial_log_text.delete("1.0", "end")
        self.serial_log_text.configure(state="disabled")

    def _schedule_serial_poll(self) -> None:
        if self._closed:
            return
        self.serial_poll_after_id = self.root.after(self.poll_ms, self._poll_serial_records)

    def _poll_serial_records(self) -> None:
        self.serial_poll_after_id = None
        records: list[SerialRecord] = []
        has_board_status = False
        accept_configuration = getattr(self, "_accept_board_configuration", None)
        while True:
            try:
                record = self.serial_records.get_nowait()
            except Empty:
                break
            records.append(record)
            if accept_configuration is not None:
                accept_configuration(record.line)
            if record.status is not None:
                has_board_status = True
                self._accept_board_status(record.status)
            if record.event is not None and self.board_events.accept(record.event):
                self._handle_board_event(record.event)
        if records:
            session_log = getattr(self, "session_log", None)
            if session_log is not None:
                session_log.write_records(records)
            self._append_serial_records(records)
        for record in records:
            if record.direction == "error":
                self._handle_serial_error(record)
        self._update_board_link_health()
        refresh_deployment = getattr(self, "_refresh_deployment_status", None)
        if refresh_deployment is not None:
            refresh_deployment()
        if has_board_status and self._point_tab_is_visible():
            source = getattr(self, "point_source", None)
            if source is None or not source.is_running:
                self._draw_current_frame(force_timeline=True)
        self._schedule_serial_poll()

    def _accept_board_status(self, status: BoardFrameStatus) -> None:
        was_stale = self.board_link_health_state == "stale"
        existing = next((item for item in self.board_statuses if item.frame_index == status.frame_index), None)
        if existing is not None:
            self.board_statuses.remove(existing)
        previous = self.board_statuses[-1] if self.board_statuses else None
        self.board_statuses.append(status)
        self.latest_board_status = status
        self.board_status_last_at_monotonic = time.monotonic()
        self.board_link_health_state = "healthy"
        if was_stale:
            self._write_session_note("板端状态日志已恢复")
        alert_popup = getattr(self, "alert_popup", None)
        if alert_popup is not None:
            alert_popup.sync_board_state(status.state)
        if status.state != 2:
            self._stop_alarm_sound("板端退出确认状态")
        self._timeline_dirty = True
        delta = board_overflow_delta(previous, status)
        self.latest_overflow_delta = delta
        frame_rate = "--"
        if previous is not None:
            delta_seconds = (status.received_at - previous.received_at).total_seconds()
            delta_frames = status.frame_index - previous.frame_index
            if delta_seconds > 0 and delta_frames > 0:
                frame_rate = f"{delta_frames / delta_seconds:.1f}Hz"
        state_label = {0: "正常", 1: "疑似", 2: "确认"}.get(status.state, str(status.state))
        association_text = board_target_association_text(board_target_association(self.current_view, status))
        target_text = (
            f"x={status.x_cm / 100.0:.2f}m y={status.y_cm / 100.0:.2f}m "
            f"z={status.z_cm / 100.0:.2f}m h={status.height_cm / 100.0:.2f}m"
            if status.has_target and status.x_cm is not None and status.y_cm is not None and status.z_cm is not None and status.height_cm is not None
            else "target=0"
        )
        self.board_status_var.set(
            f"板端 frame={status.frame_index} state={state_label} {target_text} "
            f"pts={status.point_count or 0} mot={status.motion_count or 0} "
            f"v={(status.velocity_cm_s or 0) / 100.0:.2f}m/s 帧率={frame_rate} overflowΔ={delta} {association_text}"
        )
        if status.state == 0 and not self.board_events.active:
            self._set_event_status("当前状态：正常", foreground="#dcfce7", background="#14532d")
        if self.source_mode == "device" and self.serial_reader.is_connected:
            self.serial_status_var.set(
                f"板端通信正常 {self.serial_port_var.get().strip()} @ {self.serial_baud_var.get()}"
            )
        self._refresh_deployment_status()

    def _accept_board_configuration(self, line: str) -> None:
        config = parse_board_deployment_config(line)
        if config is not None:
            previous = self.latest_board_config
            self.latest_board_config = config
            if config != previous:
                self._write_session_note(
                    "板端配置回显: "
                    f"高度={config.mount_height_cm}cm 俯角={config.pitch_deg}° "
                    f"ROI=({config.roi_x_min_cm}..{config.roi_x_max_cm},"
                    f"{config.roi_y_min_cm}..{config.roi_y_max_cm},"
                    f"{config.roi_z_min_cm}..{config.roi_z_max_cm})"
                )
            monitor_config_var = getattr(self, "monitor_config_var", None)
            if monitor_config_var is not None:
                monitor_config_var.set(
                    f"安装配置：高度 {config.mount_height_cm}cm    俯角 {config.pitch_deg}°    "
                    f"区域 X {config.roi_x_min_cm}..{config.roi_x_max_cm}cm / "
                    f"Y {config.roi_y_min_cm}..{config.roi_y_max_cm}cm"
                )
        height_cm = parse_board_mount_height(line)
        if height_cm is not None:
            self.latest_board_height_cm = height_cm
        self._refresh_deployment_status()

    def _expected_mount_height_cm(self) -> int | None:
        try:
            value = int(self.mount_height_cm_var.get().strip())
        except (AttributeError, TypeError, ValueError):
            return None
        return value if 100 <= value <= 500 else None

    def _refresh_deployment_status(self) -> None:
        status_var = getattr(self, "deployment_status_var", None)
        status_label = getattr(self, "deployment_status_label", None)
        serial_reader = getattr(self, "serial_reader", None)
        if status_var is None or serial_reader is None:
            return
        readiness = deployment_readiness(
            source_mode=getattr(self, "source_mode", "device"),
            monitoring_requested=bool(getattr(self, "deployment_check_requested", False)),
            serial_connected=bool(getattr(serial_reader, "is_connected", False)),
            point_health=getattr(self, "point_health_state", "disconnected"),
            board_health=getattr(self, "board_link_health_state", "disconnected"),
            board_config=getattr(self, "latest_board_config", None),
            expected_mount_height_cm=self._expected_mount_height_cm(),
            overflow_delta=getattr(self, "latest_overflow_delta", 0),
        )
        status_var.set(f"部署自检：{readiness.title} - {readiness.detail}")
        self._refresh_monitor_connection_summary()
        if status_label is None:
            return
        colors = {
            "ready": ("#14532d", "#dcfce7"),
            "pending": ("#713f12", "#fef3c7"),
            "error": ("#7f1d1d", "#fee2e2"),
            "info": ("#1e3a5f", "#dbeafe"),
        }
        background, foreground = colors[readiness.level]
        status_label.configure(bg=background, fg=foreground)

    def _refresh_monitor_connection_summary(self) -> None:
        status_var = getattr(self, "monitor_connection_var", None)
        serial_reader = getattr(self, "serial_reader", None)
        if status_var is None or serial_reader is None:
            return
        status_var.set(
            monitor_connection_summary(
                serial_connected=bool(getattr(serial_reader, "is_connected", False)),
                point_health=getattr(self, "point_health_state", "disconnected"),
                board_health=getattr(self, "board_link_health_state", "disconnected"),
            )
        )

    def _update_board_link_health(self) -> None:
        if self.source_mode != "device" or not self.serial_reader.is_connected:
            return
        now = time.monotonic()
        reference = self.board_status_last_at_monotonic or self.serial_connected_at_monotonic or now
        age_s = max(now - reference, 0.0)
        if age_s < BOARD_STATUS_STALE_AFTER_S:
            if self.board_status_last_at_monotonic is None:
                self.board_link_health_state = "waiting"
                self.serial_status_var.set(f"串口已连接，等待板端状态 {age_s:.1f}s")
            elif self.board_link_health_state == "stale":
                self.board_link_health_state = "healthy"
                self._write_session_note("板端状态日志已恢复")
            return
        if self.board_link_health_state != "stale":
            self.board_link_health_state = "stale"
            self._write_session_note(f"板端状态日志超时: {age_s:.1f}s 无 FALL 帧")
        self.serial_status_var.set(f"板端状态超时：{age_s:.1f}s 无 FALL 帧")

    def _handle_serial_error(self, record: SerialRecord) -> None:
        self.board_link_health_state = "error"
        self.serial_status_var.set("串口读取异常，等待自动重连")
        self._set_serial_connect_button_text("重新连接")
        self._write_session_note(record.line)
        self._schedule_serial_reconnect()

    def _write_session_note(self, message: str) -> None:
        session_log = getattr(self, "session_log", None)
        if session_log is not None:
            session_log.write_note(message)

    def _append_serial_record(self, record: SerialRecord) -> None:
        self._append_serial_records((record,))

    def _append_serial_records(self, records) -> None:
        text = "".join(self._format_serial_record(record) for record in records)
        if not text:
            return
        serial_log_text = getattr(self, "serial_log_text", None)
        if serial_log_text is None:
            return
        serial_log_text.configure(state="normal")
        serial_log_text.insert("end", text)
        serial_log_text.see("end")
        serial_log_text.configure(state="disabled")

    @staticmethod
    def _format_serial_record(record: SerialRecord) -> str:
        timestamp = record.received_at.strftime("%H:%M:%S.%f")[:-3]
        if record.direction == "tx":
            prefix = "发→"
        elif record.direction == "error":
            prefix = "错误"
        else:
            prefix = "收←"
        return f"[{timestamp}] {prefix} {record.line}\n"

    def _capture_event_evidence(self, event: FallSerialEvent) -> None:
        if self.source_mode != "device":
            return
        prefix = self._event_evidence_prefix(event)
        self._write_event_evidence(event, prefix=prefix, phase="event")
        after_id: str | None = None

        def capture_follow_up() -> None:
            if after_id is not None:
                self.event_evidence_after_ids.discard(after_id)
            if not self._closed:
                self._write_event_evidence(event, prefix=prefix, phase="after_5s")

        after_id = self.root.after(EVIDENCE_POST_EVENT_MS, capture_follow_up)
        self.event_evidence_after_ids.add(after_id)

    @staticmethod
    def _event_evidence_prefix(event: FallSerialEvent) -> str:
        frame = event.fields.get("frame", "unknown")
        safe_frame = "".join(char for char in frame if char.isdigit()) or "unknown"
        return f"{event.received_at:%Y%m%d_%H%M%S_%f}_{event.state}_frame{safe_frame}"

    def _write_event_evidence(self, event: FallSerialEvent, *, prefix: str, phase: str) -> None:
        try:
            payload = self._event_evidence_payload(event, phase=phase)
            json_path = self.evidence_directory / f"{prefix}_{phase}.json"
            json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            timeline_figure = getattr(self, "timeline_figure", None)
            if timeline_figure is not None:
                timeline_figure.savefig(
                    self.evidence_directory / f"{prefix}_{phase}_timeline.png",
                    dpi=140,
                    facecolor=timeline_figure.get_facecolor(),
                )
            self._write_session_note(f"事件留证已保存: {json_path.name}")
        except Exception as exc:
            self._write_session_note(f"事件留证保存失败: {exc}")

    def _event_evidence_payload(self, event: FallSerialEvent, *, phase: str) -> dict:
        views = self.history.views[-EVIDENCE_HISTORY_FRAME_LIMIT:]
        statuses = tuple(self.board_statuses)[-EVIDENCE_HISTORY_FRAME_LIMIT:]
        return {
            "captured_at": datetime.now().isoformat(timespec="milliseconds"),
            "phase": phase,
            "event": {
                "state": event.state,
                "fields": dict(event.fields),
                "raw_line": event.raw_line,
                "received_at": event.received_at.isoformat(timespec="milliseconds"),
            },
            "point_frames": [self._evidence_frame_payload(view) for view in views],
            "board_statuses": [self._evidence_status_payload(status) for status in statuses],
        }

    @staticmethod
    def _evidence_frame_payload(view: RawFrameView) -> dict:
        return {
            "frame": view.frame.frame_index,
            "timestamp_ms": view.frame.timestamp_ms,
            "presence_points": [
                [point.x_m, point.y_m, point.z_m, point.velocity_m_s, point.snr_db]
                for point in view.presence_points
            ],
            "motion_points": [
                [point.x_m, point.y_m, point.z_m, point.velocity_m_s, point.snr_db]
                for point in view.motion_points
            ],
        }

    @staticmethod
    def _evidence_status_payload(status: BoardFrameStatus) -> dict:
        return {
            "frame": status.frame_index,
            "state": status.state,
            "track": status.track,
            "target": status.has_target,
            "x_cm": status.x_cm,
            "y_cm": status.y_cm,
            "z_cm": status.z_cm,
            "height_cm": status.height_cm,
            "points": status.point_count,
            "motion": status.motion_count,
            "velocity_cm_s": status.velocity_cm_s,
            "overflow": status.overflow,
            "received_at": status.received_at.isoformat(timespec="milliseconds"),
        }

    def _handle_board_event(self, event: FallSerialEvent) -> None:
        background, foreground = fall_alert_colors(event.state)
        frame = event.fields.get("frame", "--")
        if event.state == "recovered":
            self._stop_alarm_sound("人员恢复")
            alert_popup = getattr(self, "alert_popup", None)
            if alert_popup is not None:
                alert_popup.sync_board_state(0)
            self._set_event_status("当前状态：正常，系统已重新布防", foreground=foreground, background=background)
            self._set_last_event(f"最近事件：已恢复  frame={frame}")
            self._capture_event_evidence(event)
            return
        title = "疑似跌倒" if event.state == "suspected" else "确认跌倒"
        self._set_event_status(
            f"当前状态：{title}  frame={frame}  请确认人员状态",
            foreground=foreground,
            background=background,
        )
        self._set_last_event(f"最近事件：{title}  frame={frame}")
        self._capture_event_evidence(event)
        self.alert_popup.show(event)
        if event.state == "confirmed":
            self._start_alarm_sound()

    def _start_alarm_sound(self) -> None:
        sound = getattr(self, "alarm_sound", None)
        enabled_var = getattr(self, "audio_alarm_enabled_var", None)
        if (
            sound is not None
            and self.source_mode == "device"
            and (enabled_var is None or enabled_var.get())
            and sound.start()
        ):
            self._write_session_note("确认跌倒：声音告警已启动")

    def _stop_alarm_sound(self, reason: str) -> None:
        sound = getattr(self, "alarm_sound", None)
        if sound is not None and sound.stop():
            self._write_session_note(f"声音告警已停止：{reason}")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for after_id in (
            self.point_poll_after_id,
            self.point_reconnect_after_id,
            self.serial_poll_after_id,
            self.serial_reconnect_after_id,
            *self.event_evidence_after_ids,
        ):
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except tk.TclError:
                    pass
        self.event_evidence_after_ids.clear()
        self._stop_point_source()
        self.serial_reader.stop()
        self.session_log.close()
        self.alarm_sound.stop()
        self.alert_popup.close()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Display MRS6240 raw point clouds and board serial fall events.")
    parser.add_argument(
        "--source",
        choices=("device", "replay"),
        default="device",
        help="point/log source: device hardware or a captured session directory",
    )
    parser.add_argument("--session-dir", default=None, help="capture-station session directory used by replay mode")
    parser.add_argument("--replay-speed", type=float, default=1.0, help="recorded-session playback speed")
    parser.add_argument("--replay-loop", action="store_true", help="loop the recorded session until stopped")
    parser.add_argument("--spi-index", type=int, default=0, help="CH347 SPI device index")
    parser.add_argument("--spi-freq-mode", type=int, default=0, help="CH347 SPI frequency mode")
    parser.add_argument("--serial-port", default="COM3", help="board control/log serial port")
    parser.add_argument("--serial-baud", type=int, default=115200, help="board control/log serial baud rate")
    parser.add_argument(
        "--ui-mode",
        choices=("engineering", "monitor"),
        default="engineering",
        help="engineering shows point-cloud diagnostics; monitor shows the simplified operator UI",
    )
    parser.add_argument("--device-transport", choices=("spi", "uart"), default="spi", help="point-cloud transport")
    parser.add_argument("--point-uart-com", default="COM13", help="point-cloud UART when --device-transport uart")
    parser.add_argument("--point-uart-baud", default="1000000", help="point-cloud UART baud rate")
    parser.add_argument("--dll-path", default=None, help="optional HifMsgDataCollectionLib.dll path")
    parser.add_argument("--poll-ms", type=int, default=20, help="GUI queue polling interval")
    parser.add_argument("--history-frames", type=int, default=600, help="raw frames kept for the timeline")
    parser.add_argument(
        "--session-log-dir",
        default=str(DEFAULT_SESSION_LOG_DIR),
        help="directory for automatically saved realtime serial logs",
    )
    parser.add_argument("--x-min", type=float, default=-2.5, help="XY plot minimum X")
    parser.add_argument("--x-max", type=float, default=2.5, help="XY plot maximum X")
    parser.add_argument("--y-min", type=float, default=-2.5, help="XY plot minimum Y")
    parser.add_argument("--y-max", type=float, default=2.5, help="XY plot maximum Y")
    parser.add_argument("--z-min", type=float, default=-1.0, help="XZ/YZ projection minimum Z")
    parser.add_argument("--z-max", type=float, default=2.5, help="XZ/YZ projection maximum Z")
    parser.add_argument(
        "--yz-rotation-deg",
        type=float,
        default=DEFAULT_POINT_YZ_ROTATION_DEG,
        help="rotate raw point-cloud Y/Z into world coordinates (90=ceiling-down installation)",
    )
    parser.add_argument(
        "--mount-height-m",
        type=float,
        default=DEFAULT_POINT_MOUNT_HEIGHT_M,
        help="radar installation height used for point-cloud display, in meters",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    device_config = HifDeviceConfig(
        transport=args.device_transport,
        spi_index=args.spi_index,
        spi_freq_mode=args.spi_freq_mode,
        uart_com=args.point_uart_com,
        uart_baud_rate=args.point_uart_baud,
        dll_path=args.dll_path,
    )
    bounds = PlotBounds(
        x_min_m=args.x_min,
        x_max_m=args.x_max,
        y_min_m=args.y_min,
        y_max_m=args.y_max,
        z_min_m=args.z_min,
        z_max_m=args.z_max,
    )
    SerialEventLiveVisualizerApp(
        root,
        device_config=device_config,
        source_mode=args.source,
        session_dir=args.session_dir,
        replay_speed=args.replay_speed,
        replay_loop=args.replay_loop,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        poll_ms=args.poll_ms,
        history_frames=args.history_frames,
        bounds=bounds,
        point_yz_rotation_deg=args.yz_rotation_deg,
        point_mount_height_m=args.mount_height_m,
        session_log_dir=args.session_log_dir,
        ui_mode=args.ui_mode,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
