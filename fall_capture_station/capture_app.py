from __future__ import annotations

import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_capture_station.camera_recorder import CameraRecorder, frame_to_photoimage
    from fall_capture_station.serial_control import SerialController, list_serial_ports
    from fall_capture_station.session_recorder import SessionRecorder, wall_time_now
    from fall_capture_station.v2_protocol import (
        DEFAULT_RADAR_PROFILE,
        PROTOCOL_VERSION,
        SESSION_SCENARIOS,
        CaptureProfile,
        CaptureProfileError,
    )
else:
    from .camera_recorder import CameraRecorder, frame_to_photoimage
    from .serial_control import SerialController, list_serial_ports
    from .session_recorder import SessionRecorder, wall_time_now
    from .v2_protocol import (
        DEFAULT_RADAR_PROFILE,
        PROTOCOL_VERSION,
        SESSION_SCENARIOS,
        CaptureProfile,
        CaptureProfileError,
    )

from fall_detection.device_source import HifDeviceConfig, HifDevicePointCloudSource
from fall_detection.live_source import LatestFrameBuffer
from fall_detection.serial_event import BoardFrameStatus, FallSerialEvent, parse_board_status, parse_fall_event


DEFAULT_DATA_ROOT = Path(__file__).resolve().parent / "data"
RADAR_START_COMMAND = "mmwc start"
RADAR_STOP_COMMAND = "mmwc stop"
PENDING_SESSION_LOG_LIMIT = 2000


def _format_board_fall_status(
    status: BoardFrameStatus,
    *,
    overflow_delta: int | None = None,
) -> tuple[str, str]:
    state_text, color = {
        0: ("正常", "#15803d"),
        1: ("疑似跌倒", "#1d4ed8"),
        2: ("确认跌倒", "#b91c1c"),
    }.get(status.state, (f"未知状态({status.state})", "#64748b"))
    overflow_text = ""
    if status.overflow is not None:
        overflow_text = f"  溢出={status.overflow}"
        if overflow_delta is not None:
            overflow_text += f" (+{overflow_delta})"
    return f"跌倒状态：{state_text}  frame={status.frame_index}{overflow_text}", color


def _overflow_delta(previous: int | None, current: int | None) -> int | None:
    if current is None:
        return None
    if previous is None or current < previous:
        return current
    return current - previous


def _format_fall_event_status(event: FallSerialEvent) -> tuple[str, str]:
    state_text, color = {
        "suspected": ("疑似跌倒", "#1d4ed8"),
        "confirmed": ("确认跌倒", "#b91c1c"),
        "recovered": ("已恢复正常", "#15803d"),
    }[event.state]
    frame = event.fields.get("frame", "--")
    return f"跌倒状态：{state_text}  frame={frame}", color


class CaptureStationApp:
    def __init__(self, root: tk.Tk, *, data_root: Path = DEFAULT_DATA_ROOT):
        self.root = root
        self.data_root = data_root
        self.camera = CameraRecorder()
        self.serial = SerialController(on_line=self._on_serial_line)
        self.radar_source: HifDevicePointCloudSource | None = None
        self.radar_buffer: LatestFrameBuffer | None = None
        self.session: SessionRecorder | None = None
        self._pending_session_log_lines: list[str] = []
        self.recording = False
        self.radar_frame_count = 0
        self.camera_frame_count = 0
        self.video_photo = None
        self._last_board_overflow: int | None = None

        self.session_name_var = tk.StringVar(value="")
        self.mount_height_var = tk.StringVar(value="1.63")
        self.pitch_deg_var = tk.StringVar(value="30")
        self.subject_id_var = tk.StringVar(value="P01")
        self.environment_id_var = tk.StringVar(value="room-a")
        self.scenario_var = tk.StringVar(value="mixed")
        self.protocol_version_var = tk.StringVar(value=PROTOCOL_VERSION)
        self.radar_profile_var = tk.StringVar(value=DEFAULT_RADAR_PROFILE)
        self.profile_widgets: list[tuple[tk.Widget, str]] = []
        self.camera_index_var = tk.IntVar(value=0)
        self.spi_index_var = tk.IntVar(value=0)
        self.spi_freq_mode_var = tk.IntVar(value=0)
        self.com_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="115200")
        self.command_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="未连接")
        self.record_var = tk.StringVar(value="未记录")
        self.camera_status_var = tk.StringVar(value="摄像头未打开")
        self.radar_status_var = tk.StringVar(value="数据口未连接")
        self.serial_status_var = tk.StringVar(value="控制口未连接")
        self.fall_status_var = tk.StringVar(value="跌倒状态：等待板端 FALL 日志")

        self._build_ui()
        self._refresh_ports()
        self.root.after(30, self._poll_radar)
        self.root.after(33, self._poll_camera)

    def _build_ui(self) -> None:
        self.root.title("MRS6240 数据采集上位机")
        self.root.geometry("1320x820")
        self.root.minsize(1080, 720)
        self.root.columnconfigure(0, weight=2)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=(10, 8))
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        top.columnconfigure(7, weight=1)
        ttk.Label(top, text="会话名").grid(row=0, column=0, padx=(0, 6))
        ttk.Entry(top, textvariable=self.session_name_var, width=28).grid(row=0, column=1, padx=(0, 8))
        ttk.Button(top, text="开始记录", command=self.start_recording).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(top, text="停止记录", command=self.stop_recording).grid(row=0, column=3, padx=(0, 12))
        ttk.Label(top, textvariable=self.record_var, foreground="#b45309").grid(row=0, column=4, sticky="w")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=5, sticky="w")
        ttk.Label(top, text="板端判断").grid(row=0, column=6, padx=(16, 6), sticky="e")
        self.fall_status_label = ttk.Label(top, textvariable=self.fall_status_var, foreground="#64748b")
        self.fall_status_label.grid(row=0, column=7, sticky="w")

        left = ttk.Frame(self.root, padding=(10, 0, 6, 10))
        left.grid(row=1, column=0, sticky="nsew")
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        ttk.Label(left, text="摄像头画面").grid(row=0, column=0, sticky="w")
        self.video_label = ttk.Label(left, background="#0b0f14", anchor="center")
        self.video_label.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        self.log_text = ScrolledText(left, height=9, bg="#0b0f14", fg="#d1d5db", insertbackground="#d1d5db")
        self.log_text.grid(row=2, column=0, sticky="ew", pady=(8, 0))

        right = ttk.Frame(self.root, padding=(6, 0, 10, 10))
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        self._build_capture_profile_panel(right)
        self._build_camera_panel(right)
        self._build_radar_panel(right)
        self._build_serial_panel(right)

    def _build_capture_profile_panel(self, parent) -> None:
        panel = ttk.LabelFrame(parent, text="V2 采集配置", padding=8)
        panel.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for column in (1, 3, 5, 7):
            panel.columnconfigure(column, weight=1)

        ttk.Label(panel, text="高度(m)").grid(row=0, column=0, sticky="w")
        height = ttk.Spinbox(
            panel,
            from_=0.5,
            to=3.0,
            increment=0.01,
            textvariable=self.mount_height_var,
            width=5,
        )
        height.grid(row=0, column=1, sticky="ew", padx=(4, 6))

        ttk.Label(panel, text="俯角(°)").grid(row=0, column=2, sticky="w")
        pitch = ttk.Spinbox(
            panel,
            from_=-90,
            to=90,
            increment=1,
            textvariable=self.pitch_deg_var,
            width=5,
        )
        pitch.grid(row=0, column=3, sticky="ew", padx=(4, 6))

        ttk.Label(panel, text="人员ID").grid(row=0, column=4, sticky="w")
        subject = ttk.Entry(panel, textvariable=self.subject_id_var, width=7)
        subject.grid(row=0, column=5, sticky="ew", padx=(4, 6))

        ttk.Label(panel, text="环境ID").grid(row=0, column=6, sticky="w")
        environment = ttk.Entry(panel, textvariable=self.environment_id_var, width=8)
        environment.grid(row=0, column=7, sticky="ew", padx=(4, 0))

        ttk.Label(panel, text="场景").grid(row=1, column=0, sticky="w", pady=(6, 0))
        scenario = ttk.Combobox(
            panel,
            textvariable=self.scenario_var,
            values=SESSION_SCENARIOS,
            state="readonly",
            width=12,
        )
        scenario.grid(row=1, column=1, columnspan=2, sticky="ew", padx=(4, 6), pady=(6, 0))

        ttk.Label(panel, text="协议").grid(row=1, column=3, sticky="w", pady=(6, 0))
        protocol = ttk.Entry(panel, textvariable=self.protocol_version_var, width=10)
        protocol.grid(row=1, column=4, sticky="ew", padx=(4, 6), pady=(6, 0))

        ttk.Label(panel, text="雷达配置").grid(row=1, column=5, sticky="w", pady=(6, 0))
        radar_profile = ttk.Entry(panel, textvariable=self.radar_profile_var, width=16)
        radar_profile.grid(row=1, column=6, columnspan=2, sticky="ew", padx=(4, 0), pady=(6, 0))

        self.profile_widgets.extend(
            (
                (height, "normal"),
                (pitch, "normal"),
                (subject, "normal"),
                (environment, "normal"),
                (scenario, "readonly"),
                (protocol, "normal"),
                (radar_profile, "normal"),
            )
        )

    def _build_camera_panel(self, parent) -> None:
        panel = ttk.LabelFrame(parent, text="电脑摄像头", padding=8)
        panel.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(panel, text="Index").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(panel, from_=0, to=8, textvariable=self.camera_index_var, width=6).grid(row=0, column=1, padx=(6, 8))
        ttk.Button(panel, text="打开摄像头", command=self.open_camera).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(panel, text="关闭", command=self.close_camera).grid(row=0, column=3)
        ttk.Label(panel, textvariable=self.camera_status_var).grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

    def _build_radar_panel(self, parent) -> None:
        panel = ttk.LabelFrame(parent, text="开发板数据口 HIF", padding=8)
        panel.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(panel, text="SPI index").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(panel, from_=0, to=8, textvariable=self.spi_index_var, width=6).grid(row=0, column=1, padx=(6, 8))
        ttk.Label(panel, text="freqMode").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(panel, from_=0, to=8, textvariable=self.spi_freq_mode_var, width=6).grid(row=0, column=3, padx=(6, 8))
        ttk.Button(panel, text="连接数据口", command=self.connect_radar).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(panel, text="断开数据口", command=self.disconnect_radar).grid(row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(panel, textvariable=self.radar_status_var).grid(row=2, column=0, columnspan=4, sticky="w", pady=(6, 0))

    def _build_serial_panel(self, parent) -> None:
        panel = ttk.LabelFrame(parent, text="控制口命令", padding=8)
        panel.grid(row=3, column=0, sticky="ew", pady=(0, 8))
        panel.columnconfigure(1, weight=1)
        ttk.Label(panel, text="COM").grid(row=0, column=0, sticky="w")
        self.com_combo = ttk.Combobox(panel, textvariable=self.com_var, values=(), width=10)
        self.com_combo.grid(row=0, column=1, sticky="ew", padx=(6, 6))
        ttk.Button(panel, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 6))
        ttk.Entry(panel, textvariable=self.baud_var, width=10).grid(row=0, column=3)
        ttk.Button(panel, text="连接控制口", command=self.connect_serial).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 4))
        ttk.Button(panel, text="断开", command=self.disconnect_serial).grid(row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0))
        self.command_entry = ttk.Entry(panel, textvariable=self.command_var)
        self.command_entry.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0), padx=(0, 6))
        self.command_entry.bind("<Return>", lambda _event: self.send_command())
        ttk.Button(panel, text="发送", command=self.send_command).grid(row=2, column=3, sticky="ew", pady=(8, 0))
        ttk.Button(panel, text="启动雷达", command=self.send_start_commands).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 4))
        ttk.Button(panel, text="停止雷达", command=self.send_stop_command).grid(row=3, column=2, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(panel, text="下发安装高度(m)").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Button(panel, text="设置高度", command=self.send_mount_height).grid(row=4, column=2, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(panel, textvariable=self.serial_status_var).grid(row=5, column=0, columnspan=4, sticky="w", pady=(6, 0))

    def open_camera(self) -> None:
        try:
            self.camera.camera_index = int(self.camera_index_var.get())
            self.camera.open()
        except Exception as exc:
            messagebox.showerror("摄像头打开失败", str(exc))
            return
        self.camera_status_var.set(f"摄像头正在打开 index={self.camera.camera_index}")
        self._log(f"camera opening index={self.camera.camera_index}")

    def close_camera(self) -> None:
        self.camera.close()
        self.camera_status_var.set("摄像头未打开")
        self._log("camera closed")

    def connect_radar(self) -> None:
        if self.radar_source is not None and self.radar_source.is_running:
            return
        self.radar_buffer = LatestFrameBuffer()
        config = HifDeviceConfig(
            transport="spi",
            spi_index=int(self.spi_index_var.get()),
            spi_freq_mode=int(self.spi_freq_mode_var.get()),
        )
        self.radar_source = HifDevicePointCloudSource(config=config)
        try:
            self.radar_source.start(self.radar_buffer)
        except Exception as exc:
            self.radar_source = None
            self.radar_buffer = None
            messagebox.showerror("数据口连接失败", str(exc))
            return
        self.radar_status_var.set(f"数据口已连接 SPI index={config.spi_index} freqMode={config.spi_freq_mode}")
        self._log(self.radar_status_var.get())

    def disconnect_radar(self) -> None:
        if self.radar_source is not None:
            self.radar_source.stop()
            self.radar_source.join(1.0)
        self.radar_source = None
        self.radar_buffer = None
        self.radar_status_var.set("数据口未连接")
        self._log("radar data disconnected")

    def connect_serial(self) -> None:
        try:
            self.serial.connect(self.com_var.get(), int(self.baud_var.get()))
        except Exception as exc:
            messagebox.showerror("控制口连接失败", str(exc))
            return
        self.serial_status_var.set(f"控制口已连接 {self.com_var.get()} @ {self.baud_var.get()}")
        self._log(self.serial_status_var.get())

    def disconnect_serial(self) -> None:
        self.serial.disconnect()
        self.serial_status_var.set("控制口未连接")
        self._log("control serial disconnected")

    def send_command(self) -> None:
        command = self.command_var.get().strip()
        if not command:
            return
        self._send_control_command(command)

    def send_start_commands(self) -> None:
        self._send_control_command(RADAR_START_COMMAND)

    def send_stop_command(self) -> None:
        self._send_control_command(RADAR_STOP_COMMAND)

    def send_mount_height(self) -> None:
        try:
            height_m = float(self.mount_height_var.get().strip())
        except ValueError:
            messagebox.showwarning("安装高度无效", "请输入米数，例如 3.10。")
            return
        height_cm = round(height_m * 100.0)
        if not 100 <= height_cm <= 500:
            messagebox.showwarning("安装高度无效", "安装高度范围为 1.00~5.00 m。")
            return
        self._send_control_command(f"fall_height {height_cm}")

    def _send_control_command(self, command: str) -> None:
        if not self.serial.is_connected:
            messagebox.showwarning("控制口未连接", "请先连接控制口。")
            return
        try:
            self.serial.send_line(command)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))
            return
        self._log(f"发 -> {command}")

    def start_recording(self) -> None:
        if self.recording:
            return
        try:
            profile = self._capture_profile()
        except CaptureProfileError as error:
            messagebox.showerror("采集参数无效", str(error))
            return
        name = self.session_name_var.get().strip() or None
        self.radar_frame_count = 0
        self.camera_frame_count = 0
        self.session = SessionRecorder.create(
            self.data_root,
            session_name=name,
            metadata={
                "app": "capture_app",
                "camera_index": int(self.camera_index_var.get()),
                "spi_index": int(self.spi_index_var.get()),
                "spi_freq_mode": int(self.spi_freq_mode_var.get()),
                **profile.to_metadata(),
            },
        )
        self._set_profile_controls_enabled(False)
        if self.camera.is_open:
            self.camera.start_recording(self.session.camera_video_path)
        self._flush_pending_session_log()
        self.recording = True
        self.record_var.set(f"记录中 {self.session.session_name}")
        self._log(f"recording started: {self.session.session_dir}")

    def stop_recording(self) -> None:
        if not self.recording:
            return
        self.recording = False
        self.camera.stop_recording()
        if self.session is not None:
            self.session.close()
        self._set_profile_controls_enabled(True)
        self.record_var.set("未记录")
        self._log("recording stopped")

    def _capture_profile(self) -> CaptureProfile:
        return CaptureProfile(
            mount_height_m=self.mount_height_var.get(),
            pitch_deg=self.pitch_deg_var.get(),
            subject_id=self.subject_id_var.get(),
            environment_id=self.environment_id_var.get(),
            scenario=self.scenario_var.get(),
            protocol_version=self.protocol_version_var.get(),
            radar_profile=self.radar_profile_var.get(),
        )

    def _set_profile_controls_enabled(self, enabled: bool) -> None:
        for widget, enabled_state in self.profile_widgets:
            widget.configure(state=enabled_state if enabled else "disabled")

    def write_mark(self, mark: str) -> None:
        if self.session is None:
            messagebox.showwarning("未创建采集", "请先点击开始记录。")
            return
        self.session.write_operator_mark(mark, wall_time=wall_time_now())
        self._log(f"mark: {mark}")

    def _poll_radar(self) -> None:
        if self.radar_buffer is not None:
            frame = self.radar_buffer.take_latest()
            if frame is not None:
                self.radar_frame_count += 1
                self.radar_status_var.set(
                    f"数据口运行 帧={self.radar_frame_count} motion={frame.motion_count} presence={frame.presence_count}"
                )
                if self.recording and self.session is not None:
                    self.session.write_radar_frame(frame, monotonic_ns=time.monotonic_ns(), wall_time=wall_time_now())
        self._update_status()
        self.root.after(30, self._poll_radar)

    def _poll_camera(self) -> None:
        error = self.camera.take_error()
        if error is not None:
            self.camera_status_var.set(f"摄像头打开失败: {error}")
            self._log(f"camera error: {error}")
            messagebox.showerror("摄像头打开失败", error)
        frame = self.camera.read_frame()
        if frame is not None:
            self.camera_frame_count += 1
            try:
                self.video_photo = frame_to_photoimage(frame.image_bgr)
                self.video_label.configure(image=self.video_photo, text="")
            except Exception as exc:
                self.video_label.configure(text=f"摄像头显示失败: {exc}", foreground="#dc2626")
            self.camera_status_var.set(f"摄像头运行 帧={self.camera_frame_count}")
            if self.recording and self.session is not None:
                self.session.write_camera_frame(
                    frame_index=frame.frame_index,
                    monotonic_ns=frame.monotonic_ns,
                    wall_time=frame.wall_time,
                    video_time_ms=frame.video_time_ms,
                )
        elif self.camera.is_open and self.camera_frame_count == 0:
            self.camera_status_var.set(
                f"摄像头已打开 index={self.camera.camera_index}，等待首帧"
            )
        self.root.after(33, self._poll_camera)

    def _refresh_ports(self) -> None:
        ports = list_serial_ports()
        if hasattr(self, "com_combo"):
            self.com_combo["values"] = ports
        if ports and not self.com_var.get():
            self.com_var.set(ports[0])

    def _on_serial_line(self, line: str) -> None:
        event = parse_fall_event(line)
        status = parse_board_status(line)
        self.root.after(0, lambda: self._handle_serial_line(line, event, status))

    def _handle_serial_line(
        self,
        line: str,
        event: FallSerialEvent | None,
        status: BoardFrameStatus | None,
    ) -> None:
        self._log(f"收 <- {line}")
        if status is not None:
            overflow_delta = _overflow_delta(self._last_board_overflow, status.overflow)
            self._last_board_overflow = status.overflow
            text, color = _format_board_fall_status(status, overflow_delta=overflow_delta)
            self.fall_status_var.set(text)
            self.fall_status_label.configure(foreground=color)
        if event is not None:
            text, color = _format_fall_event_status(event)
            self.fall_status_var.set(text)
            self.fall_status_label.configure(foreground=color)

    def _log(self, text: str) -> None:
        line = f"[{wall_time_now()}] {text}\n"
        self.log_text.insert("end", line)
        self.log_text.see("end")
        if self.session is not None:
            self._append_session_log_line(line)
        else:
            self._remember_pending_session_log_line(line)

    def _append_session_log_line(self, line: str) -> None:
        if self.session is None:
            return
        with (self.session.session_dir / "serial_control.log").open("a", encoding="utf-8") as f:
            f.write(line)

    def _flush_pending_session_log(self) -> None:
        pending = getattr(self, "_pending_session_log_lines", [])
        if self.session is None or not pending:
            return
        with (self.session.session_dir / "serial_control.log").open("a", encoding="utf-8") as f:
            f.writelines(pending)
        pending.clear()
        self._pending_session_log_lines = pending

    def _remember_pending_session_log_line(self, line: str) -> None:
        pending = getattr(self, "_pending_session_log_lines", [])
        pending.append(line)
        if len(pending) > PENDING_SESSION_LOG_LIMIT:
            del pending[: len(pending) - PENDING_SESSION_LOG_LIMIT]
        self._pending_session_log_lines = pending

    def _update_status(self) -> None:
        self.status_var.set(f"摄像头帧={self.camera_frame_count}  雷达帧={self.radar_frame_count}")

    def close(self) -> None:
        self.stop_recording()
        self.disconnect_radar()
        self.disconnect_serial()
        self.camera.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = CaptureStationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    main()
