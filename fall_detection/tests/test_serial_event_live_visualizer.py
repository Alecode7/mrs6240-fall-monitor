import sys
import unittest
from collections import deque
from datetime import datetime
from queue import Queue
from types import SimpleNamespace
from unittest.mock import patch

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.live_source import LatestFrameBuffer
from fall_detection.raw_visualization import RawFrameHistory, raw_view_from_frame
from fall_detection.serial_event import SerialRecord, parse_board_status, parse_fall_event
from fall_detection.serial_event_live_visualizer import (
    BoardEventState,
    BoardDeploymentConfig,
    FALL_CONFIG_COMMAND,
    FallAlarmSound,
    POINT_DATA_STALE_AFTER_S,
    POINT_RECONNECT_DELAY_MS,
    PlotBounds,
    SerialEventLiveVisualizerApp,
    board_target_association,
    board_target_association_text,
    board_overflow_delta,
    create_replay_source,
    deployment_readiness,
    fall_alert_colors,
    monitor_connection_summary,
    parse_args,
    parse_board_deployment_config,
    parse_board_mount_height,
)


def point_frame(index: int, *, x_m: float = 0.1) -> CombinedFrame:
    return CombinedFrame(
        frame_index=index,
        timestamp_ms=index * 50,
        timestamp_text=f"t{index}",
        presence_points=(Point3D(x_m, 1.0, 0.8, 0.0, 40.0, "presence"),),
        motion_points=(Point3D(-x_m, 1.2, 0.6, 0.3, 30.0, "motion"),),
    )


class TextValue:
    def __init__(self, value: str = ""):
        self.value = value

    def set(self, value: str) -> None:
        self.value = value


class FakeRoot:
    def __init__(self):
        self.after_calls = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))
        return "after-id"

    def after_cancel(self, _after_id):
        return None


class SerialEventLiveVisualizerTests(unittest.TestCase):
    def test_deployment_config_parser_reads_current_board_startup_line(self):
        config = parse_board_deployment_config(
            "FALL_CFG alg=3 mount_h=310 pitch=90 roi_x=-250..250 roi_y=-250..250 "
            "roi_z=0..220 terminal=40 suspect_ms=700"
        )

        self.assertEqual(
            config,
            BoardDeploymentConfig(
                mount_height_cm=310,
                pitch_deg=90,
                roi_x_min_cm=-250,
                roi_x_max_cm=250,
                roi_y_min_cm=-250,
                roi_y_max_cm=250,
                roi_z_min_cm=0,
                roi_z_max_cm=220,
            ),
        )
        self.assertIsNone(parse_board_deployment_config("FALL_CFG mount_h=310 pitch=90"))

    def test_mount_height_parser_accepts_load_set_and_query_responses(self):
        self.assertEqual(parse_board_mount_height("FALL_HEIGHT loaded=310cm"), 310)
        self.assertEqual(parse_board_mount_height("FALL_HEIGHT set=280cm saved=1"), 280)
        self.assertEqual(parse_board_mount_height("FALL_HEIGHT 350cm"), 350)
        self.assertIsNone(parse_board_mount_height("FALL_HEIGHT invalid, range=100..500cm"))

    def test_active_deployment_config_query_uses_fall_config_protocol(self):
        config = parse_board_deployment_config(
            "FALL_CFG alg=3 mount_h=310 pitch=90 roi_x=-250..250 roi_y=-250..250 "
            "roi_z=0..220 terminal=40"
        )

        self.assertEqual(FALL_CONFIG_COMMAND, "fall_config")
        self.assertEqual(config, BoardDeploymentConfig(310, 90, -250, 250, -250, 250, 0, 220))

    def test_request_deployment_configuration_sends_query_without_stopping_radar(self):
        sent: list[str] = []
        notes: list[str] = []
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.source_mode = "device"
        app.serial_reader = SimpleNamespace(send_line=sent.append)
        app.serial_status_var = TextValue()
        app._write_session_note = notes.append

        SerialEventLiveVisualizerApp.request_deployment_configuration(app, automatic=True)

        self.assertEqual(sent, ["fall_config"])
        self.assertEqual(notes, ["已自动请求板端部署配置"])

    def test_deployment_readiness_requires_telemetry_and_matching_configuration(self):
        config = BoardDeploymentConfig(310, 90, -250, 250, -250, 250, 0, 220)

        ready = deployment_readiness(
            source_mode="device",
            monitoring_requested=True,
            serial_connected=True,
            point_health="healthy",
            board_health="healthy",
            board_config=config,
            expected_mount_height_cm=310,
        )
        mismatch = deployment_readiness(
            source_mode="device",
            monitoring_requested=True,
            serial_connected=True,
            point_health="healthy",
            board_health="healthy",
            board_config=config,
            expected_mount_height_cm=280,
        )
        failed = deployment_readiness(
            source_mode="device",
            monitoring_requested=True,
            serial_connected=True,
            point_health="stale",
            board_health="healthy",
            board_config=config,
            expected_mount_height_cm=310,
            overflow_delta=2,
        )

        self.assertEqual(ready.level, "ready")
        self.assertEqual(mismatch.title, "配置不一致")
        self.assertIn("高度 310cm", mismatch.detail)
        self.assertEqual(failed.level, "error")
        self.assertIn("点云数据异常", failed.detail)
        self.assertIn("overflow 增长 +2", failed.detail)

    def test_deployment_readiness_waits_for_user_connection_attempt(self):
        status = deployment_readiness(
            source_mode="device",
            monitoring_requested=False,
            serial_connected=False,
            point_health="disconnected",
            board_health="disconnected",
            board_config=None,
            expected_mount_height_cm=310,
        )

        self.assertEqual(status.level, "pending")
        self.assertEqual(status.title, "等待连接")

    def test_cli_defaults_use_current_board_connections(self):
        with patch.object(sys, "argv", ["serial_event_live_visualizer.py"]):
            args = parse_args()

        self.assertEqual(args.spi_index, 0)
        self.assertEqual(args.spi_freq_mode, 0)
        self.assertEqual(args.serial_port, "COM3")
        self.assertEqual(args.serial_baud, 115200)
        self.assertEqual(args.ui_mode, "engineering")

    def test_cli_accepts_simplified_monitor_ui(self):
        with patch.object(sys, "argv", ["serial_event_live_visualizer.py", "--ui-mode", "monitor"]):
            args = parse_args()

        self.assertEqual(args.ui_mode, "monitor")

    def test_monitor_connection_summary_uses_operator_readable_states(self):
        healthy = monitor_connection_summary(serial_connected=True, point_health="healthy", board_health="healthy")
        failed = monitor_connection_summary(serial_connected=False, point_health="stale", board_health="error")

        self.assertEqual(healthy, "控制口：已连接    点云：正常    板端状态：正常")
        self.assertEqual(failed, "控制口：未连接    点云：超时    板端状态：异常")

    def test_cli_accepts_connection_overrides(self):
        with patch.object(
            sys,
            "argv",
            [
                "serial_event_live_visualizer.py",
                "--spi-index",
                "2",
                "--spi-freq-mode",
                "3",
                "--serial-port",
                "COM8",
                "--serial-baud",
                "9600",
            ],
        ):
            args = parse_args()

        self.assertEqual(args.spi_index, 2)
        self.assertEqual(args.spi_freq_mode, 3)
        self.assertEqual(args.serial_port, "COM8")
        self.assertEqual(args.serial_baud, 9600)

    def test_cli_accepts_recorded_session_replay_options(self):
        with patch.object(
            sys,
            "argv",
            [
                "serial_event_live_visualizer.py",
                "--source",
                "replay",
                "--session-dir",
                r"..\fall_capture_station\data\1.67-60",
                "--replay-speed",
                "2.0",
                "--replay-loop",
            ],
        ):
            args = parse_args()

        self.assertEqual(args.source, "replay")
        self.assertEqual(args.session_dir, r"..\fall_capture_station\data\1.67-60")
        self.assertAlmostEqual(args.replay_speed, 2.0)
        self.assertTrue(args.replay_loop)

    def test_replay_source_factory_loads_session_without_device_source(self):
        with patch("fall_detection.serial_event_live_visualizer.load_recorded_session") as loader, patch(
            "fall_detection.serial_event_live_visualizer.RecordedSessionReplay"
        ) as replay_factory:
            session = object()
            loader.return_value = session

            source = create_replay_source("session-dir", speed=0.5, loop=True)

        loader.assert_called_once_with("session-dir")
        replay_factory.assert_called_once_with(session, speed=0.5, loop=True)
        self.assertIs(source, replay_factory.return_value)

    def test_replay_reset_discards_pending_old_serial_records(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.source_mode = "replay"
        app.serial_records = Queue()
        app.serial_records.put(SerialRecord("rx", "old replay line", datetime.now()))
        app.board_statuses = deque()
        app.latest_board_status = object()
        app.board_status_var = SimpleNamespace(set=lambda _value: None)
        app.event_status_var = SimpleNamespace(set=lambda _value: None)
        app.serial_status_var = SimpleNamespace(set=lambda _value: None)

        SerialEventLiveVisualizerApp._reset_board_state(app)

        self.assertTrue(app.serial_records.empty())
        self.assertFalse(app.board_events.active)
        self.assertIsNone(app.latest_board_status)

    def test_event_consumer_deduplicates_and_tracks_recovery(self):
        state = BoardEventState()
        suspected = parse_fall_event("FALL_EVENT state=suspected frame=10")
        confirmed = parse_fall_event("FALL_EVENT state=confirmed frame=10")
        recovered = parse_fall_event("FALL_EVENT state=recovered frame=12 reason=stood_up")
        assert suspected and confirmed and recovered

        self.assertTrue(state.accept(suspected))
        self.assertFalse(state.accept(suspected))
        self.assertTrue(state.accept(confirmed))
        self.assertTrue(state.active)
        self.assertTrue(state.accept(recovered))
        self.assertFalse(state.active)
        self.assertEqual(state.last_event, recovered)

    def test_unrelated_event_state_does_not_trigger_alarm(self):
        state = BoardEventState()
        self.assertFalse(state.accept(None))
        self.assertFalse(state.active)

    def test_confirmed_alarm_sound_repeats_until_stopped(self):
        root = FakeRoot()
        played: list[str] = []
        sound = FallAlarmSound(
            root,
            interval_ms=1200,
            sound_player=lambda: played.append("tone"),
            play_in_background=False,
        )

        self.assertTrue(sound.start())
        self.assertFalse(sound.start())
        self.assertTrue(sound.active)
        self.assertEqual(played, ["tone"])
        self.assertEqual(root.after_calls[0][0], 1200)
        self.assertTrue(sound.stop())
        self.assertFalse(sound.active)
        self.assertFalse(sound.stop())

    def test_simplified_monitor_omits_serial_console_without_interrupting_serial_polling(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        record = SerialRecord("rx", "FALL_CFG mount_h=310", datetime.now())

        SerialEventLiveVisualizerApp._append_serial_records(app, (record,))

    def test_suspected_fall_uses_blue_alert_colors_and_confirmed_uses_red(self):
        suspected_background, suspected_foreground = fall_alert_colors("suspected")
        confirmed_background, confirmed_foreground = fall_alert_colors("confirmed")
        recovered_background, recovered_foreground = fall_alert_colors("recovered")

        self.assertEqual((suspected_background, suspected_foreground), ("#1d4ed8", "#dbeafe"))
        self.assertEqual((confirmed_background, confirmed_foreground), ("#7f1d1d", "#fee2e2"))
        self.assertEqual((recovered_background, recovered_foreground), ("#14532d", "#dcfce7"))

    def test_board_overflow_delta_reports_growth_and_handles_counter_reset(self):
        previous = parse_board_status("FALL frame=2810 state=0 target=1 overflow=8586")
        current = parse_board_status("FALL frame=2820 state=0 target=1 overflow=8949")
        reset = parse_board_status("FALL frame=10 state=0 target=0 overflow=3")
        assert previous and current and reset

        self.assertEqual(board_overflow_delta(None, previous), 0)
        self.assertEqual(board_overflow_delta(previous, current), 363)
        self.assertEqual(board_overflow_delta(current, reset), 0)

    def test_board_target_association_uses_nearby_point_cloud_centroid(self):
        status = parse_board_status("FALL frame=2 state=0 target=1 x=20 y=100 z=80")
        assert status is not None

        association = board_target_association(raw_view_from_frame(point_frame(2, x_m=0.2)), status)

        assert association is not None
        self.assertEqual(association.candidate_count, 2)
        self.assertTrue(association.is_aligned)
        self.assertIn("对齐", board_target_association_text(association))

    def test_event_evidence_payload_contains_recent_point_and_board_frames(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.history = RawFrameHistory(max_frames=10)
        app.history.append(raw_view_from_frame(point_frame(8)))
        status = parse_board_status("FALL frame=8 state=0 target=1 x=10 y=100 z=80 h=120 pts=4")
        event = parse_fall_event("FALL_EVENT state=confirmed frame=8")
        assert status is not None and event is not None
        app.board_statuses = deque((status,))

        payload = SerialEventLiveVisualizerApp._event_evidence_payload(app, event, phase="event")

        self.assertEqual(payload["event"]["state"], "confirmed")
        self.assertEqual(payload["point_frames"][0]["frame"], 8)
        self.assertEqual(payload["board_statuses"][0]["frame"], 8)

    def test_point_health_marks_stale_data_then_marks_recovery(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.point_source = SimpleNamespace(is_running=True)
        app.source_mode = "device"
        app.point_source_started_at_monotonic = 100.0
        app.point_last_frame_at_monotonic = None
        app.point_health_state = "waiting"
        app.point_status_var = TextValue()
        app.point_health_var = TextValue()

        with patch("fall_detection.serial_event_live_visualizer.time.monotonic", return_value=100.0 + POINT_DATA_STALE_AFTER_S):
            state = SerialEventLiveVisualizerApp._update_point_health(app)

        self.assertEqual(state, "stale")
        self.assertEqual(app.point_status_var.value, "点云数据超时")
        self.assertIn("无新帧", app.point_health_var.value)

        app.point_last_frame_at_monotonic = 104.0
        with patch("fall_detection.serial_event_live_visualizer.time.monotonic", return_value=104.2):
            state = SerialEventLiveVisualizerApp._update_point_health(app)

        self.assertEqual(state, "healthy")
        self.assertEqual(app.point_status_var.value, "点云已连接")
        self.assertIn("正常", app.point_health_var.value)

    def test_point_auto_reconnect_schedules_only_in_device_mode(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app._closed = False
        app.source_mode = "device"
        app.point_auto_reconnect_var = SimpleNamespace(get=lambda: True)
        app.point_reconnect_after_id = None
        app.root = FakeRoot()

        SerialEventLiveVisualizerApp._schedule_point_reconnect(app)

        self.assertEqual(app.point_reconnect_after_id, "after-id")
        self.assertEqual(app.root.after_calls[0][0], POINT_RECONNECT_DELAY_MS)

        app.source_mode = "replay"
        app.point_reconnect_after_id = None
        SerialEventLiveVisualizerApp._schedule_point_reconnect(app)
        self.assertEqual(len(app.root.after_calls), 1)

    def test_board_link_health_marks_missing_status_frames(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.source_mode = "device"
        app.serial_reader = SimpleNamespace(is_connected=True)
        app.serial_connected_at_monotonic = 200.0
        app.board_status_last_at_monotonic = None
        app.board_link_health_state = "waiting"
        app.serial_status_var = TextValue()

        with patch("fall_detection.serial_event_live_visualizer.time.monotonic", return_value=203.0):
            SerialEventLiveVisualizerApp._update_board_link_health(app)

        self.assertEqual(app.board_link_health_state, "stale")
        self.assertIn("超时", app.serial_status_var.value)

        app.board_status_last_at_monotonic = 204.0
        with patch("fall_detection.serial_event_live_visualizer.time.monotonic", return_value=204.2):
            SerialEventLiveVisualizerApp._update_board_link_health(app)

        self.assertEqual(app.board_link_health_state, "healthy")

    def test_draw_current_frame_keeps_xy_and_projections_live_but_throttles_timeline(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        calls = {"xy": 0, "projections": 0, "timeline": 0, "status": 0}
        app.current_view = raw_view_from_frame(point_frame(1))
        app.draw_timeline_every = 10
        app._point_draw_count = 0
        app._timeline_dirty = False
        app._draw_xy = lambda: calls.__setitem__("xy", calls["xy"] + 1)
        app._draw_projections = lambda: calls.__setitem__("projections", calls["projections"] + 1)
        app._draw_timeline = lambda: calls.__setitem__("timeline", calls["timeline"] + 1)
        app._update_point_status = lambda: calls.__setitem__("status", calls["status"] + 1)

        for _ in range(10):
            SerialEventLiveVisualizerApp._draw_current_frame(app)

        self.assertEqual(calls, {"xy": 10, "projections": 10, "timeline": 1, "status": 10})

    def test_point_poll_consumes_frames_without_drawing_hidden_point_tab(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app._closed = False
        app.point_source = SimpleNamespace(is_running=True)
        app.point_buffer = LatestFrameBuffer()
        app.point_buffer.publish(point_frame(7))
        app.history = RawFrameHistory(max_frames=10)
        app.current_view = None
        app.point_poll_after_id = "old-after-id"
        app._point_tab_is_visible = lambda: False
        app._draw_current_frame = lambda: self.fail("hidden point tab must not be redrawn")
        app._update_point_health = lambda: "healthy"
        status_updates = []
        app._update_point_status = lambda: status_updates.append(True)
        scheduled = []
        app._schedule_point_poll = lambda: scheduled.append(True)

        SerialEventLiveVisualizerApp._poll_point_frames(app)

        self.assertEqual(app.current_view.frame.frame_index, 7)
        self.assertEqual([view.frame.frame_index for view in app.history.views], [7])
        self.assertEqual(status_updates, [True])
        self.assertEqual(scheduled, [True])

    def test_selecting_point_tab_renders_latest_state_once(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.current_view = raw_view_from_frame(point_frame(9))
        app._point_tab_is_visible = lambda: True
        rendered = []
        app._draw_current_frame = lambda *, force_timeline=False: rendered.append(force_timeline)

        SerialEventLiveVisualizerApp._on_tab_changed(app)

        self.assertEqual(rendered, [True])

    def test_projection_redraw_reuses_artists_and_maps_xz_yz_coordinates(self):
        app = object.__new__(SerialEventLiveVisualizerApp)
        app.bounds = PlotBounds()
        app.projection_figure = Figure(figsize=(4, 4), dpi=80)
        app.xz_axis = app.projection_figure.add_subplot(211)
        app.yz_axis = app.projection_figure.add_subplot(212, sharey=app.xz_axis)
        app.projection_canvas = FigureCanvasAgg(app.projection_figure)
        app.latest_board_status = None
        app.current_view = raw_view_from_frame(point_frame(1, x_m=0.1))

        SerialEventLiveVisualizerApp._draw_projections(app)
        xz_presence_artist = app._xz_presence_artist
        yz_presence_artist = app._yz_presence_artist
        app.current_view = raw_view_from_frame(point_frame(2, x_m=0.4))
        SerialEventLiveVisualizerApp._draw_projections(app)

        self.assertIs(app._xz_presence_artist, xz_presence_artist)
        self.assertIs(app._yz_presence_artist, yz_presence_artist)
        self.assertEqual(self._offsets(app._xz_presence_artist), ((0.4, 0.8),))
        self.assertEqual(self._offsets(app._yz_presence_artist), ((1.0, 0.8),))
        self.assertEqual(self._offsets(app._xz_motion_artist), ((-0.4, 0.6),))
        self.assertEqual(self._offsets(app._yz_motion_artist), ((1.2, 0.6),))

        status = parse_board_status("FALL frame=2 state=0 target=1 x=20 y=100 z=80 h=120")
        assert status is not None
        app.latest_board_status = status
        SerialEventLiveVisualizerApp._draw_projections(app)
        self.assertEqual(self._offsets(app._xz_target_artist), ((0.2, 0.8),))
        self.assertEqual(self._offsets(app._yz_target_artist), ((1.0, 0.8),))
        self.assertEqual(app._xz_target_text.get_position(), (0.2, 0.8))
        self.assertEqual(app._yz_target_text.get_position(), (1.0, 0.8))

    @staticmethod
    def _offsets(artist) -> tuple[tuple[float, float], ...]:
        return tuple(tuple(float(value) for value in row) for row in artist.get_offsets())

    def test_serial_poll_batches_widget_append_and_processes_every_record(self):
        app = object.__new__(type("TestApp", (), {}))
        app.serial_records = Queue()
        now = datetime.now()
        status = parse_board_status("FALL frame=10 state=0 target=0", received_at=now)
        event = parse_fall_event("FALL_EVENT state=suspected frame=11", received_at=now)
        assert status is not None and event is not None
        records = (
            SerialRecord("rx", "Welcome to use POSSUMIC", now),
            SerialRecord("rx", status.raw_line, now, status=status),
            SerialRecord("rx", event.raw_line, now, event=event),
        )
        for record in records:
            app.serial_records.put(record)
        app.serial_poll_after_id = "old-after-id"
        app._closed = False
        scheduled = []
        single_appends = []
        batch_appends = []
        accepted_statuses = []
        handled_events = []
        app._schedule_serial_poll = lambda: scheduled.append(True)
        app._append_serial_record = lambda record: single_appends.append(record)
        app._append_serial_records = lambda batch: batch_appends.append(tuple(batch))
        app._accept_board_status = lambda board_status: accepted_statuses.append(board_status)
        app._handle_board_event = lambda board_event: handled_events.append(board_event)
        app._update_board_link_health = lambda: None
        app._point_tab_is_visible = lambda: True
        app.point_source = None
        app.current_view = None
        renders = []
        app._draw_current_frame = lambda *, force_timeline=False: renders.append(force_timeline)
        app.board_events = BoardEventState()

        SerialEventLiveVisualizerApp._poll_serial_records(app)

        self.assertEqual(single_appends, [])
        self.assertEqual(batch_appends, [records])
        self.assertEqual(accepted_statuses, [status])
        self.assertEqual(handled_events, [event])
        self.assertEqual(renders, [True])
        self.assertEqual(scheduled, [True])


if __name__ == "__main__":
    unittest.main()
