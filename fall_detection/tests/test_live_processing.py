import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from fall_detection.datadealer_loader import CombinedFrame, Point3D, load_datadealer_recording
from fall_detection.live_processing import LiveFrameWindow
from fall_detection.rule_detector import FallRuleConfig, FallState
from fall_detection.live_visualizer import LiveReplayVisualizerApp
from fall_detection.live_visualizer import _format_frame_frequency
from fall_detection.tracking import TrackerConfig
from fall_detection.visualization_model import bounds_from_config
from fall_detection.visualize_datadealer import DatadealerVisualizerApp
from fall_detection.visualize_datadealer import FrameAlert


def frame(index, *, x=0.0, y=1.0, z=1.0, motion_count=1):
    presence = (
        Point3D(x - 0.1, y, z - 0.1, 0.0, 35.0, "presence"),
        Point3D(x, y, z, 0.0, 35.0, "presence"),
        Point3D(x + 0.1, y, z + 0.1, 0.0, 35.0, "presence"),
    )
    motion = tuple(Point3D(x, y, z, 0.5, 35.0, "motion") for _ in range(motion_count))
    return CombinedFrame(
        frame_index=index,
        timestamp_ms=index * 100,
        timestamp_text=f"t{index}",
        presence_points=presence,
        motion_points=motion,
    )


class FakeRoot:
    def __init__(self):
        self.after_calls = []
        self.cancelled = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))
        return f"after-{len(self.after_calls)}"

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)


class CallbackScale:
    def __init__(self, callback):
        self.callback = callback
        self.configures = []
        self.values = []

    def configure(self, **kwargs):
        self.configures.append(kwargs)

    def set(self, value):
        self.values.append(value)
        self.callback(str(value))


class OneFrameBuffer:
    published_count = 1
    consumed_count = 0

    def __init__(self, next_frame):
        self.next_frame = next_frame

    def take_latest(self):
        if self.next_frame is None:
            return None
        frame_value = self.next_frame
        self.next_frame = None
        self.consumed_count += 1
        return frame_value

    def take_pending(self):
        frame_value = self.take_latest()
        return () if frame_value is None else (frame_value,)


class PendingFramesBuffer:
    def __init__(self, frames):
        self.frames = tuple(frames)
        self.published_count = len(self.frames)
        self.consumed_count = 0

    def take_latest(self):
        if not self.frames:
            return None
        latest = self.frames[-1]
        self.frames = ()
        self.consumed_count += 1
        return latest

    def take_pending(self):
        pending = self.frames
        self.frames = ()
        self.consumed_count += len(pending)
        return pending


class StubLiveWindow:
    def __init__(self, snapshot):
        self.snapshot = snapshot
        self.push_calls = []

    def push_frame(self, next_frame, *, build_timeline=True):
        self.push_calls.append((next_frame, build_timeline))
        return self.snapshot


class SequentialStubLiveWindow:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)
        self.push_calls = []

    def push_frame(self, next_frame, *, build_timeline=True):
        self.push_calls.append((next_frame, build_timeline))
        return self.snapshots.pop(0)


class PopupCapture:
    def __init__(self):
        self.calls = []

    def sync(self, alert, *, frame_index, timestamp_ms):
        self.calls.append((alert, frame_index, timestamp_ms))


class LiveProcessingTests(unittest.TestCase):
    def test_live_frame_window_matches_verified_terminal_height_fall_events(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=10.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=1.0,
            switch_confirm_frames=3,
            hold_ms=5000,
        )
        cases = {
            "1.67-60": (
                60.0,
                1.67,
                {
                    "suspected_fall": [651, 2107],
                    "confirmed_fall": [678, 2185],
                    "recovered": [1207, 2640],
                },
            ),
            "1.67-60-2": (
                60.0,
                1.67,
                {
                    "suspected_fall": [1083, 1922],
                    "confirmed_fall": [1109, 1963],
                    "recovered": [1429, 2145],
                },
            ),
            "163-30": (
                30.0,
                1.63,
                {
                    "suspected_fall": [791, 1993, 2484, 3136, 3544],
                    "confirmed_fall": [815, 2018, 2509, 3161, 3568],
                    "recovered": [1067, 2249, 2665, 3319, 3689],
                },
            ),
            "1.63-30-2": (
                30.0,
                1.63,
                {
                    "suspected_fall": [499, 869, 1331],
                    "confirmed_fall": [523, 899, 1383],
                    "recovered": [786, 1149, 1550],
                },
            ),
        }

        for stem, (yz_rotation_deg, mount_height_m, expected) in cases.items():
            with self.subTest(stem=stem):
                recording = load_datadealer_recording(data_root / stem, stem=stem)
                window = LiveFrameWindow(
                    tracker_config,
                    max_history=600,
                    fall_config=FallRuleConfig(),
                    yz_rotation_deg=yz_rotation_deg,
                    mount_height_m=mount_height_m,
                )
                actual = {state: [] for state in expected}

                for source_frame in recording.frames:
                    snapshot = window.push_frame(source_frame, build_timeline=False)
                    for state in snapshot.alerts[-1].fall_states:
                        actual[state.value].append(source_frame.frame_index)

                self.assertEqual(actual, expected)

    def test_live_application_window_factory_propagates_world_coordinate_settings(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        app.config = TrackerConfig(min_snr_db=10.0, z_min_m=0.0, z_max_m=2.2)
        app.history_frames = 600
        app.fall_config = SimpleNamespace(name="fall-config")
        app.yz_rotation_deg = 60.0
        app.mount_height_m = 1.67

        window = app._new_live_window()

        self.assertIs(window.runtime.config, app.config)
        self.assertEqual(window.max_history, 600)
        self.assertIs(window._fall_detector.config, app.fall_config)
        self.assertAlmostEqual(window.yz_rotation_deg, 60.0)
        self.assertAlmostEqual(window.mount_height_m, 1.67)

    def test_live_frame_window_keeps_incremental_tracking_and_bounded_history(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, hold_ms=3000),
            max_history=2,
        )

        window.push_frame(frame(0, x=0.0, y=1.0))
        snapshot = window.push_frame(frame(1, x=0.1, y=1.1))
        snapshot = window.push_frame(frame(2, x=0.2, y=1.2))

        self.assertEqual([view.frame.frame_index for view in snapshot.views], [1, 2])
        self.assertEqual(snapshot.timeline.frame_indices, (1, 2))
        self.assertEqual(len(snapshot.alerts), 2)
        self.assertTrue(snapshot.views[-1].target.has_target)

    def test_live_frame_window_marks_large_jump_inside_recent_window(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, hold_ms=0, switch_confirm_frames=1),
            max_history=4,
            jump_threshold_m=0.8,
        )

        window.push_frame(frame(0, x=0.0, y=1.0))
        snapshot = window.push_frame(frame(1, x=1.2, y=2.2))

        self.assertTrue(snapshot.alerts[-1].has_alert)
        self.assertIsNotNone(snapshot.alerts[-1].jump_m)
        self.assertGreater(snapshot.alerts[-1].jump_m, 0.8)
        self.assertEqual(snapshot.timeline.alert_indices, (1,))

    def test_live_frame_window_applies_yz_rotation_before_tracking(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, y_min_m=-0.2, z_min_m=-2.0),
            max_history=4,
            yz_rotation_deg=90.0,
        )

        snapshot = window.push_frame(frame(0, x=0.0, y=1.0, z=0.0, motion_count=2))
        view = snapshot.views[-1]

        self.assertAlmostEqual(view.frame.presence_points[1].y_m, 0.0, places=7)
        self.assertAlmostEqual(view.frame.presence_points[1].z_m, -1.0, places=7)
        self.assertTrue(view.target.has_target)
        self.assertAlmostEqual(view.target.center_z_m, -1.0, delta=0.11)

    def test_live_frame_window_applies_mount_height_before_z_roi_and_tracking(self):
        window = LiveFrameWindow(
            config=TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                y_min_m=-0.2,
                z_min_m=0.0,
                z_max_m=2.2,
            ),
            max_history=4,
            yz_rotation_deg=90.0,
            mount_height_m=1.67,
        )

        snapshot = window.push_frame(frame(0, x=0.0, y=1.0, z=0.0, motion_count=2))
        view = snapshot.views[-1]

        self.assertAlmostEqual(view.frame.presence_points[1].y_m, 0.0, places=7)
        self.assertAlmostEqual(view.frame.presence_points[1].z_m, 0.67, places=7)
        self.assertTrue(view.target.has_target)
        self.assertAlmostEqual(view.target.center_z_m, 0.67, delta=0.11)

    def test_live_frame_window_keeps_alerts_incrementally_when_history_rolls(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, hold_ms=0, switch_confirm_frames=1),
            max_history=2,
            jump_threshold_m=0.8,
        )

        window.push_frame(frame(0, x=0.0, y=1.0))
        window.push_frame(frame(1, x=1.2, y=2.2))
        snapshot = window.push_frame(frame(2, x=1.25, y=2.25))

        self.assertEqual([view.frame.frame_index for view in snapshot.views], [1, 2])
        self.assertEqual(len(snapshot.alerts), 2)
        self.assertTrue(snapshot.alerts[0].has_alert)
        self.assertFalse(snapshot.alerts[1].has_alert)

    def test_live_frame_window_can_skip_timeline_rebuild_between_draws(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5),
            max_history=4,
        )

        first = window.push_frame(frame(0), build_timeline=True)
        skipped = window.push_frame(frame(1), build_timeline=False)
        rebuilt = window.push_frame(frame(2), build_timeline=True)

        self.assertEqual([view.frame.frame_index for view in skipped.views], [0, 1])
        self.assertEqual(len(skipped.alerts), 2)
        self.assertEqual(skipped.timeline.frame_indices, first.timeline.frame_indices)
        self.assertEqual(rebuilt.timeline.frame_indices, (0, 1, 2))

    def test_format_frame_frequency_uses_current_and_previous_frame_timestamp(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5),
            max_history=4,
        )

        snapshot = window.push_frame(frame(0))
        self.assertEqual(_format_frame_frequency(snapshot.views, current_index=0), "频率 --Hz")

        snapshot = window.push_frame(frame(1))

        self.assertEqual(_format_frame_frequency(snapshot.views, current_index=1), "频率 10.0Hz")

    def test_format_frame_frequency_handles_missing_or_invalid_timestamp_delta(self):
        window = LiveFrameWindow(
            config=TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5),
            max_history=4,
        )
        window.push_frame(frame(1))
        snapshot = window.push_frame(frame(1))

        self.assertEqual(_format_frame_frequency(snapshot.views, current_index=1), "频率 --Hz")

    def test_live_draw_current_frame_throttles_3d_and_timeline_while_playing(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        calls = {"text": 0, "xy": 0, "3d": 0, "timeline": 0, "popup": 0}
        app.frames = [object()]
        app.current_index = 0
        app.is_playing = True
        app.draw_3d_every = 3
        app.draw_timeline_every = 5
        app._live_draw_count = 0
        app._update_text = lambda _view: calls.__setitem__("text", calls["text"] + 1)
        app._draw_xy = lambda _view: calls.__setitem__("xy", calls["xy"] + 1)
        app._draw_3d = lambda _view: calls.__setitem__("3d", calls["3d"] + 1)
        app._draw_timeline = lambda: calls.__setitem__("timeline", calls["timeline"] + 1)
        app._sync_fall_alert_popup = lambda _view: calls.__setitem__("popup", calls["popup"] + 1)
        app._clear_canvases = lambda: None

        for _ in range(6):
            LiveReplayVisualizerApp.draw_current_frame(app)

        self.assertEqual(calls["text"], 6)
        self.assertEqual(calls["xy"], 6)
        self.assertEqual(calls["popup"], 6)
        self.assertEqual(calls["3d"], 2)
        self.assertEqual(calls["timeline"], 2)

    def test_live_draw_current_frame_fully_refreshes_when_paused(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        calls = {"3d": 0, "timeline": 0}
        app.frames = [object()]
        app.current_index = 0
        app.is_playing = False
        app.draw_3d_every = 10
        app.draw_timeline_every = 10
        app._live_draw_count = 0
        app._update_text = lambda _view: None
        app._draw_xy = lambda _view: None
        app._draw_3d = lambda _view: calls.__setitem__("3d", calls["3d"] + 1)
        app._draw_timeline = lambda: calls.__setitem__("timeline", calls["timeline"] + 1)
        app._sync_fall_alert_popup = lambda _view: None
        app._clear_canvases = lambda: None

        for _ in range(3):
            LiveReplayVisualizerApp.draw_current_frame(app)

        self.assertEqual(calls["3d"], 3)
        self.assertEqual(calls["timeline"], 3)

    def test_live_3d_uses_fast_artist_update_while_playing(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        app.is_playing = True
        calls = {"fast": 0, "base": 0}
        app._draw_live_3d = lambda _view: calls.__setitem__("fast", calls["fast"] + 1)

        with patch.object(
            DatadealerVisualizerApp,
            "_draw_3d",
            lambda _self, _view: calls.__setitem__("base", calls["base"] + 1),
        ):
            LiveReplayVisualizerApp._draw_3d(app, object())

        self.assertEqual(calls["fast"], 1)
        self.assertEqual(calls["base"], 0)

    def test_live_3d_uses_full_redraw_when_paused(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        app.is_playing = False
        calls = {"fast": 0, "base": 0}
        app._draw_live_3d = lambda _view: calls.__setitem__("fast", calls["fast"] + 1)

        with patch.object(
            DatadealerVisualizerApp,
            "_draw_3d",
            lambda _self, _view: calls.__setitem__("base", calls["base"] + 1),
        ):
            LiveReplayVisualizerApp._draw_3d(app, object())

        self.assertEqual(calls["fast"], 0)
        self.assertEqual(calls["base"], 1)

    def test_live_3d_fast_artist_update_can_render(self):
        config = TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5)
        window = LiveFrameWindow(config=config, max_history=4)
        view = window.push_frame(frame(0, motion_count=4)).views[-1]
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        app.is_playing = True
        app.bounds = bounds_from_config(config)
        app.figure = Figure(figsize=(5, 4), dpi=100, facecolor="#0b0f14")
        app.ax3d = app.figure.add_subplot(111, projection="3d")
        app.figure_canvas = FigureCanvasAgg(app.figure)
        app._last_3d_elev = 24.0
        app._last_3d_azim = -58.0
        app.alerts = ()
        app.current_index = 0

        LiveReplayVisualizerApp._draw_3d(app, view)
        app.figure_canvas.draw()

        self.assertTrue(app._live_3d_ready)
        self.assertGreater(len(app.ax3d.collections), 0)

    def test_live_poll_does_not_redraw_from_programmatic_slider_set(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        view = SimpleNamespace(frame=SimpleNamespace(frame_index=0, timestamp_ms=0), clusters=(), filtered_points=())
        snapshot = SimpleNamespace(
            views=(view,),
            alerts=(),
            timeline=SimpleNamespace(frame_indices=(), height_z_m=(), motion_counts=(), max_velocity_m_s=(), alert_indices=()),
        )
        draw_calls = {"count": 0}
        app.root = FakeRoot()
        app.is_playing = True
        app.play_after_id = None
        app.poll_ms = 5
        app.draw_timeline_every = 10
        app._live_draw_count = 0
        app.live_buffer = OneFrameBuffer(object())
        app.live_window = StubLiveWindow(snapshot)
        app.live_source = SimpleNamespace(is_running=True)
        app.source_mode = "device"
        app.loop = True
        app.frames = []
        app.current_index = 0
        app.alerts = ()
        app.timeline_series = snapshot.timeline
        app.frame_scale = CallbackScale(lambda value: LiveReplayVisualizerApp._on_slider(app, value))
        app.draw_current_frame = lambda: draw_calls.__setitem__("count", draw_calls["count"] + 1)

        LiveReplayVisualizerApp._poll_live_frame(app)

        self.assertEqual(draw_calls["count"], 1)
        self.assertEqual(app.frame_scale.values, [0])
        self.assertEqual(app.root.after_calls[-1][0], 5)

    def test_live_poll_processes_all_pending_frames_but_draws_once(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        source_frames = (object(), object(), object())
        view = SimpleNamespace(frame=SimpleNamespace(frame_index=2, timestamp_ms=100), clusters=(), filtered_points=())
        snapshot = SimpleNamespace(
            views=(view,),
            alerts=(),
            timeline=SimpleNamespace(frame_indices=(), height_z_m=(), motion_counts=(), max_velocity_m_s=(), alert_indices=()),
        )
        draw_calls = {"count": 0}
        app.root = FakeRoot()
        app.is_playing = True
        app.play_after_id = None
        app.poll_ms = 5
        app.draw_timeline_every = 10
        app._live_draw_count = 0
        app.live_buffer = PendingFramesBuffer(source_frames)
        app.live_window = StubLiveWindow(snapshot)
        app.live_source = SimpleNamespace(is_running=True)
        app.source_mode = "device"
        app.loop = True
        app.frames = []
        app.current_index = 0
        app.alerts = ()
        app.timeline_series = snapshot.timeline
        app.frame_scale = CallbackScale(lambda value: LiveReplayVisualizerApp._on_slider(app, value))
        app.draw_current_frame = lambda: draw_calls.__setitem__("count", draw_calls["count"] + 1)

        LiveReplayVisualizerApp._poll_live_frame(app)

        self.assertEqual(
            [call[0] for call in app.live_window.push_calls],
            list(source_frames),
        )
        self.assertEqual(
            [call[1] for call in app.live_window.push_calls],
            [False, False, True],
        )
        self.assertEqual(app.live_buffer.consumed_count, 3)
        self.assertEqual(draw_calls["count"], 1)

    def test_live_poll_surfaces_fall_alert_from_middle_of_pending_batch(self):
        app = LiveReplayVisualizerApp.__new__(LiveReplayVisualizerApp)
        source_frames = (object(), object(), object())
        timeline = SimpleNamespace(frame_indices=(), height_z_m=(), motion_counts=(), max_velocity_m_s=(), alert_indices=())
        views = tuple(
            SimpleNamespace(
                frame=SimpleNamespace(frame_index=index, timestamp_ms=index * 50),
                clusters=(),
                filtered_points=(),
            )
            for index in range(3)
        )
        snapshots = (
            SimpleNamespace(views=(views[0],), alerts=(FrameAlert(),), timeline=timeline),
            SimpleNamespace(
                views=(views[1],),
                alerts=(FrameAlert(fall_states=(FallState.SUSPECTED,)),),
                timeline=timeline,
            ),
            SimpleNamespace(views=(views[2],), alerts=(FrameAlert(),), timeline=timeline),
        )
        popup = PopupCapture()
        app.root = FakeRoot()
        app.is_playing = True
        app.play_after_id = None
        app.poll_ms = 5
        app.draw_timeline_every = 10
        app._live_draw_count = 0
        app.live_buffer = PendingFramesBuffer(source_frames)
        app.live_window = SequentialStubLiveWindow(snapshots)
        app.live_source = SimpleNamespace(is_running=True)
        app.source_mode = "device"
        app.loop = True
        app.frames = []
        app.current_index = 0
        app.alerts = ()
        app.timeline_series = timeline
        app.frame_scale = CallbackScale(lambda value: LiveReplayVisualizerApp._on_slider(app, value))
        app.draw_current_frame = lambda: None
        app.fall_alert_popup = popup

        LiveReplayVisualizerApp._poll_live_frame(app)

        self.assertEqual(len(popup.calls), 1)
        alert, frame_index, timestamp_ms = popup.calls[0]
        self.assertEqual(alert.fall_states, (FallState.SUSPECTED,))
        self.assertEqual(frame_index, 1)
        self.assertEqual(timestamp_ms, 50)


if __name__ == "__main__":
    unittest.main()
