import unittest
from types import SimpleNamespace

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.operator_marks import OperatorMark
from fall_detection.rule_detector import FallEvent, FallState
from fall_detection.tracking import TargetFrame, TargetState
from fall_detection.visualization_model import ClusterView
from fall_detection.visualize_datadealer import DatadealerVisualizerApp
from fall_detection.visualize_datadealer import FallAlertPopup
from fall_detection.visualize_datadealer import FrameAlert
from fall_detection.visualize_datadealer import TimelineSeries
from fall_detection.visualize_datadealer import _build_frame_alerts
from fall_detection.visualize_datadealer import _build_timeline_series
from fall_detection.visualize_datadealer import _display_cluster_boxes
from fall_detection.visualize_datadealer import _fall_popup_message
from fall_detection.visualize_datadealer import _format_operator_mark_status
from fall_detection.visualize_datadealer import _format_source_frame_label
from fall_detection.visualize_datadealer import _prepare_frames_for_visualization
from fall_detection.visualize_datadealer import _replace_tracker_z_roi
from fall_detection.visualize_datadealer import _rotate_frames_for_visualization
from fall_detection.visualize_datadealer import _format_target_diagnosis
from fall_detection.visualize_datadealer import _load_operator_marks_for_recording
from fall_detection.visualize_datadealer import _next_alert_index
from fall_detection.visualize_datadealer import _target_state_suffix


class FakeRoot:
    def __init__(self):
        self.after_calls = []
        self.cancelled = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))
        return "after-id"

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)


class FakeScale:
    def __init__(self):
        self.values = []

    def set(self, value):
        self.values.append(value)


class FakeSpeedVar:
    def get(self):
        return 1


class FakeAxes3D:
    elev = 31.0
    azim = -42.0


class FakeEvent:
    def __init__(self, inaxes):
        self.inaxes = inaxes


class FakePopupWindow:
    def __init__(self, root):
        self.root = root
        self.title_text = ""
        self.config = {}
        self.geometry_text = ""
        self.resizable_value = None
        self.attributes_calls = []
        self.protocols = {}
        self.withdraw_count = 0
        self.deiconify_count = 0
        self.lift_count = 0
        self.destroy_count = 0

    def title(self, text):
        self.title_text = text

    def configure(self, **kwargs):
        self.config.update(kwargs)

    def geometry(self, text):
        self.geometry_text = text

    def resizable(self, width, height):
        self.resizable_value = (width, height)

    def attributes(self, *args):
        self.attributes_calls.append(args)

    def protocol(self, name, callback):
        self.protocols[name] = callback

    def withdraw(self):
        self.withdraw_count += 1

    def deiconify(self):
        self.deiconify_count += 1

    def lift(self):
        self.lift_count += 1

    def destroy(self):
        self.destroy_count += 1


class FakePopupLabel:
    def __init__(self, window, **kwargs):
        self.window = window
        self.config = dict(kwargs)
        self.pack_args = None

    def configure(self, **kwargs):
        self.config.update(kwargs)

    def pack(self, **kwargs):
        self.pack_args = kwargs


class FakeSyncPopup:
    def __init__(self):
        self.calls = []

    def sync(self, alert, *, frame_index, timestamp_ms):
        self.calls.append((alert, frame_index, timestamp_ms))

    def hide(self):
        self.calls.append(("hide",))


class FakePopupRoot:
    def __init__(self):
        self.after_calls = []
        self.cancelled = []

    def after(self, delay_ms, callback):
        after_id = f"after-{len(self.after_calls) + 1}"
        self.after_calls.append((after_id, delay_ms, callback))
        return after_id

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)


def cluster_view(x, y, *, is_target=False, point_count=3, motion_count=0):
    return ClusterView(
        points=(
            Point3D(x, y, 0.2, 0.0, 25.0, "presence"),
            Point3D(x + 0.1, y, 0.3, 0.0, 25.0, "presence"),
            Point3D(x, y + 0.1, 0.4, 0.4 if motion_count else 0.0, 25.0, "motion" if motion_count else "presence"),
        )[:point_count],
        is_target=is_target,
        center_x_m=x,
        center_y_m=y,
        center_z_m=0.3,
        x_min_m=x - 0.1,
        x_max_m=x + 0.1,
        y_min_m=y - 0.1,
        y_max_m=y + 0.1,
        z_min_m=0.1,
        z_max_m=0.5,
        point_count=point_count,
        motion_count=motion_count,
        max_abs_velocity_m_s=0.4 if motion_count else 0.0,
    )


def target_frame(index, x, y, z, *, state=TargetState.TRACKING, motion_count=2, velocity=0.0):
    return TargetFrame(
        frame_index=index,
        timestamp_ms=index * 100,
        timestamp_text=f"t{index}",
        has_target=True,
        center_x_m=x,
        center_y_m=y,
        center_z_m=z,
        z_min_m=z - 0.1,
        z_max_m=z + 0.1,
        height_m=0.2,
        width_x_m=0.2,
        depth_y_m=0.2,
        point_count=8,
        motion_count=motion_count,
        max_abs_velocity_m_s=velocity,
        state=state,
    )


def view_frame(index, x, y, z, *, motion_count=2, velocity=0.0):
    return SimpleNamespace(
        frame=SimpleNamespace(frame_index=index, timestamp_ms=index * 100),
        target=target_frame(index, x, y, z, motion_count=motion_count, velocity=velocity),
    )


class VisualizerPlaybackTests(unittest.TestCase):
    def make_app(self):
        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        app.root = FakeRoot()
        app.frames = [object(), object()]
        app.current_index = 0
        app.is_playing = True
        app.speed_var = FakeSpeedVar()
        app.frame_scale = FakeScale()
        app.play_after_id = None
        app._is_3d_interacting = False
        app._last_3d_elev = 24.0
        app._last_3d_azim = -58.0
        app.ax3d = FakeAxes3D()
        return app

    def test_schedule_next_does_not_advance_frame_during_3d_drag(self):
        app = self.make_app()
        app._is_3d_interacting = True

        def fail_if_redrawn():
            raise AssertionError("draw_current_frame should not run while dragging 3D axes")

        app.draw_current_frame = fail_if_redrawn

        app._schedule_next()

        self.assertEqual(app.current_index, 0)
        self.assertEqual(app.frame_scale.values, [])
        self.assertEqual(len(app.root.after_calls), 1)
        self.assertEqual(app.play_after_id, "after-id")

    def test_3d_mouse_events_toggle_interaction_and_capture_view(self):
        app = self.make_app()

        app._on_3d_mouse_press(FakeEvent(app.ax3d))
        self.assertTrue(app._is_3d_interacting)

        app._on_3d_mouse_release(FakeEvent(None))
        self.assertFalse(app._is_3d_interacting)
        self.assertEqual(app._last_3d_elev, 31.0)
        self.assertEqual(app._last_3d_azim, -42.0)

    def test_source_frame_label_uses_sdk_frame_index_text(self):
        frame = SimpleNamespace(frame_index=972, timestamp_text="frame 973")

        self.assertEqual(_format_source_frame_label(frame), "源帧 973")

    def test_rotate_frames_for_visualization_applies_yz_rotation_before_tracking(self):
        source = CombinedFrame(
            frame_index=0,
            timestamp_ms=0,
            timestamp_text="t0",
            presence_points=(Point3D(0.0, 1.0, 0.0, 0.0, 30.0, "presence"),),
            motion_points=(),
        )

        rotated = _rotate_frames_for_visualization((source,), 90.0)

        self.assertAlmostEqual(rotated[0].presence_points[0].y_m, 0.0, places=6)
        self.assertAlmostEqual(rotated[0].presence_points[0].z_m, -1.0, places=6)

    def test_prepare_frames_for_visualization_applies_mount_height_and_ground_calibration(self):
        frames = []
        for frame_index in range(3):
            floor = []
            for x_index in range(-3, 4):
                for y_index in range(1, 6):
                    x = x_index * 0.2
                    y = y_index * 0.25
                    z = 0.04 * x - 0.03 * y - 1.20
                    floor.append(Point3D(x, y, z, 0.0, 30.0, "presence"))
            frames.append(
                CombinedFrame(
                    frame_index=frame_index,
                    timestamp_ms=frame_index * 50,
                    timestamp_text=f"t{frame_index}",
                    presence_points=tuple(floor),
                    motion_points=(),
                )
            )

        transformed, plane = _prepare_frames_for_visualization(
            tuple(frames),
            yz_rotation_deg=0.0,
            mount_height_m=1.2,
            ground_calibration_frames=3,
            ground_band_m=0.2,
            min_snr_db=10.0,
        )

        self.assertIsNotNone(plane)
        floor_heights = [point.z_m for point in transformed[0].presence_points]
        self.assertLess(max(abs(height) for height in floor_heights), 0.01)

    def test_replace_tracker_z_roi_preserves_other_tracking_parameters(self):
        from fall_detection.tracking import TrackerConfig

        original = TrackerConfig(min_snr_db=10.0, hold_ms=5000)

        updated = _replace_tracker_z_roi(original, z_min_m=0.0, z_max_m=2.2)

        self.assertAlmostEqual(updated.z_min_m, 0.0)
        self.assertAlmostEqual(updated.z_max_m, 2.2)
        self.assertAlmostEqual(updated.min_snr_db, 10.0)
        self.assertEqual(updated.hold_ms, 5000)
        with self.assertRaisesRegex(ValueError, "Z 下限.*Z 上限"):
            _replace_tracker_z_roi(original, z_min_m=1.0, z_max_m=0.5)

    def test_apply_world_coordinate_settings_updates_z_roi_and_plot_bounds(self):
        from fall_detection.tracking import TrackerConfig

        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        app.config = TrackerConfig(min_snr_db=10.0)
        app.yz_rotation_var = SimpleNamespace(get=lambda: 60.0)
        app.mount_height_var = SimpleNamespace(get=lambda: 1.67)
        app.z_min_var = SimpleNamespace(get=lambda: 0.0)
        app.z_max_var = SimpleNamespace(get=lambda: 2.2)
        app.ground_calibration_frames_var = SimpleNamespace(get=lambda: 0)
        app.ground_band_var = SimpleNamespace(get=lambda: 0.25)
        reloads = []
        app.load_selected_recording = lambda: reloads.append(True)

        DatadealerVisualizerApp._apply_world_coordinate_settings(app)

        self.assertAlmostEqual(app.config.z_min_m, 0.0)
        self.assertAlmostEqual(app.config.z_max_m, 2.2)
        self.assertAlmostEqual(app.bounds.z_min_m, 0.0)
        self.assertAlmostEqual(app.bounds.z_max_m, 2.2)
        self.assertAlmostEqual(app.yz_rotation_deg, 60.0)
        self.assertAlmostEqual(app.mount_height_m, 1.67)
        self.assertEqual(reloads, [True])

    def test_display_cluster_boxes_include_all_clusters_and_put_target_last(self):
        target = cluster_view(0.0, 1.0, is_target=True, motion_count=1)
        left = cluster_view(-0.7, 1.4)
        right = cluster_view(0.8, 2.0)
        view = SimpleNamespace(clusters=(target, left, right), target_cluster=target)

        boxes = _display_cluster_boxes(view)

        self.assertEqual([box.cluster for box in boxes], [left, right, target])
        self.assertEqual([box.label for box in boxes], ["聚类", "聚类", "目标"])
        self.assertEqual(boxes[-1].color, "#f6d365")

    def test_display_cluster_boxes_include_held_target_box(self):
        live = cluster_view(-0.7, 1.4)
        held_target = cluster_view(0.0, 1.0, is_target=True, point_count=0)
        view = SimpleNamespace(clusters=(live,), target_cluster=held_target)

        boxes = _display_cluster_boxes(view)

        self.assertEqual([box.cluster for box in boxes], [live, held_target])
        self.assertEqual([box.label for box in boxes], ["聚类", "目标"])

    def test_target_diagnosis_warns_when_tiny_target_loses_large_body_cluster(self):
        target = cluster_view(0.0, 0.9, is_target=True, point_count=4, motion_count=4)
        body = cluster_view(0.0, 1.2, point_count=101, motion_count=1)
        view = SimpleNamespace(clusters=(body, target), target_cluster=target, target=SimpleNamespace(held=False))

        diagnosis = _format_target_diagnosis(view)

        self.assertIn("警告", diagnosis)
        self.assertIn("最大聚类未选中", diagnosis)
        self.assertIn("目标=点4/动4", diagnosis)
        self.assertIn("最大=点101/动1", diagnosis)

    def test_target_diagnosis_marks_stable_large_target_as_ok(self):
        target = cluster_view(0.0, 1.2, is_target=True, point_count=101, motion_count=1)
        motion_noise = cluster_view(0.0, 0.9, point_count=4, motion_count=4)
        view = SimpleNamespace(clusters=(target, motion_noise), target_cluster=target, target=SimpleNamespace(held=False))

        diagnosis = _format_target_diagnosis(view)

        self.assertIn("正常", diagnosis)
        self.assertNotIn("警告", diagnosis)

    def test_target_diagnosis_calls_static_points_lock_not_hold(self):
        target = cluster_view(0.0, 1.2, is_target=True, point_count=58, motion_count=0)
        view = SimpleNamespace(clusters=(target,), target_cluster=target, target=SimpleNamespace(held=True, point_count=58))

        diagnosis = _format_target_diagnosis(view)

        self.assertIn("静态锁定", diagnosis)
        self.assertNotIn("目标保持中", diagnosis)

    def test_target_state_suffix_distinguishes_static_lock_from_true_hold(self):
        self.assertEqual(_target_state_suffix(SimpleNamespace(held=False, point_count=58)), "")
        self.assertEqual(_target_state_suffix(SimpleNamespace(held=True, point_count=58)), " 静态锁定")
        self.assertEqual(_target_state_suffix(SimpleNamespace(held=True, point_count=0)), " 保持")

    def test_target_state_suffix_uses_explicit_tracker_state(self):
        self.assertEqual(_target_state_suffix(SimpleNamespace(state=TargetState.TRACKING, held=True, point_count=0)), "")
        self.assertEqual(_target_state_suffix(SimpleNamespace(state=TargetState.STATIC_LOCK, held=False, point_count=0)), " 静态锁定")
        self.assertEqual(_target_state_suffix(SimpleNamespace(state=TargetState.HOLD, held=False, point_count=58)), " 保持")
        self.assertEqual(_target_state_suffix(SimpleNamespace(state=TargetState.REACQUIRE, held=False, point_count=0)), " 重捕获")

    def test_datadealer_right_panel_draws_xz_and_yz_projection_views(self):
        target = cluster_view(0.2, 1.0, is_target=True, motion_count=1)
        other = cluster_view(-0.4, 1.6)
        view = SimpleNamespace(
            filtered_points=target.points + other.points,
            clusters=(target, other),
            target=target_frame(0, 0.2, 1.0, 0.3),
            target_cluster=target,
        )
        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        app.bounds = SimpleNamespace(x_min_m=-1.5, x_max_m=1.5, y_min_m=0.15, y_max_m=3.0, z_min_m=-0.8, z_max_m=1.7)
        app.figure = Figure(figsize=(5, 4), dpi=100, facecolor="#0b0f14")
        app.ax_xz = app.figure.add_subplot(211)
        app.ax_yz = app.figure.add_subplot(212)
        app.figure_canvas = FigureCanvasAgg(app.figure)
        app.alerts = ()
        app.current_index = 0

        DatadealerVisualizerApp._draw_3d(app, view)
        app.figure_canvas.draw()

        self.assertEqual(app.ax_xz.get_xlabel(), "X left + (m)")
        self.assertEqual(app.ax_yz.get_xlabel(), "Y forward (m)")
        self.assertGreater(len(app.ax_xz.collections), 0)
        self.assertGreater(len(app.ax_yz.collections), 0)

    def test_target_diagnosis_reports_reacquire_state_without_target_cluster(self):
        live = cluster_view(1.2, 2.2, point_count=8, motion_count=5)
        view = SimpleNamespace(
            clusters=(live,),
            target_cluster=None,
            target=SimpleNamespace(state=TargetState.REACQUIRE, has_target=False),
        )

        diagnosis = _format_target_diagnosis(view)

        self.assertIn("重捕获中", diagnosis)
        self.assertIn("聚类=1", diagnosis)

    def test_frame_alerts_mark_jump_and_fall_event_frames(self):
        views = [
            view_frame(0, 0.0, 1.0, 1.2),
            view_frame(1, 0.1, 1.1, 1.1),
            view_frame(2, 1.2, 2.2, 0.4),
        ]
        fall_events = [
            FallEvent(
                state=FallState.SUSPECTED,
                frame_index=2,
                timestamp_ms=200,
                drop_height_m=0.7,
                reason="fast height drop with motion",
            )
        ]

        alerts = _build_frame_alerts(views, fall_events, jump_threshold_m=0.8)

        self.assertFalse(alerts[0].has_alert)
        self.assertFalse(alerts[1].has_alert)
        self.assertTrue(alerts[2].has_alert)
        self.assertAlmostEqual(alerts[2].jump_m, 1.56, delta=0.02)
        self.assertEqual(alerts[2].fall_states, (FallState.SUSPECTED,))
        self.assertIn("跳变", alerts[2].summary)
        self.assertIn("疑似跌倒", alerts[2].summary)

    def test_next_alert_index_wraps_in_both_directions(self):
        alerts = _build_frame_alerts(
            [
                view_frame(0, 0.0, 1.0, 1.2),
                view_frame(1, 1.2, 2.2, 0.5),
                view_frame(2, 1.2, 2.2, 0.5),
            ],
            [
                FallEvent(
                    state=FallState.CONFIRMED,
                    frame_index=2,
                    timestamp_ms=200,
                    drop_height_m=0.8,
                    reason="low posture stayed still after suspected fall",
                )
            ],
            jump_threshold_m=0.8,
        )

        self.assertEqual(_next_alert_index(alerts, current_index=0, direction=1), 1)
        self.assertEqual(_next_alert_index(alerts, current_index=1, direction=1), 2)
        self.assertEqual(_next_alert_index(alerts, current_index=2, direction=1), 1)
        self.assertEqual(_next_alert_index(alerts, current_index=0, direction=-1), 2)

    def test_timeline_series_extracts_height_motion_velocity_and_alert_indices(self):
        views = [
            view_frame(0, 0.0, 1.0, 1.2, motion_count=2, velocity=0.1),
            view_frame(1, 0.1, 1.1, 0.9, motion_count=7, velocity=0.6),
            view_frame(2, 0.2, 1.2, 0.4, motion_count=1, velocity=0.0),
        ]
        alerts = (
            _build_frame_alerts(
                views,
                [
                    FallEvent(
                        state=FallState.SUSPECTED,
                        frame_index=1,
                        timestamp_ms=100,
                        drop_height_m=0.3,
                        reason="fast height drop with motion",
                    )
                ],
                jump_threshold_m=0.8,
            )
        )

        series = _build_timeline_series(views, alerts)

        self.assertEqual(series.frame_indices, (0, 1, 2))
        self.assertEqual(series.height_z_m, (1.2, 0.9, 0.4))
        self.assertEqual(series.motion_counts, (2, 7, 1))
        self.assertEqual(series.max_velocity_m_s, (0.1, 0.6, 0.0))
        self.assertEqual(series.alert_indices, (1,))

    def test_timeline_series_keeps_only_operator_marks_inside_recording(self):
        views = [
            view_frame(0, 0.0, 1.0, 1.2),
            view_frame(1, 0.1, 1.1, 0.9),
            view_frame(2, 0.2, 1.2, 0.4),
        ]
        marks = (
            OperatorMark(mark="完全蹲下", radar_frame_index=1),
            OperatorMark(mark="错误越界", radar_frame_index=99),
        )

        series = _build_timeline_series(
            views,
            (FrameAlert(), FrameAlert(), FrameAlert()),
            operator_marks=marks,
        )

        self.assertEqual(
            [(mark.radar_frame_index, mark.mark) for mark in series.operator_marks],
            [(1, "完全蹲下")],
        )

    def test_operator_mark_status_exposes_marks_for_current_frame(self):
        marks = (
            OperatorMark(mark="疑似跌倒", radar_frame_index=12),
            OperatorMark(mark="真实跌倒", radar_frame_index=12),
            OperatorMark(mark="完全站起", radar_frame_index=30),
        )

        self.assertEqual(
            _format_operator_mark_status(marks, frame_index=12),
            "人工标注: 疑似跌倒 / 真实跌倒",
        )
        self.assertEqual(
            _format_operator_mark_status(marks, frame_index=13),
            "人工标注: 无",
        )

    def test_recording_loads_operator_marks_next_to_motion_csv(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            recording_dir = Path(directory) / "sample"
            recording_dir.mkdir()
            motion_path = recording_dir / "sample_motion_point_cloud.csv"
            marks_path = recording_dir / "operator_marks.csv"
            marks_path.write_text(
                "mark,radar_frame_index,radar_timestamp_ms\n"
                "真实跌倒,42,2100\n",
                encoding="utf-8",
            )
            recording = SimpleNamespace(motion_path=motion_path)

            marks = _load_operator_marks_for_recording(recording)

        self.assertEqual([(mark.radar_frame_index, mark.mark) for mark in marks], [(42, "真实跌倒")])

    def test_draw_timeline_renders_colored_operator_mark_lines_and_labels(self):
        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        app.timeline_figure = Figure(figsize=(8, 2), dpi=100)
        app.timeline_ax_height = app.timeline_figure.add_subplot(311)
        app.timeline_ax_motion = app.timeline_figure.add_subplot(312, sharex=app.timeline_ax_height)
        app.timeline_ax_velocity = app.timeline_figure.add_subplot(313, sharex=app.timeline_ax_height)
        app.timeline_axes = (app.timeline_ax_height, app.timeline_ax_motion, app.timeline_ax_velocity)
        app.timeline_canvas = FigureCanvasAgg(app.timeline_figure)
        app.current_index = 1
        app.timeline_series = TimelineSeries(
            frame_indices=(0, 1, 2),
            height_z_m=(1.2, 0.9, 0.4),
            motion_counts=(1, 5, 2),
            max_velocity_m_s=(0.1, 0.6, 0.2),
            alert_indices=(),
            operator_marks=(
                OperatorMark(mark="真实跌倒", radar_frame_index=1),
                OperatorMark(mark="完全站起", radar_frame_index=2),
            ),
        )

        DatadealerVisualizerApp._draw_timeline(app)

        labels = {text.get_text() for text in app.timeline_ax_height.texts}
        line_colors = {line.get_color() for line in app.timeline_ax_height.lines}
        self.assertIn("真实跌倒", labels)
        self.assertIn("完全站起", labels)
        self.assertIn("#ef4444", line_colors)
        self.assertIn("#22c55e", line_colors)

    def test_update_text_exposes_operator_mark_for_current_frame(self):
        captured = {}

        def capture_var(name):
            return SimpleNamespace(set=lambda value: captured.__setitem__(name, value))

        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        view = SimpleNamespace(
            frame=SimpleNamespace(
                frame_index=12,
                timestamp_ms=600,
                timestamp_text="frame 12",
                presence_count=3,
                motion_count=1,
            ),
            target=target_frame(12, 0.0, 1.0, 0.4),
            clusters=(),
            filtered_points=(),
            target_cluster=None,
        )
        app.frames = [view]
        app.current_index = 0
        app.operator_marks = (OperatorMark(mark="真实跌倒", radar_frame_index=12),)
        app.status_var = capture_var("status")
        app.target_var = capture_var("target")
        app.diagnosis_var = capture_var("diagnosis")
        app.alert_var = capture_var("alert")
        app.manual_mark_var = capture_var("manual")
        app._current_alert = lambda: FrameAlert()

        DatadealerVisualizerApp._update_text(app, view)

        self.assertEqual(captured["manual"], "人工标注: 真实跌倒")

    def test_fall_popup_message_only_uses_fall_states_not_jump_alerts(self):
        self.assertIsNone(_fall_popup_message(FrameAlert(jump_m=1.2), frame_index=3, timestamp_ms=300))

        message = _fall_popup_message(
            FrameAlert(fall_states=(FallState.SUSPECTED,)),
            frame_index=3,
            timestamp_ms=300,
        )

        self.assertIsNotNone(message)
        self.assertIn("疑似跌倒", message)
        self.assertIn("帧 3", message)

    def test_fall_alert_popup_shows_red_window_for_suspected_fall(self):
        windows = []
        labels = []

        def make_window(root):
            window = FakePopupWindow(root)
            windows.append(window)
            return window

        def make_label(window, **kwargs):
            label = FakePopupLabel(window, **kwargs)
            labels.append(label)
            return label

        popup = FallAlertPopup(object(), window_factory=make_window, label_factory=make_label)
        popup.sync(FrameAlert(fall_states=(FallState.SUSPECTED,)), frame_index=8, timestamp_ms=800)

        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0].title_text, "跌倒告警")
        self.assertTrue(windows[0].geometry_text.startswith("1240x980"))
        self.assertEqual(windows[0].config["bg"], "#7f1d1d")
        self.assertEqual(windows[0].deiconify_count, 1)
        self.assertEqual(windows[0].lift_count, 1)
        self.assertIn("疑似跌倒", labels[0].config["text"])
        self.assertIn("帧 8", labels[0].config["text"])

    def test_fall_alert_popup_holds_red_window_for_one_second_after_alert(self):
        windows = []
        root = FakePopupRoot()

        def make_window(root):
            window = FakePopupWindow(root)
            windows.append(window)
            return window

        popup = FallAlertPopup(root, window_factory=make_window, label_factory=FakePopupLabel)
        popup.sync(FrameAlert(fall_states=(FallState.SUSPECTED,)), frame_index=8, timestamp_ms=800)
        popup.sync(FrameAlert(), frame_index=9, timestamp_ms=900)

        self.assertEqual(root.after_calls[0][1], 1000)
        self.assertEqual(windows[0].withdraw_count, 0)

        root.after_calls[0][2]()

        self.assertEqual(windows[0].withdraw_count, 1)

    def test_fall_alert_popup_reschedules_one_second_hold_on_repeated_alert(self):
        root = FakePopupRoot()
        popup = FallAlertPopup(root, window_factory=FakePopupWindow, label_factory=FakePopupLabel)

        popup.sync(FrameAlert(fall_states=(FallState.SUSPECTED,)), frame_index=8, timestamp_ms=800)
        popup.sync(FrameAlert(fall_states=(FallState.CONFIRMED,)), frame_index=9, timestamp_ms=900)

        self.assertEqual(root.cancelled, ["after-1"])
        self.assertEqual(root.after_calls[1][1], 1000)

    def test_draw_current_frame_syncs_current_fall_alert_to_popup(self):
        app = DatadealerVisualizerApp.__new__(DatadealerVisualizerApp)
        frame = SimpleNamespace(frame=SimpleNamespace(frame_index=12, timestamp_ms=1200))
        popup = FakeSyncPopup()
        app.frames = [frame]
        app.alerts = (FrameAlert(fall_states=(FallState.SUSPECTED,)),)
        app.current_index = 0
        app.fall_alert_popup = popup
        app._update_text = lambda _view: None
        app._draw_xy = lambda _view: None
        app._draw_3d = lambda _view: None
        app._draw_timeline = lambda: None

        app.draw_current_frame()

        self.assertEqual(len(popup.calls), 1)
        alert, frame_index, timestamp_ms = popup.calls[0]
        self.assertEqual(alert.fall_states, (FallState.SUSPECTED,))
        self.assertEqual(frame_index, 12)
        self.assertEqual(timestamp_ms, 1200)


if __name__ == "__main__":
    unittest.main()
