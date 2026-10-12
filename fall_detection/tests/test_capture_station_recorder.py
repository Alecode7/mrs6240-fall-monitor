import csv
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from matplotlib import rcParams

from fall_capture_station.v2_protocol import (
    ANNOTATION_CONTEXTS,
    CAPTURE_SCHEMA_VERSION,
    DEFAULT_CAPTURE_PROFILE,
    SESSION_SCENARIOS,
    CaptureProfile,
    CaptureProfileError,
    format_annotation_note,
    parse_annotation_context,
)
from fall_capture_station.annotation_app import (
    ANNOTATION_MARKS,
    AnnotationApp,
    CHINESE_FONT_CANDIDATES,
    DEFAULT_PLAYBACK_FPS,
    MARK_BUTTON_COLUMNS,
    PLAYBACK_FPS_OPTIONS,
    _annotation_mark_positions,
    _configure_chinese_matplotlib_font,
    _nearest_camera_row,
    _next_playback_index,
    _playback_interval_ms,
    _point_groups,
)
from fall_capture_station.camera_recorder import CameraRecorder
from fall_capture_station.capture_app import CaptureStationApp, RADAR_START_COMMAND, RADAR_STOP_COMMAND
from fall_capture_station.session_recorder import (
    AnnotationEventReference,
    SessionRecorder,
    append_operator_mark,
    backfill_operator_mark_references,
    load_session_timeline,
)
from fall_detection.datadealer_loader import CombinedFrame, Point3D, load_datadealer_recording


def point(x, y, z, *, kind="motion"):
    return Point3D(x_m=x, y_m=y, z_m=z, velocity_m_s=0.2, snr_db=31.0, kind=kind)


class FakeTextLog:
    def __init__(self):
        self.lines = []

    def insert(self, _position, text):
        self.lines.append(text)

    def see(self, _position):
        pass


class FakeVar:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakePlaybackRoot:
    def __init__(self):
        self.delay_ms = None
        self.callback = None
        self.cancelled = []

    def after(self, delay_ms, callback):
        self.delay_ms = delay_ms
        self.callback = callback
        return "after-1"

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)


class FakeWidget:
    def __init__(self):
        self.options = {}

    def configure(self, **options):
        self.options.update(options)


class FakeScale:
    def __init__(self):
        self.value = 0

    def set(self, value):
        self.value = value


class FakeCamera:
    is_open = False

    def start_recording(self, _path):
        raise AssertionError("camera should not start in this test")

    def stop_recording(self):
        pass


class V2CaptureProtocolTests(unittest.TestCase):
    def test_readme_documents_the_complete_v2_capture_contract(self):
        readme = (
            Path(__file__).resolve().parents[2]
            / "fall_capture_station"
            / "README.md"
        ).read_text(encoding="utf-8")

        for token in (
            "mount_height_m",
            "pitch_deg",
            "subject_id",
            "environment_id",
            "scenario",
            "radar_profile",
            "上下文",
            "至少 10 秒",
            "不会修改历史会话",
        ):
            with self.subTest(token=token):
                self.assertIn(token, readme)

    def test_default_profile_serializes_the_complete_v2_metadata_contract(self):
        metadata = DEFAULT_CAPTURE_PROFILE.to_metadata()

        self.assertEqual(metadata["capture_schema_version"], 2)
        self.assertEqual(metadata["mount_height_m"], 1.63)
        self.assertEqual(metadata["pitch_deg"], 30.0)
        self.assertEqual(metadata["subject_id"], "P01")
        self.assertEqual(metadata["environment_id"], "room-a")
        self.assertEqual(metadata["scenario"], "mixed")
        self.assertEqual(metadata["protocol_version"], "fall-v2.0")
        self.assertEqual(metadata["radar_profile"], "mrs6240-50ms-r256-d16")

    def test_profile_rejects_missing_ids_invalid_numbers_and_unknown_scenario(self):
        invalid = (
            {"subject_id": ""},
            {"environment_id": ""},
            {"mount_height_m": float("nan")},
            {"mount_height_m": 0.49},
            {"mount_height_m": 5.01},
            {"pitch_deg": -90.01},
            {"pitch_deg": 90.01},
            {"scenario": "unsupported"},
            {"protocol_version": ""},
            {"radar_profile": ""},
        )
        baseline = DEFAULT_CAPTURE_PROFILE.to_metadata()
        baseline.pop("capture_schema_version")
        for replacement in invalid:
            with self.subTest(replacement=replacement):
                with self.assertRaises(CaptureProfileError):
                    CaptureProfile(**(baseline | replacement))

    def test_context_note_round_trips_without_changing_the_primary_mark(self):
        self.assertEqual(
            format_annotation_note("held_stool"),
            "annotation_app;context=held_stool",
        )
        self.assertEqual(
            parse_annotation_context("annotation_app;context=held_stool"),
            "held_stool",
        )
        self.assertEqual(parse_annotation_context("annotation_app"), "unknown")
        self.assertEqual(parse_annotation_context(""), "unknown")
        self.assertIn("plain", ANNOTATION_CONTEXTS)
        self.assertEqual(CAPTURE_SCHEMA_VERSION, 2)
        self.assertIn("blind_test", SESSION_SCENARIOS)


class CaptureStationRecorderTests(unittest.TestCase):
    def test_session_timeline_resolves_capture_radar_and_camera_references(self):
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "sample"
            session_dir.mkdir()
            (session_dir / "session.json").write_text(
                json.dumps(
                    {
                        "session_name": "sample",
                        "started_wall_time": "2026-07-15 16:21:15.541",
                    }
                ),
                encoding="utf-8",
            )
            (session_dir / "radar_events.jsonl").write_text(
                json.dumps(
                    {
                        "frame_index": 1355,
                        "sync_frame_index": 156,
                        "source_timestamp_ms": 1784132483322,
                        "source_timestamp_text": "2026-07-15-16:21:23.322",
                        "wall_time": "2026-07-15 16:21:23.322",
                        "relative_ms": 7781,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            (session_dir / "camera_frames.csv").write_text(
                "camera_frame_index,sync_frame_index,monotonic_ns,wall_time,relative_ms,video_time_ms\n"
                "179,156,1,2026-07-15 16:21:23.338,7797,7797\n",
                encoding="utf-8",
            )

            timeline = load_session_timeline(session_dir)
            reference = timeline.resolve(
                replay_frame_index=0,
                capture_relative_ms=7781,
                camera_offset_ms=0,
            )

            self.assertEqual(reference.capture_wall_time, "2026-07-15 16:21:23.322")
            self.assertEqual(reference.source_radar_frame_index, 1355)
            self.assertEqual(reference.source_timestamp_ms, 1784132483322)
            self.assertEqual(reference.sync_frame_index, 156)
            self.assertEqual(reference.camera_frame_index, 179)
            self.assertEqual(reference.camera_relative_ms, 7797)
            self.assertEqual(reference.camera_time_delta_ms, 16)

    def test_annotation_playback_uses_adjustable_fps_interval(self):
        self.assertEqual(DEFAULT_PLAYBACK_FPS, 5)
        self.assertEqual(PLAYBACK_FPS_OPTIONS, (1, 2, 5, 10, 20))
        self.assertEqual(_playback_interval_ms(5), 200)
        self.assertEqual(_playback_interval_ms(20), 50)

    def test_annotation_playback_stops_after_last_frame(self):
        self.assertEqual(_next_playback_index(0, 3), 1)
        self.assertEqual(_next_playback_index(1, 3), 2)
        self.assertIsNone(_next_playback_index(2, 3))
        self.assertIsNone(_next_playback_index(0, 0))

    def test_annotation_playback_advances_and_restores_play_button_at_end(self):
        app = AnnotationApp.__new__(AnnotationApp)
        app.root = FakePlaybackRoot()
        app.frames = [object(), object(), object()]
        app.current_index = 0
        app.playing = False
        app.playback_after_id = None
        app.playback_fps_var = FakeVar(5)
        app.play_button = FakeWidget()
        app.frame_scale = FakeScale()
        app.status_var = FakeVar("")
        app.session_name = "sample"
        app._updating_scale = False
        app.draw_current_frame = lambda: None

        app.start_playback()
        self.assertTrue(app.playing)
        self.assertEqual(app.play_button.options["text"], "暂停")
        self.assertEqual(app.root.delay_ms, 200)

        app._playback_tick()
        self.assertEqual(app.current_index, 1)
        self.assertTrue(app.playing)

        app._playback_tick()
        self.assertEqual(app.current_index, 2)
        self.assertFalse(app.playing)
        self.assertEqual(app.play_button.options["text"], "播放")
        self.assertIn("播放完成", app.status_var.get())

    def test_annotation_app_only_exposes_terminal_state_marks(self):
        self.assertEqual(
            ANNOTATION_MARKS,
            ("无人", "有人", "蹲下", "弯腰", "坐下", "跌倒"),
        )

    def test_annotation_marks_are_laid_out_in_visible_four_column_toolbar(self):
        positions = _annotation_mark_positions()

        self.assertEqual(MARK_BUTTON_COLUMNS, 4)
        self.assertEqual(len(positions), len(ANNOTATION_MARKS))
        self.assertLessEqual(max(position.column for position in positions), 3)
        self.assertLessEqual(max(position.row for position in positions), 1)

    def test_annotation_configures_chinese_matplotlib_font(self):
        _configure_chinese_matplotlib_font()

        self.assertEqual(rcParams["axes.unicode_minus"], False)
        self.assertEqual(rcParams["font.sans-serif"][0], CHINESE_FONT_CANDIDATES[0])

    def test_annotation_camera_lookup_applies_manual_time_offset(self):
        camera_rows = (
            {"camera_frame_index": "10", "relative_ms": "1000"},
            {"camera_frame_index": "11", "relative_ms": "1050"},
            {"camera_frame_index": "12", "relative_ms": "1100"},
        )

        self.assertEqual(_nearest_camera_row(camera_rows, radar_timestamp_ms=1000, camera_offset_ms=0)["camera_frame_index"], "10")
        self.assertEqual(_nearest_camera_row(camera_rows, radar_timestamp_ms=1000, camera_offset_ms=100)["camera_frame_index"], "12")
        self.assertEqual(_nearest_camera_row(camera_rows, radar_timestamp_ms=1100, camera_offset_ms=-100)["camera_frame_index"], "10")

    def test_annotation_point_groups_distinguish_motion_and_presence_points(self):
        groups = _point_groups(
            (
                point(0.1, 1.0, 0.3, kind="presence"),
                point(0.2, 1.1, 0.4, kind="motion"),
            )
        )

        self.assertEqual([group.label for group in groups], ["存在点", "运动点"])
        self.assertNotEqual(groups[0].color, groups[1].color)
        self.assertEqual([item.kind for item in groups[0].points], ["presence"])
        self.assertEqual([item.kind for item in groups[1].points], ["motion"])

    def test_annotation_writes_context_in_note_without_changing_mark(self):
        with tempfile.TemporaryDirectory() as tmp:
            frame = CombinedFrame(
                frame_index=131,
                timestamp_ms=7781,
                timestamp_text="2026-07-15-16:21:23.322",
                motion_points=(),
                presence_points=(),
            )
            reference = AnnotationEventReference(
                capture_wall_time="2026-07-15 16:21:23.322",
                source_radar_frame_index=1355,
            )
            app = AnnotationApp.__new__(AnnotationApp)
            app.session_dir = Path(tmp)
            app.frames = [SimpleNamespace(frame=frame)]
            app.current_index = 0
            app.context_var = FakeVar("held_stool")
            app.status_var = FakeVar("")
            app.pause_playback = lambda: None
            app._event_reference = lambda _frame: reference

            with patch("fall_capture_station.annotation_app.append_operator_mark") as append:
                app.write_mark("蹲下")

            append.assert_called_once()
            call = append.call_args
            self.assertEqual(call.args[1], "蹲下")
            self.assertEqual(call.kwargs["note"], "annotation_app;context=held_stool")
            self.assertIs(call.kwargs["event_reference"], reference)

    def test_capture_station_uses_non_destructive_start_and_stop_commands(self):
        self.assertEqual(RADAR_START_COMMAND, "mmwc start")
        self.assertEqual(RADAR_STOP_COMMAND, "mmwc stop")

    def test_capture_station_flushes_pre_recording_control_log_into_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = CaptureStationApp.__new__(CaptureStationApp)
            app.data_root = Path(tmp)
            app.session = None
            app.recording = False
            app.radar_frame_count = 3
            app.camera_frame_count = 4
            app.session_name_var = FakeVar("sample")
            app.camera_index_var = FakeVar(0)
            app.spi_index_var = FakeVar(0)
            app.spi_freq_mode_var = FakeVar(0)
            app.mount_height_var = FakeVar("1.63")
            app.pitch_deg_var = FakeVar("30")
            app.subject_id_var = FakeVar("P02")
            app.environment_id_var = FakeVar("room-b")
            app.scenario_var = FakeVar("object_assisted")
            app.protocol_version_var = FakeVar("fall-v2.0")
            app.radar_profile_var = FakeVar("mrs6240-50ms-r256-d16")
            app.profile_widgets = []
            app.record_var = FakeVar("")
            app.camera = FakeCamera()
            app.log_text = FakeTextLog()

            app._log("发 -> mmwc mode 4 2")
            app._log("发 -> mmwc start")
            app.start_recording()
            app.stop_recording()

            log_text = (app.session.session_dir / "serial_control.log").read_text(encoding="utf-8")

            self.assertIn("发 -> mmwc mode 4 2", log_text)
            self.assertIn("发 -> mmwc start", log_text)
            self.assertIn("recording started", log_text)
            self.assertIn("recording stopped", log_text)
            metadata = json.loads(
                (app.session.session_dir / "session.json").read_text(encoding="utf-8")
            )["metadata"]
            self.assertEqual(metadata["capture_schema_version"], 2)
            self.assertEqual(metadata["mount_height_m"], 1.63)
            self.assertEqual(metadata["pitch_deg"], 30.0)
            self.assertEqual(metadata["subject_id"], "P02")
            self.assertEqual(metadata["environment_id"], "room-b")
            self.assertEqual(metadata["scenario"], "object_assisted")

    def test_capture_station_rejects_invalid_profile_before_creating_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = CaptureStationApp.__new__(CaptureStationApp)
            app.data_root = Path(tmp)
            app.session = None
            app.recording = False
            app.radar_frame_count = 3
            app.camera_frame_count = 4
            app.session_name_var = FakeVar("invalid")
            app.camera_index_var = FakeVar(0)
            app.spi_index_var = FakeVar(0)
            app.spi_freq_mode_var = FakeVar(0)
            app.mount_height_var = FakeVar("1.63")
            app.pitch_deg_var = FakeVar("30")
            app.subject_id_var = FakeVar("")
            app.environment_id_var = FakeVar("room-b")
            app.scenario_var = FakeVar("mixed")
            app.protocol_version_var = FakeVar("fall-v2.0")
            app.radar_profile_var = FakeVar("mrs6240-50ms-r256-d16")
            app.profile_widgets = []
            app.record_var = FakeVar("")
            app.camera = FakeCamera()
            app.log_text = FakeTextLog()

            with patch("fall_capture_station.capture_app.messagebox.showerror") as showerror:
                app.start_recording()

            self.assertIsNone(app.session)
            self.assertFalse(app.recording)
            self.assertEqual(list(Path(tmp).iterdir()), [])
            showerror.assert_called_once()

    def test_capture_station_restores_profile_widget_states_after_recording(self):
        app = CaptureStationApp.__new__(CaptureStationApp)
        entry = FakeWidget()
        combo = FakeWidget()
        app.profile_widgets = [(entry, "normal"), (combo, "readonly")]

        app._set_profile_controls_enabled(False)

        self.assertEqual(entry.options["state"], "disabled")
        self.assertEqual(combo.options["state"], "disabled")

        app._set_profile_controls_enabled(True)

        self.assertEqual(entry.options["state"], "normal")
        self.assertEqual(combo.options["state"], "readonly")

    def test_session_recorder_writes_datadealer_compatible_pointcloud_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder.create(Path(tmp), session_name="sample", metadata={"scene": "fall"})
            frame = CombinedFrame(
                frame_index=7,
                timestamp_ms=1234,
                timestamp_text="radar-t7",
                motion_points=(point(0.1, 1.0, 0.3, kind="motion"),),
                presence_points=(point(0.2, 1.1, 0.4, kind="presence"),),
            )

            recorder.write_radar_frame(frame, monotonic_ns=10_000_000, wall_time="2026-07-08 12:00:00.000")
            recorder.close()

            recording = load_datadealer_recording(recorder.session_dir, stem="sample")

            self.assertEqual(recording.stem, "sample")
            self.assertEqual(len(recording.frames), 1)
            self.assertEqual(recording.frames[0].motion_count, 1)
            self.assertEqual(recording.frames[0].presence_count, 1)
            metadata = json.loads((recorder.session_dir / "session.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["session_name"], "sample")
            self.assertEqual(metadata["metadata"]["scene"], "fall")

    def test_session_recorder_writes_camera_frame_index_and_operator_marks(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder.create(Path(tmp), session_name="sample")

            recorder.write_camera_frame(frame_index=3, monotonic_ns=20_000_000, wall_time="2026-07-08 12:00:00.033", video_time_ms=33)
            recorder.write_operator_mark("开始蹲下", monotonic_ns=30_000_000, wall_time="2026-07-08 12:00:00.066", note="manual")
            recorder.close()
            append_operator_mark(recorder.session_dir, "完全蹲下", monotonic_ns=40_000_000, wall_time="2026-07-08 12:00:00.099", note="")

            with (recorder.session_dir / "camera_frames.csv").open("r", encoding="utf-8", newline="") as f:
                camera_rows = list(csv.DictReader(f))
            with (recorder.session_dir / "operator_marks.csv").open("r", encoding="utf-8", newline="") as f:
                mark_rows = list(csv.DictReader(f))

            self.assertEqual(camera_rows[0]["camera_frame_index"], "3")
            self.assertEqual(camera_rows[0]["video_time_ms"], "33")
            self.assertEqual([row["mark"] for row in mark_rows], ["开始蹲下", "完全蹲下"])

    def test_operator_mark_writes_capture_event_reference_separately_from_click_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder.create(Path(tmp), session_name="sample")
            reference = AnnotationEventReference(
                capture_wall_time="2026-07-15 16:21:23.322",
                source_radar_frame_index=1355,
                source_timestamp_ms=1784132483322,
                source_timestamp_text="2026-07-15-16:21:23.322",
                sync_frame_index=156,
                camera_frame_index=179,
                camera_relative_ms=7797,
                camera_video_time_ms=7797,
                camera_wall_time="2026-07-15 16:21:23.338",
                camera_offset_ms=0,
                camera_time_delta_ms=16,
            )

            append_operator_mark(
                recorder.session_dir,
                "蹲下",
                monotonic_ns=30_000_000,
                wall_time="2026-07-15 16:25:28.656",
                note="annotation_app",
                radar_frame_index=131,
                radar_timestamp_ms=7781,
                event_reference=reference,
            )

            with (recorder.session_dir / "operator_marks.csv").open("r", encoding="utf-8", newline="") as f:
                row = next(csv.DictReader(f))

            self.assertEqual(row["wall_time"], "2026-07-15 16:25:28.656")
            self.assertEqual(row["capture_wall_time"], "2026-07-15 16:21:23.322")
            self.assertEqual(row["source_radar_frame_index"], "1355")
            self.assertEqual(row["camera_frame_index"], "179")
            self.assertEqual(row["camera_time_delta_ms"], "16")

    def test_backfill_upgrades_legacy_marks_with_recorded_event_references(self):
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "sample"
            session_dir.mkdir()
            (session_dir / "session.json").write_text(
                json.dumps(
                    {
                        "session_name": "sample",
                        "started_wall_time": "2026-07-15 16:21:15.541",
                    }
                ),
                encoding="utf-8",
            )
            (session_dir / "radar_events.jsonl").write_text(
                json.dumps(
                    {
                        "frame_index": 1355,
                        "sync_frame_index": 156,
                        "source_timestamp_ms": 1784132483322,
                        "source_timestamp_text": "2026-07-15-16:21:23.322",
                        "wall_time": "2026-07-15 16:21:23.322",
                        "relative_ms": 7781,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            (session_dir / "camera_frames.csv").write_text(
                "camera_frame_index,sync_frame_index,monotonic_ns,wall_time,relative_ms,video_time_ms\n"
                "179,156,1,2026-07-15 16:21:23.338,7797,7797\n",
                encoding="utf-8",
            )
            marks_path = session_dir / "operator_marks.csv"
            marks_path.write_text(
                "mark,monotonic_ns,wall_time,note,radar_frame_index,radar_timestamp_ms\n"
                "蹲下,110389890000000,2026-07-15 16:25:28.656,annotation_app,0,7781\n",
                encoding="utf-8",
            )

            updated = backfill_operator_mark_references(session_dir)

            with marks_path.open("r", encoding="utf-8", newline="") as f:
                row = next(csv.DictReader(f))
            self.assertEqual(updated, 1)
            self.assertEqual(row["wall_time"], "2026-07-15 16:25:28.656")
            self.assertEqual(row["capture_wall_time"], "2026-07-15 16:21:23.322")
            self.assertEqual(row["source_radar_frame_index"], "1355")
            self.assertEqual(row["camera_frame_index"], "179")

    def test_session_recorder_writes_shared_sync_frame_index_for_camera_and_radar(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder.create(Path(tmp), session_name="sample")
            event_ns = recorder.started_monotonic_ns + 75_000_000
            frame = CombinedFrame(
                frame_index=10,
                timestamp_ms=75,
                timestamp_text="radar-t10",
                motion_points=(point(0.1, 1.0, 0.3, kind="motion"),),
                presence_points=(),
            )

            recorder.write_radar_frame(frame, monotonic_ns=event_ns, wall_time="2026-07-08 12:00:00.075")
            recorder.write_camera_frame(frame_index=2, monotonic_ns=event_ns, wall_time="2026-07-08 12:00:00.075", video_time_ms=75)
            recorder.close()

            with (recorder.session_dir / "camera_frames.csv").open("r", encoding="utf-8", newline="") as f:
                camera_rows = list(csv.DictReader(f))
            with (recorder.session_dir / "radar_events.jsonl").open("r", encoding="utf-8") as f:
                radar_events = [json.loads(line) for line in f]

            self.assertEqual(camera_rows[0]["sync_frame_index"], "2")
            self.assertEqual(radar_events[0]["sync_frame_index"], 2)

    def test_camera_recording_resets_video_frame_index(self):
        camera = CameraRecorder()
        camera.frame_index = 42
        camera.open = lambda: None
        with tempfile.TemporaryDirectory() as tmp:
            camera.start_recording(Path(tmp) / "camera.mp4")

        self.assertEqual(camera.frame_index, 0)

    def test_camera_backend_read_runs_off_the_gui_thread_and_latest_read_is_nonblocking(self):
        class BlockingCapture:
            def __init__(self):
                self.read_started = threading.Event()
                self.released = threading.Event()

            def isOpened(self):
                return not self.released.is_set()

            def read(self):
                self.read_started.set()
                self.released.wait(2.0)
                return False, None

            def release(self):
                self.released.set()

            def get(self, _property):
                return 30.0

        class BlockingCv2:
            CAP_DSHOW = 700

            def __init__(self, capture):
                self.capture = capture

            def VideoCapture(self, _index, _backend):
                return self.capture

        capture = BlockingCapture()
        camera = CameraRecorder()
        with patch(
            "fall_capture_station.camera_recorder._import_cv2",
            return_value=BlockingCv2(capture),
        ):
            started = time.perf_counter()
            camera.open()
            open_elapsed = time.perf_counter() - started

            self.assertLess(open_elapsed, 0.1)
            self.assertTrue(capture.read_started.wait(0.5))
            started = time.perf_counter()
            self.assertIsNone(camera.read_frame())
            read_elapsed = time.perf_counter() - started
            self.assertLess(read_elapsed, 0.05)
            camera.close()

        self.assertTrue(capture.released.is_set())

    def test_capture_station_reports_camera_opening_until_worker_is_ready(self):
        class AsyncCamera:
            def __init__(self):
                self.camera_index = 0
                self.is_open = False
                self.open_called = False

            def open(self):
                self.open_called = True

        app = CaptureStationApp.__new__(CaptureStationApp)
        app.camera = AsyncCamera()
        app.camera_index_var = FakeVar(2)
        app.camera_status_var = FakeVar("")
        app.log_text = FakeTextLog()
        app.session = None
        app._pending_session_log_lines = []

        app.open_camera()

        self.assertTrue(app.camera.open_called)
        self.assertEqual(app.camera.camera_index, 2)
        self.assertEqual(app.camera_status_var.get(), "摄像头正在打开 index=2")


if __name__ == "__main__":
    unittest.main()
