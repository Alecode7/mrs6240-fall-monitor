import csv
import json
import time
import unittest
from pathlib import Path
from queue import Queue
from tempfile import TemporaryDirectory

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.live_source import LatestFrameBuffer
from fall_detection.recording_replay import (
    RecordedSessionReplay,
    load_recorded_session,
)


def _point_columns(x: float, y: float, z: float, *, velocity: float, snr: float) -> list[float]:
    return [x, y, z, velocity, snr]


def _write_point_cloud(path: Path, rows: list[tuple[str, int, list[float]]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        for timestamp_text, relative_ms, values in rows:
            writer.writerow([timestamp_text, relative_ms, *values])


def _make_session(root: Path) -> Path:
    session_dir = root / "sample"
    session_dir.mkdir()
    _write_point_cloud(
        session_dir / "sample_motion_point_cloud.csv",
        [
            ("radar-0", 0, _point_columns(-0.1, 1.0, 0.8, velocity=0.2, snr=30.0)),
            ("radar-1", 100, _point_columns(-0.2, 1.1, 0.7, velocity=0.3, snr=31.0)),
        ],
    )
    _write_point_cloud(
        session_dir / "sample_presence_point_cloud.csv",
        [
            ("radar-0", 0, _point_columns(0.1, 1.0, 0.9, velocity=0.0, snr=35.0)),
            ("radar-1", 100, _point_columns(0.2, 1.1, 0.8, velocity=0.0, snr=36.0)),
        ],
    )
    (session_dir / "session.json").write_text(
        json.dumps(
            {
                "session_name": "sample",
                "started_wall_time": "2026-07-15 10:00:00.000",
            }
        ),
        encoding="utf-8",
    )
    (session_dir / "serial_control.log").write_text(
        "\n".join(
            (
                "[2026-07-15 10:00:00.000] recording started: sample",
                "[2026-07-15 10:00:00.020] 发 -> mmwc start",
                "[2026-07-15 10:00:00.050] 收 <- FALL frame=10 state=1 target=1",
                "[2026-07-15 10:00:00.060] 收 <- malformed board line",
                "[2026-07-15 10:00:00.080] 收 <- FALL_EVENT state=suspected frame=10",
                "[2026-07-15 10:00:00.120] recording stopped",
                "[2026-07-15 10:00:00.200] 收 <- after recording",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    return session_dir


class RecordingReplayTests(unittest.TestCase):
    def test_load_recorded_session_reads_point_pair_and_serial_boundaries(self):
        with TemporaryDirectory() as temporary:
            session = load_recorded_session(_make_session(Path(temporary)))

        self.assertEqual(session.stem, "sample")
        self.assertEqual([frame.frame_index for frame in session.frames], [0, 1])
        self.assertEqual([frame.timestamp_ms for frame in session.frames], [0, 100])
        self.assertEqual(session.frames[0].motion_count, 1)
        self.assertEqual(session.frames[0].presence_count, 1)
        self.assertEqual([item.relative_ms for item in session.serial_records], [0, 20, 50, 60, 80, 120])
        self.assertEqual([item.record.direction for item in session.serial_records], ["rx", "tx", "rx", "rx", "rx", "rx"])
        self.assertIsNotNone(session.serial_records[2].record.status)
        self.assertIsNotNone(session.serial_records[4].record.event)
        self.assertEqual(session.duration_ms, 120)

    def test_replay_publishes_frames_and_serial_records_on_shared_timeline(self):
        with TemporaryDirectory() as temporary:
            session = load_recorded_session(_make_session(Path(temporary)))
            point_buffer = LatestFrameBuffer()
            serial_records = Queue()
            replay = RecordedSessionReplay(session, speed=1000.0)

            replay.start(point_buffer, serial_records)
            replay.join(timeout_s=1.0)

            self.assertFalse(replay.is_running)
            self.assertEqual([frame.frame_index for frame in point_buffer.take_pending()], [0, 1])
            received = []
            while not serial_records.empty():
                received.append(serial_records.get_nowait())
            self.assertEqual([record.line for record in received], [item.record.line for item in session.serial_records])

    def test_replay_can_be_stopped_before_all_items_are_published(self):
        frame = CombinedFrame(
            frame_index=0,
            timestamp_ms=10_000,
            timestamp_text="late",
            presence_points=(Point3D(0.0, 1.0, 0.8, 0.0, 30.0, "presence"),),
            motion_points=(),
        )
        session = type("Session", (), {"frames": (frame,), "serial_records": (), "duration_ms": 10_000})()
        point_buffer = LatestFrameBuffer()
        serial_records = Queue()
        replay = RecordedSessionReplay(session, speed=1.0)

        replay.start(point_buffer, serial_records)
        time.sleep(0.01)
        replay.stop()
        replay.join(timeout_s=1.0)

        self.assertFalse(replay.is_running)
        self.assertEqual(point_buffer.take_pending(), ())


if __name__ == "__main__":
    unittest.main()
