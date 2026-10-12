import tempfile
import unittest
from pathlib import Path

from fall_detection.operator_marks import load_operator_marks


class OperatorMarksTests(unittest.TestCase):
    def test_load_operator_marks_deduplicates_and_sorts_by_radar_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "operator_marks.csv"
            path.write_text(
                "mark,monotonic_ns,wall_time,note,radar_frame_index,radar_timestamp_ms\n"
                "完全蹲下,300,2026-07-10 10:00:03.000,annotation_app,30,1500\n"
                "开始蹲下,200,2026-07-10 10:00:02.000,annotation_app,20,1000\n"
                "开始蹲下,201,2026-07-10 10:00:02.001,duplicate,20,1000\n"
                "真实跌倒,400,2026-07-10 10:00:04.000,annotation_app,40,2000\n",
                encoding="utf-8",
            )

            marks = load_operator_marks(path)

        self.assertEqual(
            [(mark.radar_frame_index, mark.mark) for mark in marks],
            [(20, "开始蹲下"), (30, "完全蹲下"), (40, "真实跌倒")],
        )
        self.assertEqual(marks[0].radar_timestamp_ms, 1000)
        self.assertEqual(marks[0].note, "annotation_app")

    def test_load_operator_marks_exposes_recorded_event_references(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "operator_marks.csv"
            path.write_text(
                "mark,monotonic_ns,wall_time,note,radar_frame_index,radar_timestamp_ms,"
                "capture_wall_time,source_radar_frame_index,source_timestamp_ms,source_timestamp_text,"
                "sync_frame_index,camera_frame_index,camera_relative_ms,camera_video_time_ms,"
                "camera_wall_time,camera_offset_ms,camera_time_delta_ms\n"
                "蹲下,300,2026-07-15 16:25:28.656,annotation_app,131,7781,"
                "2026-07-15 16:21:23.322,1355,1784132483322,2026-07-15-16:21:23.322,"
                "156,179,7797,7797,2026-07-15 16:21:23.338,0,16\n",
                encoding="utf-8",
            )

            mark = load_operator_marks(path)[0]

        self.assertEqual(mark.capture_wall_time, "2026-07-15 16:21:23.322")
        self.assertEqual(mark.source_radar_frame_index, 1355)
        self.assertEqual(mark.source_timestamp_ms, 1784132483322)
        self.assertEqual(mark.camera_frame_index, 179)
        self.assertEqual(mark.camera_time_delta_ms, 16)


if __name__ == "__main__":
    unittest.main()
