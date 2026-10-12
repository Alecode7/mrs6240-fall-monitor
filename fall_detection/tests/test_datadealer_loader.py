import csv
import tempfile
import unittest
from pathlib import Path

from fall_detection.datadealer_loader import (
    Point3D,
    iter_sample_stems,
    iter_sdk_pointcloud_csv,
    iter_pointcloud_csv,
    load_datadealer_recording,
    parse_pointcloud_row,
)


class DatadealerLoaderTests(unittest.TestCase):
    def test_parse_pointcloud_row_splits_xyz_velocity_and_snr_arrays(self):
        row = [
            "2026_07_03_11:10:05:745",
            "1783048205745",
            "1.0",
            "2.0",
            "0.5",
            "0.6",
            "1.7",
            "-0.1",
            "0.2",
            "-0.1",
            "30",
            "40",
        ]

        frame = parse_pointcloud_row(row, kind="motion", frame_index=3)

        self.assertEqual(frame.frame_index, 3)
        self.assertEqual(frame.timestamp_text, "2026_07_03_11:10:05:745")
        self.assertEqual(frame.timestamp_ms, 1783048205745)
        self.assertEqual(frame.kind, "motion")
        self.assertEqual(
            frame.points,
            (
                Point3D(x_m=1.0, y_m=0.5, z_m=1.7, velocity_m_s=0.2, snr_db=30.0, kind="motion"),
                Point3D(x_m=2.0, y_m=0.6, z_m=-0.1, velocity_m_s=-0.1, snr_db=40.0, kind="motion"),
            ),
        )

    def test_iter_pointcloud_csv_preserves_empty_motion_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "motion.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "1000"])
                writer.writerow(["t1", "1050", "0.1", "0.2", "1.0", "0.0", "20"])

            frames = list(iter_pointcloud_csv(path, kind="motion"))

        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].point_count, 0)
        self.assertEqual(frames[1].point_count, 1)

    def test_load_datadealer_recording_merges_motion_and_presence_by_timestamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "sample_motion_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "1000", "0.1", "0.2", "1.0", "0.3", "18"])
                writer.writerow(["t1", "1050"])
            with (root / "sample_presence_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "1000", "0.2", "0.3", "1.1", "0.0", "50"])
                writer.writerow(["t1", "1050", "0.4", "0.5", "0.2", "0.0", "60"])

            recording = load_datadealer_recording(root, stem="sample")

        self.assertEqual(len(recording.frames), 2)
        self.assertEqual(recording.frames[0].motion_count, 1)
        self.assertEqual(recording.frames[0].presence_count, 1)
        self.assertEqual(recording.frames[1].motion_count, 0)
        self.assertEqual(recording.frames[1].presence_count, 1)

    def test_load_datadealer_recording_resolves_capture_session_subdir_from_stem(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            session_dir = root / "sample"
            session_dir.mkdir()
            with (session_dir / "sample_motion_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "1000", "0.1", "0.2", "1.0", "0.3", "18"])
            with (session_dir / "sample_presence_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "1000", "0.2", "0.3", "1.1", "0.0", "50"])

            recording = load_datadealer_recording(root, stem="sample")

        self.assertEqual(recording.motion_path.parent.name, "sample")
        self.assertEqual(recording.presence_path.parent.name, "sample")
        self.assertEqual(len(recording.frames), 1)
        self.assertEqual(recording.frames[0].motion_count, 1)
        self.assertEqual(recording.frames[0].presence_count, 1)

    def test_iter_sdk_pointcloud_csv_groups_table_rows_by_frame_index(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample_Point_cloud_conversion.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"])
                writer.writerow([10, 1, 0.4, 0.5, 0.6, 0.2, 30])
                writer.writerow([10, 0, 0.1, 0.2, 0.3, 0.1, 20])
                writer.writerow([12, 0, -0.1, 0.7, 0.8, -0.2, 25])

            frames = list(iter_sdk_pointcloud_csv(path, kind="motion"))

        self.assertEqual([frame.timestamp_ms for frame in frames], [10, 12])
        self.assertEqual(frames[0].timestamp_text, "frame 10")
        self.assertEqual(frames[0].point_count, 2)
        self.assertEqual(
            frames[0].points,
            (
                Point3D(x_m=0.1, y_m=0.2, z_m=0.3, velocity_m_s=0.1, snr_db=20.0, kind="motion"),
                Point3D(x_m=0.4, y_m=0.5, z_m=0.6, velocity_m_s=0.2, snr_db=30.0, kind="motion"),
            ),
        )

    def test_iter_sdk_pointcloud_csv_converts_polar_coordinates(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample_Point_cloud_conversion.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "Range", "Azimuth", "Elevation", "Velocity", "SNR"])
                writer.writerow([5, 0, 2.0, 90.0, 0.0, 0.3, 40])

            frame = next(iter_sdk_pointcloud_csv(path, kind="motion"))

        self.assertAlmostEqual(frame.points[0].x_m, 2.0)
        self.assertAlmostEqual(frame.points[0].y_m, 0.0, places=6)
        self.assertAlmostEqual(frame.points[0].z_m, 0.0)
        self.assertEqual(frame.points[0].kind, "motion")

    def test_load_datadealer_recording_can_read_sdk_05_data_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "sample_Point_cloud_conversion.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"])
                writer.writerow([10, 0, 0.1, 0.2, 0.3, 0.1, 20])
                writer.writerow([12, 0, 0.4, 0.5, 0.6, 0.2, 30])
            with (root / "sample_Point_cloud_MPC_conversion.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"])
                writer.writerow([11, 0, -0.1, 0.7, 0.8, 0.0, 45])

            recording = load_datadealer_recording(root, stem="sample")

        self.assertEqual([frame.timestamp_ms for frame in recording.frames], [0, 50, 100])
        self.assertEqual([frame.timestamp_text for frame in recording.frames], ["frame 10", "frame 11", "frame 12"])
        self.assertAlmostEqual(recording.duration_s, 0.1)
        self.assertEqual([frame.motion_count for frame in recording.frames], [1, 0, 1])
        self.assertEqual([frame.presence_count for frame in recording.frames], [0, 1, 0])
        self.assertEqual(recording.motion_path.name, "sample_Point_cloud_conversion.csv")
        self.assertEqual(recording.presence_path.name, "sample_Point_cloud_MPC_conversion.csv")

    def test_load_sdk_recording_fills_missing_empty_frame_indices(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "sample_Point_cloud_conversion.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"])
                writer.writerow([10, 0, 0.1, 0.2, 0.3, 0.1, 20])
                writer.writerow([12, 0, 0.4, 0.5, 0.6, 0.2, 30])
            with (root / "sample_Point_cloud_MPC_conversion.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"])

            recording = load_datadealer_recording(root, stem="sample")

        self.assertEqual([frame.timestamp_ms for frame in recording.frames], [0, 50, 100])
        self.assertEqual([frame.timestamp_text for frame in recording.frames], ["frame 10", "frame 11", "frame 12"])
        self.assertEqual([frame.motion_count for frame in recording.frames], [1, 0, 1])
        self.assertEqual([frame.presence_count for frame in recording.frames], [0, 0, 0])
        self.assertEqual(recording.frames[1].all_points, ())

    def test_iter_sample_stems_includes_sdk_05_data_stems(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "sample_Point_cloud_conversion.csv").write_text(
                "FrameIndex,PointIndex,X,Y,Z,Velocity,SNR\n",
                encoding="utf-8",
            )

            stems = list(iter_sample_stems(root))

        self.assertEqual(stems, ["sample"])

    def test_iter_sample_stems_includes_capture_session_subdirs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            session_dir = root / "sample"
            session_dir.mkdir()
            (session_dir / "sample_motion_point_cloud.csv").write_text("t0,1000\n", encoding="utf-8")
            (session_dir / "sample_presence_point_cloud.csv").write_text("t0,1000\n", encoding="utf-8")

            stems = list(iter_sample_stems(root))

        self.assertEqual(stems, ["sample"])


if __name__ == "__main__":
    unittest.main()
