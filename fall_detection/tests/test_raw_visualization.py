import unittest

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.raw_visualization import RawFrameHistory, raw_view_from_frame


def frame(index: int) -> CombinedFrame:
    return CombinedFrame(
        frame_index=index,
        timestamp_ms=index * 50,
        timestamp_text=f"t{index}",
        presence_points=(Point3D(0.0, 1.0, 0.8, 0.0, 40.0, "presence"),),
        motion_points=(Point3D(0.1, 1.1, 0.7, 0.3, 30.0, "motion"),),
    )


class RawVisualizationTests(unittest.TestCase):
    def test_raw_view_preserves_presence_and_motion_points_without_target(self):
        source = frame(7)

        view = raw_view_from_frame(source)

        self.assertEqual(view.frame, source)
        self.assertEqual(view.presence_points, source.presence_points)
        self.assertEqual(view.motion_points, source.motion_points)
        self.assertIsNone(view.target)
        self.assertEqual(view.total_count, 2)
        self.assertEqual(view.presence_count, 1)
        self.assertEqual(view.motion_count, 1)
        self.assertAlmostEqual(view.max_abs_velocity_m_s, 0.3)
        self.assertAlmostEqual(view.max_z_m, 0.8)
        self.assertAlmostEqual(view.p95_z_m, 0.8)

    def test_raw_view_p95_height_rejects_one_high_outlier_for_display(self):
        stable_points = tuple(Point3D(0.0, 1.0, 0.8, 0.0, 40.0, "presence") for _ in range(19))
        source = CombinedFrame(
            frame_index=8,
            timestamp_ms=400,
            timestamp_text="t8",
            presence_points=stable_points,
            motion_points=(Point3D(0.0, 1.0, 2.5, 0.0, 40.0, "motion"),),
        )

        view = raw_view_from_frame(source)

        self.assertAlmostEqual(view.max_z_m, 2.5)
        self.assertAlmostEqual(view.p95_z_m, 0.8)

    def test_raw_history_has_bounded_frames_and_timeline_metrics(self):
        history = RawFrameHistory(max_frames=2)
        history.append(raw_view_from_frame(frame(0)))
        history.append(raw_view_from_frame(frame(1)))
        history.append(raw_view_from_frame(frame(2)))

        self.assertEqual([item.frame.frame_index for item in history.views], [1, 2])
        timeline = history.timeline()
        self.assertEqual(timeline.frame_indices, (1, 2))
        self.assertEqual(timeline.motion_counts, (1, 1))
        self.assertEqual(timeline.max_velocity_m_s, (0.3, 0.3))
        self.assertEqual(timeline.p95_z_m, (0.8, 0.8))

    def test_raw_history_replaces_duplicate_frame_index(self):
        history = RawFrameHistory(max_frames=3)
        history.append(raw_view_from_frame(frame(1)))
        changed = CombinedFrame(
            frame_index=1,
            timestamp_ms=55,
            timestamp_text="updated",
            presence_points=(),
            motion_points=(),
        )
        history.append(raw_view_from_frame(changed))

        self.assertEqual(len(history.views), 1)
        self.assertEqual(history.views[0].frame.timestamp_text, "updated")


if __name__ == "__main__":
    unittest.main()
