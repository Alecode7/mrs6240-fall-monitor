import unittest

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.live_source import LatestFrameBuffer, ReplayLiveSource


def frame(index):
    return CombinedFrame(
        frame_index=index,
        timestamp_ms=index * 50,
        timestamp_text=f"t{index}",
        presence_points=(Point3D(0.0, 1.0, 1.0, 0.0, 30.0, "presence"),),
        motion_points=(),
    )


class LiveSourceTests(unittest.TestCase):
    def test_latest_frame_buffer_can_drain_all_pending_frames_in_order(self):
        buffer = LatestFrameBuffer()
        buffer.publish(frame(0))
        buffer.publish(frame(1))
        buffer.publish(frame(2))

        pending = buffer.take_pending()

        self.assertEqual([item.frame_index for item in pending], [0, 1, 2])
        self.assertEqual(buffer.published_count, 3)
        self.assertEqual(buffer.consumed_count, 3)
        self.assertEqual(buffer.take_pending(), ())
        self.assertIsNone(buffer.take_latest())

    def test_latest_frame_buffer_returns_only_newest_unconsumed_frame(self):
        buffer = LatestFrameBuffer()
        buffer.publish(frame(0))
        buffer.publish(frame(1))

        latest = buffer.take_latest()

        self.assertIsNotNone(latest)
        self.assertEqual(latest.frame_index, 1)
        self.assertIsNone(buffer.take_latest())
        self.assertEqual(buffer.published_count, 2)
        self.assertEqual(buffer.consumed_count, 1)

    def test_replay_live_source_publishes_frames_to_latest_buffer(self):
        buffer = LatestFrameBuffer()
        source = ReplayLiveSource([frame(0), frame(1), frame(2)], interval_s=0.0)

        source.start(buffer)
        source.join(timeout_s=2.0)

        latest = buffer.take_latest()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.frame_index, 2)
        self.assertEqual(buffer.published_count, 3)
        self.assertFalse(source.is_running)


if __name__ == "__main__":
    unittest.main()
