import unittest
from types import SimpleNamespace

from fall_detection.device_source import HifDevicePointCloudSource, point_clouds_to_combined_frame
from fall_detection.live_source import LatestFrameBuffer


class FakeCallbackEvent:
    def __init__(self):
        self.handlers = []

    def __iadd__(self, handler):
        self.handlers.append(handler)
        return self

    def __isub__(self, handler):
        self.handlers.remove(handler)
        return self

    def emit(self, receive_data):
        for handler in tuple(self.handlers):
            handler(self, receive_data)


class FakeHifApi:
    def __init__(self, point_clouds):
        self.callback = FakeCallbackEvent()
        self.point_clouds = point_clouds
        self.calls = []

    def OpenSpiDevice(self, index, freq_mode):
        self.calls.append(("OpenSpiDevice", index, freq_mode))
        return True

    def CloseSpiDevice(self, index):
        self.calls.append(("CloseSpiDevice", index))
        return True

    def StartCollectingData(self):
        self.calls.append(("StartCollectingData",))
        return 0

    def StopCollectingData(self):
        self.calls.append(("StopCollectingData",))
        return 0

    def ConvertPointCloudData(self, payload, frame_index, frame_len):
        self.calls.append(("ConvertPointCloudData", payload, frame_index, frame_len))
        return self.point_clouds


def fake_point_cloud(signal_name, *, xs, ys, zs, velocities, snrs):
    return SimpleNamespace(
        signalName=signal_name,
        pointNum=len(xs),
        x=xs,
        y=ys,
        z=zs,
        w=velocities,
        u=snrs,
    )


class DeviceSourceTests(unittest.TestCase):
    def test_point_cloud_objects_convert_to_combined_frame_by_signal_name(self):
        motion = fake_point_cloud(
            "Point_cloud_conversion",
            xs=[0.1, 0.2],
            ys=[1.0, 1.1],
            zs=[0.8, 0.9],
            velocities=[0.3, -0.4],
            snrs=[32.0, 33.0],
        )
        presence = fake_point_cloud(
            "Point_cloud_MPC_conversion",
            xs=[-0.1],
            ys=[1.2],
            zs=[0.7],
            velocities=[0.0],
            snrs=[40.0],
        )

        frame = point_clouds_to_combined_frame((motion, presence), frame_index=7, timestamp_ms=1234)

        self.assertEqual(frame.frame_index, 7)
        self.assertEqual(frame.timestamp_ms, 1234)
        self.assertEqual(frame.motion_count, 2)
        self.assertEqual(frame.presence_count, 1)
        self.assertEqual(frame.motion_points[0].kind, "motion")
        self.assertEqual(frame.presence_points[0].kind, "presence")
        self.assertAlmostEqual(frame.motion_points[1].velocity_m_s, -0.4)
        self.assertAlmostEqual(frame.motion_points[1].snr_db, 33.0)

    def test_observed_datacollection_point_cloud_names_and_fields_convert(self):
        motion = fake_point_cloud(
            "motion_point_cloud",
            xs=[0.050, 0.040],
            ys=[0.340, 0.330],
            zs=[-0.080, -0.070],
            velocities=[-0.210, 0.210],
            snrs=[18.310, 22.370],
        )
        presence = fake_point_cloud(
            "presence_point_cloud",
            xs=[-0.060, -0.050],
            ys=[0.090, 0.100],
            zs=[0.000, 0.010],
            velocities=[0.000, 0.000],
            snrs=[62.000, 61.000],
        )

        frame = point_clouds_to_combined_frame(
            (motion, presence),
            frame_index=27,
            timestamp_ms=1783398446769,
            timestamp_text="2026-07-07-12:27:26.769",
        )

        self.assertEqual(frame.timestamp_text, "2026-07-07-12:27:26.769")
        self.assertEqual(frame.motion_count, 2)
        self.assertEqual(frame.presence_count, 2)
        self.assertEqual(frame.motion_points[0].kind, "motion")
        self.assertAlmostEqual(frame.motion_points[0].x_m, 0.050)
        self.assertAlmostEqual(frame.motion_points[0].y_m, 0.340)
        self.assertAlmostEqual(frame.motion_points[0].z_m, -0.080)
        self.assertAlmostEqual(frame.motion_points[0].velocity_m_s, -0.210)
        self.assertAlmostEqual(frame.motion_points[0].snr_db, 18.310)
        self.assertEqual(frame.presence_points[0].kind, "presence")
        self.assertAlmostEqual(frame.presence_points[0].x_m, -0.060)
        self.assertAlmostEqual(frame.presence_points[0].y_m, 0.090)
        self.assertAlmostEqual(frame.presence_points[0].z_m, 0.000)
        self.assertAlmostEqual(frame.presence_points[0].velocity_m_s, 0.000)
        self.assertAlmostEqual(frame.presence_points[0].snr_db, 62.000)

    def test_hif_device_source_uses_collect_only_and_publishes_point_cloud_callback(self):
        point_clouds = [
            fake_point_cloud(
                "Point_cloud_conversion",
                xs=[0.0],
                ys=[1.0],
                zs=[0.5],
                velocities=[0.2],
                snrs=[35.0],
            )
        ]
        api = FakeHifApi(point_clouds)
        buffer = LatestFrameBuffer()
        source = HifDevicePointCloudSource(api_factory=lambda: api, transport="spi", spi_index=2, spi_freq_mode=3)

        source.start(buffer)
        api.callback.emit(
            SimpleNamespace(dataType=4, frameIndex=9, frameLen=64, timeStamp=1500, payloadData=b"payload")
        )
        source.stop()
        source.join()

        frame = buffer.take_latest()
        self.assertIsNotNone(frame)
        self.assertEqual(frame.frame_index, 9)
        self.assertEqual(frame.timestamp_ms, 1500)
        self.assertEqual(frame.motion_count, 1)
        self.assertEqual(api.calls[0], ("OpenSpiDevice", 2, 3))
        self.assertIn(("StartCollectingData",), api.calls)
        self.assertIn(("StopCollectingData",), api.calls)
        self.assertIn(("CloseSpiDevice", 2), api.calls)
        self.assertNotIn(("StopRadar",), api.calls)
        self.assertNotIn(("StartRadar",), api.calls)
        self.assertNotIn(("SendConfigFile",), api.calls)

    def test_hif_device_source_accepts_datetime_string_timestamp(self):
        api = FakeHifApi(
            [
                fake_point_cloud(
                    "Point_cloud_conversion",
                    xs=[0.0],
                    ys=[1.0],
                    zs=[0.5],
                    velocities=[0.2],
                    snrs=[35.0],
                )
            ]
        )
        buffer = LatestFrameBuffer()
        source = HifDevicePointCloudSource(api_factory=lambda: api)

        source.start(buffer)
        api.callback.emit(
            SimpleNamespace(
                dataType=4,
                frameIndex=10,
                frameLen=64,
                timeStamp="2026-07-07-12:23:57.563",
                payloadData=b"payload",
            )
        )
        source.stop()
        source.join()

        frame = buffer.take_latest()
        self.assertIsNotNone(frame)
        self.assertEqual(frame.frame_index, 10)
        self.assertEqual(frame.timestamp_text, "2026-07-07-12:23:57.563")
        self.assertIsInstance(frame.timestamp_ms, int)
        self.assertGreater(frame.timestamp_ms, 0)


if __name__ == "__main__":
    unittest.main()
