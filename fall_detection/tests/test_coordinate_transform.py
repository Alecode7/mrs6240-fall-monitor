import unittest

from fall_detection.coordinate_transform import (
    GroundPlane,
    calibrate_ground_plane,
    rotate_yz_frame,
    rotate_yz_point,
    transform_frame_to_world,
    transform_point_to_world,
)
from fall_detection.datadealer_loader import CombinedFrame, Point3D


def point(x, y, z, *, kind="presence"):
    return Point3D(x_m=x, y_m=y, z_m=z, velocity_m_s=0.2, snr_db=31.0, kind=kind)


class CoordinateTransformTests(unittest.TestCase):
    def test_zero_yz_rotation_keeps_point_unchanged(self):
        source = point(0.2, 1.0, 0.5, kind="motion")

        rotated = rotate_yz_point(source, 0.0)

        self.assertIs(rotated, source)

    def test_positive_yz_rotation_treats_forward_range_as_lower_height(self):
        rotated = rotate_yz_point(point(0.2, 1.0, 0.0), 90.0)

        self.assertAlmostEqual(rotated.x_m, 0.2)
        self.assertAlmostEqual(rotated.y_m, 0.0, places=7)
        self.assertAlmostEqual(rotated.z_m, -1.0, places=7)

    def test_yz_rotation_frame_preserves_metadata_and_point_kinds(self):
        source = CombinedFrame(
            frame_index=7,
            timestamp_ms=350,
            timestamp_text="t350",
            presence_points=(point(0.0, 1.0, 0.0, kind="presence"),),
            motion_points=(point(0.1, 0.0, 1.0, kind="motion"),),
        )

        rotated = rotate_yz_frame(source, 90.0)

        self.assertEqual(rotated.frame_index, 7)
        self.assertEqual(rotated.timestamp_ms, 350)
        self.assertEqual(rotated.timestamp_text, "t350")
        self.assertEqual(rotated.presence_points[0].kind, "presence")
        self.assertEqual(rotated.motion_points[0].kind, "motion")
        self.assertAlmostEqual(rotated.presence_points[0].z_m, -1.0, places=7)
        self.assertAlmostEqual(rotated.motion_points[0].y_m, 1.0, places=7)

    def test_mount_height_moves_rotated_floor_to_world_zero(self):
        source = point(0.0, 1.0, 0.0)

        transformed = transform_point_to_world(
            source,
            yz_rotation_deg=90.0,
            mount_height_m=1.0,
        )

        self.assertAlmostEqual(transformed.y_m, 0.0, places=7)
        self.assertAlmostEqual(transformed.z_m, 0.0, places=7)

    def test_ground_plane_correction_flattens_floor_and_preserves_height_above_it(self):
        plane = GroundPlane(
            x_slope=0.10,
            y_slope=-0.05,
            intercept_m=0.20,
            point_count=120,
            rmse_m=0.01,
        )
        floor_z = plane.height_at(0.4, 1.2)

        floor = transform_point_to_world(
            point(0.4, 1.2, floor_z),
            ground_plane=plane,
        )
        body = transform_point_to_world(
            point(0.4, 1.2, floor_z + 1.35),
            ground_plane=plane,
        )

        self.assertAlmostEqual(floor.z_m, 0.0, places=7)
        self.assertAlmostEqual(body.z_m, 1.35, places=7)

    def test_ground_plane_calibration_rejects_elevated_outliers(self):
        frames = []
        for frame_index in range(4):
            floor_points = []
            for x_index in range(-4, 5):
                for y_index in range(1, 7):
                    x = x_index * 0.15
                    y = y_index * 0.25
                    z = 0.08 * x - 0.04 * y + 0.12
                    floor_points.append(point(x, y, z))
            floor_points.extend(
                [
                    point(-0.2, 0.8, 0.9),
                    point(0.0, 1.0, 1.2),
                    point(0.3, 1.3, 1.5),
                ]
            )
            frames.append(
                CombinedFrame(
                    frame_index=frame_index,
                    timestamp_ms=frame_index * 50,
                    timestamp_text=f"t{frame_index}",
                    presence_points=tuple(floor_points),
                    motion_points=(),
                )
            )

        plane = calibrate_ground_plane(
            frames,
            frame_count=4,
            ground_band_m=0.35,
            min_points=80,
        )

        self.assertAlmostEqual(plane.x_slope, 0.08, delta=0.01)
        self.assertAlmostEqual(plane.y_slope, -0.04, delta=0.01)
        self.assertAlmostEqual(plane.intercept_m, 0.12, delta=0.01)
        self.assertGreaterEqual(plane.point_count, 80)
        self.assertLess(plane.rmse_m, 0.02)

    def test_world_transform_frame_preserves_metadata_and_point_kinds(self):
        source = CombinedFrame(
            frame_index=9,
            timestamp_ms=450,
            timestamp_text="t450",
            presence_points=(point(0.0, 1.0, 0.0, kind="presence"),),
            motion_points=(point(0.1, 1.0, 0.0, kind="motion"),),
        )

        transformed = transform_frame_to_world(
            source,
            yz_rotation_deg=90.0,
            mount_height_m=1.0,
        )

        self.assertEqual(transformed.frame_index, 9)
        self.assertEqual(transformed.timestamp_ms, 450)
        self.assertEqual(transformed.timestamp_text, "t450")
        self.assertEqual(transformed.presence_points[0].kind, "presence")
        self.assertEqual(transformed.motion_points[0].kind, "motion")
        self.assertAlmostEqual(transformed.presence_points[0].z_m, 0.0, places=7)


if __name__ == "__main__":
    unittest.main()
