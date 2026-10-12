import unittest

from fall_detection.datadealer_loader import CombinedFrame, Point3D
from fall_detection.tracking import TrackerConfig
from fall_detection.visualization_model import (
    Camera3D,
    PlotBounds,
    VisualizationRuntime,
    build_visualization_frames,
    format_debug_candidate_label,
    project_iso_3d,
    project_rotated_3d,
    project_xy,
)


def point(x, y, z, velocity=0.0, snr=35.0, kind="presence"):
    return Point3D(x_m=x, y_m=y, z_m=z, velocity_m_s=velocity, snr_db=snr, kind=kind)


def frame(index, t_ms, presence_points, motion_points=()):
    return CombinedFrame(
        frame_index=index,
        timestamp_ms=t_ms,
        timestamp_text=f"t{index}",
        presence_points=tuple(presence_points),
        motion_points=tuple(motion_points),
    )


class VisualizationModelTests(unittest.TestCase):
    def test_build_visualization_frames_marks_motion_person_cluster_as_target(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 0.4, 0.05),
                    point(0.1, 0.4, 0.05),
                    point(-0.1, 0.4, 0.06),
                    point(0.2, 1.2, 1.4),
                    point(0.1, 1.2, 1.2),
                    point(0.3, 1.2, 1.6),
                ],
                [point(0.2, 1.2, 1.3, velocity=0.6, kind="motion")],
            )
        ]

        views = build_visualization_frames(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, z_max_m=2.2))

        self.assertEqual(len(views), 1)
        self.assertEqual(len(views[0].clusters), 2)
        self.assertIsNotNone(views[0].target_cluster)
        self.assertGreater(views[0].target_cluster.center_z_m, 1.0)
        self.assertEqual(views[0].target_cluster.motion_count, 1)
        self.assertTrue(views[0].target_cluster.is_target)
        self.assertEqual(sum(1 for cluster in views[0].clusters if cluster.is_target), 1)

    def test_visualization_runtime_matches_batch_tracking_for_incremental_frames(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(1.0, 2.2, 1.2),
                    point(1.1, 2.2, 1.1),
                    point(0.9, 2.2, 1.3),
                ],
                [point(1.0, 2.2, 1.2, velocity=0.9, kind="motion")],
            ),
        ]
        config = TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, track_gate_m=0.5, hold_ms=3000)

        runtime = VisualizationRuntime(config)
        incremental_views = [runtime.step(item) for item in frames]
        batch_views = build_visualization_frames(frames, config)

        self.assertEqual([view.target.state for view in incremental_views], [view.target.state for view in batch_views])
        self.assertEqual([view.target.held for view in incremental_views], [view.target.held for view in batch_views])
        self.assertAlmostEqual(incremental_views[1].target.center_x_m, batch_views[1].target.center_x_m)
        self.assertAlmostEqual(incremental_views[1].target.center_y_m, batch_views[1].target.center_y_m)

    def test_project_xy_uses_x_as_horizontal_and_y_as_forward_distance(self):
        bounds = PlotBounds(x_min_m=-1.5, x_max_m=1.5, y_min_m=0.0, y_max_m=3.0, z_min_m=0.0, z_max_m=2.0)

        center_x, center_y = project_xy(point(0.0, 1.5, 1.0), bounds, width_px=300, height_px=300)
        left_x, _ = project_xy(point(1.0, 1.5, 1.0), bounds, width_px=300, height_px=300)
        _, forward_y = project_xy(point(0.0, 3.0, 1.0), bounds, width_px=300, height_px=300)

        self.assertAlmostEqual(center_x, 150.0)
        self.assertAlmostEqual(center_y, 150.0)
        self.assertGreater(left_x, center_x)
        self.assertLess(forward_y, center_y)

    def test_project_iso_3d_uses_height_to_lift_point_up(self):
        bounds = PlotBounds(x_min_m=-1.5, x_max_m=1.5, y_min_m=0.0, y_max_m=3.0, z_min_m=0.0, z_max_m=2.0)

        _, low_y = project_iso_3d(point(0.0, 1.5, 0.0), bounds, width_px=300, height_px=300)
        _, high_y = project_iso_3d(point(0.0, 1.5, 2.0), bounds, width_px=300, height_px=300)

        self.assertLess(high_y, low_y)

    def test_target_cluster_is_not_allowed_to_exceed_half_meter_xy_footprint(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(-0.8, 0.7, 1.0),
                    point(0.0, 1.0, 1.2),
                    point(0.8, 1.6, 1.1),
                    point(0.1, 1.1, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            )
        ]

        view = build_visualization_frames(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=2.0))[0]

        self.assertIsNotNone(view.target_cluster)
        self.assertLessEqual(view.target_cluster.x_max_m - view.target_cluster.x_min_m, 0.5)
        self.assertLessEqual(view.target_cluster.y_max_m - view.target_cluster.y_min_m, 0.5)

    def test_held_target_still_exposes_display_box(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(1.0, 2.2, 1.2),
                    point(1.1, 2.2, 1.1),
                    point(0.9, 2.2, 1.3),
                ],
                [point(1.0, 2.2, 1.2, velocity=0.9, kind="motion")],
            ),
        ]

        views = build_visualization_frames(
            frames,
            TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, track_gate_m=0.5, hold_ms=3000),
        )

        self.assertTrue(views[1].target.held)
        self.assertIsNotNone(views[1].target_cluster)
        self.assertTrue(views[1].target_cluster.is_target)
        self.assertEqual(views[1].target_cluster.point_count, 0)
        self.assertAlmostEqual(views[1].target_cluster.center_x_m, 0.0)
        self.assertAlmostEqual(views[1].target_cluster.center_y_m, 1.0)

    def test_far_cluster_is_not_marked_as_target_during_hold_window(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            )
        ]
        for i in range(1, 31):
            frames.append(
                frame(
                    i,
                    1000 + i * 100,
                    [
                        point(1.2, 2.2, 1.2),
                        point(1.3, 2.2, 1.1),
                        point(1.1, 2.2, 1.3),
                    ],
                    [point(1.2, 2.2, 1.2, velocity=0.9, kind="motion")],
                )
            )

        views = build_visualization_frames(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
                switch_confirm_frames=5,
            ),
        )

        for view in views[1:31]:
            self.assertTrue(view.target.has_target)
            self.assertTrue(view.target.held)
            self.assertIsNotNone(view.target_cluster)
            self.assertAlmostEqual(view.target_cluster.center_x_m, 0.0)
            self.assertAlmostEqual(view.target_cluster.center_y_m, 1.0)

    def test_target_box_does_not_drift_to_static_cluster_without_locked_core_overlap(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(-0.30, 1.70, 1.2),
                    point(-0.20, 1.70, 1.1),
                    point(-0.25, 1.80, 1.3),
                ],
                [point(-0.25, 1.70, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(0.15, 1.65, 1.2),
                    point(0.25, 1.65, 1.1),
                    point(0.20, 1.75, 1.3),
                ],
            ),
            frame(
                2,
                1200,
                [
                    point(0.60, 1.45, 1.2),
                    point(0.70, 1.45, 1.1),
                    point(0.65, 1.55, 1.3),
                ],
            ),
        ]

        views = build_visualization_frames(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        for view in views[1:]:
            self.assertTrue(view.target.held)
            self.assertIsNotNone(view.target_cluster)
            self.assertAlmostEqual(view.target_cluster.center_x_m, -0.25)
            self.assertAlmostEqual(view.target_cluster.center_y_m, 1.70)

    def test_static_core_points_keep_target_box_xy_while_refreshing_height(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(-0.30, 1.70, 1.2),
                    point(-0.20, 1.70, 1.1),
                    point(-0.25, 1.80, 1.3),
                ],
                [point(-0.25, 1.70, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(-0.05, 1.70, 0.70),
                    point(-0.02, 1.72, 0.65),
                    point(-0.08, 1.68, 0.75),
                ],
            ),
        ]

        views = build_visualization_frames(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertTrue(views[1].target.held)
        self.assertAlmostEqual(views[1].target.center_x_m, -0.25)
        self.assertAlmostEqual(views[1].target.center_y_m, 1.70)
        self.assertAlmostEqual(views[1].target.center_z_m, 0.70)
        self.assertIsNotNone(views[1].target_cluster)
        self.assertAlmostEqual(views[1].target_cluster.center_x_m, -0.25)
        self.assertAlmostEqual(views[1].target_cluster.center_y_m, 1.70)
        self.assertEqual(views[1].target_cluster.point_count, 3)
        self.assertEqual(views[1].target_cluster.motion_count, 0)

    def test_motion_core_subset_still_exposes_target_cluster(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(-0.10, 1.00, 1.2),
                    point(0.00, 1.00, 1.1),
                    point(0.10, 1.00, 1.3),
                ],
                [point(0.00, 1.00, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(-0.10, 1.00, 1.2),
                    point(0.00, 1.00, 1.1),
                    point(0.10, 1.00, 1.3),
                    point(0.28, 1.00, 1.2),
                ],
                [point(0.00, 1.00, 1.2, velocity=0.5, kind="motion")],
            ),
        ]

        views = build_visualization_frames(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertTrue(views[1].target.has_target)
        self.assertFalse(views[1].target.held)
        self.assertIsNotNone(views[1].target_cluster)
        self.assertTrue(views[1].target_cluster.is_target)
        self.assertEqual(views[1].target_cluster.motion_count, 1)

    def test_debug_candidates_expose_previous_center_gate_score_and_selection(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                1100,
                [
                    point(0.02, 1.02, 1.2),
                    point(0.12, 1.02, 1.1),
                    point(-0.08, 1.02, 1.3),
                    point(0.36, 1.02, 1.2),
                    point(0.46, 1.02, 1.1),
                    point(0.40, 1.02, 1.3),
                ],
                [point(0.02, 1.02, 1.2, velocity=0.5, kind="motion")],
            ),
        ]

        views = build_visualization_frames(
            frames,
            TrackerConfig(min_cluster_points=2, cluster_radius_m=0.18, track_gate_m=0.5, hold_ms=3000),
        )

        view = views[1]
        self.assertAlmostEqual(view.previous_target_x_m, 0.0)
        self.assertAlmostEqual(view.previous_target_y_m, 1.0)
        self.assertAlmostEqual(view.track_gate_m, 0.5)
        self.assertGreaterEqual(len(view.debug_candidates), 2)
        self.assertTrue(any(candidate.selected for candidate in view.debug_candidates))
        self.assertTrue(any(candidate.score is not None for candidate in view.debug_candidates))
        self.assertTrue(any(candidate.within_gate for candidate in view.debug_candidates))
        label = format_debug_candidate_label(view.debug_candidates[0])
        self.assertIn("点", label)
        self.assertIn("动", label)
        self.assertIn("分", label)

    def test_far_cluster_is_not_marked_as_target_before_switch_confirmation(self):
        frames = [
            frame(
                0,
                1000,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            )
        ]
        for i in range(1, 6):
            frames.append(
                frame(
                    i,
                    1000 + i * 100,
                    [
                        point(1.2, 2.2, 1.2),
                        point(1.3, 2.2, 1.1),
                        point(1.1, 2.2, 1.3),
                    ],
                    [point(1.2, 2.2, 1.2, velocity=0.9, kind="motion")],
                )
            )

        views = build_visualization_frames(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=0,
                switch_confirm_frames=5,
            ),
        )

        early_target = views[3].target
        early_target_is_far = early_target.has_target and early_target.center_x_m is not None and early_target.center_x_m > 1.0
        self.assertFalse(early_target_is_far)
        if views[3].target_cluster is not None:
            self.assertLess(views[3].target_cluster.center_x_m, 1.0)
        self.assertAlmostEqual(views[5].target.center_x_m, 1.2)
        self.assertAlmostEqual(views[5].target_cluster.center_x_m, 1.2)

    def test_project_rotated_3d_uses_camera_angle_and_height_axis(self):
        bounds = PlotBounds(x_min_m=-1.5, x_max_m=1.5, y_min_m=0.0, y_max_m=3.0, z_min_m=0.0, z_max_m=2.0)
        low = point(0.0, 1.5, 0.0)
        high = point(0.0, 1.5, 2.0)
        side = point(1.0, 1.5, 1.0)

        _, low_y = project_rotated_3d(low, bounds, Camera3D(azimuth_deg=45, elevation_deg=25), width_px=300, height_px=300)
        _, high_y = project_rotated_3d(high, bounds, Camera3D(azimuth_deg=45, elevation_deg=25), width_px=300, height_px=300)
        x_a, _ = project_rotated_3d(side, bounds, Camera3D(azimuth_deg=0, elevation_deg=25), width_px=300, height_px=300)
        x_b, _ = project_rotated_3d(side, bounds, Camera3D(azimuth_deg=90, elevation_deg=25), width_px=300, height_px=300)

        self.assertLess(high_y, low_y)
        self.assertNotAlmostEqual(x_a, x_b)


if __name__ == "__main__":
    unittest.main()
