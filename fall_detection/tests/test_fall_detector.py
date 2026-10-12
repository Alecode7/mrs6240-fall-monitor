import unittest
from pathlib import Path

from fall_detection.coordinate_transform import transform_frame_to_world
from fall_detection.datadealer_loader import CombinedFrame, Point3D, load_datadealer_recording
from fall_detection.rule_detector import FallRuleConfig, FallState, IncrementalFallDetector, detect_falls
from fall_detection.tracking import TargetFrame, TargetState, TrackerConfig, track_recording
from fall_detection.visualization_model import VisualizationRuntime


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


def firmware_capped_tracking_frame(source_frame, config, *, max_points=192):
    world_frame = transform_frame_to_world(
        source_frame,
        yz_rotation_deg=30.0,
        mount_height_m=1.63,
    )
    ordered_points = world_frame.motion_points + world_frame.presence_points
    accepted_points = tuple(
        point
        for point in ordered_points
        if config.x_min_m <= point.x_m <= config.x_max_m
        and config.y_min_m <= point.y_m <= config.y_max_m
        and config.z_min_m <= point.z_m <= config.z_max_m
        and point.snr_db >= config.min_snr_db
    )
    capped_points = accepted_points[:max_points]
    return (
        CombinedFrame(
            frame_index=world_frame.frame_index,
            timestamp_ms=world_frame.timestamp_ms,
            timestamp_text=world_frame.timestamp_text,
            presence_points=capped_points,
            motion_points=(),
        ),
        len(accepted_points) - len(capped_points),
    )


def target_frame(
    index,
    t_ms,
    z,
    *,
    x=0.0,
    y=1.0,
    motion_count=0,
    velocity=0.0,
    point_count=20,
    held=False,
    state=TargetState.TRACKING,
):
    return TargetFrame(
        frame_index=index,
        timestamp_ms=t_ms,
        timestamp_text=f"t{index}",
        has_target=True,
        center_x_m=x,
        center_y_m=y,
        center_z_m=z,
        z_min_m=z - 0.1,
        z_max_m=z + 0.1,
        height_m=0.2,
        width_x_m=0.3,
        depth_y_m=0.3,
        point_count=point_count,
        motion_count=motion_count,
        max_abs_velocity_m_s=velocity,
        held=held,
        state=state,
    )


def cluster_points(center_x, center_y, center_z, count, *, kind="presence", velocity=0.0):
    offsets = [(-0.08, -0.08), (0.0, -0.08), (0.08, -0.08), (-0.08, 0.0), (0.0, 0.0), (0.08, 0.0), (-0.08, 0.08), (0.0, 0.08), (0.08, 0.08)]
    points = []
    for index in range(count):
        dx, dy = offsets[index % len(offsets)]
        z = center_z + ((index % 3) - 1) * 0.04
        points.append(point(center_x + dx, center_y + dy, z, velocity=velocity, kind=kind))
    return points


def vertical_body_points(center_x, center_y, center_z, count, *, kind="presence", velocity=0.0):
    offsets = [(-0.08, -0.08), (0.0, -0.08), (0.08, -0.08), (-0.08, 0.0), (0.0, 0.0), (0.08, 0.0), (-0.08, 0.08), (0.0, 0.08), (0.08, 0.08)]
    z_offsets = [-0.24, -0.12, 0.0, 0.12, 0.24]
    points = []
    for index in range(count):
        dx, dy = offsets[index % len(offsets)]
        z = center_z + z_offsets[index % len(z_offsets)]
        points.append(point(center_x + dx, center_y + dy, z, velocity=velocity, kind=kind))
    return points


def _frame_in_windows(frame_index, windows):
    return any(start <= frame_index <= end for start, end in windows)


class FallDetectorTests(unittest.TestCase):
    def test_fall_rule_defaults_require_larger_drop_and_longer_low_posture(self):
        config = FallRuleConfig()

        self.assertEqual(config.drop_window_ms, 3500)
        self.assertAlmostEqual(config.min_drop_height_m, 0.80)
        self.assertEqual(config.confirm_window_ms, 7000)
        self.assertAlmostEqual(config.motion_velocity_threshold_m_s, 0.45)
        self.assertEqual(config.motion_point_threshold, 20)
        self.assertAlmostEqual(config.low_height_m, 0.55)
        self.assertAlmostEqual(config.terminal_height_m, 0.40)
        self.assertEqual(config.suspect_min_fresh_samples, 3)
        self.assertEqual(config.confirm_min_fresh_samples, 6)
        self.assertEqual(config.fresh_min_point_count, 3)
        self.assertEqual(config.suspect_low_duration_ms, 500)
        self.assertAlmostEqual(config.suspect_low_ratio, 0.75)
        self.assertEqual(config.low_still_duration_ms, 1200)
        self.assertAlmostEqual(config.confirm_low_ratio, 0.70)
        self.assertAlmostEqual(config.confirm_min_horizontal_displacement_m, 0.0)
        self.assertAlmostEqual(config.rearm_height_m, 0.85)
        self.assertEqual(config.rearm_duration_ms, 1000)

    def test_default_rules_detect_1_67m_60_degree_falls_without_squat_alerts(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=10.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=1.0,
            switch_confirm_frames=3,
            hold_ms=5000,
        )
        cases = (
            (
                "1.67-60",
                ((570, 700), (2020, 2200)),
                ((1350, 1500), (2730, 2900)),
            ),
            (
                "1.67-60-2",
                ((1030, 1160), (1850, 2050)),
                (),
            ),
        )

        for stem, fall_windows, squat_windows in cases:
            with self.subTest(stem=stem):
                recording = load_datadealer_recording(data_root / stem, stem=stem)
                runtime = VisualizationRuntime(tracker_config)
                detector = IncrementalFallDetector(FallRuleConfig())
                events = []

                for source_frame in recording.frames:
                    world_frame = transform_frame_to_world(
                        source_frame,
                        yz_rotation_deg=60.0,
                        mount_height_m=1.67,
                    )
                    view = runtime.step(world_frame)
                    events.extend(detector.step(view.target))

                confirmed_falls = [event for event in events if event.state == FallState.CONFIRMED]
                squat_alerts = [
                    event
                    for event in events
                    if event.state in (FallState.SUSPECTED, FallState.CONFIRMED)
                    and _frame_in_windows(event.frame_index, squat_windows)
                ]

                self.assertEqual(2, len(confirmed_falls), confirmed_falls)
                self.assertEqual([], squat_alerts)
                for window in fall_windows:
                    self.assertTrue(
                        any(_frame_in_windows(event.frame_index, (window,)) for event in confirmed_falls),
                        f"no confirmed fall in window {window}: {confirmed_falls}",
                    )

    def test_default_rules_detect_163_30_falls_without_crouch_alerts(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=10.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=1.0,
            switch_confirm_frames=3,
            hold_ms=5000,
        )
        recording = load_datadealer_recording(data_root / "163-30", stem="163-30")
        runtime = VisualizationRuntime(tracker_config)
        detector = IncrementalFallDetector(FallRuleConfig())
        events = []

        for source_frame in recording.frames:
            world_frame = transform_frame_to_world(
                source_frame,
                yz_rotation_deg=30.0,
                mount_height_m=1.63,
            )
            events.extend(detector.step(runtime.step(world_frame).target))

        fall_windows = ((740, 850), (1920, 2040), (2440, 2530), (3090, 3190), (3500, 3590))
        crouch_windows = ((500, 650), (1250, 1400), (1440, 1580), (1680, 1800))
        public_alerts = [event for event in events if event.state in (FallState.SUSPECTED, FallState.CONFIRMED)]
        confirmed_falls = [event for event in events if event.state == FallState.CONFIRMED]

        self.assertFalse(
            any(_frame_in_windows(event.frame_index, crouch_windows) for event in public_alerts),
            public_alerts,
        )
        self.assertEqual(len(confirmed_falls), len(fall_windows), confirmed_falls)
        for window in fall_windows:
            self.assertTrue(
                any(_frame_in_windows(event.frame_index, (window,)) for event in confirmed_falls),
                f"no confirmed fall in window {window}: {confirmed_falls}",
            )

    def test_163_30_7_15_deployment_profile_rejects_nonfall_terminal_states(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=15.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=0.5,
            switch_confirm_frames=5,
            hold_ms=3000,
        )
        rule_config = FallRuleConfig(
            suspect_low_duration_ms=700,
            suspect_low_ratio=0.92,
            confirm_min_horizontal_displacement_m=0.60,
        )
        recording = load_datadealer_recording(
            data_root / "163-30-7-15",
            stem="163-30-7-15",
        )
        runtime = VisualizationRuntime(tracker_config)
        detector = IncrementalFallDetector(rule_config)
        events = []
        overflow_frames = 0

        for source_frame in recording.frames:
            tracking_frame, overflow_count = firmware_capped_tracking_frame(
                source_frame,
                tracker_config,
            )
            overflow_frames += overflow_count > 0
            events.extend(detector.step(runtime.step(tracking_frame).target))

        fall_windows = ((620, 750), (760, 900), (920, 1050), (1080, 1240))
        negative_windows = (
            (80, 300),
            (450, 560),
            (1240, 1380),
            (1430, 1560),
            (1580, 1740),
            (1760, 1900),
            (1910, 2100),
        )
        public_alerts = [
            event
            for event in events
            if event.state in (FallState.SUSPECTED, FallState.CONFIRMED)
        ]
        confirmed_falls = [event for event in events if event.state == FallState.CONFIRMED]

        self.assertGreater(overflow_frames, 0)
        self.assertFalse(
            any(_frame_in_windows(event.frame_index, negative_windows) for event in public_alerts),
            public_alerts,
        )
        self.assertEqual(len(confirmed_falls), len(fall_windows), confirmed_falls)
        for window in fall_windows:
            self.assertTrue(
                any(_frame_in_windows(event.frame_index, (window,)) for event in confirmed_falls),
                f"no confirmed fall in window {window}: {confirmed_falls}",
            )

    def test_163_30_7_15_2_deployment_profile_keeps_object_crouches_unconfirmed(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=15.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=0.5,
            switch_confirm_frames=5,
            hold_ms=3000,
        )
        rule_config = FallRuleConfig(
            suspect_low_duration_ms=700,
            suspect_low_ratio=0.92,
            confirm_min_horizontal_displacement_m=0.60,
        )
        recording = load_datadealer_recording(
            data_root / "163-30-7-15-2",
            stem="163-30-7-15-2",
        )
        runtime = VisualizationRuntime(tracker_config)
        detector = IncrementalFallDetector(rule_config)
        events = []

        for source_frame in recording.frames:
            tracking_frame, _overflow_count = firmware_capped_tracking_frame(
                source_frame,
                tracker_config,
            )
            events.extend(detector.step(runtime.step(tracking_frame).target))

        suspected = [event for event in events if event.state == FallState.SUSPECTED]
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]

        self.assertGreaterEqual(len(suspected), 1, events)
        self.assertEqual([], confirmed)

    def test_default_rules_detect_1_63_30_2_falls_without_negative_action_alerts(self):
        data_root = Path(__file__).resolve().parents[2] / "fall_capture_station" / "data"
        tracker_config = TrackerConfig(
            z_min_m=0.0,
            z_max_m=2.2,
            min_snr_db=10.0,
            cluster_radius_m=0.45,
            min_cluster_points=3,
            max_cluster_width_x_m=0.5,
            max_cluster_depth_y_m=0.5,
            track_gate_m=1.0,
            switch_confirm_frames=3,
            hold_ms=5000,
        )
        recording = load_datadealer_recording(data_root / "1.63-30-2", stem="1.63-30-2")
        runtime = VisualizationRuntime(tracker_config)
        detector = IncrementalFallDetector(FallRuleConfig())
        events = []

        for source_frame in recording.frames:
            world_frame = transform_frame_to_world(
                source_frame,
                yz_rotation_deg=30.0,
                mount_height_m=1.63,
            )
            events.extend(detector.step(runtime.step(world_frame).target))

        fall_windows = ((460, 550), (840, 920), (1290, 1400))
        negative_windows = (
            (80, 150),
            (270, 340),
            (1690, 1800),
            (1930, 2110),
            (2260, 2360),
            (2540, 2610),
            (2800, 3010),
        )
        public_alerts = [event for event in events if event.state in (FallState.SUSPECTED, FallState.CONFIRMED)]
        confirmed_falls = [event for event in events if event.state == FallState.CONFIRMED]

        self.assertFalse(
            any(_frame_in_windows(event.frame_index, negative_windows) for event in public_alerts),
            public_alerts,
        )
        self.assertEqual(len(confirmed_falls), len(fall_windows), confirmed_falls)
        for window in fall_windows:
            self.assertTrue(
                any(_frame_in_windows(event.frame_index, (window,)) for event in confirmed_falls),
                f"no confirmed fall in window {window}: {confirmed_falls}",
            )

    def test_tracker_default_requires_five_frames_before_switching_to_far_target(self):
        self.assertEqual(TrackerConfig().switch_confirm_frames, 5)

    def test_tracker_uses_z_as_height_and_y_as_forward_distance(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(0.2, 1.0, 1.4),
                    point(0.1, 1.1, 1.2),
                    point(0.3, 1.0, 1.6),
                    point(1.8, 2.5, 0.1),
                ],
            )
        ]

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.45, z_max_m=2.2))

        self.assertTrue(track[0].has_target)
        self.assertAlmostEqual(track[0].center_x_m, 0.2)
        self.assertAlmostEqual(track[0].center_y_m, 1.0)
        self.assertAlmostEqual(track[0].center_z_m, 1.4)
        self.assertAlmostEqual(track[0].height_m, 0.4)

    def test_tracker_switches_from_static_background_to_motion_person(self):
        frames = [
            frame(
                0,
                0,
                [point(0.0, 0.4, 0.05), point(0.1, 0.4, 0.05), point(-0.1, 0.4, 0.06)],
            ),
            frame(
                1,
                100,
                [
                    point(0.0, 0.4, 0.05),
                    point(0.1, 0.4, 0.05),
                    point(-0.1, 0.4, 0.06),
                    point(0.2, 1.2, 1.4),
                    point(0.1, 1.2, 1.2),
                    point(0.3, 1.2, 1.6),
                ],
                [point(0.2, 1.2, 1.3, velocity=0.6, kind="motion")],
            ),
        ]

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.5, z_max_m=2.2))

        self.assertLess(track[0].center_z_m, 0.2)
        self.assertGreater(track[1].center_z_m, 1.0)
        self.assertEqual(track[1].motion_count, 1)

    def test_low_static_lock_is_not_stolen_by_far_low_motion_points(self):
        frames = [
            frame(
                0,
                0,
                cluster_points(-0.52, 1.22, -0.47, 9),
            ),
            frame(
                1,
                100,
                cluster_points(-0.52, 1.22, -0.47, 9)
                + cluster_points(-0.25, 2.99, -0.56, 3, kind="motion", velocity=0.5),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertAlmostEqual(track[1].center_x_m, -0.52, delta=0.02)
        self.assertAlmostEqual(track[1].center_y_m, 1.22, delta=0.02)
        self.assertNotAlmostEqual(track[1].center_y_m, 2.99, delta=0.5)
        self.assertEqual(track[1].state, TargetState.STATIC_LOCK)

    def test_tracker_does_not_merge_human_candidate_beyond_half_meter_xy_footprint(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(-0.8, 0.7, 1.0),
                    point(0.0, 1.0, 1.2),
                    point(0.8, 1.6, 1.1),
                    point(0.1, 1.1, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            )
        ]

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=2.0))

        self.assertTrue(track[0].has_target)
        self.assertLessEqual(track[0].width_x_m, 0.5)
        self.assertLessEqual(track[0].depth_y_m, 0.5)

    def test_tracker_keeps_locked_person_when_far_motion_appears(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                100,
                [
                    point(0.02, 1.02, 1.2),
                    point(0.12, 1.02, 1.1),
                    point(-0.08, 1.02, 1.3),
                    point(1.0, 2.2, 1.2),
                    point(1.1, 2.2, 1.1),
                    point(0.9, 2.2, 1.3),
                ],
                [point(1.0, 2.2, 1.2, velocity=0.9, kind="motion")],
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertAlmostEqual(track[1].center_x_m, 0.0)
        self.assertAlmostEqual(track[1].center_y_m, 1.0)
        self.assertLess(track[1].center_y_m, 1.5)
        self.assertTrue(track[1].held)

    def test_tracker_holds_last_position_when_person_stops_and_far_motion_appears(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                100,
                [
                    point(1.0, 2.2, 1.2),
                    point(1.1, 2.2, 1.1),
                    point(0.9, 2.2, 1.3),
                ],
                [point(1.0, 2.2, 1.2, velocity=0.9, kind="motion")],
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertAlmostEqual(track[1].center_x_m, 0.0)
        self.assertAlmostEqual(track[1].center_y_m, 1.0)
        self.assertTrue(track[1].held)
        self.assertEqual(track[1].state, TargetState.HOLD)

    def test_tracker_does_not_confirm_far_target_during_hold_window(self):
        frames = [
            frame(
                0,
                0,
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
                    i * 100,
                    [
                        point(1.2, 2.2, 1.2),
                        point(1.3, 2.2, 1.1),
                        point(1.1, 2.2, 1.3),
                    ],
                    [point(1.2, 2.2, 1.2, velocity=0.9, kind="motion")],
                )
            )

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
                switch_confirm_frames=5,
            ),
        )

        for target in track[1:31]:
            self.assertTrue(target.has_target)
            self.assertAlmostEqual(target.center_x_m, 0.0)
            self.assertAlmostEqual(target.center_y_m, 1.0)
            self.assertTrue(target.held)

    def test_tracker_does_not_drift_to_static_cluster_without_locked_core_overlap(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(-0.30, 1.70, 1.2),
                    point(-0.20, 1.70, 1.1),
                    point(-0.25, 1.80, 1.3),
                ],
                [point(-0.25, 1.70, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                100,
                [
                    point(0.15, 1.65, 1.2),
                    point(0.25, 1.65, 1.1),
                    point(0.20, 1.75, 1.3),
                ],
            ),
            frame(
                2,
                200,
                [
                    point(0.60, 1.45, 1.2),
                    point(0.70, 1.45, 1.1),
                    point(0.65, 1.55, 1.3),
                ],
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertAlmostEqual(track[0].center_x_m, -0.25)
        for target in track[1:]:
            self.assertTrue(target.has_target)
            self.assertAlmostEqual(target.center_x_m, -0.25)
            self.assertAlmostEqual(target.center_y_m, 1.70)
            self.assertTrue(target.held)

    def test_tracker_static_core_points_update_height_but_do_not_move_xy_center(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(-0.30, 1.70, 1.2),
                    point(-0.20, 1.70, 1.1),
                    point(-0.25, 1.80, 1.3),
                ],
                [point(-0.25, 1.70, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                100,
                [
                    point(-0.05, 1.70, 0.70),
                    point(-0.02, 1.72, 0.65),
                    point(-0.08, 1.68, 0.75),
                ],
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertAlmostEqual(track[1].center_x_m, -0.25)
        self.assertAlmostEqual(track[1].center_y_m, 1.70)
        self.assertAlmostEqual(track[1].center_z_m, 0.70)
        self.assertEqual(track[1].point_count, 3)
        self.assertEqual(track[1].motion_count, 0)
        self.assertTrue(track[1].held)
        self.assertEqual(track[1].state, TargetState.STATIC_LOCK)

    def test_tracker_prefers_locked_position_over_nearby_motion_noise(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(0.0, 1.0, 1.2),
                    point(0.1, 1.0, 1.1),
                    point(-0.1, 1.0, 1.3),
                ],
                [point(0.0, 1.0, 1.2, velocity=0.5, kind="motion")],
            ),
            frame(
                1,
                100,
                [
                    point(0.02, 1.02, 1.2),
                    point(0.12, 1.02, 1.1),
                    point(-0.08, 1.02, 1.3),
                    point(0.42, 1.0, 1.2),
                    point(0.46, 1.0, 1.1),
                    point(0.38, 1.0, 1.3),
                ],
                [point(0.42, 1.0, 1.2, velocity=0.9, kind="motion")],
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=3000,
            ),
        )

        self.assertAlmostEqual(track[1].center_x_m, 0.0)
        self.assertAlmostEqual(track[1].center_y_m, 1.0)
        self.assertEqual(track[1].motion_count, 0)
        self.assertTrue(track[1].held)

    def test_tracker_switches_from_static_shadow_to_dominant_motion_cluster_inside_gate(self):
        frames = [
            frame(
                0,
                0,
                cluster_points(0.0, 1.45, 1.0, 12),
                cluster_points(0.0, 1.45, 1.0, 3, kind="motion", velocity=0.3),
            ),
            frame(
                1,
                100,
                cluster_points(0.0, 1.45, 1.0, 18),
                cluster_points(0.0, 1.45, 1.0, 4, kind="motion", velocity=0.2)
                + cluster_points(0.05, 1.02, 1.0, 12, kind="motion", velocity=0.7),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertFalse(track[1].held)
        self.assertGreaterEqual(track[1].motion_count, 12)
        self.assertAlmostEqual(track[1].center_y_m, 1.02, delta=0.12)

    def test_tracker_switches_from_weak_motion_lock_to_stronger_motion_cluster_inside_gate(self):
        frames = [
            frame(
                0,
                0,
                [
                    point(-0.43, 0.75, 0.25),
                    point(-0.35, 0.75, 0.45),
                    point(-0.27, 0.75, 0.65),
                    point(-0.43, 0.83, 0.25),
                    point(-0.35, 0.83, 0.45),
                    point(-0.27, 0.83, 0.65),
                    point(-0.43, 0.91, 0.25),
                    point(-0.35, 0.91, 0.45),
                    point(-0.27, 0.91, 0.65),
                    point(-0.35, 0.83, 0.45),
                ],
            ),
            frame(
                1,
                100,
                cluster_points(-0.34, 0.89, 0.45, 11)
                + cluster_points(-0.24, 0.41, 0.42, 3),
                cluster_points(-0.34, 0.89, 0.45, 4, kind="motion", velocity=0.2)
                + cluster_points(-0.24, 0.41, 0.42, 9, kind="motion", velocity=0.4),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertFalse(track[1].held)
        self.assertEqual(track[1].motion_count, 9)
        self.assertAlmostEqual(track[1].center_y_m, 0.41, delta=0.12)

    def test_tracker_rebases_from_small_shadow_to_large_body_cluster_inside_gate(self):
        shadow_presence = cluster_points(-0.35, 0.88, 0.10, 9) + [
            point(-0.35, 0.88, -0.08),
            point(-0.35, 0.88, 0.28),
        ]
        frames = [
            frame(
                0,
                0,
                shadow_presence,
            ),
            frame(
                1,
                100,
                shadow_presence
                + cluster_points(0.05, 0.70, 0.00, 48),
                cluster_points(-0.35, 0.88, 0.10, 3, kind="motion", velocity=0.2)
                + cluster_points(0.05, 0.70, 0.00, 14, kind="motion", velocity=0.5),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertFalse(track[1].held)
        self.assertGreaterEqual(track[1].point_count, 50)
        self.assertGreaterEqual(track[1].motion_count, 14)
        self.assertAlmostEqual(track[1].center_x_m, 0.05, delta=0.12)
        self.assertAlmostEqual(track[1].center_y_m, 0.70, delta=0.12)

    def test_tracker_uses_full_locked_body_cluster_when_motion_evidence_is_strong(self):
        frames = [
            frame(
                0,
                0,
                cluster_points(-0.12, 0.82, 0.20, 18),
                cluster_points(-0.12, 0.82, 0.20, 6, kind="motion", velocity=0.3),
            ),
            frame(
                1,
                100,
                cluster_points(0.08, 0.66, 0.10, 48),
                cluster_points(0.08, 0.66, 0.10, 14, kind="motion", velocity=0.5),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertFalse(track[1].held)
        self.assertGreaterEqual(track[1].point_count, 60)
        self.assertGreaterEqual(track[1].motion_count, 14)
        self.assertAlmostEqual(track[1].center_x_m, 0.08, delta=0.12)
        self.assertAlmostEqual(track[1].center_y_m, 0.66, delta=0.12)

    def test_tracker_does_not_switch_from_large_static_body_to_tiny_motion_cluster(self):
        frames = [
            frame(
                0,
                0,
                cluster_points(0.0, 1.20, 1.0, 105),
                cluster_points(0.0, 1.20, 1.0, 5, kind="motion", velocity=0.2),
            ),
            frame(
                1,
                100,
                cluster_points(0.0, 1.20, 1.0, 101),
                cluster_points(0.0, 1.20, 1.0, 1, kind="motion", velocity=0.2)
                + cluster_points(0.0, 0.90, 0.15, 4, kind="motion", velocity=0.8),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                track_gate_m=0.5,
                hold_ms=3000,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertLess(track[1].motion_count, 4)
        self.assertGreater(track[1].point_count, 80)
        self.assertAlmostEqual(track[1].center_y_m, 1.20, delta=0.12)

    def test_tracker_prefers_large_locked_static_body_over_small_near_motion_noise(self):
        frames = [
            frame(
                0,
                0,
                vertical_body_points(-0.15, 1.29, -0.16, 54),
                vertical_body_points(-0.15, 1.29, -0.16, 5, kind="motion", velocity=0.2),
            ),
            frame(
                1,
                50,
                vertical_body_points(-0.15, 1.29, -0.16, 51)
                + cluster_points(-0.49, 1.37, -0.22, 5),
                cluster_points(-0.49, 1.37, -0.22, 3, kind="motion", velocity=0.5),
            ),
        ]

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=3,
                cluster_radius_m=0.18,
                max_cluster_width_x_m=0.5,
                max_cluster_depth_y_m=0.5,
                track_gate_m=1.0,
                hold_ms=5000,
                switch_confirm_frames=3,
                z_max_m=2.2,
            ),
        )

        self.assertTrue(track[1].has_target)
        self.assertGreater(track[1].point_count, 40)
        self.assertEqual(track[1].motion_count, 0)
        self.assertAlmostEqual(track[1].center_x_m, -0.15, delta=0.12)
        self.assertAlmostEqual(track[1].center_y_m, 1.29, delta=0.12)

    def test_tracker_does_not_switch_to_far_target_until_it_is_confirmed(self):
        frames = [
            frame(
                0,
                0,
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
                    i * 100,
                    [
                        point(1.2, 2.2, 1.2),
                        point(1.3, 2.2, 1.1),
                        point(1.1, 2.2, 1.3),
                    ],
                    [point(1.2, 2.2, 1.2, velocity=0.9, kind="motion")],
                )
            )

        track = track_recording(
            frames,
            TrackerConfig(
                min_cluster_points=2,
                cluster_radius_m=0.5,
                track_gate_m=0.5,
                hold_ms=0,
                switch_confirm_frames=5,
            ),
        )

        for target in track[1:5]:
            switched_to_far_target = target.has_target and target.center_x_m is not None and target.center_x_m > 1.0
            self.assertFalse(switched_to_far_target)
            self.assertEqual(target.state, TargetState.REACQUIRE)
        self.assertTrue(track[5].has_target)
        self.assertAlmostEqual(track[5].center_x_m, 1.2)
        self.assertAlmostEqual(track[5].center_y_m, 2.2)
        self.assertFalse(track[5].held)
        self.assertEqual(track[5].state, TargetState.TRACKING)

    def test_rule_detector_confirms_fall_after_fast_height_drop_and_stillness(self):
        frames = []
        for i in range(8):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, 1.5), point(0.1, 1.0, 1.3), point(-0.1, 1.0, 1.7)],
                )
            )
        for i in range(8, 13):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, 1.0), point(0.2, 1.0, 0.7), point(-0.2, 1.0, 1.1)],
                    [point(0.0, 1.0, 1.0, velocity=1.0, kind="motion")],
                )
            )
        for i in range(13, 41):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, -0.55), point(0.2, 1.0, -0.65), point(-0.2, 1.0, -0.5)],
                )
            )

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.65, z_max_m=2.2))
        events = detect_falls(track)

        self.assertTrue(any(event.state == FallState.SUSPECTED for event in events))
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertTrue(confirmed)
        self.assertGreaterEqual(confirmed[0].drop_height_m, FallRuleConfig().min_drop_height_m)

    def test_rule_detector_ignores_single_frame_low_height_spike(self):
        config = FallRuleConfig()
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        track.append(target_frame(8, 800, 0.20, motion_count=30, velocity=0.7))
        track.extend(target_frame(index, index * 100, 1.40) for index in range(9, 20))

        events = detect_falls(track, config)

        self.assertFalse(any(event.state == FallState.SUSPECTED for event in events))
        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events))

    def test_crouch_above_terminal_height_does_not_emit_public_fall_events(self):
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 31):
            track.append(target_frame(index, index * 100, 0.48, motion_count=30, velocity=0.7))

        events = detect_falls(track, FallRuleConfig())

        self.assertFalse(any(event.state == FallState.SUSPECTED for event in events), events)
        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events), events)

    def test_held_low_height_does_not_accumulate_terminal_evidence(self):
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 14):
            track.append(target_frame(index, index * 100, 0.48, motion_count=30, velocity=0.7))
        track.append(target_frame(14, 1400, 0.25, motion_count=10, velocity=0.3))
        for index in range(15, 26):
            track.append(
                target_frame(
                    index,
                    index * 100,
                    0.25,
                    point_count=0,
                    held=True,
                    state=TargetState.HOLD,
                )
            )

        events = detect_falls(track, FallRuleConfig())

        self.assertFalse(any(event.state == FallState.SUSPECTED for event in events), events)
        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events), events)

    def test_terminal_height_controls_public_suspected_and_confirmed_timing(self):
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 31):
            track.append(target_frame(index, index * 100, 0.30, motion_count=30, velocity=0.7))

        events = detect_falls(track, FallRuleConfig())

        suspected = [event for event in events if event.state == FallState.SUSPECTED]
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertEqual([event.timestamp_ms for event in suspected], [1300])
        self.assertEqual([event.timestamp_ms for event in confirmed], [2500])
        self.assertGreaterEqual(
            confirmed[0].timestamp_ms - suspected[0].timestamp_ms,
            FallRuleConfig().low_still_duration_ms,
        )

    def test_rule_detector_uses_low_frame_ratio_for_suspect_and_confirmation(self):
        config = FallRuleConfig()
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 31):
            z_m = 0.70 if index in {11, 16, 22, 27} else 0.35
            track.append(target_frame(index, index * 100, z_m, motion_count=30, velocity=0.7))

        events = detect_falls(track, config)

        suspected = [event for event in events if event.state == FallState.SUSPECTED]
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertEqual(len(suspected), 1)
        self.assertGreaterEqual(suspected[0].timestamp_ms, 1300)
        self.assertEqual(len(confirmed), 1)
        self.assertGreater(confirmed[0].timestamp_ms, suspected[0].timestamp_ms)
        self.assertGreaterEqual(confirmed[0].timestamp_ms - 800, config.low_still_duration_ms)

    def test_rule_detector_time_windows_work_with_non_divisible_frame_period(self):
        config = FallRuleConfig()
        track = [target_frame(index, index * 53, 1.40) for index in range(20)]
        for index in range(20, 75):
            track.append(
                target_frame(
                    index,
                    index * 53,
                    0.35,
                    motion_count=30,
                    velocity=0.7,
                )
            )

        events = detect_falls(track, config)

        self.assertEqual(sum(event.state == FallState.SUSPECTED for event in events), 1)
        self.assertEqual(sum(event.state == FallState.CONFIRMED for event in events), 1)

    def test_missing_target_breaks_low_posture_duration_before_confirmation(self):
        config = FallRuleConfig(suspect_low_duration_ms=0, suspect_min_fresh_samples=1)
        detector = IncrementalFallDetector(config)
        events = []
        events.extend(detector.step(target_frame(0, 0, 1.40)))
        events.extend(detector.step(target_frame(1, 100, 0.35, motion_count=30, velocity=0.7)))
        for index, timestamp_ms in enumerate(range(200, 1600, 100), start=2):
            events.extend(
                detector.step(
                    TargetFrame(
                        frame_index=index,
                        timestamp_ms=timestamp_ms,
                        timestamp_text=f"t{index}",
                        has_target=False,
                        state=TargetState.LOST,
                    )
                )
            )

        events.extend(detector.step(target_frame(16, 1600, 0.35)))

        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events))

        for index, timestamp_ms in enumerate(range(1700, 2900, 100), start=17):
            events.extend(detector.step(target_frame(index, timestamp_ms, 0.35)))

        self.assertEqual(sum(event.state == FallState.CONFIRMED for event in events), 1)

    def test_confirmed_fall_does_not_rearm_from_elapsed_time_alone(self):
        config = FallRuleConfig()
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 100):
            track.append(target_frame(index, index * 100, 0.35, motion_count=30, velocity=0.7))

        events = detect_falls(track, config)

        self.assertEqual(sum(event.state == FallState.CONFIRMED for event in events), 1)
        self.assertFalse(any(event.state == FallState.RECOVERED for event in events))

    def test_confirmed_fall_rearms_only_after_sustained_standing_height(self):
        config = FallRuleConfig()
        track = [target_frame(index, index * 100, 1.40) for index in range(8)]
        for index in range(8, 30):
            track.append(target_frame(index, index * 100, 0.35, motion_count=30, velocity=0.7))
        for index in range(30, 38):
            track.append(target_frame(index, index * 100, 1.05))
        track.append(target_frame(38, 3800, 0.50))
        for index in range(39, 50):
            track.append(target_frame(index, index * 100, 1.05))

        events = detect_falls(track, config)

        recovered = [event for event in events if event.state == FallState.RECOVERED]
        self.assertEqual(len(recovered), 1)
        self.assertEqual(recovered[0].timestamp_ms, 4900)
        self.assertEqual(recovered[0].reason, "confirmed fall target stood up")

    def test_rule_detector_sensitive_config_confirms_crouch_sized_drop_after_800ms_stillness(self):
        track = [
            target_frame(0, 0, 1.00),
            target_frame(1, 100, 0.52, motion_count=3, velocity=0.7),
        ]
        for index, timestamp_ms in enumerate(range(200, 1001, 100), start=2):
            track.append(target_frame(index, timestamp_ms, 0.52))

        events = detect_falls(
            track,
            FallRuleConfig(
                min_drop_height_m=0.46,
                low_height_m=0.75,
                terminal_height_m=0.75,
                suspect_low_duration_ms=0,
                suspect_min_fresh_samples=1,
                low_still_duration_ms=800,
            ),
        )

        self.assertTrue(any(event.state == FallState.SUSPECTED for event in events))
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].timestamp_ms, 900)
        self.assertGreaterEqual(confirmed[0].drop_height_m, 0.46)

    def test_rule_detector_default_does_not_confirm_crouch_sized_drop_after_800ms(self):
        track = [
            target_frame(0, 0, 1.00),
            target_frame(1, 100, 0.52, motion_count=3, velocity=0.7),
        ]
        for index, timestamp_ms in enumerate(range(200, 1001, 100), start=2):
            track.append(target_frame(index, timestamp_ms, 0.52))

        events = detect_falls(track)

        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events))

    def test_rule_detector_confirms_low_posture_even_when_person_keeps_moving_with_sensitive_config(self):
        track = [
            target_frame(0, 0, 1.00),
            target_frame(1, 100, 0.52, motion_count=8, velocity=0.7),
        ]
        for index, timestamp_ms in enumerate(range(200, 1001, 100), start=2):
            track.append(target_frame(index, timestamp_ms, 0.52, motion_count=12, velocity=0.32))

        events = detect_falls(
            track,
            FallRuleConfig(
                min_drop_height_m=0.46,
                low_height_m=0.75,
                terminal_height_m=0.75,
                suspect_low_duration_ms=0,
                suspect_min_fresh_samples=1,
                low_still_duration_ms=800,
            ),
        )

        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].timestamp_ms, 900)
        self.assertEqual(confirmed[0].reason, "low posture persisted with activity after suspected fall")

    def test_rule_detector_detects_crouch_drop_spread_across_1500ms_window(self):
        track = [
            target_frame(0, 0, 1.00),
            target_frame(1, 500, 0.95),
            target_frame(2, 1000, 0.90),
            target_frame(3, 1450, 0.53, motion_count=3, velocity=0.7),
        ]
        for index, timestamp_ms in enumerate(range(1550, 2351, 100), start=4):
            track.append(target_frame(index, timestamp_ms, 0.53))

        events = detect_falls(
            track,
            FallRuleConfig(
                min_drop_height_m=0.46,
                low_height_m=0.75,
                terminal_height_m=0.75,
                suspect_low_duration_ms=0,
                suspect_min_fresh_samples=1,
                low_still_duration_ms=800,
            ),
        )

        suspected = [event for event in events if event.state == FallState.SUSPECTED]
        confirmed = [event for event in events if event.state == FallState.CONFIRMED]
        self.assertEqual(len(suspected), 1)
        self.assertEqual(suspected[0].timestamp_ms, 1450)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].timestamp_ms, 2250)

    def test_rule_detector_rearms_after_confirmed_fall_when_person_stands_then_drops_again(self):
        track = [
            target_frame(0, 0, 1.20),
            target_frame(1, 100, 0.55, motion_count=3, velocity=0.7),
        ]
        for index, timestamp_ms in enumerate(range(200, 1001, 100), start=2):
            track.append(target_frame(index, timestamp_ms, 0.55))
        track.extend(
            [
                target_frame(11, 1200, 1.20, motion_count=3, velocity=0.7),
                target_frame(12, 1300, 1.20),
                target_frame(13, 1400, 0.55, motion_count=3, velocity=0.7),
            ]
        )
        for index, timestamp_ms in enumerate(range(1500, 2301, 100), start=14):
            track.append(target_frame(index, timestamp_ms, 0.55))

        events = detect_falls(
            track,
            FallRuleConfig(
                min_drop_height_m=0.60,
                low_height_m=0.75,
                terminal_height_m=0.75,
                suspect_low_duration_ms=0,
                suspect_min_fresh_samples=1,
                low_still_duration_ms=800,
                rearm_duration_ms=100,
            ),
        )

        self.assertEqual(sum(1 for event in events if event.state == FallState.SUSPECTED), 2)
        self.assertEqual(sum(1 for event in events if event.state == FallState.CONFIRMED), 2)

    def test_incremental_fall_detector_matches_batch_detector(self):
        frames = []
        for i in range(8):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, 1.5), point(0.1, 1.0, 1.3), point(-0.1, 1.0, 1.7)],
                )
            )
        for i in range(8, 13):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, 1.0), point(0.2, 1.0, 0.7), point(-0.2, 1.0, 1.1)],
                    [point(0.0, 1.0, 1.0, velocity=1.0, kind="motion")],
                )
            )
        for i in range(13, 41):
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, -0.55), point(0.4, 1.0, -0.65), point(-0.4, 1.0, -0.5)],
                )
            )

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.65, z_max_m=2.2))
        batch_events = detect_falls(track)
        detector = IncrementalFallDetector()
        incremental_events = []
        for target in track:
            incremental_events.extend(detector.step(target))

        self.assertEqual(batch_events, incremental_events)

    def test_rule_detector_does_not_confirm_slow_sit_down_as_fall(self):
        frames = []
        for i in range(30):
            z = 1.4 - min(i * 0.02, 0.6)
            frames.append(
                frame(
                    i,
                    i * 100,
                    [point(0.0, 1.0, z), point(0.1, 1.0, z - 0.2), point(-0.1, 1.0, z + 0.2)],
                )
            )

        track = track_recording(frames, TrackerConfig(min_cluster_points=2, cluster_radius_m=0.55))
        events = detect_falls(track)

        self.assertFalse(any(event.state == FallState.CONFIRMED for event in events))


if __name__ == "__main__":
    unittest.main()
