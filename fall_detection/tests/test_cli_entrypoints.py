import subprocess
import sys
import tempfile
import unittest
import csv
from unittest.mock import patch
from pathlib import Path


class CliEntrypointTests(unittest.TestCase):
    def test_analyze_datadealer_can_run_as_direct_script(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "analyze_datadealer.py"

        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=project_dir,
            capture_output=True,
            text=True,
            timeout=30,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Analyze MRS6240 point-cloud CSV files", result.stdout)

    def test_visualize_datadealer_can_run_as_direct_script(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "visualize_datadealer.py"

        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=project_dir,
            capture_output=True,
            text=True,
            timeout=10,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Visualize MRS6240 datadealer point-cloud clusters.", result.stdout)
        self.assertIn("--yz-rotation-deg", result.stdout)

    def test_visualize_datadealer_yz_rotation_cli_defaults_to_zero_and_accepts_degrees(self):
        from fall_detection.visualize_datadealer import parse_args

        with patch.object(sys, "argv", ["visualize_datadealer.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.yz_rotation_deg, 0.0)

        with patch.object(sys, "argv", ["visualize_datadealer.py", "--yz-rotation-deg", "55"]):
            args = parse_args()

        self.assertAlmostEqual(args.yz_rotation_deg, 55.0)

    def test_visualize_datadealer_world_coordinate_cli_values(self):
        from fall_detection.visualize_datadealer import parse_args

        with patch.object(sys, "argv", ["visualize_datadealer.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.mount_height_m, 0.0)
        self.assertEqual(args.ground_calibration_frames, 0)
        self.assertAlmostEqual(args.ground_band_m, 0.25)

        with patch.object(
            sys,
            "argv",
            [
                "visualize_datadealer.py",
                "--mount-height-m",
                "1.4",
                "--ground-calibration-frames",
                "200",
                "--ground-band-m",
                "0.35",
            ],
        ):
            args = parse_args()

        self.assertAlmostEqual(args.mount_height_m, 1.4)
        self.assertEqual(args.ground_calibration_frames, 200)
        self.assertAlmostEqual(args.ground_band_m, 0.35)

    def test_visualize_datadealer_z_roi_cli_defaults_and_explicit_values(self):
        from fall_detection.tracking import TrackerConfig
        from fall_detection.visualize_datadealer import parse_args

        with patch.object(sys, "argv", ["visualize_datadealer.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.z_min, TrackerConfig.z_min_m)
        self.assertAlmostEqual(args.z_max, TrackerConfig.z_max_m)

        with patch.object(
            sys,
            "argv",
            ["visualize_datadealer.py", "--z-min", "0", "--z-max", "2.2"],
        ):
            args = parse_args()

        self.assertAlmostEqual(args.z_min, 0.0)
        self.assertAlmostEqual(args.z_max, 2.2)

    def test_visualize_datadealer_tracker_config_uses_and_validates_z_roi(self):
        from fall_detection.visualize_datadealer import parse_args, tracker_config_from_args

        with patch.object(
            sys,
            "argv",
            ["visualize_datadealer.py", "--z-min", "0", "--z-max", "2.2"],
        ):
            config = tracker_config_from_args(parse_args())

        self.assertAlmostEqual(config.z_min_m, 0.0)
        self.assertAlmostEqual(config.z_max_m, 2.2)

        with patch.object(
            sys,
            "argv",
            ["visualize_datadealer.py", "--z-min", "1.0", "--z-max", "0.5"],
        ):
            args = parse_args()

        with self.assertRaisesRegex(ValueError, "z-min.*z-max"):
            tracker_config_from_args(args)

    def test_visualize_datadealer_fall_cli_values_feed_deployment_rule_config(self):
        from fall_detection.visualize_datadealer import fall_rule_config_from_args, parse_args

        with patch.object(
            sys,
            "argv",
            [
                "visualize_datadealer.py",
                "--fall-suspect-duration-ms",
                "700",
                "--fall-suspect-low-ratio",
                "0.92",
                "--fall-confirm-min-horizontal-displacement",
                "0.60",
            ],
        ):
            config = fall_rule_config_from_args(parse_args())

        self.assertEqual(config.suspect_low_duration_ms, 700)
        self.assertAlmostEqual(config.suspect_low_ratio, 0.92)
        self.assertAlmostEqual(config.confirm_min_horizontal_displacement_m, 0.60)

    def test_visualize_datadealer_main_passes_fall_config_to_application(self):
        from fall_detection.visualize_datadealer import main

        with (
            patch.object(
                sys,
                "argv",
                [
                    "visualize_datadealer.py",
                    "--fall-suspect-duration-ms",
                    "700",
                    "--fall-suspect-low-ratio",
                    "0.92",
                    "--fall-confirm-min-horizontal-displacement",
                    "0.60",
                ],
            ),
            patch("fall_detection.visualize_datadealer.tk.Tk") as root_factory,
            patch("fall_detection.visualize_datadealer.DatadealerVisualizerApp") as app_factory,
        ):
            main()

        fall_config = app_factory.call_args.kwargs["fall_config"]
        self.assertEqual(fall_config.suspect_low_duration_ms, 700)
        self.assertAlmostEqual(fall_config.suspect_low_ratio, 0.92)
        self.assertAlmostEqual(fall_config.confirm_min_horizontal_displacement_m, 0.60)
        root_factory.return_value.mainloop.assert_called_once_with()

    def test_live_visualizer_can_run_as_direct_script(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "live_visualizer.py"

        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=project_dir,
            capture_output=True,
            text=True,
            timeout=10,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Live visualize MRS6240 point-cloud clusters.", result.stdout)
        self.assertIn("--source", result.stdout)
        self.assertIn("--spi-index", result.stdout)
        self.assertIn("--y-min", result.stdout)
        self.assertIn("--y-max", result.stdout)
        self.assertIn("--z-min", result.stdout)
        self.assertIn("--yz-rotation-deg", result.stdout)
        self.assertIn("--fall-min-drop-height", result.stdout)
        self.assertIn("--fall-terminal-height", result.stdout)
        self.assertIn("--fall-low-duration-ms", result.stdout)
        self.assertIn("--draw-3d-every", result.stdout)
        self.assertIn("--draw-timeline-every", result.stdout)

    def test_serial_event_live_visualizer_can_run_as_direct_script(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "serial_event_live_visualizer.py"

        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=project_dir,
            capture_output=True,
            text=True,
            timeout=15,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Display MRS6240 raw point clouds and board serial fall events.", result.stdout)
        self.assertIn("--spi-index", result.stdout)
        self.assertIn("--serial-port", result.stdout)
        self.assertIn("--source", result.stdout)
        self.assertIn("--session-dir", result.stdout)
        self.assertIn("--replay-speed", result.stdout)

    def test_live_visualizer_roi_cli_values_feed_tracker_config(self):
        from fall_detection.live_visualizer import parse_args, tracker_config_from_args

        with patch.object(
            sys,
            "argv",
            [
                "live_visualizer.py",
                "--y-min",
                "0.05",
                "--y-max",
                "2.0",
                "--z-min",
                "-0.3",
                "--z-max",
                "1.5",
                "--min-snr",
                "10",
            ],
        ):
            args = parse_args()

        config = tracker_config_from_args(args)

        self.assertAlmostEqual(config.y_min_m, 0.05)
        self.assertAlmostEqual(config.y_max_m, 2.0)
        self.assertAlmostEqual(config.z_min_m, -0.3)
        self.assertAlmostEqual(config.z_max_m, 1.5)
        self.assertAlmostEqual(config.min_snr_db, 10.0)

    def test_live_visualizer_fall_cli_values_feed_fall_rule_config(self):
        from fall_detection.live_visualizer import fall_rule_config_from_args, parse_args

        with patch.object(
            sys,
            "argv",
            [
                "live_visualizer.py",
                "--fall-min-drop-height",
                "0.75",
                "--fall-terminal-height",
                "0.42",
                "--fall-low-duration-ms",
                "2200",
                "--fall-confirm-window-ms",
                "4500",
            ],
        ):
            args = parse_args()

        config = fall_rule_config_from_args(args)

        self.assertAlmostEqual(config.min_drop_height_m, 0.75)
        self.assertAlmostEqual(config.terminal_height_m, 0.42)
        self.assertEqual(config.low_still_duration_ms, 2200)
        self.assertEqual(config.confirm_window_ms, 4500)

    def test_live_visualizer_yz_rotation_cli_defaults_to_zero_and_accepts_degrees(self):
        from fall_detection.live_visualizer import parse_args

        with patch.object(sys, "argv", ["live_visualizer.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.yz_rotation_deg, 0.0)

        with patch.object(sys, "argv", ["live_visualizer.py", "--yz-rotation-deg", "45"]):
            args = parse_args()

        self.assertAlmostEqual(args.yz_rotation_deg, 45.0)

    def test_live_visualizer_mount_height_cli_defaults_to_zero_and_accepts_meters(self):
        from fall_detection.live_visualizer import parse_args

        with patch.object(sys, "argv", ["live_visualizer.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.mount_height_m, 0.0)

        with patch.object(
            sys,
            "argv",
            ["live_visualizer.py", "--mount-height-m", "1.67"],
        ):
            args = parse_args()

        self.assertAlmostEqual(args.mount_height_m, 1.67)

    def test_live_visualizer_main_passes_mount_height_to_application(self):
        from fall_detection.live_visualizer import main

        with (
            patch.object(
                sys,
                "argv",
                ["live_visualizer.py", "--no-autostart", "--mount-height-m", "1.67"],
            ),
            patch("fall_detection.live_visualizer.tk.Tk") as root_factory,
            patch("fall_detection.live_visualizer.LiveReplayVisualizerApp") as app_factory,
        ):
            main()

        self.assertAlmostEqual(app_factory.call_args.kwargs["mount_height_m"], 1.67)
        root_factory.return_value.mainloop.assert_called_once_with()

    def test_live_visualizer_defaults_refresh_3d_every_live_frame(self):
        from fall_detection.live_visualizer import parse_args

        with patch.object(sys, "argv", ["live_visualizer.py"]):
            args = parse_args()

        self.assertEqual(args.poll_ms, 5)
        self.assertEqual(args.draw_3d_every, 1)
        self.assertEqual(args.draw_timeline_every, 10)

    def test_visualization_model_direct_run_shows_entrypoint_hint(self):
        package_dir = Path(__file__).resolve().parents[1]
        script = package_dir / "visualization_model.py"

        result = subprocess.run(
            [sys.executable, str(script)],
            cwd=package_dir,
            capture_output=True,
            text=True,
            timeout=10,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("visualization_model.py is a library module.", result.stdout)
        self.assertIn("visualize_datadealer.py", result.stdout)

    def test_analyze_datadealer_can_print_suspicious_target_diagnostics(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "analyze_datadealer.py"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "sample_presence_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "0", *self._point_columns(0.0, 1.2, 1.0, 101, velocity=0.0, snr=35.0)])
                writer.writerow(["t1", "100", *self._point_columns(0.0, 1.2, 1.0, 101, velocity=0.0, snr=35.0)])
            with (root / "sample_motion_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "0", *self._point_columns(0.0, 1.2, 1.0, 5, velocity=0.2, snr=35.0)])
                writer.writerow(
                    [
                        "t1",
                        "100",
                        *self._point_columns(0.0, 1.2, 1.0, 1, velocity=0.2, snr=35.0),
                        *self._point_columns(0.0, 0.9, 0.15, 4, velocity=0.8, snr=35.0),
                    ]
                )

            result = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--data-dir",
                    str(root),
                    "--stem",
                    "sample",
                    "--diagnose-targets",
                    "--cluster-radius",
                    "0.18",
                    "--min-cluster-points",
                    "3",
                ],
                cwd=project_dir,
                capture_output=True,
                text=True,
                timeout=30,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("target_diagnostics=", result.stdout)
        self.assertIn("诊断:", result.stdout)

    def test_analyze_datadealer_can_print_batch_state_summary(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "analyze_datadealer.py"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._write_simple_recording(root, "sample_a", center_y=1.0)
            self._write_simple_recording(root, "sample_b", center_y=1.4)

            result = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--data-dir",
                    str(root),
                    "--batch-summary",
                    "--cluster-radius",
                    "0.18",
                    "--min-cluster-points",
                    "3",
                ],
                cwd=project_dir,
                capture_output=True,
                text=True,
                timeout=30,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("batch_recordings=2", result.stdout)
        self.assertIn("recording=sample_a", result.stdout)
        self.assertIn("recording=sample_b", result.stdout)
        self.assertIn("states tracking=", result.stdout)
        self.assertIn("max_jump_m=", result.stdout)

    def test_analyze_datadealer_can_report_large_target_jumps_with_candidates(self):
        package_dir = Path(__file__).resolve().parents[1]
        project_dir = package_dir.parent
        script = package_dir / "analyze_datadealer.py"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "sample_presence_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "0", *self._point_columns(0.0, 1.0, 1.0, 9, velocity=0.0, snr=35.0)])
                writer.writerow(["t1", "100", *self._point_columns(1.2, 2.2, 1.0, 9, velocity=0.0, snr=35.0)])
            with (root / "sample_motion_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t0", "0", *self._point_columns(0.0, 1.0, 1.0, 3, velocity=0.2, snr=35.0)])
                writer.writerow(["t1", "100", *self._point_columns(1.2, 2.2, 1.0, 4, velocity=0.7, snr=35.0)])

            result = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--data-dir",
                    str(root),
                    "--stem",
                    "sample",
                    "--jump-report",
                    "--jump-threshold",
                    "0.8",
                    "--cluster-radius",
                    "0.18",
                    "--min-cluster-points",
                    "3",
                    "--hold-ms",
                    "0",
                    "--switch-confirm-frames",
                    "1",
                ],
                cwd=project_dir,
                capture_output=True,
                text=True,
                timeout=30,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("jump_report threshold_m=0.800", result.stdout)
        self.assertIn("jump frame=0->1", result.stdout)
        self.assertIn("jump_m=", result.stdout)
        self.assertIn("prev_state=", result.stdout)
        self.assertIn("curr_state=", result.stdout)
        self.assertIn("candidates=", result.stdout)
        self.assertIn("#0 点", result.stdout)

    def _write_simple_recording(self, root, stem, *, center_y):
        with (root / f"{stem}_presence_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t0", "0", *self._point_columns(0.0, center_y, 1.0, 9, velocity=0.0, snr=35.0)])
            writer.writerow(["t1", "100", *self._point_columns(0.02, center_y + 0.02, 1.0, 9, velocity=0.0, snr=35.0)])
        with (root / f"{stem}_motion_point_cloud.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t0", "0", *self._point_columns(0.0, center_y, 1.0, 3, velocity=0.2, snr=35.0)])
            writer.writerow(["t1", "100", *self._point_columns(0.02, center_y + 0.02, 1.0, 3, velocity=0.2, snr=35.0)])

    def _point_columns(self, center_x, center_y, center_z, count, *, velocity, snr):
        offsets = [(-0.08, -0.08), (0.0, -0.08), (0.08, -0.08), (-0.08, 0.0), (0.0, 0.0), (0.08, 0.0), (-0.08, 0.08), (0.0, 0.08), (0.08, 0.08)]
        xs = []
        ys = []
        zs = []
        velocities = []
        snrs = []
        for index in range(count):
            dx, dy = offsets[index % len(offsets)]
            xs.append(center_x + dx)
            ys.append(center_y + dy)
            zs.append(center_z + ((index % 3) - 1) * 0.04)
            velocities.append(velocity)
            snrs.append(snr)
        return [*xs, *ys, *zs, *velocities, *snrs]


if __name__ == "__main__":
    unittest.main()
