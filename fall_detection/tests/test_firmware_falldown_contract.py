from __future__ import annotations

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


MRS6240_ROOT = Path(__file__).resolve().parents[2]
FALLDOWN_ROOT = (
    MRS6240_ROOT
    / "SDK"
    / "Software_Kit"
    / "02_SDK"
    / "psdf_sdk"
    / "project"
    / "mmwave"
    / "mmwave_application"
    / "Falldown"
)
PRJ_CONFIG = FALLDOWN_ROOT / "src" / "prj_config.h"


def integer_macros(path: Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8")
    matches = re.finditer(
        r"^\s*#define\s+(FALL_[A-Z0-9_]+)\s+\(?\s*(-?\d+)\s*\)?(?:\s|/|$)",
        text,
        flags=re.MULTILINE,
    )
    return {match.group(1): int(match.group(2)) for match in matches}


class FirmwareFalldownContractTests(unittest.TestCase):
    def test_firmware_macros_match_validated_python_profile(self):
        macros = integer_macros(PRJ_CONFIG)
        expected = {
            "FALL_MOUNT_HEIGHT_CM": 163,
            "FALL_MOUNT_PITCH_DEG": 30,
            "FALL_MAX_POINTS": 192,
            "FALL_MAX_CLUSTERS": 32,
            "FALL_MIN_SNR_DB": 15,
            "FALL_ROI_X_MIN_CM": -150,
            "FALL_ROI_X_MAX_CM": 150,
            "FALL_ROI_Y_MIN_CM": 15,
            "FALL_ROI_Y_MAX_CM": 300,
            "FALL_ROI_Z_MIN_CM": 0,
            "FALL_ROI_Z_MAX_CM": 220,
            "FALL_CLUSTER_RADIUS_CM": 45,
            "FALL_MIN_CLUSTER_POINTS": 3,
            "FALL_MAX_CLUSTER_WIDTH_CM": 50,
            "FALL_MAX_CLUSTER_DEPTH_CM": 50,
            "FALL_TRACK_GATE_CM": 50,
            "FALL_HOLD_MS": 3000,
            "FALL_SWITCH_CONFIRM_FRAMES": 5,
            "FALL_STATIC_BODY_MIN_POINTS": 20,
            "FALL_DROP_WINDOW_MS": 3500,
            "FALL_CONFIRM_WINDOW_MS": 7000,
            "FALL_MIN_DROP_CM": 80,
            "FALL_MOTION_VELOCITY_TRIGGER_CM_S": 45,
            "FALL_MOTION_POINT_THRESHOLD": 20,
            "FALL_CANDIDATE_LOW_HEIGHT_CM": 55,
            "FALL_TERMINAL_HEIGHT_CM": 40,
            "FALL_SUSPECT_LOW_DURATION_MS": 700,
            "FALL_SUSPECT_MIN_FRESH_SAMPLES": 3,
            "FALL_SUSPECT_LOW_RATIO_PERCENT": 92,
            "FALL_CONFIRM_LOW_DURATION_MS": 1200,
            "FALL_CONFIRM_MIN_FRESH_SAMPLES": 6,
            "FALL_CONFIRM_LOW_RATIO_PERCENT": 70,
            "FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM": 60,
            "FALL_FRESH_MIN_POINT_COUNT": 3,
            "FALL_STILL_VELOCITY_CM_S": 18,
            "FALL_STILL_MOTION_POINTS_MAX": 1,
            "FALL_REARM_HEIGHT_CM": 85,
            "FALL_REARM_DURATION_MS": 1000,
            "FALL_HISTORY_MAX_SAMPLES": 192,
            "FALL_EVENT_BATCH_MAX": 2,
            "FALL_STATUS_PERIOD_FRAMES": 10,
        }

        self.assertEqual({name: macros.get(name) for name in expected}, expected)

    def test_firmware_rejects_nonpositive_clustering_geometry(self):
        source = PRJ_CONFIG.read_text(encoding="utf-8")

        self.assertIn("#if (FALL_CLUSTER_RADIUS_CM <= 0) ||", source)
        self.assertIn("(FALL_MAX_CLUSTER_WIDTH_CM <= 0) ||", source)
        self.assertIn("(FALL_MAX_CLUSTER_DEPTH_CM <= 0) ||", source)
        self.assertIn("(FALL_TRACK_GATE_CM <= 0)", source)

    def test_algorithm_headers_exist_without_sdk_point_buffer_types(self):
        pipeline_header = FALLDOWN_ROOT / "src" / "fall_pipeline.h"
        rule_header = FALLDOWN_ROOT / "src" / "fall_rule.h"

        self.assertTrue(pipeline_header.is_file())
        self.assertTrue(rule_header.is_file())
        combined = pipeline_header.read_text(encoding="utf-8") + rule_header.read_text(encoding="utf-8")
        self.assertNotIn("PointCloudBuffer_t", combined)
        self.assertNotIn("PresencePointCloudBuffer_t", combined)
        self.assertNotIn("mmw_ctrl.h", combined)

    def test_pipeline_core_is_fixed_memory_and_sdk_independent(self):
        source_path = FALLDOWN_ROOT / "src" / "fall_pipeline.c"
        self.assertTrue(source_path.is_file())
        source = source_path.read_text(encoding="utf-8")
        for forbidden in ("OSI_Malloc", "malloc(", "calloc(", "realloc(", "free("):
            self.assertNotIn(forbidden, source)
        for sdk_type in ("PointCloudBuffer_t", "PresencePointCloudBuffer_t"):
            self.assertNotIn(sdk_type, source)

    def test_pipeline_cluster_expansion_matches_python_lifo_order(self):
        source = (FALLDOWN_ROOT / "src" / "fall_pipeline.c").read_text(encoding="utf-8")
        self.assertIn("current = runtime->queue[--queue_tail];", source)

    def test_rule_core_is_fixed_memory_and_sdk_independent(self):
        source_path = FALLDOWN_ROOT / "src" / "fall_rule.c"
        self.assertTrue(source_path.is_file())
        source = source_path.read_text(encoding="utf-8")
        for forbidden in ("OSI_Malloc", "malloc(", "calloc(", "realloc(", "free("):
            self.assertNotIn(forbidden, source)
        self.assertNotIn("mmw_ctrl.h", source)

    def test_confirmation_uses_post_suspect_time_and_horizontal_displacement(self):
        source = (FALLDOWN_ROOT / "src" / "fall_rule.c").read_text(encoding="utf-8")

        self.assertIn(
            "target->timestamp_ms - runtime->suspected_timestamp_ms >=",
            source,
        )
        self.assertIn("fall_candidate_horizontal_displacement_ready(runtime)", source)

    def test_sdk_adapter_delegates_without_starting_radar(self):
        source = (FALLDOWN_ROOT / "src" / "mmw_app_entry.c").read_text(encoding="utf-8")
        self.assertIn('#include "fall_pipeline.h"', source)
        self.assertIn('#include "fall_rule.h"', source)
        self.assertIn("fall_pipeline_finish_frame", source)
        self.assertIn("fall_rule_step", source)
        self.assertNotIn("#define FALL_", source)
        self.assertNotIn("mmw_ctrl_start(", source)

    def test_firmware_boot_log_exposes_terminal_timing_profile(self):
        source = (FALLDOWN_ROOT / "src" / "mmw_app_entry.c").read_text(encoding="utf-8")

        self.assertIn(
            "terminal=%d suspect_ms=%u suspect_ratio=%u confirm_ms=%u confirm_xy=%u",
            source,
        )
        self.assertIn("FALL_SUSPECT_LOW_DURATION_MS", source)
        self.assertIn("FALL_SUSPECT_LOW_RATIO_PERCENT", source)
        self.assertIn("FALL_CONFIRM_LOW_DURATION_MS", source)
        self.assertIn("FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM", source)

    def test_cpuf_cdk_project_registers_falldown_modules(self):
        root = ET.parse(FALLDOWN_ROOT / "cdk_6240_cpuf" / "project.cdkproj").getroot()
        names = {element.attrib["Name"].replace("\\", "/") for element in root.iter("File")}
        expected = {
            "../src/fall_pipeline.h",
            "../src/fall_pipeline.c",
            "../src/fall_rule.h",
            "../src/fall_rule.c",
        }

        self.assertTrue(expected.issubset(names), expected - names)


if __name__ == "__main__":
    unittest.main()
