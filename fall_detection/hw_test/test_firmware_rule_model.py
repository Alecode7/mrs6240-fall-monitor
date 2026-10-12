from __future__ import annotations

import unittest

from firmware_rule_model import score_confirmed_events


class FirmwareRuleScoringTests(unittest.TestCase):
    def test_matches_confirmations_to_terminal_event_windows(self):
        score = score_confirmed_events(
            confirmed_timestamps_ms=(200, 1_200, 5_000),
            marks=(
                {"name": "跌倒", "label": 1, "timestamp_ms": 100},
                {"name": "蹲下", "label": 0, "timestamp_ms": 1_000},
            ),
            event_post_duration_ms=300,
        )

        self.assertEqual(score["positive_event_count"], 1)
        self.assertEqual(score["detected_positive_events"], 1)
        self.assertEqual(score["false_confirmed_count"], 2)
        self.assertAlmostEqual(score["event_recall"], 1.0)
        self.assertAlmostEqual(score["event_precision"], 1.0 / 3.0)
        self.assertEqual(score["false_by_label"], {"background": 1, "蹲下": 1})

    def test_multiple_confirmations_inside_one_event_count_once(self):
        score = score_confirmed_events(
            confirmed_timestamps_ms=(200, 250),
            marks=({"name": "跌倒", "label": 1, "timestamp_ms": 100},),
            event_post_duration_ms=300,
        )

        self.assertEqual(score["positive_event_count"], 1)
        self.assertEqual(score["detected_positive_events"], 1)
        self.assertEqual(score["confirmed_count"], 2)
        self.assertEqual(score["false_confirmed_count"], 0)


if __name__ == "__main__":
    unittest.main()
