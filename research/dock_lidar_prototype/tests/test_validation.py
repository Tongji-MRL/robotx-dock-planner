import unittest

from dock_lidar_prototype.detector import EntranceCandidate
from dock_lidar_prototype.validation import (
    aggregate_temporal_candidates,
    select_external_persistent_candidate,
)


def _candidate(x, y, width=1.5, heading=0.0, confidence=0.9):
    return EntranceCandidate(x, y, heading, width, confidence, True)


class TemporalValidationTests(unittest.TestCase):
    def test_repeated_candidates_are_associated_across_frames(self):
        frames = (
            (_candidate(1.00, 0.00),),
            (_candidate(1.05, 0.03),),
            (_candidate(0.97, -0.02),),
        )
        result = aggregate_temporal_candidates(frames, center_tolerance_m=0.2)
        self.assertEqual(result[0].frame_hits, 3)
        self.assertAlmostEqual(result[0].center_x, 1.0067, places=3)
        self.assertAlmostEqual(result[0].width_mean_m, 1.5)

    def test_heading_is_compared_modulo_pi(self):
        frames = (
            (_candidate(1.0, 0.0, heading=0.02),),
            (_candidate(1.0, 0.0, heading=3.14159),),
        )
        result = aggregate_temporal_candidates(frames)
        self.assertLess(abs(result[0].line_normal_rad_mod_pi), 0.03)

    def test_single_entrance_rule_prefers_outer_persistent_gap(self):
        frames = (
            (_candidate(1.0, 0.0), _candidate(2.0, 0.0)),
            (_candidate(1.02, 0.0), _candidate(2.01, 0.0)),
            (_candidate(0.98, 0.0), _candidate(2.02, 0.0)),
        )
        candidates = aggregate_temporal_candidates(frames)
        selected = select_external_persistent_candidate(candidates)
        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected.center_x, 1.0, delta=0.05)


if __name__ == "__main__":
    unittest.main()
