import unittest

from dock_lidar_prototype.detector import EntranceCandidate
from dock_lidar_prototype.tracking import EntranceTrackConfig, track_entrance_candidates


def _candidate(x, y, confidence, heading=0.0):
    return EntranceCandidate(x, y, heading, 1.5, confidence, True)


class EntranceTrackingTests(unittest.TestCase):
    def test_smooth_path_beats_single_high_confidence_jump(self):
        frames = (
            (_candidate(1.0, 0.0, 0.70), _candidate(8.0, 4.0, 0.99)),
            (_candidate(1.5, 0.1, 0.70), _candidate(3.0, -4.0, 0.99)),
            (_candidate(2.0, 0.1, 0.70), _candidate(8.0, 4.0, 0.99)),
            (_candidate(2.5, 0.2, 0.70), _candidate(3.0, -4.0, 0.99)),
        )
        track = track_entrance_candidates(frames, EntranceTrackConfig(
            max_step_m=1.0,
            position_penalty=0.8,
            min_frame_hits=3,
        ))
        self.assertIsNotNone(track)
        self.assertEqual(track.frame_hits, 4)
        self.assertAlmostEqual(track.center_xy[0], 1.75, delta=0.1)

    def test_empty_frame_can_be_bridged(self):
        frames = (
            (_candidate(1.0, 0.0, 0.8),),
            (),
            (_candidate(1.5, 0.0, 0.8),),
        )
        track = track_entrance_candidates(frames, EntranceTrackConfig(
            max_step_m=1.0,
            max_gap_frames=1,
            min_frame_hits=2,
        ))
        self.assertIsNotNone(track)
        self.assertEqual(track.frame_indices, (0, 2))

    def test_short_track_is_rejected(self):
        track = track_entrance_candidates(
            (( _candidate(1.0, 0.0, 0.9),),),
            EntranceTrackConfig(min_frame_hits=2),
        )
        self.assertIsNone(track)


if __name__ == "__main__":
    unittest.main()
