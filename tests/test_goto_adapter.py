import unittest

from dock_planner import (
    GotoReferenceSample,
    GotoStatusKind,
    NEDPoint,
    Pose2D,
    ned_goal_to_goto_enu,
    parse_goto_status,
)


class GotoCoordinateTests(unittest.TestCase):
    def test_relative_conversion_cancels_different_frame_origins(self):
        reference = GotoReferenceSample(
            current_ned=NEDPoint(100.0, 50.0, 0.0),
            odom_east_m=1000.0,
            odom_north_m=-500.0,
            ned_timestamp=20.0,
            odom_timestamp=20.1,
        )
        target = ned_goal_to_goto_enu(
            Pose2D(106.0, 47.0, 1.2),
            reference,
        )
        self.assertEqual(target.east_m, 997.0)
        self.assertEqual(target.north_m, -494.0)
        self.assertEqual(target.frame_id, "map")

    def test_unsynchronized_position_samples_are_rejected(self):
        reference = GotoReferenceSample(
            NEDPoint(0.0, 0.0, 0.0), 0.0, 0.0, 10.0, 10.5,
        )
        with self.assertRaisesRegex(ValueError, "not synchronized"):
            ned_goal_to_goto_enu(Pose2D(1.0, 2.0, 0.0), reference)


class GotoStatusTests(unittest.TestCase):
    def test_known_status_prefixes(self):
        messages = {
            "WAIT_TARGET armed=false mode=MANUAL": GotoStatusKind.WAIT_TARGET,
            "PRECHECK armed=true mode=GUIDED (needs GUIDED + armed)": GotoStatusKind.PRECHECK,
            "RUNNING dist=2.10m head_err=0.20rad v=0.50 w=0.10": GotoStatusKind.RUNNING,
            "DONE arrived": GotoStatusKind.DONE,
            "ABORTED reason=ODOMETRY_STALE": GotoStatusKind.ABORTED,
        }
        for message, expected in messages.items():
            with self.subTest(message=message):
                self.assertEqual(parse_goto_status(message).kind, expected)

    def test_done_and_abort_semantics(self):
        done = parse_goto_status("DONE arrived")
        self.assertTrue(done.terminal)
        self.assertTrue(done.succeeded)

        aborted = parse_goto_status("ABORTED reason=TARGET_NEVER_APPROACHED:start=8 now=9")
        self.assertTrue(aborted.terminal)
        self.assertFalse(aborted.succeeded)
        self.assertEqual(
            aborted.reason,
            "TARGET_NEVER_APPROACHED:start=8 now=9",
        )

    def test_unknown_status_is_retained_without_becoming_terminal(self):
        status = parse_goto_status("NEW_STATE detail")
        self.assertEqual(status.kind, GotoStatusKind.UNKNOWN)
        self.assertFalse(status.terminal)


if __name__ == "__main__":
    unittest.main()
