import unittest

from dock_planner import (
    GenericVelocityCommand,
    MotionCommandArbiter,
    MotionCommandArbiterConfig,
    MotionCommandOwner,
)


def command(surge, yaw_rate, *, timestamp=10.0, valid_until=10.5, source="test"):
    return GenericVelocityCommand(
        surge_mps=surge,
        yaw_rate_rps=yaw_rate,
        timestamp=timestamp,
        valid_until=valid_until,
        phase="TEST",
        source=source,
        frame_id="base_link",
    )


class MotionCommandArbiterTests(unittest.TestCase):
    def setUp(self):
        self.arbiter = MotionCommandArbiter(MotionCommandArbiterConfig(
            max_abs_surge_mps=2.0,
            max_abs_yaw_rate_rps=1.0,
        ))

    def test_default_owner_outputs_failsafe_stop(self):
        decision = self.arbiter.select(
            now=10.1,
            goto_command=command(1.0, 0.2),
            terminal_command=command(0.2, -0.3),
        )
        self.assertEqual(decision.owner, MotionCommandOwner.STOP)
        self.assertTrue(decision.failsafe_stop)
        self.assertEqual((decision.command.surge_mps, decision.command.yaw_rate_rps), (0.0, 0.0))

    def test_goto_owner_ignores_terminal_command(self):
        self.arbiter.set_owner(MotionCommandOwner.GOTO)
        goto = command(1.0, 0.2, source="goto")
        decision = self.arbiter.select(
            now=10.1,
            goto_command=goto,
            terminal_command=command(0.2, -0.3, source="terminal"),
        )
        self.assertIs(decision.command, goto)
        self.assertEqual(decision.owner, MotionCommandOwner.GOTO)
        self.assertFalse(decision.failsafe_stop)

    def test_terminal_owner_ignores_goto_command(self):
        self.arbiter.set_owner(MotionCommandOwner.TERMINAL)
        terminal = command(0.2, -0.3, source="terminal")
        decision = self.arbiter.select(
            now=10.1,
            goto_command=command(1.0, 0.2, source="goto"),
            terminal_command=terminal,
        )
        self.assertIs(decision.command, terminal)
        self.assertEqual(decision.owner, MotionCommandOwner.TERMINAL)

    def test_missing_or_stale_selected_command_stops(self):
        self.arbiter.set_owner(MotionCommandOwner.GOTO)
        missing = self.arbiter.select(now=10.1)
        self.assertTrue(missing.failsafe_stop)
        stale = self.arbiter.select(
            now=11.0,
            goto_command=command(1.0, 0.0),
        )
        self.assertTrue(stale.failsafe_stop)
        self.assertIn("stale", stale.reason)

    def test_wrong_frame_or_limit_violation_stops(self):
        self.arbiter.set_owner(MotionCommandOwner.TERMINAL)
        wrong_frame = GenericVelocityCommand(
            0.2, 0.1, 10.0, 10.5, "TEST", "terminal", "map",
        )
        self.assertTrue(self.arbiter.select(
            now=10.1, terminal_command=wrong_frame,
        ).failsafe_stop)
        self.assertTrue(self.arbiter.select(
            now=10.1, terminal_command=command(2.1, 0.0),
        ).failsafe_stop)
        self.assertTrue(self.arbiter.select(
            now=10.1, terminal_command=command(0.1, 1.1),
        ).failsafe_stop)


if __name__ == "__main__":
    unittest.main()
