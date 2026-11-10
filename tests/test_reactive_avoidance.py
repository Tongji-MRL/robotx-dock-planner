import math
import unittest

from usv_reactive_avoidance import (
    AvoidanceAction,
    ReactiveAvoidanceConfig,
    ReactiveAvoidanceCore,
    SteeringSide,
)


def scan_with_cluster(angle_deg=0.0, distance=2.0, *, count=181):
    values = [math.inf] * count
    center = int(round(angle_deg + 90.0))
    for index in range(center - 2, center + 3):
        values[index] = distance
    return values


class ReactiveAvoidanceTest(unittest.TestCase):
    def setUp(self):
        self.core = ReactiveAvoidanceCore(ReactiveAvoidanceConfig(
            trigger_frames=2, release_frames=2,
        ))
        self.args = dict(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.1,
            range_max=12.0,
        )

    def evaluate(self, ranges, stamp, **kwargs):
        return self.core.evaluate(ranges, timestamp=stamp, **self.args, **kwargs)

    def test_obstacle_requires_consecutive_frames_then_requests_replan(self):
        scan = scan_with_cluster(distance=2.0)
        self.assertEqual(self.evaluate(scan, 1.0).action, AvoidanceAction.PROCEED)
        result = self.evaluate(scan, 1.1)
        self.assertEqual(result.action, AvoidanceAction.REPLAN)
        self.assertIn(result.steering_side, (SteeringSide.LEFT, SteeringSide.RIGHT))

    def test_emergency_cluster_holds(self):
        scan = scan_with_cluster(distance=0.6)
        self.evaluate(scan, 1.0)
        result = self.evaluate(scan, 1.1)
        self.assertEqual(result.action, AvoidanceAction.HOLD)
        self.assertEqual(result.speed_scale, 0.0)

    def test_single_ray_noise_is_ignored(self):
        scan = [math.inf] * 181
        scan[90] = 0.5
        self.assertEqual(self.evaluate(scan, 1.0).action, AvoidanceAction.PROCEED)
        self.assertEqual(self.evaluate(scan, 1.1).action, AvoidanceAction.PROCEED)

    def test_terminal_docking_bypasses_generic_avoidance(self):
        scan = scan_with_cluster(distance=0.4)
        result = self.evaluate(scan, 1.0, terminal_docking=True)
        self.assertEqual(result.action, AvoidanceAction.PROCEED)
        self.assertIn("terminal docking", result.reason)

    def test_release_hysteresis_clears_direction_lock(self):
        blocked = scan_with_cluster(distance=2.0)
        clear = [math.inf] * 181
        self.evaluate(blocked, 1.0)
        self.assertEqual(self.evaluate(blocked, 1.1).action, AvoidanceAction.REPLAN)
        self.assertNotEqual(self.evaluate(clear, 1.2).action, AvoidanceAction.PROCEED)
        self.assertEqual(self.evaluate(clear, 1.3).action, AvoidanceAction.PROCEED)


if __name__ == "__main__":
    unittest.main()
