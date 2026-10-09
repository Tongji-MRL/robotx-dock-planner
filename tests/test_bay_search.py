import unittest

from dock_planner import (
    BayCandidateAction,
    BayDisposition,
    BayVisitConfig,
    BayVisitTracker,
    DockEntranceObservationNED,
)


def entrance(north, east=0.0, *, timestamp=10.0):
    return DockEntranceObservationNED(
        north,
        east,
        0.0,
        0.9,
        timestamp,
        "test",
        dock_id="dock_1",
    )


class BayVisitTrackerTests(unittest.TestCase):
    def test_nearby_observations_are_associated_with_one_bay(self):
        tracker = BayVisitTracker(BayVisitConfig(association_distance_m=0.5))
        first = tracker.observe(entrance(1.0))
        second = tracker.observe(entrance(1.2, 0.1, timestamp=11.0))
        self.assertEqual(first.action, BayCandidateAction.INSPECT)
        self.assertEqual(second.bay_index, first.bay_index)
        self.assertEqual(len(tracker.records), 1)
        self.assertEqual(tracker.records[0].observation_count, 2)

    def test_rejected_bay_is_skipped_when_seen_again(self):
        tracker = BayVisitTracker()
        decision = tracker.observe(entrance(1.0))
        tracker.mark_color_rejected(decision.bay_index)
        repeated = tracker.observe(entrance(1.1, timestamp=11.0))
        self.assertEqual(repeated.action, BayCandidateAction.SKIP)
        self.assertEqual(
            tracker.records[0].disposition,
            BayDisposition.COLOR_REJECTED,
        )

    def test_three_unavailable_bays_exhaust_search(self):
        tracker = BayVisitTracker(BayVisitConfig(expected_bay_count=3))
        final = None
        for index, north in enumerate((1.0, 3.0, 5.0)):
            observed = tracker.observe(entrance(north, timestamp=10.0 + index))
            self.assertEqual(observed.action, BayCandidateAction.INSPECT)
            final = tracker.mark_color_rejected(observed.bay_index)
        self.assertEqual(final.action, BayCandidateAction.SEARCH_EXHAUSTED)
        self.assertTrue(tracker.search_exhausted)

    def test_terminal_failure_prevents_repeating_same_bay(self):
        tracker = BayVisitTracker(BayVisitConfig(max_terminal_failures_per_bay=1))
        observed = tracker.observe(entrance(2.0))
        tracker.mark_terminal_failure(observed.bay_index)
        repeated = tracker.observe(entrance(2.1, timestamp=11.0))
        self.assertEqual(repeated.action, BayCandidateAction.SKIP)
        self.assertEqual(
            tracker.records[0].disposition,
            BayDisposition.TERMINAL_FAILED,
        )

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            BayVisitConfig(expected_bay_count=0)
        with self.assertRaises(ValueError):
            BayVisitConfig(association_distance_m=0.0)
        with self.assertRaises(ValueError):
            BayVisitConfig(max_terminal_failures_per_bay=0)


if __name__ == "__main__":
    unittest.main()
