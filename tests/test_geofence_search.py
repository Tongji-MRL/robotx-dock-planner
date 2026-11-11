import math
import unittest

from dock_planner import (
    DockEntranceObservationNED,
    GeofenceSearchConfig,
    NEDPoint,
    Pose2D,
    StableEntranceConfig,
    plan_geofence_search,
    stabilize_entrance_layout,
)


class GeofenceSearchTests(unittest.TestCase):
    def test_plans_inset_primary_and_fallback_lines_from_unordered_corners(self):
        corners = (
            NEDPoint(20.0, 10.0, 0.0),
            NEDPoint(0.0, 0.0, 0.0),
            NEDPoint(20.0, 0.0, 0.0),
            NEDPoint(0.0, 10.0, 0.0),
        )
        plan = plan_geofence_search(
            corners,
            GeofenceSearchConfig(safety_inset_m=2.0, station_spacing_m=4.0),
        )

        self.assertAlmostEqual(plan.centroid.north, 10.0)
        self.assertAlmostEqual(plan.centroid.east, 5.0)
        # The long centreline covers this 20 x 10 rectangle first.
        self.assertGreater(plan.primary.length_m, plan.fallback.length_m)
        self.assertEqual(len(plan.primary.stations), 5)
        for pose in plan.primary.stations + plan.fallback.stations:
            self.assertGreaterEqual(pose.x, 0.0)
            self.assertLessEqual(pose.x, 20.0)
            self.assertGreaterEqual(pose.y, 0.0)
            self.assertLessEqual(pose.y, 10.0)

    def test_three_stable_entrances_are_oriented_sorted_and_given_observation_poses(self):
        observer = Pose2D(0.0, 0.0, 0.0)
        heading = math.radians(35.0)
        forward = (math.cos(heading), math.sin(heading))
        right = (-forward[1], forward[0])
        frames = []
        for frame_index in range(4):
            frame = []
            # Deliberately publish right-to-left and reverse the raw line
            # heading.  The resolver must orient and order it geometrically.
            for lateral in (3.0, 0.0, -3.0):
                frame.append(DockEntranceObservationNED(
                    entrance_center_north_m=forward[0] * 8.0 + right[0] * lateral + 0.02 * frame_index,
                    entrance_center_east_m=forward[1] * 8.0 + right[1] * lateral - 0.01 * frame_index,
                    entrance_heading_rad=heading + math.pi,
                    confidence=0.8,
                    timestamp=100.0 + frame_index,
                    source="synthetic",
                    dock_id="dock",
                ))
            frames.append(frame)

        layout = stabilize_entrance_layout(
            frames,
            observer,
            StableEntranceConfig(
                required_frame_hits=3,
                min_entrances=3,
                dock_depth_m=6.0,
                bay_observation_standoff_m=4.0,
            ),
        )

        self.assertIsNotNone(layout)
        assert layout is not None
        self.assertEqual([item.bay_id for item in layout.bays], [1, 2, 3])
        self.assertTrue(all(hit == 4 for hit in layout.frame_hits))
        self.assertAlmostEqual(layout.inward_heading_rad, heading, delta=0.03)
        self.assertAlmostEqual(layout.coarse_center_ned.north, forward[0] * 11.0 + 0.03, delta=0.08)
        self.assertAlmostEqual(layout.coarse_center_ned.east, forward[1] * 11.0 - 0.015, delta=0.08)
        for bay in layout.bays:
            distance = math.hypot(
                bay.entrance.entrance_center_north_m - bay.observation_pose.x,
                bay.entrance.entrance_center_east_m - bay.observation_pose.y,
            )
            self.assertAlmostEqual(distance, 4.0)

    def test_requires_repeated_three_entrance_layout(self):
        observation = DockEntranceObservationNED(
            5.0, 0.0, 0.0, 0.9, 1.0, "synthetic", dock_id="dock",
        )
        self.assertIsNone(stabilize_entrance_layout(
            ((observation,), (observation,)),
            Pose2D(0.0, 0.0, 0.0),
        ))


if __name__ == "__main__":
    unittest.main()
