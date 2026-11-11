import math
import unittest

from dock_planner import (
    DockEntranceObservationNED,
    GeofenceSearchConfig,
    GpsOrigin,
    NEDPoint,
    Pose2D,
    StableEntranceConfig,
    executor_state_to_fsm_progress,
    geodetic_point_to_ned,
)
from usv_only_task3.flow import (
    UsvHealth,
    UsvOnlyTask3Config,
    UsvOnlyTask3FlowManager,
    UsvOnlyTask3State,
)


def corners():
    # Deliberately unordered.
    return (
        NEDPoint(20.0, 10.0, 0.0), NEDPoint(0.0, 0.0, 0.0),
        NEDPoint(20.0, 0.0, 0.0), NEDPoint(0.0, 10.0, 0.0),
    )


def health(stamp=0.0, **changes):
    values = dict(pose_timestamp=stamp, frame_id="robotx_rtk_ned", mavros_connected=True)
    values.update(changes)
    return UsvHealth(**values)


class UsvOnlyFlowTests(unittest.TestCase):
    def make_manager(self, **changes):
        values = dict(
            pose_timeout_s=5.0,
            max_retries_per_state=1,
            max_search_passes=2,
            geofence=GeofenceSearchConfig(safety_inset_m=1.0, station_spacing_m=100.0),
            stable_entrances=StableEntranceConfig(required_frame_hits=2, min_entrances=3),
        )
        values.update(changes)
        return UsvOnlyTask3FlowManager(UsvOnlyTask3Config(**values))

    def begin(self, manager, now=0.0):
        manager.start(now)
        manager.begin(corners(), Pose2D(1.0, 1.0, 0.0), health(now), now=now)

    def stable_frame(self, stamp):
        return tuple(
            DockEntranceObservationNED(
                10.0, east, 0.0, 0.9, stamp, "test", dock_id="field_dock"
            )
            for east in (2.0, 5.0, 8.0)
        )

    def test_full_flow_has_distinct_complete_and_fsm_finish(self):
        manager = self.make_manager()
        self.begin(manager)
        self.assertEqual(manager.state, UsvOnlyTask3State.TRANSIT_TO_SEARCH)
        manager.navigation_reached(now=1.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.SEARCH_PRIMARY)
        self.assertFalse(manager.accept_dock_frame(self.stable_frame(1.0), Pose2D(0, 5, 0), now=1.0))
        self.assertTrue(manager.accept_dock_frame(self.stable_frame(2.0), Pose2D(0, 5, 0), now=2.0))
        self.assertEqual(manager.state, UsvOnlyTask3State.VERIFY_DOCK)
        manager.dock_verified(now=3.0)
        manager.navigation_reached(now=4.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.INSPECT_BAYS)
        manager.accept_bay_color("GREEN", 0.9, now=5.0)
        self.assertEqual(manager.bay_id, 1)
        manager.navigation_reached(now=6.0)
        manager.hold_confirmed(now=7.0)
        self.assertFalse(manager.aiming_ready(gimbal_ready=False, window_id=2, now=8.0))
        self.assertEqual(manager.state, UsvOnlyTask3State.AIM_WINDOW)
        self.assertTrue(manager.aiming_ready(gimbal_ready=True, window_id=2, now=9.0))
        manager.spray_complete(now=10.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.REPORT_RESULT)
        manager.result_reported(now=11.0)
        self.assertNotEqual(manager.state, UsvOnlyTask3State.COMPLETE)
        manager.navigation_reached(now=12.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.COMPLETE)
        self.assertEqual(manager.status()["fsm_progress"], "TASK3_FINISH")
        self.assertEqual(manager.progress_report(), {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": "TASK3_FINISH",
            "error": None,
            "bay_id": 1,
            "window_id": 2,
            "report": {"bay_id": 1, "window_id": 2},
        })
        self.assertEqual(
            executor_state_to_fsm_progress(manager.status())["progress"],
            "TASK3_FINISH",
        )

    def test_finish_uses_valid_window_fallback_without_encoding(self):
        manager = self.make_manager()
        manager.state = UsvOnlyTask3State.AIM_WINDOW
        manager.bay_id = 3
        self.assertTrue(manager.aiming_ready(gimbal_ready=True, window_id=None, now=1.0))
        self.assertEqual(manager.window_id, 1)
        manager.state = UsvOnlyTask3State.COMPLETE
        progress = manager.progress_report()
        self.assertNotIn("status_code", progress)
        self.assertNotIn("status_bits", progress)
        self.assertEqual(progress["report"], {"bay_id": 3, "window_id": 1})

    def test_primary_then_secondary_then_one_bounded_search_retry(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.navigation_reached(now=1.0)
        manager.navigation_reached(now=2.0)
        manager.navigation_reached(now=2.5)
        self.assertEqual(manager.state, UsvOnlyTask3State.SEARCH_SECONDARY)
        manager.navigation_reached(now=3.0)
        manager.navigation_reached(now=3.5)
        self.assertEqual(manager.state, UsvOnlyTask3State.RETRY)
        self.assertTrue(manager.retry(now=4.0))
        self.assertEqual(manager.state, UsvOnlyTask3State.TRANSIT_TO_SEARCH)
        manager.navigation_reached(now=5.0)
        manager.navigation_reached(now=6.0)
        manager.navigation_reached(now=6.5)
        manager.navigation_reached(now=7.0)
        manager.navigation_reached(now=7.5)
        self.assertEqual(manager.state, UsvOnlyTask3State.ERROR)
        self.assertIn("DOCK_NOT_FOUND", manager.error)

    def test_timeout_retries_once_then_errors(self):
        timeouts = {UsvOnlyTask3State.TRANSIT_TO_SEARCH: 1.0, UsvOnlyTask3State.RETRY: 1.0}
        manager = self.make_manager(state_timeouts_s=timeouts)
        self.begin(manager)
        manager.tick(now=1.1)
        self.assertEqual(manager.state, UsvOnlyTask3State.RETRY)
        manager.retry(now=1.2)
        manager.tick(now=2.3)
        self.assertEqual(manager.state, UsvOnlyTask3State.ERROR)
        self.assertIn("retry limit exhausted", manager.error)

    def test_abort_immediately_releases_goal(self):
        manager = self.make_manager()
        self.begin(manager)
        self.assertIsNotNone(manager.current_goal)
        manager.abort("MASTER_ABORT", now=1.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.ABORTED)
        self.assertIsNone(manager.current_goal)
        self.assertTrue(manager.releases_control)

    def test_stale_pose_pauses_and_fresh_pose_resumes(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.tick(now=6.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.PAUSED)
        self.assertEqual(manager.detail, "RTK_POSE_STALE")
        self.assertIsNone(manager.current_goal)
        manager.update_health(health(6.0), Pose2D(2, 2, 0), now=6.0)
        self.assertTrue(manager.resume(now=6.0))
        self.assertEqual(manager.state, UsvOnlyTask3State.TRANSIT_TO_SEARCH)

    def test_frame_mismatch_never_releases_first_goal(self):
        manager = self.make_manager()
        manager.start(0.0)
        manager.begin(
            corners(), Pose2D(0, 0, 0), health(0.0, frame_id="fcu_local_ned"), now=0.0
        )
        self.assertEqual(manager.state, UsvOnlyTask3State.WAIT_INPUTS)
        self.assertIsNone(manager.current_goal)
        self.assertEqual(manager.detail, "FRAME_MISMATCH")

    def test_gps_corner_can_be_expressed_in_shared_rtk_ned(self):
        origin = GpsOrigin(31.0, 121.0, 5.0)
        point = geodetic_point_to_ned(GpsOrigin(31.00001, 121.00001, 5.0), origin)
        self.assertGreater(point.north, 1.0)
        self.assertGreater(point.east, 0.9)
        self.assertAlmostEqual(point.down, 0.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
