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
        self.assertNotEqual(manager.state, UsvOnlyTask3State.FINISHED)
        manager.navigation_reached(now=12.0)
        self.assertEqual(manager.state, UsvOnlyTask3State.FINISHED)
        self.assertEqual(manager.status()["fsm_progress"], "FINISHED")
        self.assertEqual(manager.progress_report(), {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": "FINISHED",
            "error": None,
        })
        self.assertEqual(
            executor_state_to_fsm_progress(manager.status())["progress"],
            "FINISHED",
        )

    def test_finish_uses_valid_window_fallback_without_encoding(self):
        manager = self.make_manager()
        manager.state = UsvOnlyTask3State.AIM_WINDOW
        manager.bay_id = 3
        self.assertTrue(manager.aiming_ready(gimbal_ready=True, window_id=None, now=1.0))
        self.assertEqual(manager.window_id, 1)
        manager.state = UsvOnlyTask3State.FINISHED
        progress = manager.progress_report()
        self.assertNotIn("status_code", progress)
        self.assertNotIn("status_bits", progress)
        self.assertNotIn("report", progress)
        self.assertNotIn("bay_id", progress)
        self.assertNotIn("window_id", progress)

    def test_bay_color_is_ignored_until_each_observation_pose_is_reached(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.navigation_reached(now=1.0)
        manager.accept_dock_frame(self.stable_frame(1.0), Pose2D(0, 5, 0), now=1.0)
        manager.accept_dock_frame(self.stable_frame(2.0), Pose2D(0, 5, 0), now=2.0)
        manager.dock_verified(now=3.0)
        manager.navigation_reached(now=4.0)

        # First observation pose was reached by APPROACH_DOCK completion.
        self.assertTrue(manager.status()["bay_observation_ready"])
        self.assertFalse(manager.accept_bay_color("RED", 0.9, now=5.0))
        self.assertFalse(manager.status()["bay_observation_ready"])

        # A result received while moving to Bay 2 must not be accepted.
        self.assertFalse(manager.accept_bay_color("GREEN", 0.9, now=5.5))
        self.assertIsNone(manager.bay_id)
        manager.navigation_reached(now=6.0)
        self.assertTrue(manager.accept_bay_color("GREEN", 0.9, now=6.1))
        self.assertEqual(manager.bay_id, 2)

    def test_all_non_green_bays_enter_current_bay_as_best_effort_fallback(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.navigation_reached(now=1.0)
        manager.accept_dock_frame(self.stable_frame(1.0), Pose2D(0, 5, 0), now=1.0)
        manager.accept_dock_frame(self.stable_frame(2.0), Pose2D(0, 5, 0), now=2.0)
        manager.dock_verified(now=3.0)
        manager.navigation_reached(now=4.0)

        self.assertFalse(manager.accept_bay_color("RED", 0.9, now=5.0))
        manager.navigation_reached(now=6.0)
        self.assertFalse(manager.accept_bay_color("RED", 0.9, now=7.0))
        manager.navigation_reached(now=8.0)
        self.assertTrue(manager.accept_bay_color("RED", 0.9, now=9.0))

        self.assertEqual(manager.state, UsvOnlyTask3State.ENTER_BAY)
        self.assertEqual(manager.bay_id, 3)
        self.assertTrue(manager.status()["bay_fallback_used"])
        self.assertIn("ALL_BAYS_INSPECTED", manager.status()["bay_fallback_reason"])
        self.assertIsNotNone(manager.current_goal)

    def test_bay_timeout_exhaustion_falls_back_only_after_observation_arrival(self):
        timeouts = {
            UsvOnlyTask3State.INSPECT_BAYS: 1.0,
            UsvOnlyTask3State.RETRY: 1.0,
        }
        manager = self.make_manager(max_retries_per_state=0, state_timeouts_s=timeouts)
        self.begin(manager)
        manager.navigation_reached(now=1.0)
        manager.accept_dock_frame(self.stable_frame(1.0), Pose2D(0, 5, 0), now=1.0)
        manager.accept_dock_frame(self.stable_frame(2.0), Pose2D(0, 5, 0), now=2.0)
        manager.dock_verified(now=3.0)
        manager.navigation_reached(now=4.0)

        manager.update_health(health(5.1), Pose2D(0, 5, 0), now=5.1)
        manager.tick(now=5.1)

        self.assertEqual(manager.state, UsvOnlyTask3State.ENTER_BAY)
        self.assertEqual(manager.bay_id, 1)
        self.assertTrue(manager.status()["bay_fallback_used"])

    def test_dock_stability_does_not_mix_frames_between_search_stations(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.navigation_reached(now=1.0)
        self.assertFalse(manager.accept_dock_frame(
            self.stable_frame(1.0), Pose2D(0, 5, 0), now=1.0
        ))
        manager.navigation_reached(now=2.0)
        self.assertFalse(manager.accept_dock_frame(
            self.stable_frame(2.0), Pose2D(0, 5, 0), now=2.0
        ))
        self.assertTrue(manager.accept_dock_frame(
            self.stable_frame(3.0), Pose2D(0, 5, 0), now=3.0
        ))

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

    def test_paused_or_timed_out_spray_must_reaim_before_restarting_pump(self):
        timeouts = {
            UsvOnlyTask3State.SPRAY_WATER: 1.0,
            UsvOnlyTask3State.RETRY: 1.0,
        }
        manager = self.make_manager(state_timeouts_s=timeouts)
        manager.state = UsvOnlyTask3State.SPRAY_WATER
        manager.state_entered_at = 0.0
        manager.health = health(1.1)
        manager.tick(now=1.1)
        self.assertEqual(manager.state, UsvOnlyTask3State.RETRY)
        manager.retry(now=1.2)
        self.assertEqual(manager.state, UsvOnlyTask3State.AIM_WINDOW)

        manager.state = UsvOnlyTask3State.SPRAY_WATER
        manager.pause("OPERATOR_PAUSE", now=2.0)
        manager.health = health(2.1)
        self.assertTrue(manager.resume(now=2.1))
        self.assertEqual(manager.state, UsvOnlyTask3State.AIM_WINDOW)

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

class Task4IntegrationTests(unittest.TestCase):
    make_manager = UsvOnlyFlowTests.make_manager
    begin = UsvOnlyFlowTests.begin
    stable_frame = UsvOnlyFlowTests.stable_frame

    def zone(self, revision=1):
        return dict(run_id=7, revision=revision, active=True,
                    frame_id='robotx_rtk_ned',
                    center_ned={'north': 10, 'east': 5}, radius_m=2, clearance_m=1)

    def test_core_pause_is_idempotent_and_freezes_budget(self):
        manager = self.make_manager()
        self.begin(manager)
        manager._retry_counts[UsvOnlyTask3State.TRANSIT_TO_SEARCH] = 1
        checkpoint = manager.pause_for_core(now=4)
        self.assertEqual(checkpoint['resume_phase'], 'EXPLORING')
        self.assertEqual(manager.pause_for_core(now=99), checkpoint)
        manager.tick(now=1000)
        self.assertEqual(manager.state, UsvOnlyTask3State.PAUSED)
        self.assertFalse(manager.resume(now=1000))  # cannot bypass Core handshake
        manager.update_health(health(1000), Pose2D(2, 2, 0), now=1000)
        self.assertTrue(manager.resume_core(checkpoint['checkpoint_id'], motion_stopped=True, now=1000))
        self.assertEqual(manager.state_entered_at, 996)
        self.assertEqual(manager._retry_counts[UsvOnlyTask3State.TRANSIT_TO_SEARCH], 1)
        self.assertEqual(manager.state, UsvOnlyTask3State.TRANSIT_TO_SEARCH)

    def test_known_bay_reapproaches_and_requires_fresh_geometry(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.accept_dock_frame(self.stable_frame(1), Pose2D(0, 5, 0), now=1)
        manager.accept_dock_frame(self.stable_frame(2), Pose2D(0, 5, 0), now=2)
        manager.dock_verified(now=2)
        manager.navigation_reached(now=3)
        manager.accept_bay_color('GREEN', .9, now=3)
        checkpoint = manager.pause_for_core(now=4)
        self.assertEqual(checkpoint['resume_phase'], 'CORRECT_BAY_KNOWN')
        manager.update_health(health(100), Pose2D(0, 5, 0), now=100)
        manager.resume_core(checkpoint['checkpoint_id'], motion_stopped=True, now=100)
        self.assertEqual(manager.state, UsvOnlyTask3State.APPROACH_DOCK)
        manager.navigation_reached(now=101)
        with self.assertRaises(RuntimeError):
            manager.dock_verified(now=101)
        self.assertFalse(manager.accept_dock_frame(self.stable_frame(1), Pose2D(0, 5, 0), now=101))
        manager.accept_dock_frame(self.stable_frame(101), Pose2D(0, 5, 0), now=101)
        self.assertTrue(manager.accept_dock_frame(self.stable_frame(102), Pose2D(0, 5, 0), now=102))
        manager.dock_verified(now=102)
        self.assertEqual(manager.state, UsvOnlyTask3State.INSPECT_BAYS)
        self.assertEqual(manager.bay_id, 1)

    def test_completed_watering_never_restarts_pump_on_resume(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.state = UsvOnlyTask3State.SPRAY_WATER
        manager.bay_id, manager.window_id = 2, 1
        manager.spray_complete(now=2)
        checkpoint = manager.pause_for_core(now=3)
        self.assertEqual(checkpoint['resume_phase'], 'RETURNING')
        manager.update_health(health(100), Pose2D(2, 2, 0), now=100)
        self.assertFalse(manager.resume_core(checkpoint['checkpoint_id'], motion_stopped=True, now=100))
        self.assertTrue(manager.resume_core(checkpoint['checkpoint_id'], motion_stopped=True,
                                            dock_exit_completed=True, now=100))
        self.assertEqual(manager.state, UsvOnlyTask3State.FINISHED)

    def test_all_clear_does_not_resume_core(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.apply_navigation_constraint(self.zone(), now=1)
        checkpoint = manager.pause_for_core(now=2)
        manager.apply_navigation_constraint(dict(run_id=7, revision=2, active=False), now=100)
        self.assertEqual(manager.state, UsvOnlyTask3State.PAUSED)
        self.assertEqual(manager.core_checkpoint, checkpoint)
        self.assertIsNone(manager.current_goal)

    def test_constraint_survives_restart_and_generates_bounded_detour(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.apply_navigation_constraint(self.zone(), now=1)
        manager.start(2)
        self.assertIsNotNone(manager.navigation_constraint.zone)
        manager.begin(corners(), Pose2D(1, 1, 0), health(2), now=2)
        original_goal = manager.current_goal
        self.assertTrue(manager.guard_current_goal(now=2))
        self.assertIsNone(manager.navigation_hold_reason)
        self.assertIsNotNone(manager.current_goal)
        self.assertNotEqual(manager.current_goal, original_goal)
        self.assertGreater(manager.status()['keep_out_detour_waypoints_remaining'], 0)
        self.assertIn(manager.status()['keep_out_detour_side'],
                      {'CLOCKWISE', 'COUNTERCLOCKWISE'})
        self.assertFalse(manager.releases_control)

        detour_count = manager.status()['keep_out_detour_waypoints_remaining']
        for index in range(detour_count):
            manager.current_pose = manager.current_goal
            manager.health = health(3 + index)
            manager.navigation_reached(now=3 + index)
            self.assertEqual(manager.state, UsvOnlyTask3State.TRANSIT_TO_SEARCH)
        self.assertEqual(manager.current_goal, original_goal)
        manager.current_pose = original_goal
        manager.health = health(100)
        manager.navigation_reached(now=100)
        self.assertEqual(manager.state, UsvOnlyTask3State.SEARCH_PRIMARY)

    def test_no_boundary_safe_detour_releases_control_without_using_retry(self):
        manager = self.make_manager()
        self.begin(manager)
        zone = self.zone()
        zone.update(radius_m=4.9, clearance_m=0.0)
        manager.apply_navigation_constraint(zone, now=1)
        before = dict(manager._retry_counts)
        self.assertFalse(manager.guard_current_goal(now=1))
        self.assertEqual(manager.navigation_hold_reason,
                         'KEEP_OUT_NO_DETOUR_INSIDE_GEOFENCE')
        self.assertTrue(manager.releases_control)
        manager.tick(now=10000)
        self.assertEqual(manager._retry_counts, before)

    def test_live_pose_inside_zone_generates_exit_then_resumes(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.apply_navigation_constraint(self.zone(), now=1)
        mission_goal = manager._mission_goal
        manager.update_health(health(2), Pose2D(10, 5, 0), now=2)
        self.assertEqual(manager.status()['keep_out_mode'], 'EXITING')
        exit_goal = manager.current_goal
        self.assertIsNotNone(exit_goal)
        self.assertNotEqual(exit_goal, mission_goal)
        manager.update_health(health(3), exit_goal, now=3)
        self.assertFalse(manager.status()['keep_out_inside_protected_zone'])
        self.assertNotEqual(manager.status()['keep_out_mode'], 'EXITING')

    def test_keep_out_during_spray_exits_with_actuators_suspended_then_revalidates(self):
        manager = self.make_manager()
        self.begin(manager)
        manager.navigation_reached(now=1)
        manager.accept_dock_frame(self.stable_frame(1), Pose2D(0, 5, 0), now=1)
        manager.accept_dock_frame(self.stable_frame(2), Pose2D(0, 5, 0), now=2)
        manager.dock_verified(now=3)
        manager.navigation_reached(now=4)
        manager.accept_bay_color('GREEN', .9, now=5)
        manager.navigation_reached(now=6)
        manager.hold_confirmed(now=7)
        manager.aiming_ready(gimbal_ready=True, window_id=1, now=8)
        manager.update_health(health(9), Pose2D(10, 5, 0), now=9)

        manager.apply_navigation_constraint(self.zone(), now=9)
        self.assertEqual(manager.state, UsvOnlyTask3State.SPRAY_WATER)
        self.assertTrue(manager.keep_out_operation_suspended)
        self.assertEqual(manager.status()['keep_out_suspended_state'], 'SPRAY_WATER')
        checkpoint = manager.keep_out_checkpoint
        self.assertIsNotNone(checkpoint)
        self.assertEqual(checkpoint['saved_step'], 'SPRAY_WATER')
        self.assertEqual(checkpoint['context']['bay_id'], 1)
        self.assertEqual(checkpoint['context']['window_id'], 1)
        self.assertFalse(checkpoint['context']['watering_completed'])
        self.assertEqual(manager.status()['keep_out_checkpoint_id'],
                         checkpoint['checkpoint_id'])
        self.assertEqual(manager.status()['keep_out_mode'], 'EXITING')
        exit_goal = manager.current_goal
        self.assertIsNotNone(exit_goal)
        self.assertFalse(manager.releases_control)

        # A long official closure does not consume the spray retry budget.
        manager.tick(now=1000)
        self.assertEqual(manager.state, UsvOnlyTask3State.SPRAY_WATER)
        self.assertEqual(manager._retry_counts, {})

        manager.update_health(health(1001), exit_goal, now=1001)
        self.assertTrue(manager.releases_control)
        self.assertIsNone(manager.current_goal)
        self.assertTrue(manager.keep_out_operation_suspended)

        manager.apply_navigation_constraint(
            dict(run_id=7, revision=2, active=False), now=1002
        )
        self.assertFalse(manager.keep_out_operation_suspended)
        self.assertIsNone(manager.keep_out_checkpoint)
        self.assertEqual(manager.state, UsvOnlyTask3State.APPROACH_DOCK)
        self.assertTrue(manager._needs_dock_revalidation)
        self.assertFalse(manager.watering_completed)
