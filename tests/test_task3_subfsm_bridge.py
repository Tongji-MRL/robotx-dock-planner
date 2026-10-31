import unittest

from dock_planner.task3_status_codec import CommonError, PlannerCommand, Task3Step, Task4Step, TaskCode
from dock_planner.task3_subfsm_bridge import ResumePolicy, Task3SubFSMBridge


class Task3SubFSMBridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = Task3SubFSMBridge(clock_ms=lambda: 123456)
        self.bridge.handle_master_command({
            "command_id": "start-1",
            "run_id": 42,
            "command": "START_TASK3",
            "payload": {"context": {"dock_id": "dock-A"}},
        })

    def test_normal_progress_uses_command_000(self):
        report = self.bridge.update_task3_step(Task3Step.SEARCH_DOCK)
        self.assertEqual(report.status.command, PlannerCommand.NONE)
        self.assertTrue(report.status.bit_string().endswith("000"))

    def test_task4_from_search_replans_approach_after_resume(self):
        self.bridge.update_task3_step(
            Task3Step.SEARCH_DOCK,
            context={"dock_id": "dock-A", "arc_index": 5},
        )
        preempt = self.bridge.handle_master_command({
            "command_id": "task4-1",
            "run_id": 42,
            "command": "TASK4_GOTO",
            "payload": {"target_position": {"frame_id": "ned", "north": 20.0, "east": 7.0}},
        })
        self.assertEqual(preempt.status.task, TaskCode.TASK4)
        self.assertEqual(preempt.status.step, Task4Step.GO_TO_TARGET)
        self.assertEqual(preempt.status.command, PlannerCommand.ACK)
        self.assertEqual(self.bridge.checkpoint.saved_step, Task3Step.SEARCH_DOCK)
        self.assertEqual(self.bridge.checkpoint.resume_step, Task3Step.APPROACH_DOCK)

        reached = self.bridge.task4_target_reached(
            actual_position={"frame_id": "ned", "north": 20.1, "east": 7.1}
        )
        self.assertEqual(reached.status.command, PlannerCommand.REPORT_TASK_COMPLETE)

        resumed = self.bridge.handle_master_command({
            "command_id": "resume-1",
            "run_id": 42,
            "command": "RESUME_TASK3",
            "payload": {"checkpoint_id": self.bridge.checkpoint.checkpoint_id},
        })
        self.assertEqual(resumed.status.task, TaskCode.TASK3)
        self.assertEqual(resumed.status.step, Task3Step.APPROACH_DOCK)
        self.assertEqual(
            resumed.payload["resume_policy"],
            "REPLAN_APPROACH_FROM_CURRENT_POSE",
        )

    def test_task4_during_spraying_requests_pump_off_and_redock(self):
        self.bridge.update_task3_step(
            Task3Step.SPRAYING,
            context={"bay_id": 2, "window_id": 1},
        )
        report = self.bridge.handle_master_command({
            "command_id": "task4-spray",
            "run_id": 42,
            "command": "TASK4_GOTO",
            "payload": {"target_position": {"frame_id": "wgs84", "latitude": 1.2, "longitude": 103.8}},
        })
        self.assertIn("PUMP_OFF", report.payload["safe_actions"])
        self.assertIn("LEAVE_DOCK_SAFELY_FOR_TASK4", report.payload["safe_actions"])
        self.assertEqual(self.bridge.checkpoint.resume_step, Task3Step.APPROACH_DOCK)
        self.assertTrue(self.bridge.checkpoint.context["reset_watering_confirmation"])

    def test_waiting_for_uav_resumes_by_requerying_uav_state(self):
        self.bridge.update_task3_step(
            Task3Step.WAIT_UAV_DELIVERY,
            context={"request_id": "delivery-9"},
        )
        self.bridge.handle_master_command({
            "command_id": "task4-uav",
            "run_id": 42,
            "command": "TASK4_GOTO",
            "payload": {"target_position": {"north": 1.0, "east": 2.0}},
        })
        self.assertEqual(self.bridge.checkpoint.resume_step, Task3Step.WAIT_UAV_DELIVERY)
        self.assertTrue(self.bridge.checkpoint.context["requery_uav_delivery_state"])

    def test_color_sequence_uses_structured_payload(self):
        report = self.bridge.report_color_sequence(
            resource_color="red",
            destination_color="blue",
            confidence=0.91,
        )
        self.assertEqual(report.message_type, "TASK3_COLOR_SEQUENCE")
        self.assertEqual(report.payload["resource_color"], "RED")
        self.assertEqual(report.payload["destination_color"], "BLUE")
        self.assertEqual(report.status.command, PlannerCommand.NONE)

    def test_known_fault_contains_fixed_decoder_and_recovery_fields(self):
        report = self.bridge.update_task3_step(
            Task3Step.SEARCH_DOCK,
            error=0b1000,
            detail="no point cloud received for 1.5 s",
            fault_source="lidar_bridge",
            detail_code="NO_MESSAGE_1500MS",
            retry_count=2,
        )
        self.assertEqual(report.payload["fault"]["name"], "LIDAR_NO_DATA")
        self.assertEqual(report.payload["fault"]["recovery_action"], "WAIT_FOR_RECOVERY")
        self.assertEqual(report.payload["fault"]["retry_count"], 2)

    def test_unknown_fault_requires_detail_for_planner_diagnostics(self):
        self.bridge.update_task3_step(Task3Step.SEARCH_DOCK)
        with self.assertRaises(ValueError):
            self.bridge.status_report(error=CommonError.UNKNOWN_ERROR)
        report = self.bridge.status_report(
            error=CommonError.UNKNOWN_ERROR,
            detail="unexpected detector exception",
            fault_source="dock_detector",
            detail_code="EXCEPTION",
        )
        self.assertEqual(report.payload["fault"]["name"], "UNKNOWN_ERROR")
        self.assertEqual(report.payload["fault"]["recovery_action"], "SAFE_HOLD")

    def test_resume_before_target_reached_is_rejected(self):
        self.bridge.update_task3_step(Task3Step.APPROACH_DOCK)
        self.bridge.handle_master_command({
            "command_id": "task4-early",
            "run_id": 42,
            "command": "TASK4_GOTO",
            "payload": {"target_position": {"north": 1.0, "east": 2.0}},
        })
        with self.assertRaises(RuntimeError):
            self.bridge.handle_master_command({
                "command_id": "resume-too-early",
                "run_id": 42,
                "command": "RESUME_TASK3",
                "payload": {},
            })

    def test_task4_completion_waits_for_planner_task_selection(self):
        self.bridge.update_task3_step(
            Task3Step.SEARCH_DOCK,
            context={"dock_id": "dock-A", "arc_index": 3},
        )
        self.bridge.handle_master_command({
            "command_id": "task4-select",
            "run_id": 42,
            "command": "TASK4_GOTO",
            "payload": {"target_position": {"north": 5.0, "east": 6.0}},
        })
        checkpoint_id = self.bridge.checkpoint.checkpoint_id
        report = self.bridge.task4_target_reached(
            actual_position={"north": 5.0, "east": 6.0}
        )

        self.assertEqual(report.message_type, "TASK4_TARGET_REACHED")
        self.assertEqual(report.status.command, PlannerCommand.REPORT_TASK_COMPLETE)
        self.assertEqual(self.bridge.active_task, TaskCode.TASK4)
        self.assertEqual(self.bridge.task4_step, Task4Step.WAIT_RESUME)
        self.assertEqual(self.bridge.checkpoint.checkpoint_id, checkpoint_id)
        self.assertEqual(self.bridge.task3_step, Task3Step.SEARCH_DOCK)

    def test_safe_stop_ack_is_not_reported_as_an_error(self):
        report = self.bridge.handle_master_command({
            "command_id": "end-1",
            "run_id": 42,
            "command": "SAFE_STOP",
            "payload": {},
        })
        self.assertEqual(report.status.error, CommonError.NONE)
        self.assertEqual(report.payload["fsm_progress"], "TASK3_END_ACK")
        self.assertTrue(report.payload["return_to_start_required"])

    def test_task3_completion_carries_fsm_finish_progress(self):
        report = self.bridge.report_task3_complete(result={"bay_id": 2})
        self.assertEqual(report.message_type, "TASK3_COMPLETE")
        self.assertEqual(report.payload["fsm_progress"], "TASK3_FINISH")
        self.assertEqual(report.status.command, PlannerCommand.REPORT_TASK_COMPLETE)

    def test_every_task3_step_has_an_explicit_task4_resume_rule(self):
        expected = {
            Task3Step.READY: (Task3Step.READY, ResumePolicy.CONTINUE_SAME_STEP),
            Task3Step.WAIT_DOCK_OBSERVATION: (
                Task3Step.WAIT_DOCK_OBSERVATION,
                ResumePolicy.WAIT_FOR_FRESH_DOCK_DATA,
            ),
            Task3Step.APPROACH_DOCK: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REPLAN_APPROACH_FROM_CURRENT_POSE,
            ),
            Task3Step.SEARCH_DOCK: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REPLAN_APPROACH_FROM_CURRENT_POSE,
            ),
            Task3Step.CHECK_BAY_COLOR: (
                Task3Step.SEARCH_DOCK,
                ResumePolicy.REOBSERVE_DOCK_AND_BAY,
            ),
            Task3Step.NEXT_BAY: (
                Task3Step.SEARCH_DOCK,
                ResumePolicy.REOBSERVE_DOCK_AND_BAY,
            ),
            Task3Step.TERMINAL_DOCKING: (
                Task3Step.SEARCH_DOCK,
                ResumePolicy.REOBSERVE_DOCK_AND_BAY,
            ),
            Task3Step.DOCKED: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REDOCK_AND_RESTART_WATERING,
            ),
            Task3Step.WAIT_RED_WINDOW: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REDOCK_AND_RESTART_WATERING,
            ),
            Task3Step.AIMING: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REDOCK_AND_RESTART_WATERING,
            ),
            Task3Step.SPRAYING: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REDOCK_AND_RESTART_WATERING,
            ),
            Task3Step.VERIFY_GREEN: (
                Task3Step.APPROACH_DOCK,
                ResumePolicy.REDOCK_AND_RESTART_WATERING,
            ),
            Task3Step.COLOR_SEQUENCE: (
                Task3Step.COLOR_SEQUENCE,
                ResumePolicy.REQUIRE_FRESH_COLOR_SEQUENCE,
            ),
            Task3Step.WAIT_UAV_DELIVERY: (
                Task3Step.WAIT_UAV_DELIVERY,
                ResumePolicy.REQUERY_UAV_DELIVERY_STATE,
            ),
            Task3Step.EXIT_RETURN: (
                Task3Step.EXIT_RETURN,
                ResumePolicy.REPLAN_RETURN,
            ),
            Task3Step.COMPLETE: (
                Task3Step.COMPLETE,
                ResumePolicy.NO_ACTION_TASK_ALREADY_COMPLETE,
            ),
        }
        for index, (step, (resume_step, policy)) in enumerate(expected.items(), start=1):
            with self.subTest(step=step.name):
                bridge = Task3SubFSMBridge(clock_ms=lambda: 123456)
                bridge.handle_master_command({
                    "command_id": f"start-{index}",
                    "run_id": 42,
                    "command": "START_TASK3",
                    "payload": {},
                })
                bridge.update_task3_step(step, context={"marker": step.name})
                bridge.handle_master_command({
                    "command_id": f"task4-{index}",
                    "run_id": 42,
                    "command": "TASK4_GOTO",
                    "payload": {"target_position": {"north": 1.0, "east": 2.0}},
                })
                self.assertEqual(bridge.checkpoint.resume_step, resume_step)
                self.assertEqual(bridge.checkpoint.resume_policy, policy)


if __name__ == "__main__":
    unittest.main()
