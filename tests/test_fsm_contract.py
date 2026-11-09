import unittest

from dock_planner.fsm_contract import (
    FSM_TASK3_FINISH_PROGRESS,
    correct_bay_to_fsm_progress,
    docking_result_to_fsm_message,
    executor_state_to_fsm_progress,
    normalize_fsm_task_command,
    sprayer_window_to_fsm_progress,
    safe_stop_ack_progress,
    task4_ack_progress,
    validate_fsm_progress,
)


class FsmContractTests(unittest.TestCase):
    def test_task3_begin_uses_mission_planner_shape(self):
        command = normalize_fsm_task_command({
            "vehicle": "usv",
            "task": "TASK3",
            "progress": None,
            "error": None,
            "command": "begin",
        })
        self.assertEqual(command["command"], "START_TASK3")
        self.assertEqual(command["task"], "TASK3")

    def test_map_and_task1_are_left_to_their_modules(self):
        self.assertIsNone(normalize_fsm_task_command({
            "vehicle": "usv",
            "task": "MAP",
            "progress": None,
            "error": None,
            "command": "begin",
        }))

    def test_task4_begin_maps_to_goto(self):
        command = normalize_fsm_task_command({
            "vehicle": "usv",
            "task": "TASK4",
            "progress": None,
            "error": None,
            "command": {
                "type": "TASK4_BEGIN",
                "command_seq": 7,
                "target_vehicle": "usv",
                "position": {"frame_id": "ned", "north": 1.0, "east": 2.0},
            },
        })
        self.assertEqual(command["command"], "TASK4_GOTO")
        self.assertEqual(command["payload"]["command_seq"], 7)
        self.assertEqual(task4_ack_progress(command)["progress"], "TASK4_ACK")

    def test_end_command_gets_safe_stop_ack_without_task_finish(self):
        command = normalize_fsm_task_command({
            "vehicle": "usv",
            "task": "TASK3",
            "progress": "TASK3_ERROR",
            "error": "SEARCH_FAILED",
            "command": "end",
        })
        ack = safe_stop_ack_progress(command)
        self.assertEqual(ack["progress"], "TASK3_END_ACK")
        self.assertIsNone(ack["error"])
        self.assertTrue(ack["return_to_start_required"])

    def test_executor_state_uses_progress_and_error_fields(self):
        running = executor_state_to_fsm_progress({"state": "SEARCH_DOCK"})
        self.assertEqual(running, {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": "TASK3_SEARCH_DOCK",
            "error": None,
            "detail": "",
        })
        retrying = executor_state_to_fsm_progress({
            "state": "CHECK_BAY_COLOR",
            "retry": {"operation": "bay_color", "attempt": 2},
        })
        self.assertIsNone(retrying["error"])
        self.assertEqual(retrying["retry"]["attempt"], 2)
        finished = executor_state_to_fsm_progress({"state": "FINISHED"})
        self.assertEqual(finished["progress"], FSM_TASK3_FINISH_PROGRESS)
        self.assertIsNone(finished["error"])

    def test_wire_progress_rejects_packed_status_fields(self):
        with self.assertRaises(ValueError):
            validate_fsm_progress({
                "vehicle": "usv",
                "task": "TASK3",
                "progress": "TASK3_SEARCH_DOCK",
                "error": None,
                "status_bits": "1 10 0011 0000 000",
            })

    def test_wire_progress_accepts_structured_task3_result(self):
        validate_fsm_progress({
            "vehicle": "usv",
            "task": "TASK3",
            "progress": "TASK3_SPRAYER_WINDOW",
            "error": None,
            "window_id": 1,
            "color": "RED",
        })

    def test_correct_bay_and_window_keep_structured_fields(self):
        bay = correct_bay_to_fsm_progress({
            "bay_id": 2,
            "dock_id": "dock-A",
            "confidence": 0.9,
            "docking_confirmed": False,
        })
        self.assertEqual(bay["progress"], "TASK3_BAY_CONFIRMED")
        self.assertEqual(bay["bay_id"], 2)
        result = docking_result_to_fsm_message({
            "bay_id": 2,
            "dock_id": "dock-A",
            "confidence": 0.9,
        })
        self.assertEqual(result["phase"], "DOCKING_COMPLETE")
        self.assertEqual(result["official_report"]["type"], "docking")
        window = sprayer_window_to_fsm_progress({
            "window_id": 1,
            "bay_id": 2,
            "dock_id": "dock-A",
            "color": "GREEN",
            "confidence": 0.9,
            "pump_off": True,
            "green_confirmed_duration_s": 5.0,
            "completion_confirmed": True,
        })
        self.assertEqual(window["progress"], "TASK3_SPRAYER_WINDOW")
        self.assertEqual(window["bay_id"], 2)
        self.assertEqual(window["dock_id"], "dock-A")
        self.assertTrue(window["completion_confirmed"])


if __name__ == "__main__":
    unittest.main()
