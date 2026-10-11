import unittest

from dock_planner.task3_status_codec import (
    CommonError,
    PlannerCommand,
    RecoveryAction,
    TASK3_ERROR_CATALOG_VERSION,
    TASK3_ERROR_DEFINITIONS,
    Task3Step,
    TaskCode,
    TaskStatusCode,
    VehicleCode,
)


class Task3StatusCodecTests(unittest.TestCase):
    def test_exact_14_bit_layout_and_round_trip(self):
        status = TaskStatusCode(
            VehicleCode.USV,
            TaskCode.TASK3,
            Task3Step.SEARCH_DOCK,
            CommonError.NONE,
            PlannerCommand.NONE,
        )
        self.assertEqual(status.bit_string(), "1 10 0011 0000 000")
        self.assertEqual(TaskStatusCode.unpack(status.pack()), status)

    def test_normal_status_command_defaults_to_zero(self):
        status = TaskStatusCode(VehicleCode.USV, TaskCode.TASK3, Task3Step.SPRAYING)
        self.assertEqual(status.command, PlannerCommand.NONE)
        self.assertTrue(status.bit_string().endswith("000"))

    def test_common_error_code_has_same_meaning_in_different_steps(self):
        search = TaskStatusCode(
            VehicleCode.USV, TaskCode.TASK3, Task3Step.SEARCH_DOCK, CommonError.TIMEOUT
        )
        spray = TaskStatusCode(
            VehicleCode.USV, TaskCode.TASK3, Task3Step.SPRAYING, CommonError.TIMEOUT
        )
        self.assertEqual(search.describe()["error"], "TIMEOUT")
        self.assertEqual(spray.describe()["error"], "TIMEOUT")

    def test_step_specific_error_codes_restart_at_eight(self):
        search = TaskStatusCode(
            VehicleCode.USV, TaskCode.TASK3, Task3Step.SEARCH_DOCK, 0b1000
        )
        spray = TaskStatusCode(
            VehicleCode.USV, TaskCode.TASK3, Task3Step.SPRAYING, 0b1000
        )
        self.assertEqual(search.describe()["error"], "LIDAR_NO_DATA")
        self.assertEqual(spray.describe()["error"], "PUMP_START_FAILED")

    def test_undefined_step_specific_error_is_rejected(self):
        with self.assertRaises(ValueError):
            TaskStatusCode(VehicleCode.USV, TaskCode.TASK3, 16, 0b1000)

    def test_every_task3_step_has_all_eight_specific_codes_frozen(self):
        expected_codes = set(range(0b1000, 0b1_0000))
        self.assertEqual(set(TASK3_ERROR_DEFINITIONS), set(Task3Step))
        for step, definitions in TASK3_ERROR_DEFINITIONS.items():
            with self.subTest(step=step.name):
                self.assertEqual(set(definitions), expected_codes)
                self.assertEqual(len({item.name for item in definitions.values()}), 8)

    def test_error_description_includes_planner_recovery_contract(self):
        status = TaskStatusCode(
            VehicleCode.USV,
            TaskCode.TASK3,
            Task3Step.APPROACH_DOCK,
            0b1011,
        )
        description = status.describe()
        self.assertEqual(description["error"], "NO_FEASIBLE_SAFETY_RADIUS")
        self.assertFalse(description["error_retryable"])
        self.assertEqual(description["recovery_action"], RecoveryAction.SAFE_HOLD.value)
        self.assertEqual(
            description["error_catalog_version"], TASK3_ERROR_CATALOG_VERSION
        )

    def test_unforeseen_fault_uses_common_unknown_error(self):
        status = TaskStatusCode(
            VehicleCode.USV,
            TaskCode.TASK3,
            Task3Step.SEARCH_DOCK,
            CommonError.UNKNOWN_ERROR,
        )
        self.assertEqual(status.describe()["error"], "UNKNOWN_ERROR")
        self.assertEqual(status.describe()["recovery_action"], "SAFE_HOLD")


if __name__ == "__main__":
    unittest.main()
