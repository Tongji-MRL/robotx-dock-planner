import unittest

from dock_planner.task3_ros_contract import (
    CorrectBayInfo,
    HeartbeatMonitor,
    SprayerWindowInfo,
    Task3RosMessageFactory,
    normalize_task_command,
    validate_vehicle_heartbeat,
)


class Task3RosContractTests(unittest.TestCase):
    def setUp(self):
        ids = iter(("message-1", "message-2", "message-3", "message-4"))
        self.factory = Task3RosMessageFactory(
            42,
            clock_ms=lambda: 123456,
            message_id_factory=lambda: next(ids),
        )

    def test_executor_state_uses_fsm_progress_shape(self):
        message = self.factory.progress_from_executor_state({
            "state": "SEARCH_DOCK",
            "ready_for_task_logic": True,
            "dry_run": False,
        })
        self.assertEqual(
            {key: message[key] for key in ("vehicle", "task", "progress", "error")},
            {
                "vehicle": "usv",
                "task": "TASK3",
                "progress": "TASK3_SEARCH_DOCK",
                "error": None,
            },
        )
        self.assertNotIn("status_code", message)
        self.assertNotIn("status_bits", message)

    def test_docking_report_requires_authoritative_confirmation(self):
        raw = {
            "bay_id": 2,
            "dock_id": "dock-A",
            "confidence": 0.94,
            "confirmed_at_unix_ms": 123450,
            "docking_confirmed": False,
            "source": "task3_flow",
        }
        unconfirmed = CorrectBayInfo.from_mapping(raw)
        progress = self.factory.correct_bay_progress(unconfirmed)
        self.assertEqual(progress["progress"], "TASK3_BAY_CONFIRMED")
        self.assertEqual(progress["bay_id"], 2)
        self.assertFalse(progress["docking_confirmed"])
        with self.assertRaises(ValueError):
            self.factory.docking_report(unconfirmed)

        raw["docking_confirmed"] = True
        report = self.factory.docking_report(CorrectBayInfo.from_mapping(raw))
        official = report["official_report"]
        self.assertEqual(official, {
            "type": "docking",
            "vehicle_id": "T-Wave",
            "bay_id": 2,
            "dock_id": "dock-A",
            "confidence": 0.94,
        })

    def test_bay_id_is_limited_to_official_range(self):
        with self.assertRaises(ValueError):
            CorrectBayInfo.from_mapping({
                "bay_id": 4,
                "dock_id": "dock-A",
                "confidence": 0.9,
                "confirmed_at_unix_ms": 123,
                "docking_confirmed": True,
                "source": "test",
            })

    def test_window_progress_does_not_imply_completion(self):
        info = SprayerWindowInfo.from_mapping({
            "window_id": 1,
            "color": "RED",
            "confidence": 0.88,
            "observed_at_unix_ms": 123450,
            "pump_off": False,
            "green_confirmed_duration_s": 0.0,
            "completion_confirmed": False,
            "source": "sprayer_vision",
        })
        progress = self.factory.sprayer_window_progress(info)
        self.assertEqual(progress["progress"], "TASK3_SPRAYER_WINDOW")
        self.assertEqual(progress["color"], "RED")
        self.assertFalse(info.firefighting_complete)
        with self.assertRaises(ValueError):
            self.factory.firefighting_report(info)

    def test_firefighting_report_requires_green_five_seconds_and_pump_off(self):
        base = {
            "window_id": 3,
            "color": "GREEN",
            "confidence": 0.92,
            "observed_at_unix_ms": 123450,
            "pump_off": True,
            "green_confirmed_duration_s": 5.0,
            "completion_confirmed": True,
            "source": "watering_controller",
        }
        info = SprayerWindowInfo.from_mapping(base)
        self.assertTrue(info.firefighting_complete)
        report = self.factory.firefighting_report(info)
        self.assertEqual(
            report["official_report"],
            {
                "type": "firefighting",
                "vehicle_id": "T-Wave",
                "window_id": 3,
                "color": "GREEN",
            },
        )

        for field, bad in (
            ("color", "RED"),
            ("pump_off", False),
            ("green_confirmed_duration_s", 4.99),
            ("completion_confirmed", False),
        ):
            with self.subTest(field=field):
                value = dict(base)
                value[field] = bad
                bad_info = SprayerWindowInfo.from_mapping(value)
                self.assertFalse(bad_info.firefighting_complete)
                with self.assertRaises(ValueError):
                    self.factory.firefighting_report(bad_info)

    def test_structured_and_legacy_commands_normalize(self):
        structured = normalize_task_command({
            "command_id": "resume-1",
            "run_id": 42,
            "command": "RESUME_TASK3",
            "payload": {"checkpoint_id": "checkpoint-7"},
        })
        self.assertEqual(structured["command"], "RESUME_TASK3")

        legacy = normalize_task_command({
            "task_id": 4,
            "action": "execute_task",
            "run_id": 42,
            "command_seq": 9,
            "parameters": {
                "target_position": {"frame_id": "wgs84", "latitude": 1.2, "longitude": 3.4}
            },
        })
        self.assertEqual(legacy["command_id"], "legacy-9")
        self.assertEqual(legacy["command"], "TASK4_GOTO")

    def test_resume_requires_checkpoint(self):
        with self.assertRaises(ValueError):
            normalize_task_command({
                "command_id": "resume-1",
                "run_id": 42,
                "command": "RESUME_TASK3",
                "payload": {},
            })

    def test_vehicle_heartbeat_validation_and_receive_time_watchdog(self):
        heartbeat = {
            "protocol_version": "1",
            "message_id": "heartbeat-1",
            "sent_at_unix_ms": 123456,
            "run_id": 42,
            "vehicle_id": "T-Sky",
            "source": "T-Sky",
            "frame_id": "wgs84",
            "kind": "heartbeat",
            "payload": {
                "state": "AUTO",
                "current_task": "TASK3",
                "position": {
                    "frame_id": "wgs84",
                    "latitude_deg": 1.28,
                    "longitude_deg": 103.85,
                    "altitude_m": 10.0,
                },
            },
        }
        validate_vehicle_heartbeat(heartbeat, "T-Sky")
        monitor = HeartbeatMonitor(timeout_s=3.0)
        self.assertTrue(monitor.stale("T-Sky", 10.0))
        monitor.note("T-Sky", heartbeat, 10.0)
        self.assertFalse(monitor.stale("T-Sky", 12.9))
        self.assertTrue(monitor.stale("T-Sky", 13.1))

        with self.assertRaises(ValueError):
            validate_vehicle_heartbeat(heartbeat, "T-Wave")


if __name__ == "__main__":
    unittest.main()
