import unittest

from dock_planner import (
    ResourceColorSequenceObservation,
    SprayerTargetObservation,
    WateringCompletionTracker,
    WateringSequenceController,
    WateringSequenceState,
    WateringVerificationConfig,
    WateringVerificationState,
    parse_logistics_request_json,
)


class LogisticsRequestTests(unittest.TestCase):
    def test_valid_sequence_is_normalized_and_actionable(self):
        observation = parse_logistics_request_json(
            '{"valid":true,"rule":"disruptive",'
            '"resource_color":"blue","destination_color":"green"}',
            timestamp=20.0,
        )
        self.assertIsInstance(observation, ResourceColorSequenceObservation)
        self.assertEqual(observation.rule, "DISRUPTIVE")
        self.assertEqual(observation.resource_color, "BLUE")
        self.assertEqual(observation.destination_color, "GREEN")
        self.assertTrue(observation.is_actionable)
        self.assertTrue(observation.is_fresh_at(20.5, 1.0))

    def test_missing_unknown_or_invalid_sequence_is_not_actionable(self):
        cases = (
            ({"valid": True, "resource_color": None, "destination_color": "GREEN"},),
            ({"valid": True, "resource_color": "UNKNOWN", "destination_color": "GREEN"},),
            ({"valid": False, "resource_color": "BLUE", "destination_color": "GREEN"},),
        )
        for (payload,) in cases:
            with self.subTest(payload=payload):
                observation = parse_logistics_request_json(payload, timestamp=20.0)
                self.assertFalse(observation.is_actionable)

    def test_invalid_json_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_logistics_request_json("not-json", timestamp=20.0)


class WateringCompletionTests(unittest.TestCase):
    def setUp(self):
        self.tracker = WateringCompletionTracker(
            WateringVerificationConfig(
                green_confirmation_s=5.0,
                observation_timeout_s=1.1,
            )
        )

    @staticmethod
    def observation(timestamp, *, green=False, target_valid=True):
        return SprayerTargetObservation(
            target_valid=target_valid,
            delivery_confirmed=green,
            timestamp=timestamp,
        )

    def test_red_target_does_not_stop_pump_or_complete(self):
        result = self.tracker.update(self.observation(10.0))
        self.assertEqual(result.state, WateringVerificationState.WAITING_FOR_GREEN)
        self.assertFalse(result.stop_pump_requested)
        self.assertFalse(result.watering_complete)

    def test_first_green_requests_immediate_stop_then_five_seconds_complete(self):
        first = self.tracker.update(
            self.observation(10.0, green=True, target_valid=False)
        )
        self.assertEqual(first.state, WateringVerificationState.VERIFYING_GREEN)
        self.assertTrue(first.stop_pump_requested)
        self.assertFalse(first.watering_complete)

        result = first
        for timestamp in (11.0, 12.0, 13.0, 14.0, 15.0):
            result = self.tracker.update(
                self.observation(timestamp, green=True, target_valid=False)
            )
        self.assertEqual(result.state, WateringVerificationState.COMPLETE)
        self.assertTrue(result.stop_pump_requested)
        self.assertTrue(result.watering_complete)
        self.assertEqual(result.confirmed_green_s, 5.0)

    def test_interrupted_green_restarts_confirmation_without_restarting_pump(self):
        self.tracker.update(self.observation(10.0, green=True, target_valid=False))
        interrupted = self.tracker.update(
            self.observation(10.5, green=False, target_valid=False)
        )
        self.assertTrue(interrupted.stop_pump_requested)
        self.assertFalse(interrupted.watering_complete)
        restarted = self.tracker.update(
            self.observation(11.0, green=True, target_valid=False)
        )
        self.assertEqual(restarted.confirmed_green_s, 0.0)

    def test_stale_observation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "stale"):
            self.tracker.update(self.observation(10.0), now=12.0)


class WateringSequenceTests(unittest.TestCase):
    @staticmethod
    def observation(timestamp, *, red=False, green=False):
        return SprayerTargetObservation(
            target_valid=red,
            delivery_confirmed=green,
            timestamp=timestamp,
        )

    def test_docked_red_aim_pump_then_green_completion(self):
        controller = WateringSequenceController(
            WateringVerificationConfig(green_confirmation_s=5.0, observation_timeout_s=1.1)
        )
        self.assertEqual(controller.state, WateringSequenceState.WAITING_FOR_DOCKED)
        controller.on_docked()
        self.assertEqual(controller.state, WateringSequenceState.WAITING_FOR_RED)
        controller.on_window_observation(self.observation(10.0, red=True), now=10.1)
        self.assertEqual(controller.state, WateringSequenceState.AIMING)
        started = controller.on_gimbal_ready(True)
        self.assertEqual(started.state, WateringSequenceState.SPRAYING)
        self.assertTrue(started.pump_on_requested)

        stopped = controller.on_window_observation(
            self.observation(11.0, green=True), now=11.1
        )
        self.assertEqual(stopped.state, WateringSequenceState.VERIFYING_GREEN)
        self.assertTrue(stopped.pump_off_requested)
        self.assertFalse(stopped.watering_complete)
        result = stopped
        for timestamp in (12.0, 13.0, 14.0, 15.0, 16.0):
            result = controller.on_window_observation(
                self.observation(timestamp, green=True), now=timestamp + 0.1
            )
        self.assertEqual(result.state, WateringSequenceState.COMPLETE)
        self.assertTrue(result.watering_complete)

    def test_stale_red_does_not_start_aiming(self):
        controller = WateringSequenceController()
        controller.on_docked()
        result = controller.on_window_observation(
            self.observation(10.0, red=True), now=12.0
        )
        self.assertEqual(result.state, WateringSequenceState.WAITING_FOR_RED)
        self.assertIn("stale", result.reason)

    def test_green_before_red_does_not_start_pump(self):
        controller = WateringSequenceController()
        controller.on_docked()
        result = controller.on_window_observation(
            self.observation(10.0, green=True), now=10.1
        )
        self.assertEqual(result.state, WateringSequenceState.WAITING_FOR_RED)
        self.assertFalse(result.pump_on_requested)


if __name__ == "__main__":
    unittest.main()
