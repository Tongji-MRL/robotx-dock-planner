import unittest

from dock_planner import (
    SprayerTargetObservation,
    WateringSequenceController,
    WateringSequenceState,
    WateringVerificationConfig,
)


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
