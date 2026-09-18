import unittest

from dock_planner import (
    DEFAULT_RATE_HZ,
    DEFAULT_TOPIC,
    GenericVelocityCommand,
    RosTwistAdapter,
    to_twist_message,
)
from dock_planner.ros_twist_adapter import twist_values


class _Vector3:
    def __init__(self):
        self.x = self.y = self.z = None


class _FakeTwist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeNode:
    def __init__(self):
        self.calls = []
        self.publisher = _FakePublisher()

    def create_publisher(self, message_type, topic, queue_depth):
        self.calls.append((message_type, topic, queue_depth))
        return self.publisher


class GenericVelocityCommandTests(unittest.TestCase):
    def make_command(self, **overrides):
        values = dict(
            surge_mps=1.2,
            yaw_rate_rps=-0.3,
            timestamp=10.0,
            valid_until=10.2,
            phase="align",
            source="task3_controller",
        )
        values.update(overrides)
        return GenericVelocityCommand(**values)

    def test_fields_and_validity_window(self):
        command = self.make_command()
        self.assertEqual((command.surge_mps, command.yaw_rate_rps), (1.2, -0.3))
        self.assertEqual((command.phase, command.source, command.frame_id),
                         ("align", "task3_controller", "base_link"))
        self.assertTrue(command.is_valid_at(10.0))
        self.assertTrue(command.is_valid_at(10.2))
        self.assertFalse(command.is_valid_at(9.99))
        self.assertFalse(command.is_valid_at(10.21))

    def test_validation_rejects_bad_numeric_and_metadata_values(self):
        for field, value in (("surge_mps", float("nan")), ("yaw_rate_rps", float("inf")),
                             ("timestamp", True), ("valid_until", float("nan"))):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.make_command(**{field: value})
        with self.assertRaises(ValueError):
            self.make_command(valid_until=9.9)
        for field in ("phase", "source", "frame_id"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.make_command(**{field: " "})
        with self.assertRaises(ValueError):
            self.make_command().is_valid_at(float("inf"))

    def test_twist_mapping_and_default_contract(self):
        command = self.make_command()
        message = to_twist_message(command, _FakeTwist)
        self.assertEqual((message.linear.x, message.angular.z), (1.2, -0.3))
        self.assertEqual((message.linear.y, message.linear.z,
                          message.angular.x, message.angular.y), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(twist_values(command), (1.2, -0.3))
        self.assertEqual(DEFAULT_TOPIC, "/usv/control/cmd_vel")
        self.assertEqual(DEFAULT_RATE_HZ, 10.0)

    def test_ros_adapter_publishes_and_checks_expiry(self):
        node = _FakeNode()
        adapter = RosTwistAdapter(node, message_type=_FakeTwist)
        self.assertEqual(node.calls, [(_FakeTwist, DEFAULT_TOPIC, 10)])
        adapter.publish(self.make_command(), now=10.1)
        self.assertEqual(len(node.publisher.messages), 1)
        self.assertEqual(node.publisher.messages[0].linear.x, 1.2)
        with self.assertRaises(ValueError):
            adapter.publish(self.make_command(valid_until=10.05), now=10.1)
        self.assertEqual(len(node.publisher.messages), 1)


if __name__ == "__main__":
    unittest.main()
