import unittest

from dock_planner import (
    COLOR_SEQUENCE_MESSAGE_TYPE,
    COLOR_SEQUENCE_TOPIC,
    ColorCode,
    ColorSequenceVector,
    decode_color_sequence,
)


class ColorSequenceTests(unittest.TestCase):
    def test_contract(self):
        self.assertEqual(COLOR_SEQUENCE_TOPIC, "/tongji_mrl/twave/task3/color_sequence")
        self.assertEqual(COLOR_SEQUENCE_MESSAGE_TYPE, "std_msgs/msg/Int32MultiArray")

    def test_round_trip_vector(self):
        value = ColorSequenceVector(ColorCode.RED, ColorCode.GREEN)
        self.assertEqual(value.as_ros_data(), [1, 2])
        self.assertEqual(decode_color_sequence([1, 2]), value)
        self.assertTrue(value.valid)

    def test_unknown_is_not_valid(self):
        self.assertFalse(decode_color_sequence([0, 2]).valid)

    def test_bad_shape_or_code_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_color_sequence([1])
        with self.assertRaises(ValueError):
            decode_color_sequence([99, 2])
