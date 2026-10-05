import json
import unittest

from dock_planner import (
    ActiveBayContext,
    BayColorObservation,
    CoarseDockInfo,
    DataFreshnessConfig,
    NEDPoint,
    DockVisionObservation,
    dock_vision_to_bay_color_observation,
    parse_dock_vision_json,
)


class DockVisionAdapterTests(unittest.TestCase):
    def payload(self, **overrides):
        data = {
            "valid": True,
            "heading_valid": True,
            "heading_quality": "HIGH",
            "entrance_center": {
                "x": 0.25,
                "y": -0.10,
                "z": 2.50,
                "pixel_u": 320.0,
                "pixel_v": 240.0,
            },
            "range_m": 2.52,
            "forward_distance_m": 2.50,
            "lateral_error_m": 0.25,
            "bearing_x_deg": 5.7,
            "heading_error_deg": -3.0,
            "frame_id": "oak_rgb_camera_optical_frame",
            "stamp": {"sec": 12, "nanosec": 345000000},
            "face_class": "FRONT",
            "confidence": 0.91,
            "visible_faces": [{"class": "FRONT", "confidence": 0.91}],
            "bay": "BAY_1",
            "bay_color": "GREEN",
            "bay_yolo_confidence": 0.88,
            "bay_color_confidence": 0.94,
            "alignment_status": "FRONT_ALIGNED",
            "plane_points": 42,
            "plane_rmse_m": 0.012,
            "geometry_source": "FACE_KEYPOINTS",
            "coordinate_convention": "OPTICAL_X_RIGHT_Y_DOWN_Z_FORWARD",
            "invalid_reason": "NONE",
        }
        data.update(overrides)
        return data

    def test_complete_valid_json_preserves_camera_frame_data(self):
        observation = parse_dock_vision_json(json.dumps(self.payload()))

        self.assertIsInstance(observation, DockVisionObservation)
        self.assertTrue(observation.valid)
        self.assertTrue(observation.heading_valid)
        self.assertTrue(observation.can_convert_to_ned)
        self.assertEqual(observation.entrance_center_camera.z, 2.5)
        self.assertEqual(observation.frame_id, "oak_rgb_camera_optical_frame")
        self.assertEqual(observation.timestamp, 12.345)
        self.assertEqual(observation.visible_faces[0].face_class, "FRONT")

    def test_freshness_uses_source_stamp_and_generates_valid_until(self):
        freshness = DataFreshnessConfig(perception_timeout_s=2.0)
        observation = parse_dock_vision_json(self.payload(), freshness=freshness)

        self.assertEqual(observation.valid_until, 14.345)
        self.assertTrue(observation.is_fresh_at(14.345, freshness))
        self.assertFalse(observation.is_fresh_at(14.346, freshness))
        self.assertFalse(observation.is_fresh_at(12.344, freshness))

    def test_stale_vision_is_not_fresh(self):
        freshness = DataFreshnessConfig(perception_timeout_s=2.0)
        observation = parse_dock_vision_json(self.payload(), freshness=freshness)

        self.assertFalse(observation.is_fresh_at(20.0, freshness))

    def test_active_context_uses_coarse_dock_id_and_not_bay_as_dock_id(self):
        coarse = CoarseDockInfo(
            "dock_7",
            NEDPoint(1.0, 2.0, 0.0),
            (
                NEDPoint(0.0, 1.0, 0.0),
                NEDPoint(2.0, 1.0, 0.0),
                NEDPoint(2.0, 3.0, 0.0),
                NEDPoint(0.0, 3.0, 0.0),
            ),
            timestamp=10.0,
            valid=True,
        )
        context = ActiveBayContext.from_coarse_dock(coarse, "BAY_1")

        self.assertEqual(context.dock_id, "dock_7")
        self.assertEqual(context.bay_id, "BAY_1")
        self.assertNotEqual(context.dock_id, context.bay_id)

    def test_heading_invalid_does_not_become_convertible(self):
        observation = parse_dock_vision_json(self.payload(heading_valid=False))

        self.assertFalse(observation.heading_valid)
        self.assertFalse(observation.can_convert_to_ned)

    def test_invalid_source_result_does_not_produce_color_decision(self):
        observation = parse_dock_vision_json(self.payload(valid=False))

        self.assertFalse(observation.valid)
        self.assertIsNone(dock_vision_to_bay_color_observation(observation))

    def test_green_and_red_colors_convert_to_existing_observation(self):
        context = ActiveBayContext("dock_7", "BAY_1")
        for color in ("GREEN", "RED"):
            with self.subTest(color=color):
                observation = parse_dock_vision_json(self.payload(bay_color=color))
                color_observation = dock_vision_to_bay_color_observation(
                    observation, active_bay=context
                )

                self.assertIsInstance(color_observation, BayColorObservation)
                self.assertEqual(color_observation.color, color)
                self.assertEqual(color_observation.confidence, 0.94)
                self.assertEqual(color_observation.timestamp, 12.345)

    def test_unknown_color_and_none_bay_are_blocked(self):
        for overrides in (
            {"bay_color": "UNKNOWN"},
            {"bay": "NONE"},
            {"bay_color": None},
            {"bay_color_confidence": None},
        ):
            with self.subTest(overrides=overrides):
                observation = parse_dock_vision_json(self.payload(**overrides))
                self.assertIsNone(dock_vision_to_bay_color_observation(observation))

    def test_bay_mismatch_cannot_supply_color_for_active_bay(self):
        context = ActiveBayContext("dock_7", "BAY_2")
        observation = parse_dock_vision_json(self.payload(bay="BAY_1", bay_color="GREEN"))

        self.assertFalse(observation.matches_active_bay(context))
        self.assertIsNone(
            dock_vision_to_bay_color_observation(observation, active_bay=context)
        )

    def test_missing_or_null_geometry_is_kept_as_unusable_raw_observation(self):
        observation = parse_dock_vision_json(
            self.payload(entrance_center=None, heading_error_deg=None)
        )

        self.assertIsNone(observation.entrance_center_camera)
        self.assertIsNone(observation.heading_error_deg)
        self.assertFalse(observation.can_convert_to_ned)

    def test_heading_invalid_remains_fresh_but_cannot_convert(self):
        freshness = DataFreshnessConfig(perception_timeout_s=2.0)
        observation = parse_dock_vision_json(
            self.payload(heading_valid=False), freshness=freshness
        )

        self.assertTrue(observation.is_fresh_at(13.0, freshness))
        self.assertFalse(observation.can_convert_to_ned)

    def test_missing_fields_and_nulls_are_safe(self):
        observation = parse_dock_vision_json(
            {
                "valid": False,
                "heading_valid": False,
                "stamp": None,
                "entrance_center": None,
                "bay": None,
                "bay_color": None,
            }
        )

        self.assertFalse(observation.valid)
        self.assertIsNone(observation.timestamp)
        self.assertIsNone(observation.frame_id)
        self.assertIsNone(observation.bay)
        self.assertIsNone(observation.bay_color)


if __name__ == "__main__":
    unittest.main()
