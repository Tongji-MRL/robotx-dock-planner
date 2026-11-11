"""Publish a deterministic three-bay cloud for Task3 search integration tests."""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from dock_planner import GeofenceSearchConfig, NEDPoint, plan_geofence_search
from robotx_perception_common.lidar_transform import transform_base_link_points_to_lidar


class Task3GeofenceSearchSimulator(Node):
    """Small ROS simulator that exercises the production perception bridge.

    It is intentionally not a vehicle dynamics simulator.  The vessel is
    stationary at the NED origin while a rotated three-bay dock is sampled as
    a noisy low-profile PointCloud2.  The production node must recover all
    three entrances without receiving a coarse dock centre.
    """

    def __init__(self) -> None:
        super().__init__("task3_geofence_search_simulator")
        self.declare_parameter("pointcloud_topic", "/task3/sim/livox/lidar")
        self.declare_parameter("pose_topic", "/tongji_mrl/twave/ned_position")
        self.declare_parameter("search_plan_topic", "/task3/sim/geofence_search_plan")
        self.declare_parameter("marker_topic", "/task3/sim/markers")
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("dock_bearing_rad", 1.10)
        self.declare_parameter("dock_entrance_range_m", 4.5)
        self.declare_parameter("dock_depth_m", 6.0)
        self.declare_parameter("bay_width_m", 3.0)
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("ned_frame_id", "robotx_rtk_ned")

        self._cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("pointcloud_topic").value), 10
        )
        self._pose_pub = self.create_publisher(
            String, str(self.get_parameter("pose_topic").value), 10
        )
        self._plan_pub = self.create_publisher(
            String, str(self.get_parameter("search_plan_topic").value), 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10
        )
        self._detected_coarse = None
        self._detected_entrances = []
        self.create_subscription(
            String,
            "/tongji_mrl/twave/task3/coarse_dock",
            self._on_detected_coarse,
            10,
        )
        self.create_subscription(
            String,
            "/tongji_mrl/twave/task3/dock_perception",
            self._on_detected_entrances,
            10,
        )
        self._sequence = 0
        rate = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "Synthetic three-bay Task3 search scene ready; no coarse dock is published."
        )

    def _on_detected_coarse(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if value.get("source") == "mid360_task3_unseeded_multiframe":
                self._detected_coarse = value
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _on_detected_entrances(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if value.get("source") == "mid360_task3_unseeded_multiframe":
                self._detected_entrances = list(value.get("entrance_observations", []))
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _base_cloud(self) -> np.ndarray:
        heading = float(self.get_parameter("dock_bearing_rad").value)
        entrance_range = float(self.get_parameter("dock_entrance_range_m").value)
        depth_m = float(self.get_parameter("dock_depth_m").value)
        bay_width = float(self.get_parameter("bay_width_m").value)
        forward = np.asarray((math.cos(heading), math.sin(heading)), dtype=float)
        cross = np.asarray((-forward[1], forward[0]), dtype=float)
        rng = np.random.default_rng(20261111 + self._sequence)
        parts = []
        finger_offsets = tuple(
            (index - 1.5) * bay_width for index in range(4)
        )
        for offset in finger_offsets:
            for z_m in (-0.20, 0.0, 0.20, 0.40):
                depth = np.linspace(0.0, depth_m, 80)
                keep = rng.random(len(depth)) > 0.08
                xy = (
                    forward[None, :] * (entrance_range + depth[keep, None])
                    + cross[None, :] * offset
                )
                parts.append(np.column_stack((
                    xy + rng.normal(0.0, 0.012, xy.shape),
                    np.full(keep.sum(), z_m)
                    + rng.normal(0.0, 0.008, keep.sum()),
                )))
        # Sparse water and distant clutter exercise the height/range filters
        # without making the expected entrance geometry ambiguous.
        clutter_count = 120
        clutter_angle = rng.uniform(-math.pi, math.pi, clutter_count)
        clutter_range = rng.uniform(1.2, 11.0, clutter_count)
        clutter = np.column_stack((
            clutter_range * np.cos(clutter_angle),
            clutter_range * np.sin(clutter_angle),
            rng.normal(-0.55, 0.04, clutter_count),
        ))
        return np.ascontiguousarray(np.vstack(parts + [clutter]), dtype=float)

    def _pointcloud_message(self, base_points: np.ndarray) -> PointCloud2:
        lidar_points = transform_base_link_points_to_lidar(base_points).astype("<f4")
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.height = 1
        message.width = len(lidar_points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = lidar_points.tobytes(order="C")
        message.is_dense = True
        return message

    def _pose_message(self, stamp: float) -> String:
        payload = {
            "position_ned_m": {"north": 0.0, "east": 0.0, "down": 0.0},
            "heading_rad": 0.0,
            "stamp_unix_sec": stamp,
            "sequence": self._sequence,
            "frame_id": str(self.get_parameter("ned_frame_id").value),
            "body_reference": "base_link",
            "valid": True,
        }
        result = String()
        result.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        return result

    def _search_plan_message(self) -> String:
        plan = self._search_plan()
        payload = {
            "frame_id": str(self.get_parameter("ned_frame_id").value),
            "centroid": {"north": plan.centroid.north, "east": plan.centroid.east},
            "lines": [
                {
                    "priority": index + 1,
                    "start": {"north": line.start.north, "east": line.start.east},
                    "end": {"north": line.end.north, "east": line.end.east},
                    "stations": [
                        {"north": pose.x, "east": pose.y, "heading_rad": pose.yaw}
                        for pose in line.stations
                    ],
                }
                for index, line in enumerate(plan.lines)
            ],
        }
        result = String()
        result.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        return result

    @staticmethod
    def _search_plan():
        corners = (
            NEDPoint(-6.0, -12.0, 0.0),
            NEDPoint(24.0, -12.0, 0.0),
            NEDPoint(24.0, 12.0, 0.0),
            NEDPoint(-6.0, 12.0, 0.0),
        )
        return plan_geofence_search(
            corners,
            GeofenceSearchConfig(safety_inset_m=3.0, station_spacing_m=4.0),
        )

    def _lidar_point(self, north: float, east: float, z: float = 0.0) -> Point:
        # The synthetic USV is at NED origin with heading zero.  base_link is
        # FLU, therefore NED east maps to negative local Y.
        xyz = transform_base_link_points_to_lidar(
            np.asarray(((north, -east, z),), dtype=float)
        )[0]
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def _marker(
        self,
        marker_id: int,
        namespace: str,
        marker_type: int,
        rgba: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter("frame_id").value)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _visualization_markers(self) -> MarkerArray:
        plan = self._search_plan()
        markers = []

        boundary = self._marker(1, "geofence", Marker.LINE_STRIP, (0.65, 0.65, 0.65, 0.9))
        boundary.scale.x = 0.10
        boundary.points = [
            self._lidar_point(item.north, item.east, 0.05)
            for item in plan.ordered_corners + (plan.ordered_corners[0],)
        ]
        markers.append(boundary)

        for index, line in enumerate(plan.lines):
            color = (0.05, 0.85, 1.0, 1.0) if index == 0 else (0.25, 0.35, 1.0, 0.75)
            route = self._marker(10 + index, "search_lines", Marker.LINE_STRIP, color)
            route.scale.x = 0.16 if index == 0 else 0.10
            route.points = [
                self._lidar_point(item.x, item.y, 0.12 + index * 0.03)
                for item in line.stations
            ]
            markers.append(route)
            stations = self._marker(20 + index, "search_stations", Marker.SPHERE_LIST, color)
            stations.scale.x = stations.scale.y = stations.scale.z = 0.34
            stations.points = [
                self._lidar_point(item.x, item.y, 0.18)
                for item in line.stations
            ]
            markers.append(stations)

        vessel = self._marker(30, "usv", Marker.ARROW, (1.0, 0.55, 0.05, 1.0))
        vessel.scale.x, vessel.scale.y, vessel.scale.z = 0.16, 0.36, 0.36
        vessel.points = [self._lidar_point(0.0, 0.0, 0.35), self._lidar_point(1.8, 0.0, 0.35)]
        markers.append(vessel)

        if self._detected_coarse is not None:
            center = self._detected_coarse.get("center_ned", {})
            coarse = self._marker(40, "detected_coarse", Marker.SPHERE, (1.0, 0.85, 0.05, 1.0))
            coarse.scale.x = coarse.scale.y = coarse.scale.z = 0.65
            coarse.pose.position = self._lidar_point(
                float(center.get("north", 0.0)), float(center.get("east", 0.0)), 0.55
            )
            markers.append(coarse)
            for item in self._detected_coarse.get("bay_observation_poses", []):
                pose = self._marker(50 + int(item["bay_id"]), "bay_observation", Marker.SPHERE, (1.0, 0.45, 0.05, 0.95))
                pose.scale.x = pose.scale.y = pose.scale.z = 0.42
                pose.pose.position = self._lidar_point(float(item["north"]), float(item["east"]), 0.35)
                markers.append(pose)

        for index, item in enumerate(self._detected_entrances, 1):
            north = float(item["entrance_center_north_m"])
            east = float(item["entrance_center_east_m"])
            entrance = self._marker(70 + index, "detected_entrances", Marker.CUBE, (0.1, 1.0, 0.25, 1.0))
            entrance.scale.x, entrance.scale.y, entrance.scale.z = 0.30, 0.70, 0.55
            entrance.pose.position = self._lidar_point(north, east, 0.40)
            markers.append(entrance)
            label = self._marker(80 + index, "bay_labels", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
            label.scale.z = 0.65
            label.pose.position = self._lidar_point(north, east, 1.15)
            label.text = f"Bay {index}"
            markers.append(label)
        return MarkerArray(markers=markers)

    def _publish(self) -> None:
        self._sequence += 1
        stamp = time.time()
        self._pose_pub.publish(self._pose_message(stamp))
        self._plan_pub.publish(self._search_plan_message())
        self._cloud_pub.publish(self._pointcloud_message(self._base_cloud()))
        self._marker_pub.publish(self._visualization_markers())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3GeofenceSearchSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
