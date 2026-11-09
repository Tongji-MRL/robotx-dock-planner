"""Bridge the new Task 3 vision result into the existing team contract."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String
from robotx_gimbal_interfaces.msg import PumpStatus

from dock_planner import (
    BestEffortWindowId,
    COLOR_SEQUENCE_TOPIC,
    ColorCode,
    encode_json_object,
)


SPRAYER_WINDOW_TOPIC = "/tongji_mrl/twave/task3/sprayer_window"


class Task3VisionContractAdapter(Node):
    """Publish a validated two-colour sequence for communication and the UAV."""

    def __init__(self) -> None:
        super().__init__("task3_vision_contract_adapter")
        self.declare_parameter(
            "logistics_request_topic",
            "/robotx/perception/task3/logistics_request",
        )
        self.request_sub = self.create_subscription(
            String,
            str(self.get_parameter("logistics_request_topic").value),
            self._on_request,
            10,
        )
        self.sequence_pub = self.create_publisher(
            Int32MultiArray, COLOR_SEQUENCE_TOPIC, 10
        )
        self.declare_parameter("fallback_window_id", 1)
        self.declare_parameter("green_confirmation_s", 5.0)
        self.window_ids = BestEffortWindowId(
            int(self.get_parameter("fallback_window_id").value)
        )
        self.green_confirmation_s = float(
            self.get_parameter("green_confirmation_s").value
        )
        if self.green_confirmation_s < 0.0:
            raise ValueError("green_confirmation_s must be non-negative")
        self.pump_off = False
        self.pump_initialized = False
        self.green_since_monotonic: float | None = None
        self.last_state_fields: dict[str, str] = {}
        self.last_published_key: tuple[object, ...] | None = None
        self.window_pub = self.create_publisher(String, SPRAYER_WINDOW_TOPIC, 10)
        self.create_subscription(
            String,
            "/robotx/perception/sprayer/task_state",
            self._on_window_state,
            10,
        )
        self.create_subscription(PumpStatus, "/pump/status", self._on_pump_status, 10)
        self.create_subscription(
            Bool,
            "/robotx/task3/docked_confirmed",
            self._on_docked_confirmed,
            10,
        )
        self.completion_timer = self.create_timer(0.1, self._publish_completion_if_ready)

    @staticmethod
    def _color_code(value: object) -> int:
        if not isinstance(value, str):
            raise ValueError("colour must be a string")
        try:
            code = ColorCode[value.strip().upper()]
        except KeyError as exc:
            raise ValueError(f"unsupported colour: {value!r}") from exc
        if code is ColorCode.UNKNOWN:
            raise ValueError("UNKNOWN is not a publishable colour")
        return int(code)

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict) or payload.get("valid") is not True:
                raise ValueError("request must be a valid JSON object")
            output = Int32MultiArray()
            output.data = [
                self._color_code(payload.get("resource_color")),
                self._color_code(payload.get("destination_color")),
            ]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid logistics request: {exc}")
            return
        self.sequence_pub.publish(output)
        self.get_logger().info(
            f"Published Task 3 colour sequence: {list(output.data)}"
        )

    @staticmethod
    def _state_fields(raw: str) -> dict[str, str]:
        return {
            key.strip().upper(): value.strip().upper()
            for part in raw.split(";")
            if "=" in part
            for key, value in [part.split("=", 1)]
            if key.strip()
        }

    def _on_docked_confirmed(self, msg: Bool) -> None:
        if msg.data:
            return
        self.window_ids.reset()
        self.green_since_monotonic = None
        self.last_state_fields = {}
        self.last_published_key = None

    def _on_pump_status(self, msg: PumpStatus) -> None:
        self.pump_initialized = bool(msg.initialized)
        self.pump_off = not bool(msg.pump_on)
        self._publish_completion_if_ready()

    def _on_window_state(self, msg: String) -> None:
        fields = self._state_fields(msg.data)
        self.last_state_fields = fields
        observed_id = self.window_ids.observe_side(fields.get("ACTIVE"))
        signal = fields.get("SIGNAL", "UNKNOWN")
        now = time.monotonic()
        if signal == "GREEN":
            if self.green_since_monotonic is None:
                self.green_since_monotonic = now
        else:
            self.green_since_monotonic = None

        # Publish RED progress only when perception actually knows the side.
        # A missing side never blocks the vision/gimbal/pump chain; the fallback
        # is used only for the post-delivery completion report.
        if signal == "RED" and observed_id is not None:
            self._publish_window(
                window_id=observed_id,
                color="RED",
                confidence=1.0,
                green_duration_s=0.0,
                completion=False,
            )
        self._publish_completion_if_ready()

    def _publish_completion_if_ready(self) -> None:
        if (
            self.green_since_monotonic is None
            or not self.pump_initialized
            or not self.pump_off
        ):
            return
        duration = time.monotonic() - self.green_since_monotonic
        if duration < self.green_confirmation_s:
            return
        window_id = self.window_ids.report_id
        self._publish_window(
            window_id=window_id,
            color="GREEN",
            confidence=1.0 if not self.window_ids.used_fallback else 0.0,
            green_duration_s=duration,
            completion=True,
        )

    def _publish_window(
        self,
        *,
        window_id: int,
        color: str,
        confidence: float,
        green_duration_s: float,
        completion: bool,
    ) -> None:
        key = (window_id, color, completion)
        if key == self.last_published_key:
            return
        self.last_published_key = key
        payload = {
            "window_id": window_id,
            "color": color,
            "confidence": confidence,
            "observed_at_unix_ms": max(1, int(time.time() * 1000)),
            "pump_off": self.pump_off,
            "green_confirmed_duration_s": green_duration_s,
            "completion_confirmed": completion,
            "source": "task3_window_vision_best_effort_id",
        }
        output = String(data=encode_json_object(payload))
        self.window_pub.publish(output)
        if completion:
            mode = "fallback" if self.window_ids.used_fallback else "vision"
            self.get_logger().info(
                f"Published firefighting completion with window_id={window_id} ({mode})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3VisionContractAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
