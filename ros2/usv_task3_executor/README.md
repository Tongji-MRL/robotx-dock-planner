# Task3 Perception and Communication Adapters

Task3 与 FSM 主规划器的 progress、result、command/ACK、正确泊位、喷水窗口以及双机
heartbeat 监测由当前联调节点 `task3_communication_adapter` 提供。ROS 外部消息严格使用
FSM `mission_planner_node.py` 的 `vehicle/task/progress/error/command` JSON 格式。完整接口
见仓库根目录 `TASK3_ROS_INTERFACE.md`。

统一 Orin 工作区中，车辆级 heartbeat 的生产发布者属于 `usv_state_machine`，通用
通信转发属于 `robotx_communication`；本节点只保留 Task3 事件到 FSM legacy JSON 的联调
适配，不应在正式工作区复制第二份主规划器节点。

只启动通信 adapter：

```bash
ros2 launch usv_task3_executor task3_communication_adapter.launch.py run_id:=42
```

该节点不启动或控制导航、PX4、泵和云台。

The camera perception adapter records the interfaces currently available on the USV. It does not modify perception, plan routes, command PX4, or publish Task3 navigation outputs.

## Available Inputs

| Topic | Type | Adapter behavior |
|---|---|---|
| `/robotx/perception/docking/safe_bay` | `std_msgs/msg/String` | Parses `BAY=...;COLOR=...;X=...;Y=...;Z=...;CONF=...` and records the result. X/Y/Z are in the camera optical frame, in metres; the String carries no frame or timestamp. They are not NED coordinates. |
| `/robotx/perception/docking/alignment` | `robotx_msgs/msg/DockingStatus` | Stores status, `alignment_error_m`, `heading_error_rad`, `range_m`, and the image `header.stamp` as diagnostics only. `ALIGNED` is not an arrival event. |
| `/robotx/perception/resources` | `robotx_msgs/msg/ResourceObservation` | Records resource identity, label, confidence, frame, and header timestamp. Resource observations are not used to infer dock geometry. |

The adapter deliberately does not subscribe to `/robotx/perception/docking/entrance`; no publisher for that topic is currently available.

## Shared Semantic-map Input

`task3_semantic_map_adapter` converts the retained shared map into Task3 internal
messages. The map is owned by `robotx_semantic_map`; Task3 does not read a Markdown
file and does not publish directly to UAV or OCS.

| Topic | Type | Behavior |
|---|---|---|
| `/task/semantic_map` | `std_msgs/msg/String` | Retained, versioned map snapshot. Selects `platform` or legacy `dock` objects with a finite three-element `NED_position`. |
| `/tongji_mrl/twave/task3/platforms` | `std_msgs/msg/String` | Internal platform list for the USV main state machine. The main state machine controls external task reporting. |
| `/tongji_mrl/twave/task3/coarse_dock` | `std_msgs/msg/String` | Selected center with `source: "semantic_map"`, `mock: false`, and `frame_id: "ned"`. |

The selected platform ID defaults to `0`. The standard launch starts the adapter and
executor together:

```bash
ros2 launch usv_task3_executor task3_executor.launch.py platform_id:=0
```

The executor records the center observation. Starting the circular search also requires a
current USV planar pose including heading; the current position String does not yet carry
heading, so this adapter does not fabricate one or start navigation by itself.

For a selected object, the published JSON is:

```json
{"dock_id":"0","center_ned":{"north":1.0,"east":2.0,"down":3.0},"timestamp":123.456,"valid":true,"source":"semantic_map","mock":false,"frame_id":"ned","map_epoch":"run-3","map_version":7}
```

The timestamp is the adapter's ROS clock time at message receipt. Confidence is preserved
when present. No corners or entrance pose are invented. Invalid JSON, a missing target, or
malformed/non-finite coordinates produce a warning and no publication.

## Outputs Not Produced by the Camera Perception Adapter

The camera perception adapter does not publish any of the following:

- `/tongji_mrl/twave/task3/coarse_dock`
- `/tongji_mrl/twave/task3/dock_perception`
- `/tongji_mrl/twave/task3/approach_reached`

The available camera-frame safe-bay position cannot be converted into a valid NED dock center/corner set without a real camera-to-NED transform and the required dock geometry. The inputs also lack an absolute entrance heading and a verified vehicle arrival signal. The camera perception adapter therefore does not fabricate `coarse_dock`, `entrance_observations`, or `approach_reached`, regardless of `dry_run`.

## Running

The default is diagnostic-only:

```bash
ros2 launch usv_task3_executor task3_perception_adapter.launch.py
```

To set `dry_run` false:

```bash
ros2 launch usv_task3_executor task3_perception_adapter.launch.py dry_run:=false
```

`dry_run=false` does not enable publication of incomplete navigation data. Until camera-to-NED, absolute heading, and verified arrival interfaces are supplied, the adapter continues to parse and record inputs only.
