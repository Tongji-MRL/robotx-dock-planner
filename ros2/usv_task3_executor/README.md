# Task3 Perception Adapters

Task3 与总规划器的 progress、report、command/ACK、正确泊位、喷水窗口以及双机
heartbeat 监测由 `task3_communication_adapter` 提供。完整接口见仓库根目录
`TASK3_ROS_INTERFACE.md`。

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

## Semantic-map Mock Coarse Dock

`task3_semantic_map_mock_adapter` is a separate, dry-run-only input bridge:

| Topic | Type | Behavior |
|---|---|---|
| `/task/semantic_map` | `std_msgs/msg/String` | Selects one object with `type == "dock"` and matching integer `id` (or `dock_id`), then validates its three-element finite `NED_position`. |
| `/tongji_mrl/twave/task3/coarse_dock` | `std_msgs/msg/String` | Publishes a center-only mock record marked with `source: "semantic_map_mock"`, `mock: true`, and `frame_id: "mock_ned"`. |

The selected dock ID defaults to `0`. Run the adapter and executor in separate terminals:

```bash
ros2 launch usv_task3_executor task3_semantic_map_mock_adapter.launch.py
ros2 launch usv_task3_executor task3_executor.launch.py
```

Set another ID with `dock_id:=N`. Mock publication is disabled when `dry_run:=false`. The executor records this center observation only while its own `dry_run` parameter is true. It does not pass this center-only mock into `Task3FlowManager`: the existing flow requires four approximate corners, which the semantic-map input does not provide. The executor does not require or confirm `px4_local_ned`; this adapter preserves the honest `mock_ned` label, and the coordinates must not be treated as verified PX4 local NED.

For a selected object, the published JSON is:

```json
{"dock_id":"0","center_ned":{"north":1.0,"east":2.0,"down":3.0},"timestamp":123.456,"valid":true,"source":"semantic_map_mock","mock":true,"frame_id":"mock_ned"}
```

The timestamp is the adapter's ROS clock time at message receipt. No confidence value or dock corners are generated. Invalid JSON, a missing target, or malformed/non-finite coordinates produce a warning and no publication.

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
