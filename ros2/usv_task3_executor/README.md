# Task3 Perception and Communication Adapters

Task3 与 FSM 主规划器的 progress、result、command/ACK、正确泊位、喷水窗口以及双机
heartbeat 监测由当前联调节点 `task3_communication_adapter` 提供。ROS 外部消息严格使用
FSM `mission_planner_node.py` 的 `vehicle/task/progress/error/command` JSON 格式。完整接口
见仓库根目录 `TASK3_ROS_INTERFACE.md`。

统一 Orin 工作区中，车辆级 heartbeat 的生产发布者属于 `usv_state_machine`，通用
通信转发属于 `robotx_communication`；本节点只保留 Task3 事件到 FSM legacy JSON 的联调
适配，不应在正式工作区复制第二份主规划器节点。

## 新版 Task3 视觉整链

`task3_vision_stack.launch.py` 接入新版 OAK 视觉代码，并复用原有 LiDAR
入口几何结果：

- `/robotx/perception/leds` 的红/绿 `LEDObservation` 只用于判断当前正在检查的
  bay；其像素坐标不会被当作 NED 导航坐标。
- 正确 bay 默认为官方绿色指示灯，参数为 `target_color:=GREEN`。
- executor 到达最终靠泊点后持续发布
  `/robotx/task3/docked_confirmed=true`，随后才放行窗口识别。
- 窗口模块的 logistics JSON 会校验并转换为
  `/tongji_mrl/twave/task3/color_sequence` 的 `Int32MultiArray`，顺序为
  `[resource_color, destination_color]`。
- 窗口状态中的 `ACTIVE=LEFT/RIGHT` 分别映射为 `window_id=1/2`。若喷水完成前始终
  无法判断左右，adapter 默认用 `window_id=1` 完成尽力报告；可在 launch 中通过节点
  参数 `fallback_window_id` 改为 2。窗口 ID 不阻塞瞄准、喷水或绿灯成功判定。

安全检查命令如下。它会启动相机与两套视觉，规划保持 dry run，云台和水泵不动：

```bash
ros2 launch usv_task3_executor task3_vision_stack.launch.py
```

泊位灯 YOLO 若位于单独 Python 环境，可增加：

```bash
ros2 launch usv_task3_executor task3_vision_stack.launch.py \
  detector_python:=$HOME/venvs/robotx-gpu/bin/python
```

只有监督下的正式测试才显式开启相应输出，例如
`dry_run:=false enable_goto_output:=true`。云台、水泵分别还需要
`start_aiming:=true start_gimbal:=true` 和
`start_pump:=true auto_pump:=true`；默认全部关闭。

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
| `/tongji_mrl/twave/task3/coarse_dock` | `std_msgs/msg/String` | Publishes a center-only mock record marked with `source: "semantic_map_mock"`, `mock: true`, and default `frame_id: "robotx_rtk_ned"`. |

The selected dock ID defaults to `0`. Run the adapter and executor in separate terminals:

```bash
ros2 launch usv_task3_executor task3_semantic_map_mock_adapter.launch.py
ros2 launch usv_task3_executor task3_executor.launch.py
```

Set another ID with `dock_id:=N`. Mock publication is disabled when
`dry_run:=false`. The executor starts the bounded center-search flow after it
has both the center and a fresh USV pose with the same `frame_id`. Formal inputs
from UAV semantic mapping and USV positioning both use the fixed shared RTK frame
`robotx_rtk_ned`; a mismatch is rejected instead of silently mixing coordinates.

## USV pose from shared RTK NED

正式运行直接订阅公共定位节点发布的 `/tongji_mrl/twave/ned_position`。其 USV 位置、
UAV 位姿和 UAV 语义地图必须都源于同一个固定 RTK 参考点，并明确填写
`frame_id=robotx_rtk_ned`。Task3 不再进行 UAV-NED 到 USV-NED 原点转换。
executor 默认通过 `required_ned_frame_id:=robotx_rtk_ned` 拒绝其他坐标系。

下面的 `task3_ned_pose_adapter` 只用于没有共享 RTK 输出时的 MAVROS 本地里程计测试：

`task3_ned_pose_adapter` converts MAVROS ENU odometry into the Task 3 NED
contract, including heading and sequence number:

```bash
ros2 launch usv_task3_executor task3_ned_pose_adapter.launch.py
```

It subscribes to `/mavros/local_position/odom` and publishes
`/tongji_mrl/twave/ned_position` with frame `usv_fcu_local_ned`. This frame is
local to the FCU and must not be relabelled as `robotx_rtk_ned`. Do not run it
at the same time as the shared RTK publisher.

使用这条旧测试路径时，还必须显式让 executor 接受该测试 frame：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  required_ned_frame_id:=usv_fcu_local_ned
```

Only for a dry-run message deliberately written in that FCU-local frame, override:

```bash
ros2 launch usv_task3_executor task3_semantic_map_mock_adapter.launch.py \
  frame_id:=usv_fcu_local_ned
```

Setting the same label does not transform coordinates. Formal semantic-map data
must keep `robotx_rtk_ned`; legacy data with a different origin must be converted
offline before it enters Task3.

For a selected object, the published JSON is:

```json
{"dock_id":"0","center_ned":{"north":1.0,"east":2.0,"down":3.0},"timestamp":123.456,"valid":true,"source":"semantic_map_mock","mock":true,"frame_id":"robotx_rtk_ned"}
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
