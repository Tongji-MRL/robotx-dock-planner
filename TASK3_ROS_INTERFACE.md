# Task3 子状态机与 FSM 主规划器 ROS 2 接口

更新时间：2026-10-18
外部协议：FSM `mission_planner_node` 简化 JSON（`fsm-simple-v1`）

主规划器以 `Tongji-MRL/FSM` 的 `mission_planner_node.py` 为准。Task3 ROS 边界使用
`std_msgs/msg/String`，进度上报的核心字段为 `vehicle/task/progress/error`；主规划器发给
车辆的命令另带 `command`。泊位、喷水窗口、颜色序列和 checkpoint 作为同一 JSON 中的
结构化字段传输。原来的 `task3-ros-v1` envelope 和 14 位状态码只保留在纯 Python
内部诊断，不作为主规划器线上协议。详细表见 `TASK3_FSM_ENCODING.md`。

## 1. 责任边界

`task3_communication_adapter` 负责按 FSM 格式校验和转换 Task3 progress、正确泊位结果、
喷水窗口结果、命令 ACK/转发以及 UAV/USV heartbeat 超时监测。

该节点不直接控制船、PX4、推进器、泵或云台。FSM progress 中的 `*_ACK` 只代表命令通过
校验并转发给 Task3 executor，不代表动作完成。

## 1.1 统一 Orin 工作区中的归属

当前实现位于 `usv_task3_executor`，作为 Task3 侧的 ROS 适配器和联调版本。统一 Orin
工作区的最终归属应保持以下边界：

- `usv_state_machine` 发布整条 USV 的 heartbeat，并把当前任务和 Task3 子状态作为字段；
- Task3 子状态机产生 `correct_bay` 和 `sprayer_window` 事件，但不单独发布车辆 heartbeat；
- `robotx_communication` 负责通用转发；FSM 主规划器负责本组 `task_cmd`、`progress`、`result`、`status` 和 `fault`；
- `task3` 保留 bay 选择、靠泊完成和喷水完成语义；14 位 Task3 码只用于本地诊断，
  不作为 FSM 线上字段。

在通信负责人确认字段前，不要把本文件中的 adapter 再复制成第二个可执行包。最终同步到
`~/robotx_ws/src` 时，应通过 staging 组合为 `robotx_communication`、`task3` 和
`usv_state_machine` 三个职责目录。

## 2. Topic

全部使用 `std_msgs/msg/String`，内容为 UTF-8 JSON。

| Topic | 方向 | 频率/时机 | 内容 |
|---|---|---|---|
| `/tongji_mrl/twave/task_cmd` | 总规划器 → adapter | 事件触发 | Task3/Task4 命令 |
| `/tongji_mrl/twave/task_ack` | adapter → 本地诊断 | 每条有效命令 | FSM progress ACK 的镜像 |
| `/tongji_mrl/twave/task3/control_command` | adapter → Task3 executor | 每条首次有效命令 | 规范化命令；实际执行方必须另行报告结果 |
| `/tongji_mrl/twave/task3/state` | Task3 executor → adapter | 当前 1 Hz | executor 状态 |
| `/tongji_mrl/twave/progress` | adapter → FSM | 状态/窗口变化时 | `vehicle/task/progress/error` JSON |
| `/tongji_mrl/twave/task3/correct_bay` | Task3 流程 → adapter | bay 确认/靠泊确认时 | 正确泊位权威输入 |
| `/tongji_mrl/twave/task3/sprayer_window` | 喷水流程 → adapter | 窗口状态变化时 | RED/GREEN、泵状态、绿色确认时长 |
| `/tongji_mrl/twave/result` | adapter → FSM | 权威完成事件 | docking/firefighting result JSON |
| `/tongji_mrl/twave/checkpoint` | Task3 流程 → FSM | Task4 抢占前 | 当前任务断点 |
| `/tongji_mrl/twave/heartbeat` | USV 状态机 → adapter | 2 Hz | T-Wave 车辆 heartbeat |
| `/tongji_mrl/tsky/heartbeat` | UAV 状态机 → adapter | 2 Hz | T-Sky 车辆 heartbeat |

adapter 只订阅和监测 heartbeat，不伪造 GPS、姿态或车辆模式。两台车辆各自负责发布真实
heartbeat。默认超过 3 秒未收到视为失联；启动后默认有 5 秒宽限期。

## 3. Progress

现有 `/tongji_mrl/twave/task3/state` 会映射为 FSM 可直接判断的 progress。例如：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": "TASK3_SEARCH_DOCK",
  "error": null,
  "detail": "",
  "retry": null
}
```

可恢复的感知、搜索和颜色确认问题不会立即填充 `error`。Task3 executor 会在当前步骤
内部重试，期间只在状态消息中带 `retry`，例如：

```json
"retry": {
  "operation": "bay_color",
  "attempt": 2,
  "max_attempts": 3,
  "elapsed_s": 4.2,
  "timeout_s": 30.0,
  "reason": "bay color confidence is below threshold"
}
```

只有达到 `max_attempts` 或 `timeout_s` 后，executor 才进入 `ERROR`，并把最终错误写入
`error`；FSM 主规划器随后按失败流程处理。输入格式错误、坐标系/边界安全错误等不可安全
重试的问题仍会立即进入 `ERROR`。默认值可通过 executor 参数
`retry_max_attempts`（3）和 `retry_operation_timeout_s`（30 秒）调整。

Task3 完成时，内部状态 `FINISHED`/`COMPLETE` 统一转换为 FSM 要求的：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": "TASK3_FINISH",
  "error": null
}
```

不能把 `TASK3_COMPLETE` 当作对外完成状态；`TASK3_COMPLETE` 仅可作为内部结构化报告名。
FSM 只有在 UAV 和 USV 都收到 `TASK3_FINISH` 且 `error` 为 `null` 时，才会把总状态推进到
`COMPLETE`。

14 位状态码仍可保留在 Task3 内部日志和错误目录中，但不能出现在发送给 FSM 的 JSON。
FSM 主规划器只解析 progress 字符串和 error 字段。完整简化表见
`TASK3_FSM_ENCODING.md`；内部错误目录仍见 `TASK3_ERROR_CATALOG.md`。

## 4. 正确泊位信息

Task3 流程在确认正确 bay 后发布，并在最终靠泊确认后以同一格式再次发布：

```json
{
  "run_id": 42,
  "bay_id": 2,
  "dock_id": "dock-A",
  "confidence": 0.94,
  "confirmed_at_unix_ms": 1791900000000,
  "docking_confirmed": false,
  "source": "task3_flow"
}
```

约束：

- `bay_id` 只能为 1、2、3；
- `confidence` 为 `[0,1]`；
- `docking_confirmed=false` 时只生成 `TASK3_CORRECT_BAY` progress，让总规划器及时获得
  `dock_id/bay_id/confidence`；
- `docking_confirmed=true` 时除 progress 外再生成正式 docking report；
- 感知到绿色 bay、入口对齐或进入入口均不能代替最终靠泊确认。

输出到 `/tongji_mrl/twave/result` 的核心字段：

```json
{
  "task": "TASK3",
  "phase": "DOCKING_COMPLETE",
  "authority": "USV",
  "official_report": {
    "type": "docking",
    "vehicle_id": "T-Wave",
    "bay_id": 2
  }
}
```

## 5. 喷水窗口信息

窗口状态输入：

```json
{
  "run_id": 42,
  "window_id": 1,
  "color": "GREEN",
  "confidence": 0.92,
  "observed_at_unix_ms": 1791900005000,
  "pump_off": true,
  "green_confirmed_duration_s": 5.0,
  "completion_confirmed": true,
  "source": "watering_controller"
}
```

每条有效窗口信息都会形成 `TASK3_SPRAYER_WINDOW` progress。只有同时满足以下四个条件，
才生成正式 firefighting report：

1. `color == GREEN`；
2. `pump_off == true`；
3. `green_confirmed_duration_s >= 5.0`；
4. `completion_confirmed == true`。

正式报告核心字段：

```json
{
  "task": "TASK3",
  "phase": "FIREFIGHTING_COMPLETE",
  "authority": "USV",
  "official_report": {
    "type": "firefighting",
    "vehicle_id": "T-Wave",
    "window_id": 1
  }
}
```

RED、未停泵、绿色不足 5 秒或未得到流程控制器确认时只上报 progress。

## 6. Heartbeat

两个 heartbeat topic 都使用 OCS 车辆协议 `protocol_version=1`：

```json
{
  "protocol_version": "1",
  "message_id": "uuid",
  "sent_at_unix_ms": 1791900000000,
  "run_id": 42,
  "vehicle_id": "T-Wave",
  "source": "T-Wave",
  "frame_id": "wgs84",
  "kind": "heartbeat",
  "payload": {
    "state": "AUTO",
    "current_task": "TASK3",
    "position": {
      "frame_id": "wgs84",
      "latitude_deg": 1.28,
      "longitude_deg": 103.85,
      "altitude_m": 0.0
    },
    "speed_mps": 0.3,
    "heading_deg": 90.0,
    "health": {
      "fcu_connected": true,
      "sensors_ok": true
    }
  }
}
```

Topic 与 `vehicle_id` 必须匹配。未知数据使用 `null`，不能用 0 冒充；位置整体未知时可将
`position` 设为 `null`。正式连接 OCS 前仍需保证 OCS 对未知位置的处理方式一致。

## 7. Command、ACK 与 Task4 恢复

主规划器实际发送的命令以 FSM 节点为准：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": null,
  "error": null,
  "command": "begin"
}
```

Task4 使用：

```json
{
  "vehicle": "usv",
  "task": "TASK4",
  "progress": null,
  "error": null,
  "command": {
    "type": "TASK4_BEGIN",
    "command_seq": 17,
    "target_vehicle": "usv",
    "position": {"frame_id": "ned", "north": 1.0, "east": 2.0}
  }
}
```

总规划器发送 `command="end"` 时，Task3 adapter 将其转换为 `SAFE_STOP`，并回传
`TASK3_END_ACK`。这不是 `TASK3_FINISH`：子状态机需要先取消当前导航、关闭水泵、停止云台，
再由导航/控制模块执行返航；如果返航失败，才上报最终错误供总规划器判断是否需要外部救援。

Task3 的 `begin`、`resume`、`end` 分别映射到内部的开始、恢复和安全停止命令。首次有效
命令被转发到 `/tongji_mrl/twave/task3/control_command`；重复的 FSM 命令不重复转发。

Task4 到达目标后，Task3 必须保持 `WAIT_RESUME`。只有总规划器选择继续 Task3 并向
`/tongji_mrl/twave/task_cmd` 下发 `task=TASK3, command=resume` 后才恢复；恢复时携带
checkpoint 的 `progress` 字段。总规划器选择 Task1 时，Task3 checkpoint 保留，Task3 不自行恢复。

## 8. 构建和 dry-run 联调

```bash
cd ~/robotx_ws
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash

# 只有源码或依赖变化后才重新构建；统一工作区不再单独 source livox_ws/mavros_ws
colcon build --base-paths src --symlink-install --executor sequential
source install/setup.bash
ros2 launch usv_task3_executor task3_communication_adapter.launch.py run_id:=42
```

该启动只运行通信 adapter，不启动导航、PX4、泵或云台。
