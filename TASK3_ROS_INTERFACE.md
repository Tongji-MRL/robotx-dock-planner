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
| `/tongji_mrl/twave/result` | adapter → FSM | 权威事件 | docking/firefighting result 或 Task4 `incident_ack` JSON |
| `/tongji_mrl/twave/checkpoint` | Task3 流程 → FSM | Task4 抢占前 | 当前任务断点 |
| `/tongji_mrl/twave/heartbeat` | USV 状态机 → adapter | 2 Hz | T-Wave 车辆 heartbeat |
| `/tongji_mrl/tsky/heartbeat` | UAV 状态机 → adapter | 2 Hz | T-Sky 车辆 heartbeat |

adapter 只订阅和监测 heartbeat，不伪造 GPS、姿态或车辆模式。两台车辆各自负责发布真实
heartbeat。默认超过 3 秒未收到视为失联；启动后默认有 5 秒宽限期。

### 2.1 正式坐标系

Task3 正式导航数据统一使用固定 RTK 参考点建立的 `robotx_rtk_ned`。UAV 语义地图、
UAV 位姿和 `/tongji_mrl/twave/ned_position` 中的 USV 位姿必须携带完全相同的 frame ID；
Task3 只做一致性检查，不再执行 UAV 局部原点到 USV 局部原点的转换。MAVROS
`local_position/odom` 转换得到的 `usv_fcu_local_ned` 仅用于独立测试，不能通过改标签
冒充共享 RTK 坐标。

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

Task3 完成时，子状态机和 FSM 接口统一使用 `FINISHED`。转换层仍兼容旧代码输入的
`COMPLETE`，但不会再向外发布该名称：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": "FINISHED",
  "error": null
}
```

不能把 `TASK3_COMPLETE` 当作对外完成状态；`TASK3_COMPLETE` 仅可作为内部结构化报告名。
FSM 只有在 UAV 和 USV 都收到 `FINISHED` 且 `error` 为 `null` 时，才会把总状态推进到
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

- 比赛图中面对 Dock 正面时，三个 bay 从左到右依次为 `bay_id=1,2,3`；
- `bay_id` 只能为 1、2、3；
- `dock_id` 是整个 Dock/语义地图对象的内部标识，不等于比赛要求上报的 `bay_id`；
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

面对 Dock 正面时，每个 bay 内左侧窗口为 `window_id=1`，右侧窗口为
`window_id=2`。窗口 ID 只有这两个合法值。

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

### 5.1 ID 判定与锁定

Task3 子状态机必须按以下顺序生成事件：

1. 绿色指示灯识别稳定后确定 `bay_id`，发布 `correct_bay`；
2. 进入该 bay 后跟踪亮起窗口；能判断左右时分别记录 `window_id=1/2`；
3. adapter 将 `dock_id + bay_id + window_id` 作为同一次 Task3 的目标上下文保存；
4. 未确认 bay 的窗口消息和范围外 ID 均被拒绝；
5. 新的 `START_TASK3` 或 `RESET` 命令会清空锁定，允许下一次任务重新识别。

`TASK3_SPRAYER_WINDOW` 会附带已锁定的 `dock_id` 和 `bay_id`，便于总状态机确认
窗口属于本次已选择的 bay。正式 docking report 上报 `bay_id`；正式 firefighting
report 上报 `window_id`，两者分别对应比赛要求的两个报告阶段。

入口候选不能按点云检测先后或置信度直接编号。规划流程将同一批入口按“船从水面
正对 Dock 时的左到右”排序后依次检查；完整识别出三个入口时，该顺序才对应
`bay_id=1,2,3`。若当帧只有部分入口，需结合已建立的 Dock 横向坐标轴和历史入口
位置完成关联，不能把第一个被发现的入口直接叫作 bay 1。

`window_id` 是喷水成功后的尽力报告字段，不是瞄准、开泵或绿灯完成判定的前置条件。
相机能判断目标相对当前 bay 正面的左右位置时，左侧为 1、右侧为 2，并保存到本次喷水
上下文。云台 yaw 会随船姿态、零位和跟踪过程变化，不能作为窗口编号依据。若整个喷水
过程中始终只能看到单窗且缺少 bay 中心证据，完成后使用启动参数
`fallback_window_id`（默认 1）发送合法 ID；不得因 ID 不确定延迟喷水或否定绿灯成功。

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
    "center": {"latitude": 31.00001, "longitude": 121.00001},
    "radius_m": 500.0,
    "vehicle_type": "USV"
  }
}
```

USV 接受后在 `/tongji_mrl/twave/result` 返回：

```json
{"incident_ack":{"command_seq":17}}
```

Task4 结束时主状态机发送：

```json
{"vehicle":"usv","task":"TASK4","progress":null,"error":null,"command":"resume"}
```

USV 清除禁行区并恢复原 Task3 后，再用保存的 `command_seq=17` 返回一次同样的
`IncidentAck`。`resume` 本身不携带新序号。

总规划器发送 `command="end"` 时，Task3 adapter 将其转换为 `SAFE_STOP`，并回传
`TASK3_END_ACK`。这不是 `FINISHED`：子状态机需要先取消当前导航、关闭水泵、停止云台，
再由导航/控制模块执行返航；如果返航失败，才上报最终错误供总规划器判断是否需要外部救援。

Task3 的 `begin`、`resume`、`end` 分别映射到内部的开始、恢复和安全停止命令。首次有效
命令被转发到 `/tongji_mrl/twave/task3/control_command`；重复的 FSM 命令不重复转发。

禁行区在 Dock 操作期间触发时，Task3 保存 Dock 进度、退出禁区并等待上述 Task4
`resume`；恢复后重新验证 Dock 和瞄准结果，不能沿用旧结果直接续喷。圈外航行则把
禁行区作为持续约束进行绕行。

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
