# Task3 与 FSM 的简化编码表

版本：`fsm-simple-v1`  
依据：`Tongji-MRL/FSM/src/mission_planner_node.py`

## 1. 线上消息只保留 JSON 字段

FSM 主规划器不解析 14 位整数、`status_bits` 或错误二进制表。USV 在
`/tongji_mrl/twave/progress` 发布 `std_msgs/msg/String`，最小格式是：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": "TASK3_SEARCH_DOCK",
  "error": null
}
```

允许在这四个字段旁边增加结构化结果字段（例如 `bay_id`、`window_id`、颜色和
置信度）。主规划器只用四个核心字段做状态转换：

| 字段 | 允许值/含义 |
|---|---|
| `vehicle` | `uav` 或 `usv` |
| `task` | `MAP`、`TASK1`、`TASK3`、`TASK4` |
| `progress` | 本表中的字符串；完成必须为 `FINISHED` |
| `error` | 正常为 `null`；最终失败为可读字符串 |

普通进度上报不需要 `command`。`command` 只出现在主规划器发给车辆的
`/tongji_mrl/twave/task_cmd` 消息中。

## 2. Task3 progress

| progress | 含义 |
|---|---|
| `TASK3_READY` | 已收到开始命令，准备检查依赖 |
| `TASK3_WAIT_DOCK_OBSERVATION` | 等待 UAV dock 粗中心 |
| `TASK3_APPROACH_DOCK` | 前往安全观察圆周 |
| `TASK3_SEARCH_DOCK` | 沿圆周搜索入口 |
| `TASK3_CHECK_BAY_COLOR` | 检查 bay 颜色和目标泊位 |
| `TASK3_NEXT_BAY` | 当前 bay 不匹配，退出并尝试下一 bay |
| `TASK3_TERMINAL_DOCKING` | 执行最终靠泊 |
| `TASK3_DOCKED` | 靠泊确认 |
| `TASK3_WAIT_RED_WINDOW` | 等待红色喷水窗口 |
| `TASK3_AIMING` | 云台瞄准 |
| `TASK3_SPRAYING` | 正在喷水 |
| `TASK3_VERIFY_GREEN` | 停泵并确认绿色窗口 |
| `TASK3_COLOR_SEQUENCE` | 识别并上报颜色序列 |
| `TASK3_WAIT_UAV_DELIVERY` | 等待 UAV 接收/投放 |
| `TASK3_EXIT_RETURN` | 退出 dock 并返航 |
| `FINISHED` | Task3 完成；FSM 以此推进总状态 |
| `TASK3_ERROR` | Task3 最终失败，`error` 必须非空 |

`TASK3_BAY_CONFIRMED`、`TASK3_DOCKING_RESULT`、`TASK3_SPRAYER_WINDOW` 和
`TASK3_FIREFIGHTING_RESULT` 是结果/事件进度，可携带结构化字段，但不会代替
`FINISHED`。

## 3. 错误处理

子状态机内部可以重试。重试期间保持当前 `progress`，增加可选的 `retry` 字段，
`error` 仍为 `null`。达到次数或时间上限后才发送：

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": "TASK3_ERROR",
  "error": "LIDAR_NO_DATA"
}
```

错误字符串必须来自 Task3 错误目录；主规划器只要求它是非空字符串并转入失败流程，
不会按 14 位表解码。

## 4. 主规划器命令

### 开始 Task3

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": null,
  "error": null,
  "command": "begin"
}
```

### 结束/安全停止

```json
{
  "vehicle": "usv",
  "task": "TASK3",
  "progress": null,
  "error": "...",
  "command": "end"
}
```

Task3 adapter 收到 `end` 后只做安全停止转发：取消导航、关闭水泵、停止云台，
并发送 `TASK3_END_ACK`。这不表示 Task3 完成。

### Task4 抢占

FSM 使用对象命令 `command.type="TASK4_BEGIN"`，携带 `command_seq`、
`target_vehicle` 和 `position`。Task3 adapter 保存 checkpoint 后向执行器转发，
并发送 `TASK4_ACK`。Task4 完成后，主规划器发送 `resume`，Task3 根据 checkpoint
继续当前流程。

## 5. 泊位、喷水和 heartbeat

- 正确泊位：发送 `progress="TASK3_BAY_CONFIRMED"`，附 `bay_id`、`dock_id`、
  `confidence`、`docking_confirmed`。
- 靠泊结果：发送 `progress="TASK3_DOCKING_RESULT"`，附 `official_report`。
- 喷水窗口：发送 `progress="TASK3_SPRAYER_WINDOW"`，附窗口颜色、泵状态和
  绿色确认时长。
- 喷水完成：满足绿色、停泵、绿色持续至少 5 秒且 `completion_confirmed=true` 后，
  发送 `TASK3_FIREFIGHTING_RESULT`。
- heartbeat 不使用 Task3 14 位码；继续使用独立 heartbeat topic。失联时在 progress
  上报 `progress="HEARTBEAT_STALE"`、`error="COMMUNICATION_LOST"`。

## 6. 兼容边界

`dock_planner/task3_status_codec.py` 和 `TASK3_ERROR_CATALOG.md` 可以继续用于本地
日志、调试和恢复策略，但不能把 `status_code` 或 `status_bits` 放进发送给 FSM 的
progress 消息。`fsm_contract.py` 和 `task3_communication_adapter` 是线上接口的唯一
转换边界。
