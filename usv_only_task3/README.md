# RobotX Task 3 USV-only 子状态机

## 与原流程的差异

| 项目 | 原流程 | USV-only 流程 |
|---|---|---|
| Dock 初始信息 | 等 UAV 粗中心或语义地图 | 主状态机给四个 geofence GPS 角点 |
| 搜索路线 | 围绕粗中心的圆弧 | 四边形内部主搜索线和备用搜索线 |
| 搜索终止 | 圆弧耗尽后重试 | 主线、备用线、有限搜索轮次后报错 |
| Dock 确认 | 单个可用入口可进入颜色判断 | 三个入口必须通过多帧稳定确认 |
| Bay 编号 | 依观测顺序尝试 | 船面对 Dock 时从左到右固定为 1、2、3 |
| Bay 判断 | 当前入口直接看颜色 | 逐个驶到 Bay 观察位确认绿色 Bay |
| 任务结束 | `FINISHED` | 独立 `COMPLETE`，随后上报 `TASK3_FINISH` |

保留 `geofence_search`、`DockPlanner`、三入口稳定器、视觉 Bay/窗口结果、
`safe_passage` 航点反馈、避障仲裁和现有 FSM progress 转换。旧
`task3_executor` 保留作回退，新流程由 `task3_usv_only_executor` 启动。

## 状态、超时和失败转移

| 状态 | 进入条件 | 完成条件 | 默认超时 | 失败转移 |
|---|---|---|---:|---|
| `IDLE` | 初始化或 RESET | START | 无 | 无 |
| `WAIT_INPUTS` | START | 四角点、新鲜 RTK 位姿、正确 frame、MAVROS 正常 | 30 s | `RETRY`，耗尽 `ERROR` |
| `PLAN_SEARCH` | 输入齐全 | 生成两条搜索线 | 5 s | 非法边界 `ERROR` |
| `TRANSIT_TO_SEARCH` | 计划完成/新一轮 | 控制器 `DONE` | 180 s | `RETRY`，耗尽 `ERROR` |
| `SEARCH_PRIMARY` | 到主线 | 三入口稳定或主线结束 | 300 s | 稳定转验证；线结束转备用线 |
| `SEARCH_SECONDARY` | 主线无结果 | 三入口稳定或备用线结束 | 300 s | 有限整轮重试，耗尽 `ERROR` |
| `VERIFY_DOCK` | 三入口多帧稳定 | 验证 ACK | 15 s | `RETRY`，耗尽 `ERROR` |
| `APPROACH_DOCK` | 布局验证 | 到首个 Bay 观察位 | 120 s | `RETRY`，耗尽 `ERROR` |
| `INSPECT_BAYS` | 到观察位 | 高置信绿色 Bay | 120 s | 换观察位；全部失败后有限重试 |
| `ENTER_BAY` | 目标 Bay 确认 | 控制器 `DONE` | 120 s | `RETRY`，耗尽 `ERROR` |
| `HOLD_POSITION` | 到 Bay 内停车点 | 停稳确认 | 20 s | `RETRY`，耗尽 `ERROR` |
| `AIM_WINDOW` | 停稳 | 云台 ready | 90 s | `RETRY`，耗尽 `ERROR` |
| `SPRAY_WATER` | 云台 ready | 喷水完成确认 | 90 s | `RETRY`，耗尽 `ERROR` |
| `REPORT_RESULT` | 喷水完成 | 主状态机/通信层 ACK | 15 s | `RETRY`，耗尽 `ERROR` |
| `EXIT_DOCK` | 结果 ACK | 控制器确认退出 | 120 s | `RETRY`，耗尽 `ERROR` |
| `COMPLETE` | 退出完成 | 终态 | 无 | 发布 `TASK3_FINISH` |
| `PAUSED` | RTK/frame/MAVROS/人工接管/主状态机暂停 | 健康恢复并 RESUME | 30 s | 恢复超时 `ERROR` |
| `RETRY` | 可恢复失败 | 重新发起该状态 | 10 s | 次数耗尽 `ERROR` |
| `ERROR` | 重试耗尽/不可恢复错误 | RESET | 终态 | 无 |
| `ABORTED` | ABORT/END/SAFE_STOP | RESET | 终态 | 无 |

`PAUSED/RETRY/ERROR/ABORTED` 不发布新航点。`dry_run=true` 只发布预览。

## 输入

| Topic | 类型 | 字段/语义 |
|---|---|---|
| `/tongji_mrl/twave/task_cmd` | String JSON | `task=TASK3`；START/PAUSE/RESUME/ABORT/RESET |
| `/tongji_mrl/twave/task3/geofence` | String JSON | `frame_id=robotx_rtk_ned` 和四角点 |
| `/tongji_mrl/twave/ned_position` | String JSON | `position_ned_m`、`heading_rad`、时间戳、frame |
| `/tongji_mrl/twave/task3/vehicle_health` | String JSON | `mavros_connected`、`manual_takeover` |
| `/mavros/state` | `mavros_msgs/State` | 实际 FCU 连接状态；未连接时立即暂停 |
| `/tongji_mrl/twave/task3/dock_perception` | String JSON | 现有 `entrance_observations[]` |
| `/safe_passage/goto/status` | String | `DONE` 或 `ABORTED reason=...` |
| `/tongji_mrl/twave/task3/dock_verified` | Bool | 三入口布局验证 ACK |
| `/tongji_mrl/twave/task3/bay_observation` | String JSON | `color`、`confidence` |
| `/robotx/task3/hold_confirmed` | Bool | Bay 内停车稳定 |
| `/tongji_mrl/twave/task3/gimbal_ready` | String JSON | `ready`、可选 `window_id` 1/2 |
| `/tongji_mrl/twave/task3/spray_complete` | Bool | 喷水完成 |
| `/tongji_mrl/twave/task3/result_ack` | Bool | 主状态机已收结果 |

四角点顺序不作假设。可直接输入 NED：

```json
{"frame_id":"robotx_rtk_ned","corners_ned":[
 {"north":0,"east":0,"down":0},{"north":20,"east":0,"down":0},
 {"north":20,"east":10,"down":0},{"north":0,"east":10,"down":0}]}
```

也可输入 GPS 和建立公共 NED 的同一个 RTK 原点：

```json
{"frame_id":"robotx_rtk_ned",
 "rtk_origin_gps":{"latitude_deg":31,"longitude_deg":121,"altitude_ellipsoid_m":5},
 "corners_gps":[
  {"latitude_deg":31,"longitude_deg":121,"altitude_ellipsoid_m":5},
  {"latitude_deg":31.0001,"longitude_deg":121,"altitude_ellipsoid_m":5},
  {"latitude_deg":31.0001,"longitude_deg":121.0001,"altitude_ellipsoid_m":5},
  {"latitude_deg":31,"longitude_deg":121.0001,"altitude_ellipsoid_m":5}]}
```

GPS 高度须与 RTK 原点采用相同的椭球高定义。

## 输出

| Topic | 类型 | 内容 |
|---|---|---|
| `/tongji_mrl/twave/task3/state` | String JSON | state/detail/error/bay_id/window_id/search 信息及完成标志 |
| `/tongji_mrl/twave/task3/goto_preview` | String JSON | NED 高层目标预览 |
| `/safe_passage/goto/target` | String JSON | 仅关闭 dry-run 且启用输出时发布 |
| `/tongji_mrl/twave/task3/release_control` | Bool | true 时公共控制器须释放旧目标 |
| `/tongji_mrl/twave/task3/perception_mode` | String | 搜索时 `STOP_SCAN`，其他 `TRACK` |

航点 JSON 含 `avoidance_enabled`。Dock 外为 true，`ENTER_BAY` 为 false，表示公共避障层
切换到 Dock 通道策略；紧急停车、定位失效和通道占用保护仍由公共安全层保留。

## 明确的完成状态

`REPORT_RESULT` 和 `EXIT_DOCK` 均不是任务完成。只有退出完成才发布：

```json
{"state":"COMPLETE","task_complete":true,"fsm_progress":"TASK3_FINISH"}
```

现有 `executor_state_to_fsm_progress()` 会将 `COMPLETE` 映射为主状态机使用的
`progress=TASK3_FINISH`。主状态机应以此判定整个 Task 3 完成。

当前另一个 `FSM` 仓库的 `mission_planner_node.py` 仍同时等待 UAV 和 USV 两条
`TASK3_FINISH`。在 USV-only 比赛配置中，主状态机还必须改为只等待 USV，或显式配置
`task3_requires_uav=false`；这不是子状态机可以通过伪造 UAV 完成消息解决的。

## 启动

```bash
ros2 launch usv_task3_executor task3_usv_only_executor.launch.py
```

默认 `dry_run=true`。真实航点输出必须同时设置：

```bash
ros2 launch usv_task3_executor task3_usv_only_executor.launch.py \
  dry_run:=false enable_goto_output:=true
```

LiDAR 输入选择仍由现有感知 launch 决定：优先 `/cloud_registered_body`，不可用时用
`/livox/lidar`。状态机只消费统一 `dock_perception`，没有复制或修改感知算法。
