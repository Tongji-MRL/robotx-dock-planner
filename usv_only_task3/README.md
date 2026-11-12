# RobotX Task 3 USV-only 子状态机

## 与原流程的差异

| 项目 | 原流程 | USV-only 流程 |
|---|---|---|
| Dock 初始信息 | 等 UAV 粗中心或语义地图 | 主状态机转发官方 course_config 的四个 GPS 角点 |
| 搜索路线 | 围绕粗中心的圆弧 | 四边形内部主搜索线和备用搜索线 |
| 搜索终止 | 圆弧耗尽后重试 | 主线、备用线、有限搜索轮次后报错 |
| Dock 确认 | 单个可用入口可进入颜色判断 | 三个入口必须通过多帧稳定确认 |
| Bay 编号 | 依观测顺序尝试 | 船面对 Dock 时从左到右固定为 1、2、3 |
| Bay 判断 | 当前入口直接看颜色 | 逐个驶到 Bay 观察位确认绿色 Bay |
| 任务结束 | `FINISHED` | 最终状态和上报值均为 `FINISHED` |

保留 `geofence_search`、`DockPlanner`、三入口稳定器、视觉 Bay/窗口结果、
`safe_passage` 航点反馈、避障仲裁和现有 FSM progress 转换。旧
`task3_executor` 保留作回退，新流程由 `task3_usv_only_executor` 启动。

## 状态、超时和失败转移

| 状态 | 进入条件 | 完成条件 | 默认超时 | 失败转移 |
|---|---|---|---:|---|
| `IDLE` | RESET 后 | 自动进入等待 | 无 | 无 |
| `WAIT_INPUTS` | launch 启动 | course_config、Orin 本机 GPS/odom、MAVROS 正常 | 无 | 输入齐全后自动开始 |
| `PLAN_SEARCH` | 输入齐全 | 生成两条搜索线 | 5 s | 非法边界 `ERROR` |
| `TRANSIT_TO_SEARCH` | 计划完成/新一轮 | 控制器 `DONE` | 180 s | `RETRY`，耗尽 `ERROR` |
| `SEARCH_PRIMARY` | 到主线 | 三入口稳定或主线结束 | 300 s | 稳定转验证；线结束转备用线 |
| `SEARCH_SECONDARY` | 主线无结果 | 三入口稳定或备用线结束 | 300 s | 有限整轮重试，耗尽 `ERROR` |
| `VERIFY_DOCK` | 三入口多帧稳定 | 默认自动通过结构验证 | 15 s | 结构无效或超时后 `RETRY`，耗尽 `ERROR` |
| `APPROACH_DOCK` | 布局验证 | 到首个 Bay 观察位 | 120 s | `RETRY`，耗尽 `ERROR` |
| `INSPECT_BAYS` | 到观察位 | 控制器确认到达后，高置信绿色 Bay | 120 s | 换观察位；未到新观察位时忽略颜色；全部失败后有限重试 |
| `ENTER_BAY` | 目标 Bay 确认 | 控制器 `DONE` | 120 s | `RETRY`，耗尽 `ERROR` |
| `HOLD_POSITION` | 到 Bay 内停车点 | 停稳确认 | 20 s | `RETRY`，耗尽 `ERROR` |
| `AIM_WINDOW` | 停稳 | 云台 ready | 90 s | `RETRY`，耗尽 `ERROR` |
| `SPRAY_WATER` | 云台 ready | 喷水完成确认 | 90 s | `RETRY`，耗尽 `ERROR` |
| `REPORT_RESULT` | 喷水完成 | 整理最终 `bay_id/window_id` 后自动进入退出流程 | 15 s | ID 非法则 `ERROR` |
| `EXIT_DOCK` | 结果 ACK | 控制器确认退出 | 120 s | `RETRY`，耗尽 `ERROR` |
| `FINISHED` | 退出完成 | 终态 | 无 | 发布 `FINISHED` |
| `PAUSED` | RTK/frame/MAVROS/人工接管/主状态机暂停 | 健康恢复并 RESUME | 30 s | 恢复超时 `ERROR` |
| `RETRY` | 可恢复失败 | 重新发起该状态 | 10 s | 次数耗尽 `ERROR` |
| `ERROR` | 重试耗尽/不可恢复错误 | RESET | 终态 | 无 |
| `ABORTED` | ABORT/END/SAFE_STOP | RESET | 终态 | 无 |

`PAUSED/RETRY/ERROR/ABORTED` 不发布新航点。`dry_run=true` 只发布预览。

## 输入

| Topic | 类型 | 字段/语义 |
|---|---|---|
| `/tongji_mrl/twave/course_config` | String JSON | 正式课程配置；`corners` 为四个 GPS 经纬度对象 |
| `/mavros/state` | `mavros_msgs/State` | 实际 FCU 连接状态；未连接时立即暂停 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | USV 自己的 RTK/GNSS 位置；Task3 在 Orin 内转换为 NED |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | 与 RTK NED 位置同步，用于把规划目标转换到控制器的本地 map/ENU 坐标 |
| `/tongji_mrl/twave/task3/dock_perception` | String JSON | 现有 `entrance_observations[]` |
| `/safe_passage/goto/status` | String | `DONE` 或 `ABORTED reason=...` |
| `/tongji_mrl/twave/task3/dock_verified` | Bool | 可选外部验证 ACK；`auto_verify_stable_dock=true` 时不需要 |
| `/tongji_mrl/twave/task3/bay_observation` | String JSON | `color`、`confidence` |
| `/robotx/task3/hold_confirmed` | Bool | Bay 内停车稳定 |
| `/tongji_mrl/twave/task3/gimbal_ready` | String JSON | `ready`、可选 `window_id` 1/2 |
| `/tongji_mrl/twave/task3/spray_complete` | Bool | 完整执行“红灯、瞄准、喷水、绿灯、停泵、绿色稳定 5 秒”后才可置 true |

启动 launch 后，Task3 自动进入 `WAIT_INPUTS`；收到课程配置、MAVROS GPS、MAVROS
odom 和已连接的 FCU 后自动开始搜索，不需要 Task3 `START` 命令。主状态机只需原样转发
官方 `/tongji_mrl/twave/course_config`，例如：

```json
{
  "schema_version":"twave-course-config-v1",
  "course_id":"RX2026-A",
  "corners":[
    {"latitude":1.2801,"longitude":103.8501},
    {"latitude":1.2801,"longitude":103.8503},
    {"latitude":1.2803,"longitude":103.8503},
    {"latitude":1.2803,"longitude":103.8501}
  ]
}
```

Task3 将第一角作为本轮本地 NED 原点，并用 Orin 本机 MAVROS 的 RTK/GNSS 位姿转换到
同一坐标系。`corners` 必须恰好包含四个互不重复的 GPS 点。

## 输出

| Topic | 类型 | 内容 |
|---|---|---|
| `/tongji_mrl/twave/progress` | String JSON | 只包含主状态机需要的 `vehicle/task/progress/error` |
| `/tongji_mrl/twave/result` | String JSON | 入坞稳定后发布 `DockingReport`；喷水验证完成后发布 `FirefightingReport` |
| `/tongji_mrl/twave/task3/goto_preview` | String JSON | NED 高层目标预览 |
| `/safe_passage/goto/target` | `geometry_msgs/PointStamped` | 本地 `map`/ENU 目标；仅关闭 dry-run、启用输出且 NED/odom 样本同步时发布 |
| `/safe_passage/goto/cancel` | Bool | 暂停、ABORT、错误或控制释放时取消旧目标；控制器用 `ABORTED` 状态确认 |
| `/tongji_mrl/twave/task3/action_preview` | String JSON | 云台和水泵请求预览，dry-run 中也可观察 |
| `/tongji_mrl/twave/task3/gimbal_request` | String JSON | `AIM_WINDOW` 或 `STOP`；仅启用辅助输出时发布 |
| `/tongji_mrl/twave/task3/spray_request` | String JSON | `START` 或 `STOP`；仅启用辅助输出时发布 |
| `/tongji_mrl/twave/task3/release_control` | Bool | true 时公共控制器须释放旧目标 |
| `/tongji_mrl/twave/task3/perception_mode` | String | 搜索时 `STOP_SCAN`，其他 `TRACK` |

航点 JSON 含 `avoidance_enabled`。Dock 外为 true，`ENTER_BAY` 为 false，表示公共避障层
切换到 Dock 通道策略；紧急停车、定位失效和通道占用保护仍由公共安全层保留。

## 明确的完成状态

不使用状态位或二进制编码。progress 与比赛报告分开；`REPORT_RESULT` 和
`EXIT_DOCK` 均不是任务完成，只有退出完成才发布：

```json
{
  "vehicle":"usv",
  "task":"TASK3",
  "progress":"FINISHED",
  "error":null
}
```

Bay 内停车稳定后另行发布：

```json
{"team_id":"TONG","vehicle_id":"usv","seq":1,
 "sent_at":"2026-10-17T10:12:00Z","docking":{"bay_id":2}}
```

喷水绿色稳定确认后另行发布：

```json
{"team_id":"TONG","vehicle_id":"usv","seq":2,
 "sent_at":"2026-10-17T10:15:00Z","firefighting":{"window_id":1}}
```

每条 result 只含一种报告主体。主状态机应转发 result，并以
`progress=FINISHED` 判定整个 Task 3 完成。窗口左右无法稳定判断时使用配置的合法
fallback ID（默认 1），不会因此永久卡住。`report_seq_start` 用于与整车公共报告序号衔接；
如果公共通信节点统一分配序号，应由该节点覆盖此参数。

当前另一个 `FSM` 仓库的 `mission_planner_node.py` 仍同时等待 UAV 和 USV 两条
`FINISHED`。在 USV-only 比赛配置中，主状态机还必须改为只等待 USV，或显式配置
`task3_requires_uav=false`；这不是子状态机可以通过伪造 UAV 完成消息解决的。

## 启动

```bash
ros2 launch usv_task3_executor task3_usv_only_executor.launch.py
```

默认 `dry_run=true`。真实航点输出必须同时设置：

```bash
ros2 launch usv_task3_executor task3_usv_only_executor.launch.py \
  dry_run:=false enable_goto_output:=true enable_aux_output:=true
```

`enable_aux_output=false` 时只发布动作预览，不会请求云台或水泵。真实运行前，云台/水泵
适配节点必须确认消费请求话题；`spray_complete=true` 的生产者必须承担红转绿和持续 5 秒
验证，子状态机不会把未经验证的单帧绿色当作完成。

LiDAR 输入选择仍由现有感知 launch 决定：FAST-LIO 模式默认使用
`/fast_lio/cloud_registered_body`（`fast_lio_body`），不可用时用 `/livox/lidar`。
状态机只消费统一 `dock_perception`，没有复制或修改感知算法。

比赛核心链路可用一个 launch 启动 Livox/避障桥、MAVROS、FAST-LIO、入口感知、
航点控制和 USV-only executor：

```bash
ros2 launch usv_task3_executor task3_competition_core.launch.py \
  dry_run:=false control_enabled:=true
```

它不启动总状态机，也不会伪造 RTK 数据。总状态机只须将官方
`/tongji_mrl/twave/course_config` 转发给 USV；Task3 在 Orin 内读取 MAVROS 的 GPS 和 odom。
云台/水泵适配完成前保持
`enable_aux_output:=false`；该参数为 false 时不会发送喷水动作。

Orin 比赛运行使用一条命令：`scripts/start_task3_competition_staged.sh`。它在 tmux 中依次
启动 Livox/避障桥、MAVROS、FAST-LIO、导航与 Task3 状态机、OAK/云台、水泵、泊位灯和
窗口通信适配；每阶段收到下一层所需 topic 后才继续。自动水泵链路默认启动，但泵仍保持关闭，
只有收到有效红色目标且云台锁定后才会喷水。台架排障才使用 `--no-pump`。

## Task4 Core / Advanced 衔接

参见 [Task4 接口与恢复说明](TASK4_HANDOFF.md)。新执行器支持四阶段存档、
Core 暂停冻结动作计时、恢复入口复核，以及主状态机下发的圆形禁行区约束。
`PAUSED` 的原 30 秒超时只用于普通故障；Core 暂停等待主状态机显式恢复。
禁行区穿越检查和绕行覆盖搜索、入坞与退出。
对于船外的圆形禁行区，执行器会在四角 geofence 内生成切线和圆弧绕行航点，
选择顺、逆时针中较短的可行路线；无边界内可行路线时释放控制并报告原因。
