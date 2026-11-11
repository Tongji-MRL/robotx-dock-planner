# Task3 与 Task4 主状态机的接口

本次修改对象：`task3_usv_only_executor`，即新的 USV-only 子状态机。
旧的 `task3_executor` 保留。停车、判断船是否在禁行区内、圈内退出、
撤销底层控制目标以及水泵/云台停止，均由主状态机与执行层负责。
Task3 的 `release_control=true` 是释放控制请求，不是“船已停稳”的证据。

## Task4 Core：存档与恢复

收到 `/tongji_mrl/twave/task_cmd`（`std_msgs/String` JSON）：

```json
{"vehicle":"usv","task":"TASK4","command":{"type":"TASK4_BEGIN"}}
```

Task3 冻结进度、停止产生新目标，向 `/tongji_mrl/twave/checkpoint` 发出存档。
重复暂停不会覆盖第一次存档，也不会把原步骤变成 `PAUSED`。
主状态机需等待存档并完成自己的停车/任务切换；Task3 不执行 Task4 导航。

存档增加四种概括阶段，但保留细分动作：

| resume_phase | 保留内容与恢复方式 |
|---|---|
| EXPLORING | 搜索线、站点、轮次；从新位姿继续检查路线 |
| DOCK_KNOWN | 三入口布局、当前 Bay 观察位；先到观察位，再重新确认入口 |
| CORRECT_BAY_KNOWN | 正确 Bay、窗口 ID、喷水进度；重验入口和绿灯，不能直接恢复喷水 |
| RETURNING | 已完成喷水的事实；避免再次喷水，等待主状态机确认已安全离开 Dock |

`RETURNING` 在当前执行器里对应退出 Dock 到任务完成，尚不表示回到比赛起点。

存档还包含 `checkpoint_id`、`saved_step`、动作已用时间、重试次数、坐标系和布局。
Core 暂停不消耗原动作超时预算，不受原来的 30 秒故障恢复超时约束。
普通定位失效仍按原故障超时处理。Core 结束后使用返回的真实 ID：

```json
{"vehicle":"usv","task":"TASK3","command":"resume","checkpoint_id":"从存档复制","motion_stopped":true}
```

恢复要求新鲜定位、正确坐标系、MAVROS 健康；有禁行区等待条件时不能恢复。
若存档已确认喷水完成，还必须提供 `dock_exit_completed:true`，Task3 才报告完成。
该字段必须来自主状态机的实际退出确认，不能为跳过流程随意设置。

存档目前支持同一进程内暂停/恢复，并输出可保存的 JSON；没有实现进程崩溃后的反序列化恢复。

## Task4 Advanced：持续的导航约束

直接向 `/tongji_mrl/twave/task3/navigation_constraint` 发布完整 JSON 快照：

```json
{
  "run_id": 7,
  "revision": 1,
  "motion_stopped": true,
  "active": true,
  "inside": false,
  "frame_id": "robotx_rtk_ned",
  "center_ned": {"north": 30.0, "east": 12.0},
  "radius_m": 5.0,
  "clearance_m": 1.5
}
```

这些是队内接口字段，不是官方 RoboCommand 字段。`clearance_m` 示例值只是演示，
实际需覆盖船体外廓和定位/控制误差。GPS 圆心先按比赛统一 RTK 原点转换为 NED，
不能直接把经纬度当米传入。Task3 不重新判断 `inside`：

- `inside:true`：等待主状态机退出禁行区，不自行向最近圆边开船。
- `inside:false`：继续原 Task3，检查每个导航线段是否穿过扩张后的圆。
- 路径穿圆：计算顺、逆时针两条“切线—圆弧—切线”候选，选择较短且完全位于
  四角 geofence 内的路线，逐航点绕过后接回原 Task3 目标。
- 实际规划半径为 `radius_m + clearance_m + keep_out_detour_margin_m`；默认额外
  余量 0.5 m，圆弧航点默认间距不超过 1.0 m，均可通过 launch 参数调整。
- 坐标系不同、起终点位于扩张圆内、或两侧绕行都会越界：禁止下发目标、
  请求释放控制并报告明确原因。
- 入坞与出坞同样检查禁行区，不受 `avoidance_enabled` 的终端开关影响。
- 不设自动过期时间；START/RESET 不清掉禁行区。

AllClear：

```json
{"run_id":7,"revision":2,"motion_stopped":true,"active":false}
```

`revision` 每次更新递增；旧版本被忽略，同版本不同内容被拒绝。
圈内退出后，主状态机应发更高版本的 `active:true, inside:false` 完整快照。
AllClear 只移除导航约束，不恢复 Core、不清重试次数、不重新开始 Task3。

也可通过 `task_cmd` 的 `task=TASK4`、`command.type=TASK4_KEEP_OUT` 或
`TASK4_ALL_CLEAR` 携带上述字段，省略 `active`，由类型确定。
主状态机尚未改成发送这些新字段，需要对接；这里没有改主状态机仓库。

## 当前边界与调试

新增 `/tongji_mrl/twave/task3/state`，可看 `resume_phase`、`core_paused`、
`navigation_constraint`、`navigation_hold_reason` 和 `retry_counts`。

- `WAIT_MAIN_ZONE_EXIT`：等待主状态机完成退出。
- `KEEP_OUT_START_INSIDE_CLEARANCE`：船仍处于扩张圆内，交回主状态机退出。
- `KEEP_OUT_TARGET_INSIDE_CLEARANCE`：原目标位于禁区，需主状态机调整任务目标。
- `KEEP_OUT_NO_DETOUR_INSIDE_GEOFENCE`：两侧圆弧都会越过四角边界。
- `KEEP_OUT_FRAME_MISMATCH`：修正输入坐标系。
- `INVALID_NAVIGATION_CONSTRAINT`：修正快照并以更高版本重发。

本次已实现圆形禁行区的 Task3 局部绕行路径。物理障碍物仍走现有公共避障接口；
新执行器输出的 `avoidance_enabled` 只是交给导航适配层的标志，不能据此声称
实船避障已联调通过。禁行区绕行是几何参考路径，底层控制器仍须执行船体碰撞余量、
动态障碍物避让和航点跟踪；没有可行圆弧时不会把圆删掉或径直穿过去。

沿用新执行器的 JSON NED 航点接口。若底层使用原 `PointStamped` ENU goto，
仍需原有 NED/ENU 和消息适配，不能将两种消息类型直接连同一个 topic。
导航完成需先收到当前目标的 RUNNING 再接收 DONE，防止暂停前的旧 DONE 推进步骤。

节点重启后主状态机须先重发当前完整约束快照，再允许启动任务。
本次仅本地纯逻辑与隔离 ROS 节点检查；没有在 Orin 或实船验证。
