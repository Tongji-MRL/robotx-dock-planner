# Task3 与 Task4 主状态机的接口

本次修改对象：`task3_usv_only_executor`，即新的 USV-only 子状态机。
旧的 `task3_executor` 保留。Task3 自己根据实时 RTK 位姿判断船是否在禁行区，
并负责生成圈内退出点或圈外绕行点。主状态机只转发官方区域消息。

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

官方协议字段已经固定为：

```proto
message KeepOutZone { LatLng center = 1; float radius_m = 2; VehicleType vehicle_type = 3; }
message AllClear { VehicleType vehicle_type = 1; }
```

主状态机通过 `/tongji_mrl/twave/task_cmd` 转发，不需要计算圈内外：

```json
{
  "run_id": 7,
  "command": "TASK4_KEEP_OUT_ZONE",
  "parameters": {
    "official_command_seq": 18,
    "vehicle_type": 1,
    "center": {"latitude": 31.0, "longitude": 121.0},
    "radius_m": 5.0
  }
}
```

Task3 使用 geofence 输入中的 `rtk_origin_gps` 将 GPS 圆心转换到
`robotx_rtk_ned`。若区域先于 RTK 原点到达，立即停止产生旧目标并报告
`WAIT_RTK_ORIGIN_FOR_KEEP_OUT`，原点到达后自动继续。

- 每次收到实时 `/tongji_mrl/twave/ned_position` 都重新计算到圆心的距离。
- 位于 `radius_m + keep_out_safety_clearance_m` 内：撤销旧目标，生成最近的
  geofence 内安全退出点；默认安全余量 1.0 m。
- 离开安全半径后：从当前真实位置重新检查原 Task3 目标并继续任务。
- 位于圈外：检查每个导航线段是否穿过扩张后的圆。
- 路径穿圆：计算顺、逆时针两条“切线—圆弧—切线”候选，选择较短且完全位于
  四角 geofence 内的路线，逐航点绕过后接回原 Task3 目标。
- 绕行半径为 `radius_m + keep_out_safety_clearance_m + keep_out_detour_margin_m`；
  默认再增加 0.5 m，圆弧航点默认间距不超过 1.0 m，均可通过 launch 参数调整。
- 坐标系不同、起终点位于扩张圆内、或两侧绕行都会越界：禁止下发目标、
  请求释放控制并报告明确原因。
- 入坞与出坞同样检查禁行区，不受 `avoidance_enabled` 的终端开关影响。
- 不设自动过期时间；START/RESET 不清掉禁行区。

AllClear：

```json
{"run_id":7,"command":"TASK4_ALL_CLEAR","parameters":{"official_command_seq":19,"vehicle_type":1}}
```

AllClear 只移除导航约束，不恢复 Core、不清重试次数、不重新开始 Task3。
节点也兼容带 `task=TASK4` 的嵌套命令，以及已经转换成 NED 的内部
`navigation_constraint` 快照，便于独立调试。

## 当前边界与调试

新增 `/tongji_mrl/twave/task3/state`，可看 `resume_phase`、`core_paused`、
`navigation_constraint`、`navigation_hold_reason`、`keep_out_mode`、
`keep_out_inside_official_zone`、`keep_out_inside_protected_zone` 和 `retry_counts`。

- `WAIT_RTK_ORIGIN_FOR_KEEP_OUT`：缺少共享 RTK 原点，禁止继续旧路线。
- `WAIT_TASK3_GEOFENCE_FOR_KEEP_OUT_EXIT`：已知船在圈内，但还没有安全边界。
- `KEEP_OUT_TARGET_INSIDE_CLEARANCE`：原目标位于禁区，需主状态机调整任务目标。
- `KEEP_OUT_NO_EXIT_INSIDE_GEOFENCE`：找不到位于安全边界内的退出点。
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
