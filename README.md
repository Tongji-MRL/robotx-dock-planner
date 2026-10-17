# DockPlanner V0.1

独立纯 Python 几何规划器，仅依赖标准库，建议 Python 3.8+。
不依赖 reference 仓库、固定场景、ROS 2 或 Gazebo。

## 使用

在本目录执行：

```bash
python3 -m examples.basic
python3 -m unittest discover -s tests -v
```

```python
from dock_planner import DockPlanner, DockSpec, PlannerConfig, Pose2D

planner = DockPlanner(PlannerConfig(approach_distance=4, stop_distance_inside=2))
result = planner.plan(
    Pose2D(2, 3, 0),
    DockSpec(10, 20, 1.5707963267948966, bay_width=3, bay_depth=5),
    obstacles=None,
)
```

## 坐标与结果契约

- 全部输入输出在调用者提供的同一个二维笛卡尔坐标系；距离为米。
- yaw 为从 +X 轴逆时针旋转的弧度，不要求输入归一化。
- `entrance_heading` 明确指向港内，不能直接使用 dock 本体 pose 的 yaw。
- `bay_width` 是入口净宽，`bay_depth` 是从入口朝内的净深。
- approach = entrance − approach_distance × 朝内单位方向。
- stop = entrance + stop_distance_inside × 朝内单位方向。
- approach_distance > 0；0 < stop_distance_inside < bay_depth。
- waypoints 顺序固定为 USV、approach、entrance、stop，共四个语义点。
  即使 USV 与 approach 重合，也保留两个点以表达各自姿态。
- 第一个 waypoint 保留输入 USV yaw；其余三个使用 entrance_heading。
- `SUCCESS` 只表示几何规划成功，不表示实际靠泊成功。
- 这是稀疏 waypoint 折线，不是可直接执行的光滑轨迹；不检查初始转向、
  船体尺寸、曲率、运动学、碰撞或 USV 到 approach 段是否穿过 dock。
  bay_width 当前仅校验有效性，stop_depth 只约束参考点而非整船占地。
- 非法输入返回 INVALID_INPUT、空 waypoints、空 approach_pose 和 reason。
- `obstacles` 仅接受 None 或空 list/tuple。非空输入返回 UNSUPPORTED_INPUT，
  避免在尚未实现避障时给出误导性的成功结果。
- DockSpec 的 confidence（None 或 [0,1]）与 dock_id（None 或字符串）
  仅为预留 metadata，不参与目标选择或几何计算。

## 文件

- `dock_planner/types.py`：输入、输出 dataclass 和状态枚举。
- `dock_planner/planner.py`：输入校验、几何计算和单目标 API。
- `dock_planner/integration.py`：Task 3 感知、终端入口和通用速度的集成契约。
- `dock_planner/dock_search.py`：中心点粗定位到安全圆边缘与有限圆弧目标的纯几何层。
- `dock_planner/ros_twist_adapter.py`：通用速度命令到 ROS `Twist` 的消息适配。
- `dock_planner/task3_flow.py`：不依赖 ROS 的搜索、颜色判定和终端规划流程骨架。
- `dock_planner/dock_perception.py`：离线 NumPy 感知原型和 synthetic mock；当前不接 ROS。
- `dock_planner/color_sequence.py`：颜色序列通信契约；不依赖 ROS。
- `dock_planner/sprayer_interfaces.py`：云台/喷水感知结果的数据契约。
- `dock_planner/watering.py`：停泊后的瞄准、喷水和绿色确认状态逻辑；不直接控制硬件。
- `dock_planner/bay_search.py`：入口中心关联、bay 去重、颜色拒绝和 terminal 失败记忆。
- `dock_planner/task3_status_codec.py`：Task3 14 位状态码、固定错误目录和恢复动作解码。
- `dock_planner/task3_subfsm_bridge.py`：Task3 子状态机与总规划器通信、Task4 抢占和恢复逻辑。
- `dock_planner/task3_ros_contract.py`：Task3 侧 progress/report/command/ACK、正确泊位、喷水窗口和双机 heartbeat 的纯 JSON 契约；统一 Orin 中的通用通信归 `robotx_communication`，车辆 heartbeat 由 `usv_state_machine` 发布。
- `dock_planner/__init__.py`：公开导入接口。
- `TASK3_COMMUNICATION_PROTOCOL.md`：Task3 步骤、全部错误码、command、跨机数据和 Task4 恢复协议。
- `TASK3_ERROR_CATALOG.md`：Task3 128 个步骤特有错误及通用错误的独立查表文档。
- `TASK3_ERROR_CATALOG.csv`：供总规划器程序或表格软件读取的错误目录。
- `TASK3_ROS_INTERFACE.md`：供总规划器和 Task3 联调的 ROS 2 Topic、消息样例、完成门控与启动方法。
- `TASK3_ABNORMAL_SCENARIOS.md`：任务前、中、后 158 项异常场景、当前覆盖状态和处置方案。
- `TASK3_READINESS_CHECKLIST.md`：从离线逻辑到比赛级闭环的完成度清单。
- `examples/basic.py`：打印一组示例输入和结果。
- `examples/__init__.py`：使示例可通过模块方式运行。
- `tests/test_planner.py`：平移、旋转、起点变化、非法输入及预留接口测试。

## Task 3 已确定的实际执行流程

以下流程是 Task 3 后续真实接入的固定流程，仿真 thrust 控制器只是独立的验证路径：

1. **UAV 粗定位**：UAV 至少提供 dock ID、转换到 USV NED 后的粗中心、timestamp
   和 valid；如果以后能提供四个大致角点，则使用 `CoarseDockInfo` 保存。角点只是
   粗几何，不能作为精确边界，也不能直接推导入口。
2. **Approach Dock**：中心点方案根据已知 dock 外形尺寸、停船距离和定位误差计算
   搜索半径，先生成靠近当前 USV 一侧的圆周 staging 目标。
3. **Dock Local Search**：USV 使用 Mid-360 LiDAR 发现 dock 结构、dock 方向和入口候选。
   LiDAR 不负责颜色识别。
4. **Bay Inspection**：USV 在一个入口附近调整姿态，使相机朝向入口后方墙面，然后把
   camera image 交给颜色识别模型。
5. **Bay Decision**：颜色符合目标时选择该入口；否则退出当前入口，沿 dock 方向继续
   搜索下一个入口，重复局部定位、姿态调整和颜色识别。
6. **Terminal Docking**：选定入口后生成 `PreciseEntrancePoseNED`，交给 DockPlanner
   生成 align、entrance、stop 等终端目标。
7. **速度输出**：上层控制逻辑将终端控制结果表示为 `GenericVelocityCommand`，再由
   `RosTwistAdapter` 转为 `/usv/control/cmd_vel` 的 `Twist`。

当前仓库提供数据边界、流程状态机、中心点 approach/有限圆弧目标生成，以及只用于
离线 synthetic 测试的 NumPy dock 感知原型。它不包含生产可用的 Mid-360/PointCloud2
接入、颜色模型、真实 USV 运动控制或 ArduPilot 连接。`gz_full_docking.cc` 继续使用
已有左右 thrust 控制，不属于上述真实 USV 速度输出链路。

点云处理的边界是 `DockPerception`：未来 ROS adapter 将 `sensor_msgs/msg/PointCloud2`
交给外部感知实现，再返回带时间戳的 `DockPerceptionResult`；`Task3FlowManager` 只消费
结果，不解析点云。颜色处理同理通过 `ColorDetector` 消费视觉模型输出。

```text
UAV coarse center in USV NED
        |
        v
safe-circle staging goal
        |
        v
bounded circular-arc goals + Mid-360 entrance search
        |
        v
camera pose adjustment -> color detection -> bay decision
        |                         |
        | target color            | other color
        v                         v
PreciseEntrancePoseNED       exit and next bay search
        |
        v
DockPlanner -> align / entrance / stop
        |
        v
GenericVelocityCommand -> RosTwistAdapter -> /usv/control/cmd_vel
```

## Task 3 感知输入与集成接口

`dock_planner/integration.py` 定义独立于 ROS 2、Gazebo、UAV 通信和全局规划实现的
输入契约。NED 点使用 `north/east/down`，单位米；平面 `x_ned=north`、
`y_ned=east`。`heading_rad` 从北朝东为正，指向泊位内部；数值上与 core 在
`x=north, y=east` 下使用的 `yaw` 一致。接入方必须统一 NED 原点和时间戳时钟，
并把传感器原生坐标和航向转换到这里。`DataFreshnessConfig` 负责纯逻辑层的
粗定位和近距离感知超时判断。

```text
CoarseDockInfo -> USV approach -> Mid-360 dock perception -> entrance candidates
                                                        |
                                                        v
camera image -> color detection -> selected bay -> PreciseEntrancePoseNED
                                                        |
                                                        v
                                         DockSpec -> DockPlanner -> terminal targets
                                                        |
                                                        v
                                         GenericVelocityCommand -> ROS Twist adapter
```

- `CoarseDockInfo` 的四个 `approximate_corners_ned` 只校验数量、点类型、
  有限坐标及彼此不同；不假设角点顺序、矩形、物理边界或入口位置。`valid=False`
  表示观测不可用于转换。
- `CoarseDockCenterObservationNED` 是中心点方案的独立粗定位输入，适合在 UAV
  暂时只提供一个 NED 中心点时做 approach/search mock；它不会被当作四个角点或精确入口。
  `CoarseDockSearchConfig` 用已知 dock 外接尺寸、船体包络、尺寸测量余量、停船距离和
  定位误差计算搜索半径，默认值可由现场标定覆盖。当前默认半径约为 `7.643 m`，其中
  已计入 `0.92 m` 船体包络和 `0.10 m` 粗测余量。`coarse_dock_staging_pose()` 根据当前
  USV 位置把中心点转换成搜索圈上的临时 staging 点；它不生成入口 pose，也不替代
  Mid-360 局部感知。
- `classify_coarse_dock_circle_position()` 明确区分圆外、已到圆周、圆内、进入不可缩减
  最小包络和与粗中心重合五种情况。圆内时 staging 位于同一径向的外侧，船先退出到
  圆周；已在圆周时 `Task3FlowManager` 直接进入搜索；与粗中心重合时不猜测退出方向。
- `CircularDockSearchConfig` 和 `plan_circular_dock_search()` 从 staging 点开始生成
  一段可配置方向、弧长和点间距的圆弧目标。默认只搜索半圆。这里生成的是目标点，
  不会驱动船运动，也不保证整圈无障碍；真实任务必须由 executor/控制器逐点执行，
  并由任务层确认所选圆弧处于可航水域。
- `DockEntranceObservationNED` 表示 Mid-360/USV 近距离感知给出的入口中心、朝内 heading、
  confidence、timestamp 和 source；`entrance_observation_to_precise_pose()` 只接受这个
  观测类型，并将有效观测转换为 `PreciseEntrancePoseNED`。
- `DockPerceptionResult` 表示一次 dock 感知更新：`dock_id`、入口候选
  `entrance_observations`、`timestamp`、`source`、`valid` 和可选 `valid_until`。
  `DataFreshnessConfig` 当前默认粗定位超时 30 s、近距离感知超时 2 s；过期结果不会进入
  终端规划。
- `BayColorObservation` 表示感知组颜色模型的输出。项目只消费 color、confidence、
  timestamp 和 source，不实现或绑定具体颜色识别模型。
- `PreciseEntrancePoseNED` 表示 USV 完成 Mid-360 局部搜索、入口姿态调整和颜色判定后
  选中的最终入口，包括有效性、时间戳、可选置信度和可选 dock ID。当前仓库只保留
  这个结果接口，不实现上游感知和 bay 选择。
- `ObstaclePointNED` 仅有 `center_ned`，没有尺寸或半径。
  `GlobalApproachPlanner` 是协议接口：输入当前 pose、approach goal、障碍物点，
  可选 planner 侧 `safety_radius` 与 NED 边界点序列；输出 waypoint path 或失败原因。
  这里不实现避障、膨胀、A* 或 RRT*。
- `prepare_terminal_plan_input()` 仅在粗观测和精确入口均有效时，将精确入口与
  **另行提供的泊位净宽/净深**转换成现有 `DockSpec`。可选入口 dock ID 若存在，
  必须匹配粗 dock ID；若入口没有 ID，调用方必须确认关联关系。粗中心和角点
  不参与入口 pose 推算。函数保留 obstacle points；将非空点传入现有 core 时，
  core 仍返回 `UNSUPPORTED_INPUT`，不会报告避障成功。
- core 的 `SUCCESS` 仍仅表示 terminal docking 语义目标几何生成成功。Approach、
  局部搜索、颜色决策和 terminal control 由外层流程协调；DockPlanner 不承担这些感知
  或控制职责。泊位净宽/净深、感知有效期和安全半径仍由外部接口提供。

### 速度控制接口

`GenericVelocityCommand` 是控制逻辑与具体执行后端之间的内部速度契约，包含
`surge_mps`（前向速度，m/s）、`yaw_rate_rps`（偏航角速度，rad/s）、
`timestamp`、`valid_until`、`phase`、`source` 和 `frame_id`。时间戳使用调用方
选定的时钟域；命令只有在有效时间窗口内才可发布。

`RosTwistAdapter` 将该契约转换为 `geometry_msgs/msg/Twist`：

- 默认 topic：`/usv/control/cmd_vel`
- `linear.x = surge_mps`
- `angular.z = yaw_rate_rps`
- 其它线速度和角速度分量置零

适配器只负责消息转换和发布，不连接或假设真实 ArduPilot；10 Hz 是上层控制契约的
默认发布频率，具体定时器由 ROS 节点负责。当前 `gz_full_docking.cc` 仍使用既有
左右推力控制，不通过该适配器。

最小调用方式：

```python
request = prepare_terminal_plan_input(coarse, precise_entrance, bay_width, bay_depth, obstacle_points)
result = planner.plan(current_pose_ned, request.dock, obstacles=request.obstacles)
```

`current_pose_ned` 是 `Pose2D(north, east, heading_rad)`。已有 VRX adapter
继续使用原接口与世界坐标，不依赖新数据结构。

### 场地边界、重试和喷水完成监督

`mission_safety.py` 提供不依赖 ROS 的任务监督接口：

- `NavigableBoundaryNED` 表示每次场地单独提供的可航行区域多边形；
- `plan_boundary_safe_circular_search()` 同时检查顺/逆时针搜索圆弧，只保留从 staging
  点开始、连续且满足岸边净距的安全段；两侧都不足时返回失败，不生成实船控制；
- `plan_adaptive_boundary_safe_circular_search()` 先使用原始安全半径；只有两个方向都没有
  足够长的安全圆弧时，才按配置减小 stand-off 后重新检查。它不会缩小 dock 尺寸和
  船体包络、尺寸余量、粗定位误差，也不会越过人工配置的最小 stand-off；当前位置到
  staging 的完整直线路段也会做精确几何检查，覆盖船一开始位于搜索圆内、需要向外退出的情况；
- `DockAttemptSupervisor` 同时限制单次尝试时间、总时间和最大尝试次数；
- 达到限制后返回 `EXIT_AND_RETURN`，`plan_exit_and_return()` 生成 dock 外退出目标和
  另行配置的岸边等待目标。真实场地的边界和等待点必须由任务配置提供。

`parse_logistics_request_json()` 解析
`/robotx/perception/task3/logistics_request` 的资源颜色与目标圆颜色。缺字段、
`UNKNOWN`、`NONE` 或 `valid=false` 不会成为可执行序列。通信到 UAV/OCS 仍由 USV
状态机/通信模块负责。

### 颜色序列通信契约

Task3 规划侧固定颜色序列的建议通信接口如下：

```text
topic: /tongji_mrl/twave/task3/color_sequence
type:  std_msgs/msg/Int32MultiArray
data:  [tin_color, circle_color]
```

颜色编码为 `UNKNOWN=0`、`RED=1`、`GREEN=2`、`BLUE=3`、`WHITE=4`、`AMBER=5`。
向量必须恰好有两个元素，第一个是锡罐颜色，第二个是圆圈颜色；任一元素为
`UNKNOWN` 时结果无效。`dock_planner/color_sequence.py` 提供纯 Python 编解码和
校验，ROS publisher/subscriber 仍由通信模块接入。对端确认订阅相同 topic 和消息类型后，
再做正式 ROS 联调。

`WateringCompletionTracker` 在首次收到 `delivery_confirmed` 时请求立即停泵，随后要求
连续确认 GREEN 5 秒才报告完成；感知中断会重新计时，但不会请求自动重启水泵。
它只产生决策，不调用真实水泵接口。

`WateringSequenceController` 将停泊后的喷水步骤串起来：

```text
DOCKED -> WAITING_FOR_RED -> AIMING -> SPRAYING
       -> VERIFYING_GREEN -> COMPLETE
```

红色窗口由 `target_valid=true, delivery_confirmed=false` 表示；云台报告 ready 后
才产生一次 `pump_on_requested`。首次收到绿色窗口时产生一次
`pump_off_requested`，并要求绿色连续保持 5 秒才报告 `watering_complete`。
该类只输出决策，不调用 `/pump/set`，也不直接控制云台；ROS adapter 负责执行这些
决策。

`MotionCommandArbiter` 为 goto、terminal 和安全停止提供显式控制权。只有当前 owner 的
新鲜 `GenericVelocityCommand` 可以通过；缺命令、过期、frame 不符或超过配置限制时
输出零速度。ROS 层必须把 goto/terminal 输出重映射到不同输入 topic，并只让 arbiter
发布最终 `/usv/control/cmd_vel`。

`ObstacleAvoidanceDecision` 是预留给 Task1/Orin 公共避障模块的纯接口，动作包括
`PROCEED`、`HOLD`、`REPLAN` 和 `ABORT`。仲裁器只在收到新鲜的 `PROCEED` 时允许当前
控制器继续输出；其他动作和过期结果都会输出零速度。实船可开启
`require_avoidance_decision=True`，让避障结果缺失也触发停止。该接口不绑定 ROS topic，
后续只需由公共模块 adapter 填充，不需要修改 DockPlanner。

### 异常场景审计

[`TASK3_ABNORMAL_SCENARIOS.md`](TASK3_ABNORMAL_SCENARIOS.md) 将任务前、粗定位与接近、
LiDAR 搜索、颜色与终端靠泊、喷水、返航和系统级故障拆成 158 项。每一项明确标为
已覆盖、部分覆盖或未覆盖，并给出默认安全动作和下一步。当前统计为 52 项已覆盖、
34 项部分覆盖、72 项未覆盖；其中 61 项未覆盖问题属于实船自主控制前必须处理的 P0。

规划安全层现在检查 waypoint 之间的完整线段，而不只检查端点。approach、搜索圆弧、
terminal、退出和返航均可复用相同的多边形求交与净距逻辑；自交或重复顶点的边界会被
拒绝。`BayVisitTracker` 用入口中心距离关联 bay，避免重复检查已拒绝或 terminal 失败的
入口，并在三个预期 bay 全部不可用时结束搜索。
颜色不匹配后，`Task3FlowManager.prepare_non_target_bay_exit()` 会生成“当前船位 →
入口中心 → 入口外侧”的连续退出路径；执行器完成退出后再调用 `next_bay()`，恢复搜索，
不会把船留在错误 bay 内直接继续绕行。
`select_safe_return_goal()` 会优先保留人工配置且安全的等待点；首选点不满足边界净距或
dock 最小距离时，才在边界内确定性选择最近的安全替代点。
Task3 flow 还提供独立的入口和 bay 颜色最低置信度门；低于门限的结果不会触发状态前进。

`pointcloud_adapter.py` 按实际 Mid-360 PointCloud2 字段布局提取 x/y/z，不要求
intensity、tag、line、timestamp 与 XYZ 使用相同 datatype。它支持有限值过滤和质量
统计；默认还会去掉 Mid-360 原始数据中的 `(0,0,0)` 占位点。

`lidar_transform.py` 使用现场测得的 Mid-360 外参把点从 `livox_frame` 转到
`base_link`：当前初始值为 `x=+0.15 m`、`y=0`、`z=+0.22 m`，安装角为 0。入口识别和
避障都应先调用 `pointcloud_message_to_base_link_xyz()` 或
`transform_lidar_points_to_base_link()`，再使用点的位置。这样障碍物位置不会停留在
LiDAR 自身坐标系中。若需要全局 NED 位置，还要继续使用当时的 `base_link → usv_ned`
位姿变换；本模块不猜测该动态位姿。

`examples.export_mid360_bag_frames` 可以只读 rosbag，将抽样后的 PointCloud2 清洗并保存
为独立 `.npy` 帧和 `manifest.json`。该工具只供离线检查，不启动节点、不发布 topic、
不发送控制命令。入口检测和图片输出由独立的
`research/dock_lidar_prototype/run_recorded_frame.py` 完成。

面向 Orin 的公共 ROS 2 包模板位于
`ros2/robotx_perception_common/`。它包含 Task1 和 Task3 共用的 PointCloud2 解析、
`livox_frame → base_link` 转换，以及 `task3_lidar_bringup.launch.py`。部署时将该目录
随本仓库放入 Orin 工作区的 `src/` 后构建；Task3 的 dock 入口识别仍留在 Task3 包中。

Orin 现有的 ROS executor 已收回本仓库：ROS 包元数据和 launch 文件位于
`ros2/usv_task3_executor/`，节点源码位于 `usv_task3_executor/`。该 ROS 包在构建时直接
安装仓库根目录唯一一份 `dock_planner/`，避免现场工作区再维护规划核心副本。把整个仓库
克隆到 colcon 工作区 `src/` 后，可同时发现 `usv_task3_executor` 和
`robotx_perception_common` 两个 ROS 包。统一 Orin 工作区还应由 `robotx_communication`
提供通用通信适配，由 `usv_state_machine` 提供车辆级 heartbeat；当前 Task3 adapter
用于接口联调，暂不在 Task3 包外复制一份。

`ned_map_adapter.py` 实现 UAV 建图 NED 到 USV 本地 NED 的纯坐标适配。它使用 UAV
建图起点和 USV 任务起点的 WGS84 经纬度及椭球高，先计算固定 NED 平移，再把
`/task/semantic_map` 中的 dock 中心转换成 `CoarseDockCenterObservationNED`。转换不
依赖 ROS、OCS 或 PX4，也不会根据车辆当前移动位置重复计算原点。项目接口已确认两套
NED 的北、东、下方向一致，因此当前转换只做平移，不增加 yaw 旋转。

坐标链纯 Python 演示：

```bash
python3 -m examples.task3_ned_map_dry_run
```

随机场地边界、三次失败和退出返航的纯逻辑演示：

```bash
python3 -m examples.task3_safety_recovery_dry_run
```

纯 Python 整链演示（无 ROS、无真实控制、无硬件）：

```bash
python3 -m examples.task3_full_dry_run
```

该演示覆盖粗中心、staging、圆弧搜索、synthetic 点云入口候选、非目标颜色继续搜索、
目标颜色选择、terminal plan、DOCKED、WATERING 和 FINISHED。到达目标点的事件由脚本
模拟，因此不能当作实船运动验证或真实 Mid-360 验证。

## VRX 几何适配/测试层

`dock_planner/vrx_reference.py` 单独提供
`vrx_dock_2022_bay_to_dockspec(dock_world_x, dock_world_y, dock_world_yaw, bay_index)`。
bay_index 为 **1、2、3**；输入 yaw 单位为弧度，输出 heading 归一化到 [-π, π)。
非法输入抛出 ValueError。核心 planner 与其数据结构保持不变。

**VRX_REFERENCE_GEOMETRY**：三个局部入口为 (-1,3)、(-1,9)、(-1,15)，
朝内 +X，每个 bay 净宽 4 m、净深 6 m。这不是 RobotX 2026 官方尺寸，
也不是未来正式模型的默认值。来源为 VRX humble 提交 dc30ed8d 的
`vrx_gz/models/dock_2022_base/model.sdf` 与 `dock_block_4x4/model.sdf`。
入口参考线是碰撞体隔堤前端 x=-1，后墙内沿 x=5。
该模块不读取 VRX、不加载插件，也不要求本机存在 VRX 仓库。

运行坐标表与完整测试：

```bash
python3 -B -m examples.vrx_reference
python3 -B -m unittest discover -s tests -v
```

示例使用官方本地世界 `scan_dock_deliver_task.sdf` 中整体 dock pose
(-480,190,yaw=1.0)，分别规划三个 bay；USV 起点 (-500,180,1.0) 为测试自选，
不是官方出生位置。approach_distance=4，stop_distance_inside=2。
`tests/test_vrx_reference.py` 验证坐标变换、heading、相对位置及 planner 输出顺序。
bay_id 仅在单个整体 dock 内唯一，多 dock 场景应由调用方加入实例前缀。

## Gazebo Garden 静态核对

先生成（不启动仿真）：

```bash
cd ~/robotx_ws/dock_planner_v0
python3 -B -m examples.gazebo_static
python3 -B -m unittest discover -s tests -v
```

打开静态 world：

```bash
bash scripts/run_gazebo_static.sh
```

脚本直接调用 Garden，不启动 ROS 节点；只需已安装的 gz sim 与个人 VRX
模型目录。源码模型路径优先，沿用本机 `LIBGL_ALWAYS_SOFTWARE=1` 设置；
默认保持暂停。关闭窗口或在启动终端 Ctrl-C 结束。

生成器引用原版 `model://dock_2022_base`，整体 pose=(-480,190,0,0,0,1.0)。
所有九个点由现有 adapter 和 DockPlanner 计算，独立写成世界坐标，无重复旋转。
场景没有 WAM-V。测试 USV pose 仅用于满足 planner 输入，不在场景中生成。

- 橙色球：approach，在入口外 4 m。
- 青色球/横线：entrance/入口线，位于隔堤前端。
- 绿色球：stop，在入口内 2 m。
- 黄色箭头：朝港内方向；白线连接三个语义点。
- 所有标记为无碰撞静态 visual；球中心 z=0.7 m，箭头 z=1.02 m，
  高度只用于可视化。入口线长 4 m；九个姿态 heading 均为 1 rad。
- 入口参考线依据碰撞体前端 x=-1；可以在 Entity Tree 对 dock 使用
  右键 View/Collisions（碰撞显示）进一步核对 mesh 与碰撞边界。

`generated/vrx_dock_static.json` 提供完整坐标与来源。
`generated/runtime/` 保存本次 GUI 配置与缓存，日志与截图也放在 generated 下。
`generated/logs/gazebo_static.log` 是本次启动的完整输出；
`generated/logs/tests.log` 是测试结果；`generated/gazebo_static.png` 为窗口截图。
这些均为可重新生成的实验产物。

几何仍是 VRX_REFERENCE_GEOMETRY（4×6 m），不是 RobotX 2026 官方尺寸。
静态显示对齐不等于整船碰撞检查或控制执行验证。

### 静态 WAM-V 对齐检查

当前生成器还会在每个 bay 的 approach、entrance、stop 各放置一艘静态 WAM-V，
共 9 艘。它使用个人 VRX WAM-V base mesh，并从个人 VRX `wamv_base.urdf`
复制完整的 25 个 collision primitive；没有推进器、传感器、控制器或 ROS 插件，所有模型
均为 `<static>true>`，世界重力为零。该 WAM-V 仅用于位置、艏向和净空目视检查。

运行同一命令即可重新生成并打开：

```bash
bash scripts/run_gazebo_static.sh
```

在 Gazebo GUI 中通过 Scene/Visualization 能力打开 `Show collisions`（或选择
实体后启用碰撞显示）。重点检查 entrance 和 stop 的两侧浮筒与隔堤距离；以
VRX_REFERENCE_GEOMETRY 的 4 m bay 宽度，完整碰撞投影的横向范围为 -1.23 到
1.23 m，入口和 stop 两侧各留 0.77 m。stop 位于局部 x=1 m 时，沿 +X 的
后缘距后墙约 1.451 m。approach 船体
应在 bay 外侧。碰撞可视化只显示 SDF 中的静态 primitive，不代表真实动力学。

### WAM-V 直线推进校准工具

`scripts/gz_dual_thrust_calibration.cc` 使用单个 Gazebo Transport node 创建左右推进器
publisher，并在同一 20 Hz 循环中连续发布相同命令。它通过 `HasConnections()` 确认左右
订阅端存在，读取 `dynamic_pose/info` 中的 `wamv`，等待 3 秒后估算初始速度，然后执行
不超过 2 秒、100 N 的受限测试。结束、异常或信号中断后均连续三次发布左右 `0.0`。

编译与运行示例：

```bash
bash scripts/build_gz_dual_thrust_calibration.sh
./generated/gz_dual_thrust_calibration 30 1.0 20
```

参数依次为左右相同推力（N）、持续时间（s）和发布频率（Hz）。应在官方
`/vrx/release` 完成并确认出生区安全后运行；该工具只用于低速直线校准，不执行导航。

### Phase 1：start 到 bay_2 approach

`examples/phase1_target.py` 使用现有 `vrx_dock_2022_bay_to_dockspec()` 与
`DockPlanner.plan()` 生成 bay_2、approach_distance=12 m 的目标，并写入
`generated/phase1_target.json`。控制器不包含手写的 approach 世界坐标。

```bash
python3 -B -m examples.phase1_target
bash scripts/build_gz_phase1_approach.sh
./generated/gz_phase1_approach generated/phase1_target.json
```

`gz_phase1_approach` 使用单个 Gazebo Transport node 在同一个 20 Hz 循环发布左右
推力。基础前进推力为 30 N，单侧限幅 50 N；大航向误差时降低前进分量，目标 10 m
内逐渐减速。接受半径为 1.5 m，超时为 180 s。正常结束、异常、超时或信号中断均会
连续五次发布左右 0.0。本工具只执行 start 到 approach，不追踪 entrance 或 stop。

2026-09-12 的实测结果保存在 `generated/phase1_result.json`：位置目标在 138.751 s
进入接受半径，最终距离 1.430 m，推力成功归零。该次测试只把 bearing 作为控制航向，
因此最终船艏没有对齐规划的 approach yaw；进入 entrance 阶段前应先增加独立的姿态对齐步骤。

### Phase 1.5：approach 航向对齐

`gz_phase1_5_align` 先复用 Phase 1 的位置控制到达 approach，再切换到 ALIGN。
目标文件中的 approach 和 entrance 均由 adapter 与 planner 生成；ALIGN 只读取
`entrance_pose.yaw`，不追踪 entrance 位置。

```bash
python3 -B -m examples.phase1_target
bash scripts/build_gz_phase1_5_align.sh
./generated/gz_phase1_5_align generated/phase1_target.json
```

ALIGN 开始后先保持零推力 2 秒，再使用 `left=-turn, right=turn` 的纯差速转向，单侧
最大 12 N，前进分量为零。航向误差归一化到 [-pi, pi]；误差小于 5 度并连续保持
1.5 秒才成功。ALIGN 漂移超过 2 m 或运行超过 60 秒会失败并归零，不会进入 entrance。

2026-09-12 实测：初始 yaw=0.457796 rad，目标 heading=1.0 rad；31.950 秒后最终
yaw=0.924189 rad，误差 4.345 度，位置漂移 0.193 m，左右推力成功归零。完整摘要见
`generated/phase1_5_result.json`。

### Phase 2：approach 到 entrance

`gz_phase2_entrance` 依次复用 Phase 1 和 ALIGN，只有两者成功后才启动 ENTRANCE。
入口段以 planner 生成的 entrance 轴线计算纵向和横向误差，基础推力 12 N，单侧限幅
20 N。控制器使用最大 0.18 rad 的小航向偏置消除横向误差，并在距离入口 5 m 内减速。

```bash
python3 -B -m examples.phase1_target
bash scripts/build_gz_phase2_entrance.sh
./generated/gz_phase2_entrance generated/phase1_target.json
```

入口接受条件为距离小于 0.9 m 且入口航向误差小于 7 度。控制器订阅
`/vrx/contacts`；ENTRANCE 期间发现 WAM-V collision 会立即停止。距离入口 4 m 内，
结合 4 m VRX bay 和真实 WAM-V footprint，参考点横向偏差限制为 0.70 m；此外还检查
走廊越界、越过入口和 100 秒超时。所有退出路径发布左右 0.0，不进入 stop。

2026-09-12 实测：ENTRANCE 初始横向偏差 0.704 m，90.450 秒后到入口距离
0.864 m，航向误差 -0.693 度，横向偏差 -0.035 m；无 contact、走廊越界、超时或
持续振荡，最大绝对推进器命令 14.502 N，成功归零。官方日志同时报告进入 bay2 的
external dock activation zone。完整摘要见 `generated/phase2_result.json`。

### Phase 3：entrance 到 stop

`gz_phase3_stop` 只执行 entrance → stop，并由 shell 的成功退出门控接在
`gz_phase2_entrance` 后；因此前置流程失败时不会进入 STOP。entrance、stop 和
entrance_heading 均来自 `examples.phase1_target` 写出的 adapter/planner 结果。

```bash
python3 -B -m examples.phase1_target
python3 -B -m examples.phase3_world
bash scripts/build_gz_phase2_entrance.sh
bash scripts/build_gz_phase3_stop.sh
./generated/gz_phase2_entrance generated/phase1_target.json && \
  ./generated/gz_phase3_stop generated/phase1_target.json
```

STOP 基础推力 6 N、单侧限幅 10 N，并在接近 stop 时连续衰减；参考点距 stop
纵向剩余 0.40 m 后锁存零前推。控制器同时约束 0.70 m corridor、stop overshoot、
后墙净空、contact、pose stale、90 s 超时和 publish failure，任何失败路径都重复归零。

完整 Phase 2 前置流程约 261 s，官方 world 的 300 s running 状态不足以完成独立的
低速停车与稳定观察。因此 `examples.phase3_world.py` 从本地官方
`scan_dock_deliver_task.sdf` 生成一个不提交的副本，只把 scoring lifecycle 的
`running_state_duration` 从 300 s 延长到 480 s，并加入 120 s `exit_delay`；几何、物理、
WAM-V、release 和其余语义保持不变，测试会做 XML 语义比对。

2026-09-13 实测：Phase 2 前置成功后，Phase 3 从距 stop 2.861 m 处开始，46.701 s
后稳定停车。最终距 stop 0.377 m，heading error 0.215°，横向误差 0.053 m，最小
后墙净空 1.673 m，最大绝对推力 6.091 N；无 collision、走廊越界、stop overshoot、
后墙风险、超时或振荡，推力成功归零。VRX 日志确认 internal activation 并持续在泊位
10 s 后进入 `<docked>`。完整摘要见 `generated/phase3_result.json`。

### 完整连续流程：start 到 docked

`gz_full_docking` 是独立的连续验证控制器，不修改已经验证过的四个分段控制器。
它显式实现 `GO_TO_APPROACH -> ALIGN -> GO_TO_ENTRANCE -> GO_TO_STOP -> DONE`
状态机，并保留 `FAILED` 终态。只有当前阶段满足原成功条件后才切换；collision、
corridor exceeded、pose stale、阶段或官方任务 timeout、publish failure、stop overshoot
和 back-wall risk 均会立即进入 `FAILED`，随后重复发布双侧零推力。

```bash
python3 -B -m examples.phase1_target
bash scripts/build_gz_full_docking.sh
./generated/gz_full_docking generated/phase1_target.json
```

控制目标仍全部读取 adapter + DockPlanner 生成的 `phase1_target.json`。连续控制器复用
Phase 1/1.5/2/3 参数；官方 300 s 基线中 `GO_TO_APPROACH` 用时 138.500 s，是明确
瓶颈，完整流程在 STOP 阶段被官方时限终止。随后只把连续控制器的 approach 基础推力
从 30 N 提至 45 N；其余阶段逻辑和参数、DockPlanner 几何及 world 时限均未改。

2026-09-13 在未经修改的官方原始 `scan_dock_deliver_task` 上实测成功。控制器从官方
elapsed=10 s 开始，在 elapsed=284 s 进入 `DONE`，控制器总运行 273.451 s，距 300 s
截止尚有 16 s。阶段耗时依次为 103.250、33.150、89.950、46.600 s。最终距 stop
0.379 m，heading error 0.115°，横向误差 0.046 m，最小后墙净空 1.676 m；无
collision、corridor 越界、pose stale、timeout、publish failure、stop overshoot 或
back-wall risk。bay_2 internal contain 连续保持至少 10 s，`docked` 被观察到，双侧
推力成功归零。完整基线、优化和成功结果见 `generated/full_docking_result.json`。
