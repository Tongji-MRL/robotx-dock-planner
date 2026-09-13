# DockPlanner V0 工作进度与续接说明

更新时间：2026-09-12（Asia/Shanghai）

本文档用于在新的 Codex thread 中无损继续 RobotX 2026 Task 3 靠泊规划工作。当前已完成 Phase 2 动态验证；**尚未开始 Phase 3，也不应在读取本文档后自动运行 Phase 3**。

## 1. 项目目标与工作区边界

项目目标是开发 RobotX 2026 Task 3 的 USV docking planner，使规划器接收 USV 位姿和泊位入口几何，生成：

```text
USV pose -> approach pose -> entrance pose -> stop pose
```

当前阶段先验证几何语义和 WAM-V 的基本可执行性。Planner 的 `SUCCESS` 只表示几何规划成功，不表示真实船已经完成靠泊，也不包含动力学可行性、避障或碰撞保证。

- 个人实验工作区：`/home/y/robotx_ws`
- 当前项目：`/home/y/robotx_ws/dock_planner_v0`
- 宋词留下的旧规划仓库：`/home/y/robotx_ws/references/plan_world`，只作为 reference，未修改
- 本地官方 VRX：`/home/y/robotx_ws/src/vrx`，只读使用，未修改
- 团队正式工作区：`/home/y/robotx_team_ws/robotx_workspace`，当前实验阶段不得修改
- 团队仿真仓库位于正式工作区的 `src/robotx_sim`，当前没有向其中同步代码

当前 `dock_planner_v0` 本身不是 Git 仓库；不要假定存在可提交的 Git 状态。本轮没有执行 `git push`。

## 2. 当前环境与仿真入口

- ROS：ROS 2 Humble
- Gazebo：Gazebo Garden / Gazebo Sim 7（`gz-sim7`，此前环境检查为 7.9.0）
- 本地 VRX 源码：`/home/y/robotx_ws/src/vrx`
- 已安装工作区：`/home/y/robotx_ws/install`
- 官方 VRX world：`scan_dock_deliver_task`
- world 文件：`/home/y/robotx_ws/src/vrx/vrx_gz/worlds/scan_dock_deliver_task.sdf`
- 启动脚本：`/home/y/robotx_ws/run_vrx.sh`

已验证的启动方式：

```bash
bash /home/y/robotx_ws/run_vrx.sh \
  world:=scan_dock_deliver_task \
  sim_mode:=sim \
  headless:=true
```

`run_vrx.sh` 会 source `/opt/ros/humble/setup.bash` 和 `/home/y/robotx_ws/install/setup.bash`，设置 Gazebo resource path，再通过 `ros2 launch vrx_gz competition.launch.py` 启动比赛 world。

动态测试使用 Gazebo Transport 的本地 socket。在受限 sandbox 中可能需要允许相关命令在 sandbox 外启动。交接时没有 Gazebo、launch 或控制器进程在运行。

## 3. DockPlanner V0.1 当前设计

### 3.1 数据结构

定义在 `dock_planner/types.py`：

```python
Pose2D(x: float, y: float, yaw: float)

DockSpec(
    entrance_center_x: float,
    entrance_center_y: float,
    entrance_heading: float,
    bay_width: float,
    bay_depth: float,
    confidence: float | None = None,
    dock_id: str | None = None,
)

PlannerConfig(
    approach_distance: float,
    stop_distance_inside: float,
)

PlanResult(
    approach_pose: Pose2D | None,
    waypoints: list[Pose2D],
    status: PlanStatus,
    reason: str,
)
```

`PlanStatus` 包含 `SUCCESS`、`INVALID_INPUT`、`UNSUPPORTED_INPUT`。

### 3.2 approach / entrance / stop 生成逻辑

`entrance_heading` 明确定义为**从入口中心朝泊位内部的方向**，不能直接用 dock model pose 的 yaw 代替。

令：

```text
u = (cos(entrance_heading), sin(entrance_heading))
```

则 `DockPlanner.plan()` 生成：

```text
approach = entrance_center - approach_distance * u
entrance = entrance_center
stop     = entrance_center + stop_distance_inside * u
```

三个目标 pose 的 yaw 都是 `entrance_heading`。当前 `waypoints` 顺序是：

```text
[输入 USV pose, approach pose, entrance pose, stop pose]
```

`obstacles=None` 或空列表可以传入；非空 obstacles 返回 `UNSUPPORTED_INPUT`。V0.1 没有 RRT*、动力学模型、正式 ROS 2 接口或控制器依赖。

### 3.3 VRX dock_2022_base adapter

适配器在 `dock_planner/vrx_reference.py`：

```python
vrx_dock_2022_bay_to_dockspec(
    dock_world_x,
    dock_world_y,
    dock_world_yaw,
    bay_index,
) -> DockSpec
```

VRX 参考局部几何：

| Bay | 局部入口中心 | 局部朝内 heading | 宽度 | 深度 |
|---|---:|---:|---:|---:|
| bay_1 | `(-1, 3)` | `0 rad` | `4 m` | `6 m` |
| bay_2 | `(-1, 9)` | `0 rad` | `4 m` | `6 m` |
| bay_3 | `(-1, 15)` | `0 rad` | `4 m` | `6 m` |

局部到世界的转换：

```text
x_world = dock_x + cos(dock_yaw) * x_local - sin(dock_yaw) * y_local
y_world = dock_y + sin(dock_yaw) * x_local + cos(dock_yaw) * y_local
entrance_heading_world = wrap(dock_yaw + local_inward_heading)
```

代码用 `VRX_REFERENCE_GEOMETRY` 标识这组几何。**4 m × 6 m 只是本地 VRX 模型的参考尺寸，不是 RobotX 2026 官方尺寸，不能固化为未来正式 Task 3 模型。**

### 3.4 当前 bay_2 目标

动态验证统一使用：

```text
dock world pose = (-480.0, 190.0, yaw=1.0 rad)
bay_index = 2
approach_distance = 12.0 m
stop_distance_inside = 2.0 m
```

adapter 和 planner 自动计算：

| 点 | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| approach | `-494.59716883955684` | `183.92359795031060` | `1.0` |
| entrance | `-488.11354116913920` | `194.02124976800536` | `1.0` |
| stop | `-487.03293655740290` | `195.70419173762115` | `1.0` |

这些数值只用于核对。新控制程序仍必须调用 `vrx_dock_2022_bay_to_dockspec()` 和 `DockPlanner.plan()`，不能复制坐标。

## 4. 当前重要文件及用途

### `dock_planner/`

- `dock_planner/__init__.py`：导出公共 API。
- `dock_planner/types.py`：核心数据结构和状态枚举。
- `dock_planner/planner.py`：V0.1 纯几何规划核心。
- `dock_planner/vrx_reference.py`：VRX `dock_2022_base` 三 bay 几何适配。
- `dock_planner/wamv_collision.py`：读取真实 VRX WAM-V URDF collision primitive，用于静态 footprint/净空检查；未进入 planner 核心。

### `examples/`

- `examples/basic.py`：最小 DockPlanner API 示例。
- `examples/vrx_reference.py`：VRX 三 bay 的 DockSpec 和规划结果示例。
- `examples/gazebo_static.py`：调用 adapter/planner 生成最小静态 Gazebo world、markers、heading 线和 JSON。
- `examples/phase1_target.py`：为动态控制自动生成 bay_2 目标 JSON。当前 JSON 显式包含 DockSpec、approach、entrance；planner 已生成 stop。Phase 3 应扩展此输出和相应测试以显式携带 stop，不能手写 stop。
- `examples/__init__.py`：examples 包标记。

### `scripts/`

- `scripts/run_gazebo_static.sh`：启动静态可视化 world，不控制船。
- `scripts/gz_dual_thrust_calibration.cc`：单进程 Gazebo Transport 直线推力校准，同一循环近似同时发布左右推力。
- `scripts/build_gz_dual_thrust_calibration.sh`：构建校准工具。
- `scripts/gz_phase1_approach.cc`：实际 Phase 1，执行 start → approach。
- `scripts/build_gz_phase1_approach.sh`：构建 Phase 1 控制器。
- `scripts/gz_phase1_5_align.cc`：实际 Phase 1.5，先到 approach，再对齐 `entrance_heading`，不进入 entrance。
- `scripts/build_gz_phase1_5_align.sh`：构建 Phase 1.5 控制器。
- `scripts/gz_phase2_entrance.cc`：实际 Phase 2，执行 start → approach → ALIGN → entrance，包含 corridor、contact、超时和归零检查；不会驶向 stop。
- `scripts/build_gz_phase2_entrance.sh`：构建 Phase 2 控制器。

这些 C++ 程序是 standalone Gazebo Transport 实验工具，不是正式 ROS 2 controller。Phase 2 实测后修正了结果统计：源码不再把驶向 entrance 的位移误记为 ALIGN drift，并已重新构建；控制行为未改变。

### `tests/`

- `tests/test_planner.py`：V0.1 平移、90°/180°旋转、不同起点、非法 bay 尺寸及输入验证。
- `tests/test_vrx_reference.py`：VRX adapter、三个 bay 相对位置、任意平移旋转、heading 和官方 world 数值。
- `tests/test_gazebo_static.py`：静态 SDF/JSON 一致性和真实 WAM-V collision footprint 净空。
- `tests/test_phase1_target.py`：动态目标必须由 adapter + planner 自动生成。

### `generated/`

- `generated/vrx_dock_static.sdf`、`generated/vrx_dock_static.json`：静态 world 和自动生成坐标。
- `generated/gazebo_static.png`、`generated/gazebo_static_wamv.png`：静态俯视图。
- `generated/gz_dual_thrust_calibration`：已构建校准程序。
- `generated/gz_phase1_approach`：已构建 Phase 1 程序。
- `generated/gz_phase1_5_align`：已构建 Phase 1.5 程序。
- `generated/gz_phase2_entrance`：已构建 Phase 2 程序。
- `generated/phase1_target.json`：Python adapter/planner 自动生成的动态目标。
- `generated/phase1_result.json`：Phase 1 实测结果。
- `generated/phase1_5_result.json`：Phase 1.5 实测结果。
- `generated/phase2_result.json`：Phase 2 实测结果。
- `generated/logs/`：静态 Gazebo 和测试历史日志。

### 顶层文档

- `README.md`：API、VRX adapter、静态/动态验证和测试说明。
- `answer.txt`：此前阶段给用户的回答快照，不是代码输入。
- `PROGRESS.md`：本文档，作为跨 thread 的交接记录。

## 5. WAM-V 动态接口

### World pose

```text
topic: /world/scan_dock_deliver_task/dynamic_pose/info
type:  gz.msgs.Pose_V
```

从 `Pose_V` 中筛选 `name == "wamv"`。动态控制使用 Gazebo world frame，与 adapter 输出的 world 坐标一致。

四元数转 yaw：

```text
yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
```

角度误差归一化到 `[-pi, pi]`。

### 左右 thrust

```text
left topic:  /wamv/thrusters/left/thrust
right topic: /wamv/thrusters/right/thrust
type:        gz.msgs.Double
```

它们是 command topic。没有 publisher 是正常的；需要确认 WAM-V 插件存在 subscriber。命令有保持最近值的效果，控制器必须固定频率持续发布，并在正常、异常、信号和超时退出路径都发布左右 `0.0`，最好重复数次。

推进器位置约为：

```text
left:  (-2.373776, +1.027135, 0.318237)
right: (-2.373776, -1.027135, 0.318237)
```

当前差速约定下，正 yaw 修正使用 `right > left`：

```text
left  = base - turn
right = base + turn
```

### 官方 release / DetachableJoint

```text
topic: /vrx/release
type:  gz.msgs.Empty
```

`scan_dock_deliver_task.sdf` 的 scoring plugin 配置 `<release_topic>/vrx/release</release_topic>`。`vrx_gz/src/ScoringPlugin.cc` 广告 `gz::msgs::Empty` publisher，并在比赛 Ready/Running 流程发布 release。

WAM-V 的 DetachableJoint 定义在：

```text
/home/y/robotx_ws/src/vrx/vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro
```

`gz::sim::systems::DetachableJoint` 订阅 `/vrx/release`，把 WAM-V 从出生平台的 dummy joint 解开。官方 release 的关键日志：

```text
ScoringPlugin::OnReady
Removing entity: 377
Detaching joint [377]
ScoringPlugin::OnRunning
```

动态测试必须等这些日志/状态，不能在 joint attached 时解释推进器效果，也不应自行猜 release 消息格式。

### Contact

```text
topic: /vrx/contacts
type:  gz.msgs.Contacts
```

WAM-V contact system发布该 topic，VRX scoring plugin 也订阅。Phase 2 只在 ENTRANCE 状态启用 contact 追踪，筛选碰撞实体名称中包含 `wamv` 的 pair。

当前未使用单独速度 topic。直线校准的线速度和角速度由连续 world pose 差分估算。

## 6. 已解决的重要问题

1. **WAM-V 最初被固定**：小推力没有位移是因为 DetachableJoint 仍 attached。之后改为等待官方 scoring plugin 在 Ready 阶段自动发布 `/vrx/release`，以 `Detaching joint` 和 `OnRunning` 确认解锁。
2. **对称 100 N / 0.5 s 曾产生约 -14° yaw**：早期用两个独立、阻塞的 `gz topic -p` 进程先后发送左右命令。进程启动差和命令保持造成了一段单侧推力；该现象不代表推进器本身严重不对称。
3. **单进程同步发布解决偏航**：C++ 工具用一个 `gz::transport::Node`，在同一个固定频率循环近似同时发布左右命令，并统一归零。直线校准 publish skew 仅约 23–26 µs，相同左右推力已能基本直线前进。

## 7. 直线推进校准

使用 `scripts/gz_dual_thrust_calibration.cc`，20 Hz 连续发布 1 s。

### 30 N / 1 s

```text
initial: x=-532.070075228, y=161.989626286, yaw=0.999897019
estimated initial vx=-0.002307576 m/s
estimated initial vy=-0.000973098 m/s
estimated initial yaw_rate=-0.000015641 rad/s
actual duration=1.000061071 s, cycles=20
max left/right publish skew=23.382 us
displacement=0.188804520 m
forward component=0.188802580 m
lateral component=-0.000855998 m
delta yaw=-0.000082878 rad (-0.00475 deg)
zero published=true
```

### 50 N / 1 s

```text
initial: x=-531.798279645, y=162.430226826, yaw=0.999728418
estimated initial vx=0.000131789 m/s
estimated initial vy=-0.001312323 m/s
estimated initial yaw_rate=0.000012134 rad/s
actual duration=1.000060135 s, cycles=20
max left/right publish skew=25.942 us
displacement=0.310243067 m
forward component=0.310240051 m
lateral component=-0.001368021 m
delta yaw=0.000016762 rad (0.00096 deg)
zero published=true
```

结论：对称命令可基本直线前进。Phase 1 推荐并采用 `30 N` 基础推力。

## 8. Phase 1：start → approach

结果：`generated/phase1_result.json`

控制参数：

```text
rate=20 Hz
base thrust=30 N
max per-thruster thrust=50 N
turn gain=30
max turn term=25 N
acceptance radius=1.5 m
timeout=180 s
```

实测：

```text
initial pose:
  x=-532.070317100, y=161.988696026, yaw=0.999943396
approach:
  x=-494.597168840, y=183.923597950, yaw=1.0
initial distance=43.420925405 m
initial bearing heading error=-0.470365713 rad
runtime=138.750676009 s
control cycles=2770
max publish skew=121.045 us
final pose:
  x=-495.880911229, y=183.293676931, yaw=0.457994133
final distance=1.429963291 m
final bearing heading error=-0.001821304 rad
error to planned approach yaw=0.542005867 rad (约 31.1 deg)
reached=true, timed_out=false, interrupted=false
publish_failed=false, zero_published=true, world_stopped=true
```

位置目标成功，但船头未对齐入口方向，因此增加 Phase 1.5。

## 9. Phase 1.5：approach 航向对齐

结果：`generated/phase1_5_result.json`

参数：

```text
turn gain=25
maximum differential term per side=12 N
forward thrust=0 N
initial zero settle=2.0 s
heading tolerance=5 deg
stable duration=1.5 s
maximum drift=2.0 m
ALIGN timeout=60 s
```

实测：

```text
ALIGN initial:
  x=-495.943449069, y=183.265559337, yaw=0.457796438
target entrance_heading=1.0 rad
initial heading error=0.542203562 rad = 31.066991 deg
ALIGN final:
  x=-495.787976917, y=183.380611477, yaw=0.924188702
final heading error=0.075811298 rad = 4.344696 deg
position drift=0.193412990 m
duration=31.950055841 s, cycles=639
final distance to approach=1.308762082 m
max publish skew=653.842 us
aligned=true, timed_out=false, drift_exceeded=false
publish_failed=false, zero_published=true
entered_entrance_phase=false, world_stopped=true
```

## 10. Phase 2：approach → entrance

结果：`generated/phase2_result.json`

完整流程：

```text
start -> approach -> ALIGN to entrance_heading -> entrance -> thrust zero
```

没有驶向 stop。

### 控制参数

```text
rate=20 Hz
entrance base thrust=12 N
entrance max per-thruster thrust=20 N
entrance turn gain=25
entrance max turn term=8 N
maximum centerline heading offset=0.18 rad (约 10.3 deg)
entrance acceptance radius=0.9 m
entrance heading tolerance=7 deg
entrance timeout=100 s
whole controller timeout=290 s
VRX bay width=4.0 m
WAM-V collision half-width including margin=1.30 m
entrance lateral limit=4.0/2-1.30=0.70 m
```

近距离段根据入口中心线横向误差生成有限 heading offset，再叠加入口航向差速修正。超出 corridor、穿过 entrance、检测到 WAM-V contact、pose stale、超时或 publish failure 都会停止。

### 实测结果

```text
Phase 2 initial:
  x=-495.782251379, y=183.380585987, yaw=0.921208160
entrance target:
  x=-488.113541169, y=194.021249768, yaw=1.0
initial lateral error=+0.703821955 m
duration=90.450079711 s
cycles=1809
final pose:
  x=-488.550371699, y=193.275677137, yaw=1.012101472
final distance to entrance=0.864117735 m
final heading error=-0.012101472 rad = -0.693340 deg
final lateral error=-0.035254396 m
maximum absolute thrust=14.501586489 N
heading-error sign changes=1
oscillation=false
collision=false, collision pair empty
corridor_exceeded=false, passed_entrance=false
timed_out=false, publish_failed=false
zero_published=true, world_stopped=true
```

官方 VRX activation-zone 日志：

```text
Entering external dock activation zone in [bay2]
OnExternalActivationEvent(): 1
```

因此 WAM-V 已进入官方 bay_2 外部 activation zone，无 contact/collision，入口横向误差和航向误差均合格。Phase 2 成功。

整个程序从出生点完成 Phase 1、ALIGN、Phase 2 的实测总运行约 `260.95 s`。官方 scoring plugin 在仿真时间 `320 s` 附近出现 `OnFinished`；当前证明的是几何路径和基本可执行性，未证明正式比赛时间性能。

静态真实 WAM-V footprint 检查此前确认：4 m bay 中，按约 `±1.23 m` 横向 collision footprint，居中时每侧约 `0.77 m` 余量。因此 `0.70 m` corridor limit 已接近可用宽度，Phase 3 仍需保守。

## 11. 测试基线

当前静态单元测试基线是 **24/24 通过**：

```bash
cd /home/y/robotx_ws/dock_planner_v0
python3 -B -m unittest discover -s tests -v
```

测试覆盖 planner 几何、VRX adapter、静态 Gazebo world/markers、真实 WAM-V collision footprint，以及动态目标自动生成。它们不启动 Gazebo，不验证真实动力学。

实验控制工具的构建命令：

```bash
bash scripts/build_gz_dual_thrust_calibration.sh
bash scripts/build_gz_phase1_approach.sh
bash scripts/build_gz_phase1_5_align.sh
bash scripts/build_gz_phase2_entrance.sh
```

## 12. 已知但暂未处理的问题

1. **WaveVisual teardown segmentation fault**：Ctrl-C 停止 world 后偶尔发生。它出现在控制器退出、推力归零之后，目前没有证据表明影响 Phase 结果。
2. **RobotX 2026 官方 dock 尺寸未确认**：当前 4 m × 6 m 和三 bay 坐标仅为 `VRX_REFERENCE_GEOMETRY`。
3. **正式团队 ROS 2 interface 未接入**：尚未接团队 workspace、ROS 2 topic/action、DDS 或 ArduPilot。
4. **obstacle / RRT* 未接入**：obstacles 只是 optional 预留，非空输入尚不支持。
5. **perception 未接入**：尚未接 dock pose、confidence、TF、严格 entrance pose 或 UAV 感知。
6. **Phase 2 接受半径较宽**：最终 reference point 距入口中心 `0.864 m`，符合 0.9 m 条件；Phase 3 不能假定船恰好在入口中心。
7. **正式任务耗时未验证**：简单控制器较慢，要区分分段可行性验证和比赛时间性能。
8. **控制代码尚未抽象**：Phase 1/1.5/2 的 standalone C++ 有重复状态机逻辑。先保留可复现实验，Phase 3 后再考虑重构。

## 13. 下一步：Phase 3（尚未执行）

下一步明确为：

```text
Phase 3 = entrance -> stop 单段动态验证
```

本次交接**没有创建 Phase 3 控制器、没有运行 Phase 3、没有启动 Gazebo、没有发送任何推力**。

下一 thread 应先读取本文档和 Phase 2 源码/结果，再设计 Phase 3。建议最低要求：

- 继续由 adapter + planner 自动取得 stop，禁止手写坐标。
- 以已验证流程到达 entrance，只有 entrance 成功后才进入 STOP 状态。
- STOP 段保持 `entrance_heading`，使用更低推力和受限横向修正。
- 考虑 Phase 2 结束时距入口中心最多仍有 0.9 m，不能假定初始误差为零。
- 继续使用 `/vrx/contacts`，发生 WAM-V contact 立即归零。
- 保留 0.70 m corridor 检查，并增加不得穿过后壁/超出 bay depth 的纵向安全检查。
- 设置 stop 接受半径、heading tolerance、单段和总超时；所有退出路径左右 thrust 归零。
- 记录 stop 初始/最终 pose、distance、heading/lateral/longitudinal error、最大推力、碰撞、走廊、后壁、振荡、超时、归零结果。
- 只做 Phase 3，不继续 RRT*、perception、正式 ROS 2 或团队仓库集成。

## 14. 下一条 Codex thread 可直接复制的继续工作提示词

```text
继续 RobotX Task 3 DockPlanner 工作。

请先完整读取：
~/robotx_ws/dock_planner_v0/PROGRESS.md

它是当前项目的交接基线。当前已经完成 DockPlanner V0.1、VRX dock_2022 adapter、静态 Gazebo/真实 WAM-V footprint 验证、直线推进校准，以及动态 Phase 1、Phase 1.5、Phase 2；24/24 静态测试通过。Phase 2 已成功到达 bay_2 entrance，并可靠归零。Phase 3 尚未执行。

现在继续 Phase 3：entrance -> stop 单段动态验证。

约束：
1. 只修改 ~/robotx_ws/dock_planner_v0。
2. 不修改 ~/robotx_ws/src/vrx。
3. 不修改 ~/robotx_team_ws/robotx_workspace。
4. 使用官方 scan_dock_deliver_task、真实动态 WAM-V 和官方自动 /vrx/release。
5. 使用现有 vrx_dock_2022_bay_to_dockspec() + DockPlanner.plan() 自动生成 bay_2 的 approach、entrance、stop；approach_distance=12 m，禁止手写目标坐标。
6. 先复现 start -> approach -> ALIGN -> entrance，只有 entrance 成功后才进入 STOP 状态。
7. STOP 段保持 entrance_heading，以低推力同时修正横向位置和 heading，沿泊位内部方向前进到 stop。
8. 保留 /vrx/contacts、0.70 m corridor limit、pose stale、超时、publish failure 检查，并增加不得越过 bay 后壁/超出 bay depth 的纵向安全检查。
9. 所有正常、异常、信号、碰撞和超时退出路径都必须将左右 thrust 归零。
10. 记录 Phase 3 初始/最终 pose、到 stop 的距离、heading error、lateral/longitudinal error、最大 thrust、振荡、collision、corridor、back-wall、timeout 和归零状态。
11. 只完成 Phase 3；不要继续 RRT*、障碍物、perception、正式 ROS 2/DDS/ArduPilot 或团队仓库集成。

开始前先核对现有 Phase 2 源码、generated/phase2_result.json、target 生成链和当前测试。先说明 Phase 3 的最小控制/安全方案，再按我的确认执行动态测试；不要在没有确认时发送非零推力。
```
