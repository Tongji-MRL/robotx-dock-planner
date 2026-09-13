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
- `dock_planner/__init__.py`：公开导入接口。
- `examples/basic.py`：打印一组示例输入和结果。
- `examples/__init__.py`：使示例可通过模块方式运行。
- `tests/test_planner.py`：平移、旋转、起点变化、非法输入及预留接口测试。

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
