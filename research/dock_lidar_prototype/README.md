# RobotX Task3 Dock LiDAR Geometry Prototype

这是仓库内隔离的纯 Python 离线 prototype，不属于 `dock_planner` 运行包。

## 输入和输出

输入是同一个 local/base 2D frame 中的 `points_xyz (N, 3)` 和
`coarse_dock_center_xy`。prototype 不做 TF、不转 NED、不发布 ROS 消息。

输出 `EntranceCandidate`：

```text
center_x, center_y       入口中心（local 2D frame）
heading_rad              从 gap 指向粗 dock 中心的法向
width_m                  gap 宽度
confidence               几何支持度、法向一致性和内部空闲度的组合分数
valid                    是否通过全部阈值
```

## 算法

```text
XYZ
 -> finite / z / ROI filter
 -> XY projection
 -> grid-connected clustering
 -> deterministic seeded RANSAC line segments
 -> pair approximately parallel wall segments
 -> endpoint gap width check
 -> gap interior occupancy check
 -> normal orientation toward coarse center
 -> confidence and best candidate
```

第一版没有完整 dock 模板 ICP。它只把“两条近似平行结构之间的空段”作为入口候选，
因此可以处理斜视，但不能保证真实 RobotX dock 的所有结构都能被正确分成墙段。

## 运行

```bash
cd /home/y/robotx_ws/dock_planner_v0/research/dock_lidar_prototype
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
  python3 -m unittest discover -s tests -v
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
  python3 -m dock_lidar_prototype.run_prototype
```

示例图会写到 `artifacts/synthetic_detection.png`。图中包括 XY 点、检测墙段、橙/绿
gap candidate、红色入口中心和 heading 箭头、紫色 coarse center。

## 现场数据离线检查

先从 rosbag 抽取清洗后的单帧。该步骤只读 bag，不会启动节点或发布控制：

```bash
source /opt/ros/humble/setup.bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0:$PYTHONPATH \
  python3 -m examples.export_mid360_bag_frames \
  /path/to/dock_bag /tmp/dock_frames --stride 10 --max-frames 20
```

再使用和点云相同坐标系中的 dock 粗中心检测一帧：

```bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
  python3 -m dock_lidar_prototype.run_recorded_frame \
  /tmp/dock_frames/frame_000000.npy \
  --center-x 4.0 --center-y 0.0 --observer-x 0.0 --observer-y 0.0 \
  --output-png /tmp/dock_detection.png \
  --output-json /tmp/dock_detection.json
```

这里的中心点必须已经和 `livox_frame` 点云处于同一坐标系。当前工具不会猜测
`livox_frame → base_link` 外参，也不会把 UAV NED 中心直接当作 LiDAR 坐标。

## 当前边界

- 不接 `sensor_msgs/msg/PointCloud2`。
- 不使用真实 Mid-360 外参或时间同步。
- 不输出 `DockEntranceObservationNED`。
- 不调用 `DockPlanner`、`Task3FlowManager`、PX4 或车辆控制。
- 当前 synthetic dock 是两侧墙加后墙的简化 U 形结构。

完整的离线验证结果、真实 bag 连续帧统计和后续测试门槛见
[`VALIDATION_REPORT_20261008.md`](VALIDATION_REPORT_20261008.md)。

## 没有 UAV 粗中心时的离线搜索

真实采集数据还没有 `livox_frame -> base_link` 外参，也没有把 UAV 中心同步到
LiDAR frame。因此可以先用独立搜索工具做几何可行性检查：它在输入点云的有限
局部范围内生成多个候选中心，逐个调用上面的 gap 检测，再按入口中心距离去重。
`min-range` 用来排除 LiDAR 原点附近的 USV 自身结构。

```bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
  python3 -m dock_lidar_prototype.run_unseeded_search \
  /path/to/frame.npy \
  --min-range 1.5 --max-range 10 --grid-spacing 2 \
  --z-min -0.5 --z-max 0.5 --roi-radius 6 \
  --output-png /tmp/unseeded_search.png \
  --output-json /tmp/unseeded_search.json
```

输出仍然是输入点云的 local frame，JSON 中包含多个候选的中心、朝向、宽度和
置信度。这个搜索只能用于离线探索；运行时仍应优先使用 UAV 粗中心作为 ROI
和朝向先验，并在确认外参后再接 ROS/Task3。

## 跨帧验证

无中心搜索现在默认增加了一个保守的 LiDAR 近场 USV 结构屏蔽，并提供
`tracking.py` 的跨帧候选跟踪。屏蔽范围在当前 `livox_frame` 输入中约为
`x=[-0.70, 0.85] m, y=[-0.55, 0.55] m`，可通过命令行参数调整；跟踪器按候选
中心移动距离、朝向变化和几何置信度选择连续轨迹。它只用于离线验证，不会替代
真实运行时的 TF、USV 位姿或 UAV 粗中心。

对一组已导出的连续帧运行批量分析：

```bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
  python3 -m dock_lidar_prototype.run_recorded_sequence \
  /home/y/robotx_ws/downloads/task3_lidar/derived_custommsg \
  /tmp/dock_lidar_sequence_report
```

输出包括每个场景的候选帧、跨帧轨迹、代表帧图片和 `batch_report.json`。
