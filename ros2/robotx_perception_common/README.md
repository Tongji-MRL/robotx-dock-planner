# robotx_perception_common

公共 LiDAR 基础包，供 Task1、Task3 及其他感知节点复用。

## 包含内容

- Mid-360 `PointCloud2` 和 Livox `CustomMsg` 的 XYZ 提取；
- NaN/Inf 和 `(0, 0, 0)` 占位点过滤；
- 按现场测量外参将 `livox_frame` 转换到 `base_link`；
- 一个统一启动文件，同时启动 Mid-360 驱动和静态 TF。

当前外参：

```text
base_link → livox_frame
x = +0.15 m, y = +0.10 m, z = +0.22 m
roll = pitch = yaw = 0
```

## 安装和启动

把本目录复制到 Orin 工作区的 `src/robotx_perception_common`，然后构建：

```bash
cd /home/robotx/robotx_ws
colcon build --packages-select robotx_perception_common
source install/setup.bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py
```

外参需要临时覆盖时可以传 launch 参数；默认值就是当前实测值：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py \
  lidar_x:=0.15 lidar_y:=0.10 lidar_z:=0.22 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

如果驱动已经在另一个终端运行，只启动公共 TF：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py start_driver:=false
```

Task1 和 Task3 的 ROS 回调都应先调用：

```python
from robotx_perception_common import pointcloud_message_to_base_link_xyz
points_base = pointcloud_message_to_base_link_xyz(pointcloud_msg)
```

该函数输出的是 `base_link` 局部坐标。转换到任务 NED 需要另行使用实时 USV 位姿。

## Task3 LiDAR 感知桥接

`task3_lidar_perception` 是一个独立 ROS 2 节点。当前 Orin 实测驱动默认发布
`sensor_msgs/msg/PointCloud2`，因此默认输入格式是 `pointcloud2`：

```text
/livox/lidar (PointCloud2)
        +
/tongji_mrl/twave/task3/coarse_dock (JSON)
        +
/tongji_mrl/twave/ned_position (JSON with heading_rad)
        |
        v
PointCloud2/CustomMsg -> base_link -> coarse-centre seeded line/gap detector -> NED
        |
        v
/tongji_mrl/twave/task3/dock_perception (std_msgs/msg/String JSON)
```

The pose message must include `valid`, `position_ned_m`, `stamp_unix_sec`
(or `timestamp`) and `heading_rad` (or `yaw_rad`). A position-only message is
rejected because rotating `base_link` points into NED without heading would be
unsafe. The node does not publish vehicle commands.

The default freshness limits are 30 s for the coarse dock and 2 s for the USV
pose. Stale inputs are ignored. The default `seeded_lines` detector first
limits the cloud around the UAV coarse centre, removes self returns and points
outside the measured low-dock height band, then extracts short line segments.
It accepts either two longitudinal side edges forming a channel or two
transverse front-face fragments forming an opening. In both cases the gap and
the approach corridor must be clear. This avoids one large PCA box being formed
by sparse water returns that connect the dock to nearby clutter.

The node prints one compact diagnostic line every two seconds. `filtered=0`
points to a height/ROI issue; nonzero clusters with `segments=0` points to line
thresholds; nonzero segments with `candidates=0` means the gap, direction, or
clear-corridor checks rejected the scene. Override `diagnostic_log_period_s`
only when a different log rate is needed.

`legacy_numpy` keeps the earlier whole-cluster/PCA implementation for replay
comparison:

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  detector_mode:=legacy_numpy
```

For a controlled field recognition check before the UAV coarse-centre and NED
position adapters are available, the node has an explicitly opt-in local
fallback. It searches a bounded 12 m `base_link` cloud for parallel dock-edge
segments and publishes the candidates on the test output topic. This mode is
for recognition bring-up only; keep the Task3 executor stopped and do not use
its output for vehicle control:

```bash
ros2 run robotx_perception_common task3_lidar_perception --ros-args \
  -p input_format:=pointcloud2 \
  -p enable_local_fallback:=true \
  -p coarse_timeout_s:=-1.0 \
  -p pose_timeout_s:=-1.0
```

When a coarse centre and a heading-bearing USV NED pose are available, keep
`enable_local_fallback` false. If it is explicitly enabled, the fallback output
replaces the seeded result and must only be used for stationary recognition
bring-up.

Start it after the official Livox driver is already running:

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py
```

The output is the existing Task3 JSON contract consumed by
`usv_task3_executor`. The executor still requires its existing multi-frame
stability check before it releases any target to control.

## 无 UAV 粗中心搜索

`unseeded_lines` 用于只有 USV、RTK 和 Mid-360 的方案。四个 geofence
角点先转换到共用的 `robotx_rtk_ned`，规划器在两组对边中点之间生成两条向内
缩进的搜索线，并按固定间距生成“停车观察”点。第一条线覆盖不足时再使用第二条。

船停稳后，感知节点在 360° 点云中寻找三个平行入口。入口必须在至少 3 帧中重复
出现，随后按面向 dock 时的左到右顺序编号为 1、2、3，并发布粗中心和三个 bay 的
相机观察位姿。该模式仍只发布目标，不直接控制推进器。

本地合成仿真会生成一个相对船体旋转 1.1 rad 的低矮三 bay dock、水面杂点和 RTK
位姿，并把它送进正式感知节点：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
ros2 launch robotx_perception_common task3_geofence_search_sim.launch.py
```

看到 `UNSEEDED DOCK CONFIRMED: 3 entrances` 即表示三入口、左右顺序和粗中心链路
已经确认。仿真只验证搜索几何与感知接口，不包含实船动力学和避障控制。

带 RViz 图形界面的版本：

```bash
ros2 launch robotx_perception_common task3_geofence_search_rviz.launch.py
```

界面中灰线是 geofence，青色和蓝色分别是主搜索线与备用搜索线，橙色箭头是
USV，点云是合成 Mid-360 数据；稳定识别后会显示绿色入口及 Bay 1/2/3、黄色
粗中心和橙色相机观察点。

实船启动 Livox、MAVROS 和 RTK 位姿后，使用：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  detector_mode:=unseeded_lines \
  local_max_candidates:=6 \
  local_max_range_m:=10.0 \
  local_lateral_half_width_m:=12.0 \
  unseeded_required_detections:=3 \
  unseeded_min_entrances:=3
```

## 航行中使用 FAST-LIO 去畸变点云

沿 geofence 搜索线航行时，直接使用一帧原始扫描会受船体平移、转向和水面起伏
影响。可选启动文件会同时启动 FAST-LIO，并把它输出的当前机体系去畸变点云
`/cloud_registered_body` 送入同一个入口检测器：

```bash
ros2 launch robotx_perception_common task3_lio_perception.launch.py
```

该模式仍使用 RTK NED 作为全局任务坐标；FAST-LIO 只负责扫描内去畸变和短时
运动估计。它不会替代 RTK，也不会使用持续累积的 `/cloud_registered` 地图来识别
Dock。启动前需确保工作区内已有 ROS 2 包 `fast_lio`，并且 `/livox/lidar` 为
`sensor_msgs/msg/PointCloud2`、`/livox/imu` 为 `sensor_msgs/msg/Imu`。

若 FAST-LIO 未正常输出，停止该启动文件后仍可用原始点云回退：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  detector_mode:=unseeded_lines \
  local_max_candidates:=6 \
  local_max_range_m:=15.0 \
  local_lateral_half_width_m:=15.0
```

组合启动会校验输入点云的 `frame_id` 必须为 `body`，防止误把世界系累计地图
当成船体系当前扫描。FAST-LIO 应在船静止时先完成 IMU 初始化，再开始搜索航行。

如果驱动切换为 Livox CustomMsg，可以显式选择：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  input_format:=custom_msg
```
