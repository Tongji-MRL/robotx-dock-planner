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

如果驱动切换为 Livox CustomMsg，可以显式选择：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  input_format:=custom_msg
```
