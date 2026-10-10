# robotx_perception_common

公共 LiDAR 基础包，供 Task1、Task3 及其他感知节点复用。

## 包含内容

- Mid-360 `PointCloud2` 的 XYZ 提取；
- NaN/Inf 和 `(0, 0, 0)` 占位点过滤；
- 按现场测量外参将 `livox_frame` 转换到 `base_link`；
- 一个统一启动文件，同时启动 Mid-360 驱动和静态 TF。

当前外参：

```text
base_link → livox_frame
x = +0.15 m, y = 0.00 m, z = +0.22 m
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
  lidar_x:=0.15 lidar_y:=0.0 lidar_z:=0.22 \
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

`task3_lidar_perception` 是一个独立 ROS 2 节点：

```text
/livox/lidar (PointCloud2)
        +
/tongji_mrl/twave/task3/coarse_dock (JSON)
        +
/tongji_mrl/twave/ned_position (JSON with heading_rad)
        |
        v
PointCloud2 -> base_link -> USV pose -> NED -> NumpyDockPerception
        |
        v
/tongji_mrl/twave/task3/dock_perception (std_msgs/msg/String JSON)
```

The pose message must include `valid`, `position_ned_m`, `stamp_unix_sec`
(or `timestamp`) and `heading_rad` (or `yaw_rad`). A position-only message is
rejected because rotating `base_link` points into NED without heading would be
unsafe. The node does not publish vehicle commands.

The default freshness limits are 30 s for the coarse dock and 2 s for the USV
pose. Stale inputs are ignored.

Start it after the official Livox driver is already running:

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py
```

The output is the existing Task3 JSON contract consumed by
`usv_task3_executor`; the detector remains a geometry prototype and should be
validated on recorded/real dock scans before control integration.
