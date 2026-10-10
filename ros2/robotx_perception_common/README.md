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
