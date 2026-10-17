# Task3 水上 LiDAR 最小测试步骤

第一轮目标：确认 Mid-360 在水上不同距离能稳定看到 dock 入口，并把 rosbag 保存到统一日志目录。

第一轮保持人工遥控，不启动 Task3 executor、状态机、泵或自动 setpoint。

## 0. 下水前检查

USV 操作：

- 确认遥控器处于人工模式，驾驶员随时可以接管。
- 确认电池、急停、GPS/定位和 Mid-360 供电正常。
- 确认 dock 周围没有人员进入 USV 航行区域。
- 记录当天日期、天气和 dock 入口的大致位置。
- 手机准备拍摄 USV 与 dock 的合照。

## 1. 终端 A：确认当前 Orin 终端和公共包

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
hostname
date
df -h ~/robotx_ws
ros2 pkg prefix robotx_perception_common
ros2 pkg prefix livox_ros_driver2
```

如果上一条能输出公共包路径，保持这个终端打开。每次录包的目录名会在终端 G 中由当前时间自动生成，不需要手动改文件名。

如果找不到公共包或驱动包，先停止本次采集，让负责人按统一工作区流程构建；正常采集时不临时加载旧的 `livox_ws` 或 `mavros_ws`。

## 2. 终端 B：启动 Mid-360 和静态 TF

新开一个 Orin 终端，执行统一启动文件：

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py
```

这个终端会同时启动：

- Mid-360 驱动；
- `base_link → livox_frame` 静态 TF。

保持这个终端运行，不要关闭。

## 3. 终端 C：启动共享 LiDAR 感知入口（可选）

如果本次同时验证共享感知输出，在新终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py
```

如果只采集原始点云，可以跳过本终端。不要启动 `task3_executor`、状态机、泵或自动控制。

## 4. 终端 D：确认点云

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash

ros2 topic list -t | grep -E '/livox/lidar|/tf|/fix|odom|ned_position'
ros2 topic info /livox/lidar --verbose
```

确认列表中出现：

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

## 5. 终端 E：确认点云频率

再开一个 Orin 终端，只执行下面这一条：

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
ros2 topic hz /livox/lidar
```

确认平均频率约 10 Hz 后，按 `Ctrl+C` 停止频率检查。

## 6. 终端 F：确认静态 TF

再开一个 Orin 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

确认持续输出：

```text
x = 0.150
y = 0.000
z = 0.220
roll = pitch = yaw = 0
```

## 7. 终端 G：开始录包

先在终端 G 创建本次录包目录。目录名使用当前时间自动生成：

```bash
source /opt/ros/humble/setup.bash
source ~/robotx_ws/install/setup.bash
mkdir -p /home/robotx/robotx_logs/task3_lidar
BAG="/home/robotx/robotx_logs/task3_lidar/dock_$(date +%Y%m%d_%H%M%S)"
echo "即将录制：$BAG"
```

确认目录名无误后开始录包：

```bash
ros2 bag record \
  -o "$BAG" \
  /livox/lidar /tf /tf_static
```

如果终端 C 确认 `/fix` 或实际 `/odom` 在线，把它们加到录包命令末尾：

```bash
/fix /odom
```

不要录入不存在的 topic，也不要在这一轮启动控制节点。

## 8. 近距离静态点：约 2 m

USV 操作：

1. 人工遥控到 dock 入口外侧，距离入口约 2 m。
2. 船停稳，船头大致朝向入口。
3. 保持静止 20 秒。
4. 手机拍一张能同时看到 USV 和 dock 的照片。
5. 在终端 G 按 `Ctrl+C` 停止录包。

终端 G：

```bash
ros2 bag info "$BAG"
```

确认消息数量大于 0、包含 `/livox/lidar` 后，再进入下一组。

## 9. 中距离静态点：约 4 m

重新在终端 G 生成当前时间目录并开始录包：

```bash
BAG="/home/robotx/robotx_logs/task3_lidar/dock_$(date +%Y%m%d_%H%M%S)"
echo "即将录制：$BAG"
ros2 bag record \
  -o "$BAG" \
  /livox/lidar /tf /tf_static
```

USV 操作：

1. 人工后退到 dock 入口约 4 m。
2. 保持船头大致朝向入口。
3. 静止 20 秒并拍照。
4. 按 `Ctrl+C` 停止录包。

检查：

```bash
ros2 bag info "$BAG"
```

## 10. 远距离静态点：约 6 m

```bash
BAG="/home/robotx/robotx_logs/task3_lidar/dock_$(date +%Y%m%d_%H%M%S)"
echo "即将录制：$BAG"
ros2 bag record \
  -o "$BAG" \
  /livox/lidar /tf /tf_static
```

USV 操作：

1. 人工后退到 dock 入口约 6 m。
2. 静止 20 秒并拍照。
3. 按 `Ctrl+C` 停止录包。

如果水域安全范围不允许 6 m，改成现场允许的“近、中、远”三个距离，并在记录中写下估计距离。

## 11. 一次慢速经过

终端 G：

```bash
BAG="/home/robotx/robotx_logs/task3_lidar/dock_$(date +%Y%m%d_%H%M%S)"
echo "即将录制：$BAG"
ros2 bag record \
  -o "$BAG" \
  /livox/lidar /tf /tf_static
```

USV 操作：

1. 保持人工遥控。
2. 在安全距离外沿 dock 正面慢速经过。
3. 转过约 60°～90°即可，不需要绕完整一圈。
4. 全程约 1～2 分钟。
5. 回到安全位置后停止录包。

这一段不需要专门安排遮挡物，也不需要启动自动控制。

## 12. 停止顺序

1. 先在终端 G 按 `Ctrl+C` 停止 rosbag。
2. 对每个 bag 执行 `ros2 bag info`。
3. 确认 bag 文件已经写完后，在终端 B 按 `Ctrl+C` 停止统一启动程序；如果启动了终端 C 的共享感知，再按 `Ctrl+C` 停止终端 C。
4. 最后由驾驶员确认 USV 处于安全状态并断电。

## 13. 回传文件

先在 Orin 上确认大小：

```bash
du -sh /home/robotx/robotx_logs/task3_lidar/*
find /home/robotx/robotx_logs/task3_lidar -maxdepth 2 -type f -name 'metadata.yaml' -print
```

本地电脑执行：

```bash
mkdir -p /home/y/robotx_ws/downloads/task3_lidar
scp -r robotx@100.91.153.16:/home/robotx/robotx_logs/task3_lidar/* \
  /home/y/robotx_ws/downloads/task3_lidar/
```

## 14. 第一轮通过条件

- 三个静态距离都有有效点云消息。
- 每个静态 bag 能导出至少 5 帧。
- 入口宽度大致保持在 1.1～1.9 m。
- 同一静态位置连续 5 帧中至少 4 帧检测到同一 gap。
- 慢速经过时没有持续跳到其他结构。
- 所有结果仍保留在 `livox_frame`/`base_link`，没有未经确认的 NED 转换。

这一轮通过后，再把实测外参、粗中心和入口结果接到 `dock_perception` dry-run；本轮不启动船舶自动控制。
