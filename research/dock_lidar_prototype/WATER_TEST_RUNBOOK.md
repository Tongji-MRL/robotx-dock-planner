# Task3 水上 LiDAR 最小测试步骤

第一轮目标：确认 Mid-360 在水上不同距离能稳定看到 dock 入口，并保存可复现的 rosbag。

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
source /home/robotx/livox_ws/install/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
hostname
date
df -h /home/robotx
ros2 pkg prefix robotx_perception_common
```

如果上一条能输出公共包路径，保持这个终端打开。每次录包的目录名会在终端 F 中由当前时间自动生成，不需要手动改文件名。

如果提示找不到公共包，只需首次部署时执行：

```bash
cd /home/robotx/robotx_ws
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
colcon build --packages-select robotx_perception_common
source install/setup.bash
```

## 2. 终端 B：启动 Mid-360 和静态 TF

新开一个 Orin 终端，执行统一启动文件：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py
```

这个终端会同时启动：

- Mid-360 驱动；
- `base_link → livox_frame` 静态 TF。

保持这个终端运行，不要关闭。

如果驱动已经在其他终端运行，则改用：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py start_driver:=false
```

## 3. 终端 C：确认点云

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
source /home/robotx/robotx_ws/install/setup.bash

ros2 topic list -t | grep -E '/livox/lidar|/tf|/fix|odom|ned_position'
ros2 topic info /livox/lidar --verbose
```

确认列表中出现：

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

## 4. 终端 D：确认点云频率

再开一个 Orin 终端，只执行下面这一条：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
ros2 topic hz /livox/lidar
```

确认平均频率约 10 Hz 后，按 `Ctrl+C` 停止频率检查。

## 5. 终端 E：确认静态 TF

再开一个 Orin 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

确认持续输出：

```text
x = 0.150
y = 0.000
z = 0.220
roll = pitch = yaw = 0
```

## 6. 终端 F：开始录包

先在终端 F 创建本次录包目录。目录名使用当前时间自动生成：

```bash
source /opt/ros/humble/setup.bash
mkdir -p /home/robotx/livox_ws/bags/task3_water
BAG="/home/robotx/livox_ws/bags/task3_water/$(date +%Y%m%d_%H%M%S)"
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

## 7. 近距离静态点：约 2 m

USV 操作：

1. 人工遥控到 dock 入口外侧，距离入口约 2 m。
2. 船停稳，船头大致朝向入口。
3. 保持静止 20 秒。
4. 手机拍一张能同时看到 USV 和 dock 的照片。
5. 在终端 F 按 `Ctrl+C` 停止录包。

终端 F：

```bash
ros2 bag info "$BAG"
```

确认消息数量大于 0、包含 `/livox/lidar` 后，再进入下一组。

## 8. 中距离静态点：约 4 m

重新在终端 F 生成当前时间目录并开始录包：

```bash
BAG="/home/robotx/livox_ws/bags/task3_water/$(date +%Y%m%d_%H%M%S)"
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

## 9. 远距离静态点：约 6 m

```bash
BAG="/home/robotx/livox_ws/bags/task3_water/$(date +%Y%m%d_%H%M%S)"
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

## 10. 一次慢速经过

终端 F：

```bash
BAG="/home/robotx/livox_ws/bags/task3_water/$(date +%Y%m%d_%H%M%S)"
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

## 11. 停止顺序

1. 先在终端 F 按 `Ctrl+C` 停止 rosbag。
2. 对每个 bag 执行 `ros2 bag info`。
3. 确认 bag 文件已经写完后，在终端 B 按 `Ctrl+C` 停止统一启动程序。
4. 最后由驾驶员确认 USV 处于安全状态并断电。

## 12. 回传文件

先在 Orin 上确认大小：

```bash
du -sh /home/robotx/livox_ws/bags/task3_water/*
find /home/robotx/livox_ws/bags/task3_water -maxdepth 2 -type f -name 'metadata.yaml' -print
```

本地电脑执行：

```bash
mkdir -p /home/y/robotx_ws/downloads/task3_water
scp -r robotx@100.91.153.16:/home/robotx/livox_ws/bags/task3_water/* \
  /home/y/robotx_ws/downloads/task3_water/
```

## 13. 第一轮通过条件

- 三个静态距离都有有效点云消息。
- 每个静态 bag 能导出至少 5 帧。
- 入口宽度大致保持在 1.1～1.9 m。
- 同一静态位置连续 5 帧中至少 4 帧检测到同一 gap。
- 慢速经过时没有持续跳到其他结构。
- 所有结果仍保留在 `livox_frame`/`base_link`，没有未经确认的 NED 转换。

这一轮通过后，再把实测外参、粗中心和入口结果接到 `dock_perception` dry-run；本轮不启动船舶自动控制。
