# USV Mid-360 Dock 点云采集：岸上静态多角度测试

以下命令都在 **USV Orin** 上执行，除非特别标注“本地电脑”。

## 0. 测试前

- [ ] USV 上电
- [ ] Orin 上电
- [ ] Mid-360 上电
- [ ] LiDAR 网线已接
- [ ] USV 已固定在岸上或测试支架上
- [ ] 有人员负责手动调整 USV 朝向/位置
- [ ] 手机/相机准备好

本次不下水、不启动自动驾驶、喷水或自动靠泊，只采集不同角度的静态点云。

## 1. 检查网络

### 终端 1

```bash
ip -4 addr
ping -c 4 192.168.1.188
```

要求：能收到 `64 bytes from 192.168.1.188`。

## 2. 检查 LiDAR topic

### 终端 1

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
ros2 topic list -t | grep -E '/livox/lidar|/tf$|/tf_static'
ros2 topic info /livox/lidar --verbose
```

要求：

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

并且有 publisher。

## 3. 检查频率

### 终端 2

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
ros2 topic hz /livox/lidar --window 5
```

等待 5～10 秒。

要求：约 `10 Hz`。

保持终端 2 不关闭。

## 4. 记录 frame_id

### 终端 1

```bash
ros2 topic echo /livox/lidar --once | grep -A3 frame_id
```

记录结果，预期是：

```text
livox_frame
```

截图：`01_lidar_frame.png`

## 5. 准备录制

### 终端 3

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
mkdir -p /home/robotx/livox_ws/bags
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/home/robotx/livox_ws/bags/dock_mid360_${STAMP}
echo "$OUT"
```

记下输出路径。

## 6. 开始录制

### 终端 3

确认 `/tf`、`/tf_static` 存在后输入：

```bash
ros2 bag record -o "$OUT" /livox/lidar /tf /tf_static
```

如果 TF 不存在，改为：

```bash
ros2 bag record -o "$OUT" /livox/lidar
```

要求看到：

```text
Recording...
Subscribed to topic '/livox/lidar'
```

看到后才开始录制静态数据；调整角度时先停止录制。

## 7. 记录开始时间和现场

### 终端 4或手机

```bash
date -Is
```

拍三张照片：

```text
02_dock_overview.jpg       dock整体
03_usv_dock_position.jpg   USV和dock相对位置
04_mid360_mounting.jpg     Mid-360安装方向
```

## 8. 场景一：正对 dock（0 度）

### USV动作

1. 将 USV 放在 dock 正前方；
2. 将船头对准 dock 中间；
3. 固定 USV，不要移动；
4. 录制 15～30 秒；
5. 拍照。

记录：`scene=front_static`、大致距离、船头方向。

照片：`05_front_static.jpg`

## 9. 场景二：左侧斜视（约 -30 度）

### USV动作

1. 停止上一段录制；
2. 将 USV 手动转向 dock 左侧约 30 度，或将 USV 放到左前方位置；
3. 固定 USV；
4. 重新开始录制；
5. 录制 15～30 秒；
6. 拍照。

记录：`scene=oblique_left`、大致距离、船头方向。

照片：`06_oblique_left.jpg`

## 10. 场景三：右侧斜视（约 +30 度）

### USV动作

1. 停止上一段录制；
2. 将 USV 手动转向 dock 右侧约 30 度，或将 USV 放到右前方位置；
3. 固定 USV；
4. 重新开始录制；
5. 录制 15～30 秒；
6. 拍照。

记录：`scene=oblique_right`、大致距离、船头方向。

照片：`07_oblique_right.jpg`

## 11. 场景四：多角度静态采集

### USV动作

每个角度都执行一次“停止上一段录制 → 调整位置/朝向 → 固定 → 重新开始录制”：

1. 0 度：正对 dock；
2. -30 度：左侧斜视；
3. +30 度：右侧斜视；
4. -60 度：更大左侧斜视；
5. +60 度：更大右侧斜视；
6. 如果空间允许，再采集 dock 侧面约 90 度；
7. 每个角度录制 15～30 秒；
8. 每个角度拍一张现场照片。

不要让 USV 在录制过程中移动。每个角度最好单独生成一个 bag，便于后面比较。

### 每换一个角度时：终端 3

先按：

```text
Ctrl+C
```

再输入：

```bash
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/home/robotx/livox_ws/bags/dock_mid360_${STAMP}
ros2 bag record -o "$OUT" /livox/lidar /tf /tf_static
```

看到 `Recording...` 后，再开始该角度的 15～30 秒静态采集。

如果前面确认没有 TF，使用：

```bash
ros2 bag record -o "$OUT" /livox/lidar
```

记录：`scene=static_angle`、角度、距离、船头方向。

照片：

```text
08_angle_minus60.jpg
09_angle_plus60.jpg
10_angle_side.jpg
```

## 12. 停止录制

### USV动作

1. 停止最后一段录制；
2. 确认 USV 已固定；
3. 不再调整位置；
4. 保存所有 bag 和照片。

### 终端 3

按：

```text
Ctrl+C
```

然后输入：

```bash
ros2 bag info "$OUT"
du -sh "$OUT"
```

要求：`/livox/lidar` 消息数量大于 0，录制时长大于 0。

截图：`11_rosbag_info.png`

## 13. Orin 上生成质量报告

### 终端 3

如果 staging 测试包存在：

```bash
source /opt/ros/humble/setup.bash
cd /home/robotx/task3_update_staging/20261006_lidar_ros_test/ws
source install/setup.bash
PYTHONPATH="$PWD/src/lidar_ros_test:$PYTHONPATH" \
python3 -m lidar_ros_test.lidar_bag_report \
  "$OUT" --output-json ../dock_mid360_quality.json
```

检查：

```text
received_frames > 0
actual_frequency_hz 接近 10
finite_ratio 接近 1
frame_id 为 livox_frame
```

## 14. 拷回本地电脑

### 本地电脑终端

把路径替换成实际 bag 路径：

```bash
mkdir -p /home/y/robotx_ws/downloads
scp -r \
  robotx@100.91.153.16:/home/robotx/livox_ws/bags/dock_mid360_YYYYMMDD_HHMMSS \
  /home/y/robotx_ws/downloads/
```

## 15. 本地导出点云帧

### 本地电脑终端

```bash
source /opt/ros/humble/setup.bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0:$PYTHONPATH \
python3 -m examples.export_mid360_bag_frames \
  /home/y/robotx_ws/downloads/dock_mid360_YYYYMMDD_HHMMSS \
  /tmp/dock_frames \
  --topic /livox/lidar \
  --stride 5 --max-frames 20
```

要求生成：

```text
/tmp/dock_frames/frame_*.npy
/tmp/dock_frames/manifest.json
```

## 16. 本地运行入口识别

只有粗中心已经转换到和点云相同坐标系后运行。

### 本地电脑终端

```bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
python3 -m dock_lidar_prototype.run_recorded_frame \
  /tmp/dock_frames/frame_000000.npy \
  --center-x 填写粗中心x \
  --center-y 填写粗中心y \
  --observer-x 填写观测位置x \
  --observer-y 填写观测位置y \
  --output-png /tmp/dock_detection.png \
  --output-json /tmp/dock_detection.json
```

查看：

```bash
cat /tmp/dock_detection.json
```

打开：`/tmp/dock_detection.png`

确认：入口中心在 gap 中，箭头方向合理，`selected.valid=true`。

如果粗中心仍是 UAV NED，或者没有 `livox_frame` 外参，只做质量检查，不使用入口识别结果。

## 17. 最终保存

- [ ] rosbag 目录
- [ ] `ros2 bag info` 截图
- [ ] LiDAR 质量报告 JSON
- [ ] `manifest.json`
- [ ] 点云帧 `.npy`
- [ ] 现场照片
- [ ] LiDAR frame_id 截图
- [ ] USV 距离、朝向和动作记录
