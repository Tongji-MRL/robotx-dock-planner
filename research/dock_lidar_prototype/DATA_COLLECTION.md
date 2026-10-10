# Mid-360 Dock 点云采集流程

这份流程只采集数据，不发布控制指令，也不启动 DockPlanner 或 Task3 控制节点。

**第 1～5 节在 USV Orin 上执行，第 6 节在你的本地 Ubuntu 执行。**
不要把 Orin 路径和本地路径混用：Orin 的用户目录是 `/home/robotx`，本地目录是 `/home/y/robotx_ws`。

建议准备三个终端：

- **终端 A**：启动或查看 Mid-360 驱动；
- **终端 B**：查看 topic 和频率；
- **终端 C**：录制 rosbag。

所有终端都要使用同一个 ROS 环境和同一个 `ROS_DOMAIN_ID`。如果驱动已经由系统服务启动，不要重复启动第二个驱动。

## 现场按键顺序（照这个顺序做）

下面假设你已经通过 VS Code SSH 打开了 USV Orin。为了避免“船已经动了但还没开始录”的情况，严格按顺序操作。

### 终端 1：确认驱动和 ROS 环境

在 Orin 打开第一个终端，输入：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
ros2 topic list -t | grep -E '/livox/lidar|/tf$|/tf_static|semantic_map|odom'
ros2 topic info /livox/lidar --verbose
```

你需要看到：

- `/livox/lidar` 存在；
- 类型是 `sensor_msgs/msg/PointCloud2`；
- 有 publisher；
- 能记下 `frame_id`，通常是 `livox_frame`。

如果这里没有 `/livox/lidar`，先停下来处理驱动，不要让 USV 开始移动。

### 终端 2：确认实时频率

在 Orin 打开第二个终端，输入：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
ros2 topic hz /livox/lidar --window 5
```

等待几秒，确认输出频率大约接近 10 Hz。确认后不要关闭这个终端，让它一直显示频率。

如果频率为 0，或者没有输出，先不要录制。

### 终端 3：准备录 bag，但先不要按回车

在 Orin 打开第三个终端，先输入下面内容，但在最后一行执行前检查输出目录：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash
mkdir -p /home/robotx/livox_ws/bags
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/home/robotx/livox_ws/bags/dock_mid360_${STAMP}
echo "录制目录: $OUT"
```

如果 `/tf` 和 `/tf_static` 存在，推荐输入：

```bash
ros2 bag record -o "$OUT" /livox/lidar /tf /tf_static
```

如果确认还有地图或里程计 topic，再额外加入：

```bash
ros2 bag record -o "$OUT" \
  /livox/lidar /tf /tf_static \
  /task/semantic_map /mavros/local_position/odom
```

注意：**输入这条命令并按回车后，才算正式开始录制。**

### 终端 4：记录现场信息和拍照

建议再开一个终端记录时间；如果没有第四个终端，也可以用手机或纸笔记录。

开始录制后，先在终端 4 输入：

```bash
date -Is
```

同时拍一张现场照片，照片中尽量包含：

- USV 所在位置；
- dock 的整体方向；
- LiDAR 安装方向；
- 周围是否有遮挡物。

这张照片不是算法输入，用来以后把点云结果和真实场景对应起来。

## 现场动作顺序

### 阶段 A：静止正对 dock

下面的“让 USV 移动”由驾驶员或现场已有的驾驶控制完成，采集终端本身不会控制船。

1. 确认终端 1 有 `/livox/lidar`；
2. 确认终端 2 频率接近 10 Hz；
3. 在终端 3 执行 `ros2 bag record ...`；
4. 等待 3～5 秒，确认终端 3 显示正在录制；
5. 让 USV 停在 dock 前方，正对 dock 中间；
6. 保持静止 15～30 秒；
7. 拍一张“正对 dock”的现场照片；
8. 在终端 4 记录：`scene=front_static`、大致距离和船头方向。

这段数据用于测试最容易的正对场景。

### 阶段 B：低速斜视 dock

1. 确认终端 3 仍在录制；
2. 让 USV 低速向 dock 左侧或右侧移动；
3. 到达斜视位置后停船 5～10 秒；
4. 拍一张“斜视 dock”的现场照片；
5. 在终端 4 记录 `scene=oblique_left` 或 `scene=oblique_right`；
6. 如果船有明显颠簸，记录 `rough_motion=true`。

### 阶段 C：沿安全圆周绕行

1. 确认周围没有其他船只或人员；
2. 让 USV 低速沿 dock 外侧安全圆周移动；
3. 每移动约 30～45 度，停船 5～10 秒；
4. 每个停留点记录当前大致方位；
5. 尽量覆盖 dock 的三个入口方向；
6. 如果完成一圈，记录 `scene=arc_complete`；
7. 再停船 10～15 秒，作为最后的静止参考帧；
8. 拍一张绕行结束后的整体照片。

这一阶段最重要，因为后续算法要比较不同观察角度下的墙段和 gap。

### 阶段 D：停止录制

确认已经完成所有动作后：

1. 让 USV 停稳；
2. 等待 5 秒，让最后的点云写入 bag；
3. 回到终端 3，按 `Ctrl+C`；
4. 不要直接关闭 SSH 或 VS Code；
5. 记录终端 3 显示的 bag 路径。

然后在终端 3 输入：

```bash
ros2 bag info "$OUT"
du -sh "$OUT"
```

确认 `/livox/lidar` 消息数量大于 0，录制时长大于 0。

## 推荐拍摄清单

每个 bag 至少保存这些照片或截图：

1. dock 整体照片；
2. USV 正对 dock 的照片；
3. USV 左侧斜视 dock 的照片；
4. USV 右侧斜视 dock 的照片；
5. 绕行结束后的照片；
6. 终端 1 的 topic 类型和 publisher 截图；
7. 终端 2 的频率截图；
8. 终端 3 的 `ros2 bag info` 截图。

照片文件名建议带上场景：

```text
20261006_front_static.jpg
20261006_oblique_left.jpg
20261006_oblique_right.jpg
20261006_arc_complete.jpg
```

照片和 bag 不需要严格同步到毫秒，主要用于确认当时 LiDAR 看到的实际场景。

## 1. Orin 上采集前检查

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/livox_ws/install/setup.bash

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

ros2 topic list -t | grep -E '/livox/lidar|/tf$|/tf_static|semantic_map|odom'
ros2 topic info /livox/lidar --verbose
ros2 topic hz /livox/lidar --window 5
```

确认：

- `/livox/lidar` 类型是 `sensor_msgs/msg/PointCloud2`；
- 能看到发布节点；
- 频率大约接近 10 Hz；
- 记下点云的 `frame_id`，目前实测是 `livox_frame`。

如果没有 `/livox/lidar`，先不要录 bag，检查 Mid-360 驱动和 ROS domain。

如果 `ros2 topic info` 显示有 publisher，但 `ros2 topic hz` 没有数据，可能是 QoS 不匹配或驱动没有真正输出。先把这一步的终端输出保存下来，不要直接进入算法测试。

不要用普通的 `ros2 topic echo /livox/lidar` 长时间打印点云，会产生大量终端输出。只需要查看 topic 类型、频率和少量 header 信息即可。

## 2. 录制内容

先列出实际存在的辅助 topic：

```bash
ros2 topic list
```

### 2.1 最小录制

最小录制只需要 LiDAR；这样即使暂时没有 TF，也能保留点云数据：

```bash
mkdir -p /home/robotx/livox_ws/bags
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/home/robotx/livox_ws/bags/dock_mid360_${STAMP}

ros2 bag record -o "$OUT" \
  /livox/lidar
```

### 2.2 推荐录制

如果 topic 存在，推荐把 TF、地图和里程计一起录下来。可以先检查：

```bash
for t in /tf /tf_static /task/semantic_map /mavros/local_position/odom; do
  if ros2 topic list | grep -Fxq "$t"; then
    echo "available: $t"
  else
    echo "missing:   $t"
  fi
done
```

然后只把实际存在的 topic 放进录制命令：

```bash
ros2 bag record -o "$OUT" \
  /livox/lidar /tf /tf_static \
  /task/semantic_map /mavros/local_position/odom
```

如果某个 topic 不存在，就从命令中删掉它。不存在的 topic 不要硬写进命令。

录制开始后应看到类似信息：

```text
[INFO] Recording...
[INFO] Subscribed to topic '/livox/lidar'
```

记录下开始时间：

```bash
date -Is
```

## 3. 建议的现场动作

每个动作之间保持几秒静止，方便检查点云质量和运动畸变：

建议每个 bag 录制 1～3 分钟，并至少包含以下片段：

1. 船停在 dock 前方，保持 15～30 秒；
2. 缓慢沿 dock 外侧安全圆周移动；
3. 每转过约 30～45 度停 5～10 秒；
4. 尽量覆盖 dock 的三个入口方向；
5. 最后再停 10～15 秒，作为静止参考帧。

如果场地允许，建议分别录三种数据：

- **正对**：船正对 dock 中间；
- **斜视**：从左侧或右侧观察 dock；
- **绕行**：沿安全圆周移动一段或完整一圈。

每次开始和结束都记录：时间、船的大致位置、船头方向、与 dock 的距离、当时正在观察 dock 的哪一侧。可以直接写到一个文本文件中：

```bash
date -Is > /tmp/dock_run_notes.txt
echo 'scene=front_static' >> /tmp/dock_run_notes.txt
echo 'distance_m=待填写' >> /tmp/dock_run_notes.txt
echo 'heading=待填写' >> /tmp/dock_run_notes.txt
```

不要在采集过程中启动喷水、PX4 控制或自动靠泊。

如果船运动过程中出现明显颠簸，先保留这段 bag，不要删除；它可以用来检查运动畸变和过滤效果。但入口几何识别应优先使用停船或低速片段。

## 4. 停止并检查 bag

采集完成后在录 bag 的终端按 `Ctrl+C`，然后执行：

```bash
ros2 bag info "$OUT"
du -sh "$OUT"
```

确认：

- `/livox/lidar` 有消息；
- 录制时长和预期一致；
- `/tf_static` 是否有内容；
- 是否记录到了 `livox_frame` 到船体坐标的变换。

推荐把结果保存下来：

```bash
ros2 bag info "$OUT" > "${OUT}_info.txt"
du -sh "$OUT" > "${OUT}_size.txt"
sha256sum "${OUT}"/* > "${OUT}_sha256.txt" 2>/dev/null || true
```

`ros2 bag info` 中至少应看到 `/livox/lidar` 的消息数量大于 0、时长大于 0。若消息数量为 0，说明本次没有录到有效点云，需要重新采集。

## 5. 点云质量检查

可以在 Orin 上直接读取 bag，也可以把 bag 拷回本地后检查。检查内容包括：

- 点数和频率；
- NaN/Inf；
- 全零点数量；
- XYZ 范围；
- `point_step` 和 XYZ 字段类型；
- `frame_id`；
- 是否存在 LiDAR 到船体的 TF。

质量检查通过只代表点云数据正常，不能代表已经识别出入口。

当前已知的合格参考范围是：点云约 10 Hz，XYZ 为 `FLOAT32`，有限值比例接近 100%，全零点会在解析时过滤。若出现大量 NaN/Inf、坐标突然达到几十或几百米、频率接近 0，应先排查驱动或录包，不要继续调入口算法参数。

如果 Orin 上已经有独立的 LiDAR 测试包，可以直接对刚录好的 bag 做离线报告：

```bash
source /opt/ros/humble/setup.bash
cd /home/robotx/task3_update_staging/20261006_lidar_ros_test/ws
PYTHONPATH="$PWD/src/lidar_ros_test:$PYTHONPATH" \
python3 -m lidar_ros_test.lidar_bag_report \
  /home/robotx/livox_ws/bags/dock_mid360_YYYYMMDD_HHMMSS \
  --output-json ../dock_mid360_quality.json
```

如果要在 LiDAR 正在运行时实时监视，必须使用**驱动所在的 ROS domain**。之前的 `ROS_DOMAIN_ID=89` 只是隔离测试用的示例，不要盲目照搬：

```bash
source /opt/ros/humble/setup.bash
cd /home/robotx/task3_update_staging/20261006_lidar_ros_test/ws
source install/setup.bash

ros2 run lidar_ros_test lidar_quality_monitor \
  --topic /livox/lidar \
  --frames 100 \
  --timeout 15 \
  --output-json ../dock_mid360_live_quality.json
```

这个监视器只订阅和保存统计，不发布任何控制 topic。若 15 秒内收到 100 帧，它会提前结束；如果没有消息，也会超时退出并保存报告。

## 6. 拷回本地并运行入口 prototype

在本地执行：

```bash
mkdir -p /home/y/robotx_ws/downloads
REMOTE_BAG=/home/robotx/livox_ws/bags/dock_mid360_YYYYMMDD_HHMMSS
scp -r robotx@100.91.153.16:"$REMOTE_BAG" /home/y/robotx_ws/downloads/
```

然后把 bag 导出成 `.npy` 点云帧，再运行：

```bash
cd /home/y/robotx_ws/dock_planner_v0
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0:$PYTHONPATH \
python3 -m examples.export_mid360_bag_frames \
  /path/to/dock_mid360_YYYYMMDD_HHMMSS \
  /tmp/dock_frames --topic /livox/lidar --stride 1 --max-frames 50
```

第一次建议只导出少量帧：

```bash
--stride 5 --max-frames 20
```

确认格式没有问题后，再导出全部或更密集的帧。

入口 prototype 的输入必须满足：

```text
点云坐标系 == 粗中心坐标系
```

当前点云通常是 `livox_frame`。如果粗中心来自 UAV NED，必须先经过真实坐标转换；不能直接把 UAV NED 数值填入 prototype。

如果暂时还没有 `livox_frame` 到船体坐标的外参，可以先做两种检查：

1. 只检查点云质量和可视化；
2. 用人工测量、明确写下来的同坐标系粗中心做临时算法调试。

这种临时调试结果不能作为比赛运行结果，不能把它直接接入 DockPlanner。

单帧识别命令示例：

```bash
PYTHONPATH=/home/y/robotx_ws/dock_planner_v0/research \
python3 -m dock_lidar_prototype.run_recorded_frame \
  /tmp/dock_frames/frame_000000.npy \
  --center-x 4.0 --center-y 0.0 \
  --observer-x 0.0 --observer-y 0.0 \
  --output-png /tmp/dock_detection.png \
  --output-json /tmp/dock_detection.json
```

检查输出：

- `candidate_count`：找到的 gap 候选数量；
- `selected`：最终选中的入口；
- `selected.valid`：是否通过筛选；
- `center_x/center_y`：入口中心；
- `heading_rad`：入口朝向；
- `width_m`：入口宽度；
- `confidence`：几何置信度；
- PNG：用于人工确认检测位置是否合理。

## 7. 每次采集应保存的资料

- rosbag 目录；
- `ros2 bag info` 输出；
- LiDAR `frame_id`；
- 点云和 TF topic 列表；
- 采集时的船位、船头方向和 dock 大致距离；
- 采集动作说明（正对、斜视、绕行到哪一侧）。

## 8. 这次采集是否成功

满足下面条件，说明“数据采集”成功：

| 检查项 | 合格标准 |
|---|---|
| LiDAR topic | `/livox/lidar` 存在且类型为 PointCloud2 |
| 消息数量 | 大于 0 |
| 频率 | 约 10 Hz |
| XYZ 数值 | 没有明显 NaN/Inf 或异常大值 |
| frame_id | 已记录，通常为 `livox_frame` |
| 场景覆盖 | 至少有正对、斜视或绕行中的一种 |
| bag 文件 | 能用 `ros2 bag info` 正常打开 |

满足这些条件后，下一步才是“入口识别测试”。入口识别还要额外确认粗中心和点云坐标系一致，并且真实 dock 结构确实进入 LiDAR 视野。
