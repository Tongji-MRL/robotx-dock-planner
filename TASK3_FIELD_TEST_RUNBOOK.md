# RobotX Task3 实船快速测试手册

适用环境：Ubuntu 测试电脑 + VS Code Remote SSH 连接 USV Orin

Orin：`robotx@192.168.206.66`

地面端：`192.168.206.200`

移动端：`192.168.206.202`

正式工作区：`/home/robotx/robotx_ws`

## 1. 今天要测什么

按下面顺序执行，前一项不通过就不要进入后面的运动或喷水测试：

```text
网络与人工接管
→ 代码和 ROS 环境
→ LiDAR、TF、GPS/NED
→ Task3 dry-run 与粗中心
→ dock 入口实时识别
→ 通信、云台和水泵接口
→ 0.5 m / 1.0 m 最小运动测试
```

不再采集 LiDAR rosbag，不要求现场一次跑完整任务。每项只需记录“通过/失败”和一张关键
截图。当前外参必须为：

```text
base_link -> livox_frame
translation = [0.15, 0.10, 0.22] m
RPY = [0, 0, 0]
```

## 2. 安全规则

- 至少有一人全程握遥控器，能立即切人工、零推力或断推进电源。
- 水泵测试前必须明确物理断电方法，喷嘴方向不得有人。
- 软件 `Ctrl+C` 不能代替遥控接管或物理断电。
- 网络连续丢包、定位跳变、船反向运动、多控制发布者或泵关不掉时立即停止。
- 未确认 goto 的 frame、单位和到达反馈前，不发送真实目标。

## 3. 终端安排

在 Ubuntu 测试电脑上用 VS Code Remote SSH 连接：

```bash
ssh robotx@192.168.206.66
```

VS Code 连接成功后，使用“终端 → 新建终端”建立以下终端。每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
```

| 终端 | 用途 | 是否保持 |
|---|---|---|
| Ubuntu 本机 `L0` | 持续 ping Orin | 运动测试期间保持 |
| 远程 `R0-query` | 所有一次性查询 | 可重复使用 |
| 远程 `R1-lidar` | Mid-360 驱动和 TF | 启动后保持 |
| 远程 `R2-task3` | semantic map / Task3 dry-run | 启动后保持 |
| 远程 `R3-perception` | 入口感知 | 启动后保持 |
| 远程 `R4-aux` | 通信、云台、水泵，分项测完后停止再复用 | 按当前项目保持 |
| 远程 `R5-control` | goto/控制器 | 仅运动测试时保持 |

凡是 `ros2 launch`、`ros2 run`、持续 `ros2 topic echo`，命令不会自动返回。不要在该终端
继续输入下一条命令，应新开终端或回到 `R0-query`。

## 4. 网络检查

在 Ubuntu 本机终端执行：

```bash
ip -4 addr
ping -c 10 192.168.206.66
ping -c 10 192.168.206.200
ping -c 10 192.168.206.202
nc -vz 192.168.206.66 22
```

三台设备都必须可达，Orin 的 SSH 端口必须成功。记录测试电脑 IPv4，登录 Orin 后在
`R0-query` 反向检查：

```bash
ping -c 10 TEST_PC_IP
ping -c 10 192.168.206.200
ping -c 10 192.168.206.202
```

把 `TEST_PC_IP` 换成测试电脑实际地址。进入水上运动前，在 Ubuntu 本机 `L0` 持续执行：

```bash
ping 192.168.206.66
```

连续超时 3 次立即人工接管。ping 正常只代表网络可达，地面端和移动端还要在第 9 节验证
heartbeat/ACK。

## 5. 环境和代码检查

船在岸上，推进器和水泵断电。在 `R0-query` 执行：

```bash
hostname
date -Ins
df -h /home/robotx
git -C /home/robotx/robotx_ws/src/task3 rev-parse --short HEAD
git -C /home/robotx/robotx_ws/src/task3 status --short
ros2 pkg prefix usv_task3_executor
ros2 pkg prefix robotx_perception_common
ros2 pkg prefix robotx_semantic_map
ros2 pkg prefix mavros
ros2 pkg prefix livox_ros_driver2
```

所有 ROS 包都应来自 `/home/robotx/robotx_ws/install/`。然后运行核心测试：

```bash
cd /home/robotx/robotx_ws/src/task3
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$PWD \
python3 -m unittest discover -s tests -p 'test_*.py'
```

预期：`Ran 181 tests`、`OK`。

如果当天更新过 Task3 源码，先执行一次单包构建；没有更新则跳过以节省时间：

```bash
cd /home/robotx/robotx_ws
colcon build --symlink-install --packages-select usv_task3_executor
source install/setup.bash
```

再检查语义地图接口：

```bash
python3 - <<'PY'
from robotx_semantic_map import parse_snapshot, select_objects_by_type
print("semantic map API: PASS")
PY
```

若出现 `cannot import name 'parse_snapshot'`，记录为公共语义地图版本不兼容。此时可以继续
测网络、LiDAR、定位、云台和泵，但不能进行完整 Task3 dry-run。

## 6. LiDAR、TF 和定位

先在 `R0-query` 查看 LiDAR 是否已经运行：

```bash
ros2 topic info /livox/lidar --verbose
```

如果没有 publisher，在 `R1-lidar` 启动并保持：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py \
  lidar_x:=0.15 lidar_y:=0.10 lidar_z:=0.22 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

回到 `R0-query`：

```bash
timeout 12s ros2 topic hz /livox/lidar
ros2 topic echo --once /livox/lidar --field header
timeout 5s ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/global_position/global
ros2 topic echo --once /mavros/local_position/odom
ros2 topic echo --once /tongji_mrl/twave/ned_position
```

通过条件：

- LiDAR 约 10 Hz，`frame_id=livox_frame`；
- TF 为 `[0.15, 0.10, 0.22]`，不能还是旧的 `y=0.00`；
- MAVROS 已连接，GPS、odom、NED 持续更新；
- heading 的来源、单位和零方向明确。

任一定位或 TF 不可信时，不做运动和入口方向验证。

## 7. Task3 dry-run 和粗中心

只有 `parse_snapshot` 检查通过才执行。在 `R2-task3` 启动并保持：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  platform_id:=0 dry_run:=true
```

回到 `R0-query`：

```bash
ros2 node list | grep -E 'task3|semantic'
ros2 topic echo --once /task/semantic_map
ros2 topic echo --once /tongji_mrl/twave/task3/platforms
ros2 topic echo --once /tongji_mrl/twave/task3/coarse_dock
ros2 topic echo --once /tongji_mrl/twave/task3/state
```

通过条件：两个节点无 traceback；目标 platform 唯一；粗中心是有效 NED；Task3 状态可读。
没有 UAV/semantic map 输入时记录“缺少上游输入”，不要伪造为成功。

## 8. dock 入口实时识别

不录 rosbag。确保 LiDAR、NED 和 coarse dock 都在更新。在 `R3-perception` 启动并保持：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  input_topic:=/livox/lidar input_format:=livox_custom
```

如果 `/livox/lidar` 实际类型是 `sensor_msgs/msg/PointCloud2`，把参数改成
`input_format:=pointcloud2`。回到 `R0-query`：

```bash
ros2 topic echo /tongji_mrl/twave/task3/dock_perception
```

观察到几帧后按 `Ctrl+C`。通过条件：入口中心、朝向、confidence、timestamp、valid 字段
合理；看不到入口时应 invalid/不发布。方向相反先检查 TF、heading、粗中心和输入格式。

## 9. 通信、云台和水泵

### 9.1 通信

在 `R4-aux` 启动：

```bash
ros2 launch usv_task3_executor task3_communication_adapter.launch.py run_id:=42
```

在 `R0-query` 检查：

```bash
ros2 topic info /tongji_mrl/twave/task_cmd --verbose
ros2 topic info /tongji_mrl/twave/progress --verbose
ros2 topic info /tongji_mrl/twave/result --verbose
ros2 topic info /tongji_mrl/twave/heartbeat --verbose
ros2 topic info /tongji_mrl/tsky/heartbeat --verbose
```

确认地面端 `.200` 和移动端 `.202` 在线；begin/end、heartbeat、ACK、颜色序列能双向传递。
只有 ping、没有 heartbeat/ACK 时，通信测试不通过。测完在 `R4-aux` 按 `Ctrl+C`。

### 9.2 云台和水泵

由云台负责人在空闲的 `R4-aux` 启动现有云台/水泵节点。先在 `R0-query` 检查：

```bash
ros2 topic info /robotx/perception/sprayer/active_window_3d --verbose
ros2 topic info /vision/target_observation --verbose
ros2 topic info /gimbal/status --verbose
ros2 topic info /pump/status --verbose
ros2 service type /pump/set
ros2 interface show robotx_gimbal_interfaces/srv/SetPump
```

测试顺序：

1. 泵物理断电，先测窗口识别、目标丢失和云台反馈。
2. 确认 `/pump/set` 请求字段和物理断电方法。
3. 接通泵，第一次开泵不超过 1 秒。
4. 立即停泵，确认 `/pump/status=OFF`，等待 10 秒检查继电器。
5. 最后测试“红色开泵 → 绿色停泵 → 绿色持续 5 秒完成”。

泵无法关闭、无状态反馈或目标丢失后仍喷水时，立即物理断电并停止本阶段。

## 10. 最小运动测试

仅当前面所有必要检查通过、遥控接管有效、水域开阔时执行。水泵保持断电。

在 `R0-query` 先检查最终控制 topic 和 goto 接口：

```bash
ros2 topic info /usv/control/cmd_vel --verbose
ros2 topic info /safe_passage/goto/target --verbose
ros2 topic info /safe_passage/goto/status --verbose
```

最终控制 topic 只能有一个已确认的 publisher。由控制负责人在 `R5-control` 启动 goto/
控制器，并确认 target 的 frame、单位和到达反馈。不要照抄未知坐标发送目标。

测试顺序：

1. 手动低速确认左右推进方向和遥控接管。
2. 发送当前位置前方 `0.5 m` 的目标，使用最低安全速度。
3. 到达后立即切人工，确认没有残留非零输出。
4. 再测一次低速转向和 `1.0 m` 直行。
5. 任一项反向、振荡、无到达反馈或网络中断，立即人工接管，不增加距离。

现场时间足够时，才按“安全圆周 → 短圆弧 → 入口对齐 → 低速进入 → 退出”逐段测试；
每段结束都停船确认，不发送一整套自动轨迹。

## 11. 快速故障处理

| 现象 | 处理 |
|---|---|
| VS Code SSH 连不上 | 本机 ping `192.168.206.66`，检查网段、Orin 电源和 SSH 22 端口 |
| `ros2` 找不到包 | 新终端只 source ROS Humble 和正式工作区 |
| `parse_snapshot` 导入失败 | 公共 semantic map 版本不兼容；跳过完整 Task3 dry-run |
| LiDAR 无数据 | 检查供电、网线、驱动、设备 IP 和重复 publisher |
| TF 仍为 `y=0.00` | 用 `lidar_y:=0.10` 重启，未修正前不测入口 |
| NED/heading 不更新 | 停止自主测试，检查 MAVROS、GPS 和共享位置节点 |
| 入口无结果 | 检查粗中心、NED、TF、heading 和 `input_format` |
| ping 正常但无 ACK | 检查应用进程、ROS domain、topic 和消息类型 |
| 多个控制 publisher | 停止所有自主节点，只保留批准的唯一控制出口 |
| 船反向、振荡或网络断开 | 立即遥控接管、零推力，不自动重试 |
| 泵无法关闭 | 立即物理断泵电源，结束泵测试 |

## 12. 结束与结果

结束顺序：人工接管并零推力 → 泵关闭并断电 → 停 goto → 停 Task3/通信 → 停感知 →
停 LiDAR/MAVROS → 船上岸后关闭飞控和 Orin。

最终只需填写：

| 项目 | 结果 |
|---|---|
| `.66/.200/.202` 网络与 SSH | 通过 / 失败 |
| 181 项核心测试、semantic API | 通过 / 失败 |
| LiDAR 10 Hz、TF `[0.15,0.10,0.22]` | 通过 / 失败 |
| MAVROS、GPS、NED、heading | 通过 / 失败 |
| Task3 dry-run、coarse dock | 通过 / 失败 |
| 入口中心和 heading | 通过 / 失败 |
| 地面端/移动端 heartbeat、ACK | 通过 / 失败 |
| 窗口、云台、水泵开关 | 通过 / 失败 |
| goto 0.5 m / 1.0 m | 通过 / 失败 / 未执行 |
| 主要错误和截图编号 |  |

只要运动前检查未通过，最终结论就是“静态接口测试完成，禁止自主运动”。
