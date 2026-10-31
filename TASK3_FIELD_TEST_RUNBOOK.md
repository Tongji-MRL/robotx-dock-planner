# RobotX Task3 实船分阶段测试手册

版本日期：2026-10-31

适用主机：USV Orin

现场 SSH：`robotx@192.168.206.66`
正式工作区：`/home/robotx/robotx_ws`

## 1. 测试目标和当前边界

本手册用于在没有原开发者到场时，让队员安全完成出发前和现场最重要的 Task3 验证，
并留下可复查的日志。测试按以下顺序逐级放行：

```text
人员与硬件安全
  -> 网络、版本、构建
  -> 无控制 ROS dry-run
  -> LiDAR、TF、定位
  -> semantic map 与 Task3 输入
  -> 单点 goto 低速测试
  -> dock 数据采集与入口感知
  -> 云台/泵单独测试
  -> 通信与结果上报
```

当前必须知道的限制：

1. Orin 上 `task3_executor` 已能启动，181 项纯 Python 测试和单包构建已通过。
2. 完整 Task3 launch 当前被公共 `robotx_semantic_map` 缺少 `parse_snapshot` 接口阻塞。
3. Task3 当前不会单独完成真实车辆控制闭环；goto、终端控制、避障和控制权仲裁属于公共模块。
4. `base_link -> livox_frame` 最新外参是 `(+0.15,+0.10,+0.22) m`，RPY 暂为零。
5. 未验证唯一控制发布者、人工接管和泵默认关闭之前，禁止开启自主运动或喷水。

明天现场的最低目标按优先级排列：

| 优先级 | 目标 | 允许的最终结论 |
|---:|---|---|
| P0 | 完成安全、版本、构建、语义地图 API、LiDAR/TF、定位和人工接管检查 | 可以或不可以继续现场测试 |
| P1 | 采集多个方位的真实 dock 点云、位姿、照片和视频 | `PASS_FOR_DATA_COLLECTION_ONLY` |
| P2 | 在开阔水面完成 0.5 m 和 1.0 m 单点 goto | 单点控制接口可用 |
| P3 | 分别完成窗口感知、云台、泵、通信 dry-run | 对应单模块可用 |
| 可选 | 分段执行安全圆周、短弧、入口识别和低速靠泊 | 只记录已实际完成的分段 |

P0 或 P1 完成就有有效测试产出。不得为了追求“完整流程”跳过停止条件。

## 2. 人员、设备和职责

至少四人到场，角色不能省略：

| 角色 | 职责 |
|---|---|
| 测试负责人 | 唯一有权宣布进入下一阶段或终止测试 |
| 遥控/安全员 | 全程握住遥控器，能立即切手动、急停或断推进电源 |
| 岸边观察员 | 观察船体、人员、岸边、dock、绳索和航行边界，不操作电脑 |
| 电脑操作员 | SSH、启动节点、记录日志，不擅自发送控制或泵命令 |
| 泵/云台负责人 | 只有进入泵测试阶段后才接通水泵电源，并负责物理断电 |

现场必须准备：

- 充满电的遥控器、备用电池和可用的人工接管模式；
- 可靠的推进器急停/断电方式；
- 水泵独立断电方式；
- 救生衣、回收绳、长杆或回收艇；
- Orin 同网段电脑、电源、网线/路由器；
- 一名人员负责拍摄船体运动和喷水状态；
- 足够磁盘空间保存 rosbag、日志和照片。

任何人进入推进器、水泵喷射方向或船体可能运动的区域时，推进和水泵必须物理断电。

## 3. 统一停止口令和停止动作

任何人都可以喊“停止”。听到后按固定顺序处理：

1. 遥控安全员立即切换人工/安全模式并给零推力；
2. 无法确认零推力时，直接断推进电源；
3. 泵负责人关闭泵并断泵电源；
4. 电脑操作员停止当前 launch，保存终端输出；
5. 船稳定后再检查软件，禁止边运动边排查。

软件 `Ctrl+C` 不能替代遥控接管或物理断电。节点崩溃后，底层可能暂时保持最后一条命令。

立即停止条件：

- 船向岸、人员、其他船或 dock 非预期加速；
- 遥控接管无效、控制模式不明确；
- `/cmd_vel` 或最终控制 topic 出现多个未知发布者；
- 定位、heading、TF 或时间戳突变；
- LiDAR、避障或 goto 状态中断；
- 水泵无法关闭、继电器粘连、云台失控；
- Orin 过热、掉电、CPU/内存异常或网络持续中断。

## 4. 测试记录目录

在 SSH 登录后创建本次记录目录：

```bash
TEST_ID=$(date +%Y%m%d_%H%M%S)
TEST_LOG=/home/robotx/robotx_logs/task3_field_${TEST_ID}
mkdir -p "$TEST_LOG"
echo "$TEST_LOG"
```

每个终端先加载唯一正式环境：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
```

不要 source `~/livox_ws`、`~/mavros_ws`、历史 staging 或其他旧工作区。

推荐使用 tmux：

```bash
tmux new -s task3_field
```

建议窗口：`00_health`、`01_mavros`、`02_lidar`、`03_semantic`、`04_task3`、
`05_goto`、`06_gimbal_pump`、`07_bag`。使用 `Ctrl+b c` 新建窗口，
`Ctrl+b ,` 修改窗口名。

## 5. 阶段 A：上电前物理检查

船仍在岸上支架，推进器和水泵保持物理断电。

逐项确认并拍照：

- [ ] 船体无进水，电池、接头和保险固定；
- [ ] 推进器无杂物，左右桨与方向确认；
- [ ] Mid-360、GPS、相机、云台固定，无松动；
- [ ] Mid-360 相对中央盖板参考点为前 `0.15 m`、左 `0.10 m`、上 `0.22 m`；
- [ ] LiDAR 正对船艏，安装角暂按零；
- [ ] 遥控器接管和急停已由安全员现场演示；
- [ ] 泵默认断电，喷嘴朝向无人区域；
- [ ] 航行区、岸边、浅水、绳索和禁止进入区已口头确认。

任一项不确定，停止测试，不靠软件猜测补偿。

## 6. 阶段 B：网络、SSH、版本和资源检查

测试电脑先检查网络：

```bash
ping -c 3 192.168.206.66
ssh robotx@192.168.206.66
```

登录后执行：

```bash
hostname
date -Ins
uptime
df -h /home/robotx
free -h
git -C /home/robotx/robotx_ws/src/task3 rev-parse --short HEAD
git -C /home/robotx/robotx_ws/src/task3 status --short
```

通过标准：

- 主机名为 `t-wave-orin`；
- Task3 代码基线为 `a46a96c`，或队长明确批准的更新版本；
- 只允许清单文档有已知改动，不能有未知源码修改；
- `/home/robotx` 可用空间至少 10 GB；
- 时间与测试电脑相差不超过 1 秒。

包来源检查：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
ros2 pkg prefix usv_task3_executor
ros2 pkg prefix robotx_perception_common
ros2 pkg prefix robotx_semantic_map
ros2 pkg prefix mavros
ros2 pkg prefix livox_ros_driver2
```

所有输出都必须位于 `/home/robotx/robotx_ws/install/`。

重复包检查：

```bash
cd /home/robotx/robotx_ws
colcon list --base-paths src | awk '{print $1}' | sort | uniq -d
```

通过标准：无输出。出现重复包时，不启动任何节点，先把重复源码移到工作区外备份目录。

## 7. 阶段 C：离线测试和构建

推进器和泵继续断电。

Task3 纯逻辑测试：

```bash
cd /home/robotx/robotx_ws/src/task3
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=$PWD \
python3 -m unittest discover -s tests -p 'test_*.py'
```

预期：`Ran 181 tests`、`OK`。

只构建 Task3：

```bash
cd /home/robotx/robotx_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select usv_task3_executor
source install/setup.bash
ros2 pkg executables usv_task3_executor
```

应看到：

- `task3_executor`
- `task3_semantic_map_adapter`
- `task3_perception_adapter`
- `task3_communication_adapter`

公共语义地图接口检查：

```bash
python3 - <<'PY'
from robotx_semantic_map import parse_snapshot, select_objects_by_type
print("semantic map API: PASS")
PY
```

当前已知问题：如果这里出现 `ImportError: cannot import name 'parse_snapshot'`，禁止运行完整
Task3 launch。由公共语义地图负责人更新 `robotx_semantic_map` 到包含这两个接口的版本，
重新构建该包和 Task3 后再执行本阶段。不能在 Task3 内临时复制同名函数绕过。

## 8. 阶段 D：无控制 ROS 健康检查

推进器和泵仍断电。确认所有终端使用同一 `ROS_DOMAIN_ID`：

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
ros2 node list
ros2 topic list -t | tee "$TEST_LOG/topic_list.txt"
ros2 service list -t | tee "$TEST_LOG/service_list.txt"
```

检查最终控制 topic 的发布者数量：

```bash
ros2 topic info /usv/control/cmd_vel --verbose
```

如果现场最终控制 topic 不是 `/usv/control/cmd_vel`，由控制负责人先给出真实 topic，后续
检查全部替换为该 topic，禁止根据名字猜测。未进入控制测试前允许 0 个发布者；进入控制
测试后只允许队长确认的唯一仲裁节点发布。
出现第二个未知发布者，立即停止所有自主节点。

只启动 Task3 executor，不启动 semantic adapter：

```bash
ros2 run usv_task3_executor task3_executor --ros-args -p dry_run:=true
```

另一个终端检查：

```bash
ros2 node list | grep usv_task3_executor
ros2 topic echo --once /tongji_mrl/twave/task3/state
```

通过标准：节点进入 `IDLE`，没有车辆控制或泵命令发布。完成后 `Ctrl+C`。

公共语义地图 API 修复后，再运行完整 dry-run：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  platform_id:=0 dry_run:=true
```

通过标准：executor 和 semantic adapter 都保持运行，无 traceback。

## 9. 阶段 E：LiDAR、TF 和点云检查

船仍在岸上支架。启动 Mid-360：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py \
  lidar_x:=0.15 lidar_y:=0.10 lidar_z:=0.22 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

另一个终端检查：

```bash
ros2 node list | grep -E 'livox|task3_lidar'
ros2 topic info /livox/lidar --verbose
timeout 12s ros2 topic hz /livox/lidar
ros2 topic echo --once /livox/lidar --field header
timeout 5s ros2 run tf2_ros tf2_echo base_link livox_frame
```

通过标准：

- `/livox/lidar` 有 1 个 publisher；
- 类型是当前配置对应的 `livox_ros_driver2/msg/CustomMsg`；
- 频率约 10 Hz，短时波动可以接受；
- `frame_id=livox_frame`；
- TF translation 为 `[0.150, 0.100, 0.220] m`；
- RPY 接近 `[0,0,0]`。

若 TF 仍为 `y=0.00`，停止后续入口识别，检查启动参数和感知包默认值；禁止在日志里把
旧 TF 当作新外参。

点云异常处理：

| 现象 | 处理 |
|---|---|
| 无 `/livox/lidar` | 检查 LiDAR 电源、网线、Orin 网卡 IP、驱动节点和配置文件 |
| publisher 有但频率为 0 | 重启驱动；检查设备 IP 冲突和网线；保存驱动日志 |
| frame 不是 `livox_frame` | 停止感知，统一驱动配置和 TF frame 名称 |
| 有全零点 | 原始 Mid-360 数据中约 3.8% 全零占位点可以存在，适配器必须过滤 |
| TF 不存在或方向相反 | 禁止入口识别和自动运动，重新核对父子 frame 和安装方向 |
| 频率持续低于 8 Hz | 检查 CPU、网络和点云处理负载，未恢复前不进入运动测试 |

## 10. 阶段 F：MAVROS、GPS、NED 和 heading

本阶段仍不发送 setpoint。启动方式由飞控负责人给出，随后只读检查：

```bash
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/global_position/global
ros2 topic echo --once /mavros/local_position/odom
ros2 topic echo --once /tongji_mrl/twave/ned_position
```

通过标准：

- MAVROS 显示飞控已连接；
- GPS 经纬度不是零值或旧值；
- NED 消息 `valid=true`、时间戳持续更新；
- `frame_id` 与 Task3 约定一致；
- heading 来源、单位和零方向已确认；
- 船静止时位置无明显跳变，人工缓慢转船时 heading 方向正确。

若 NED 消息没有 heading，Task3 只能记录粗中心，不能开始圆周搜索。不得伪造 heading。

低速转船检查：在推进器仍断电时由两人手动缓慢转船，观察 GPS/NED。若 `base_link`
位置随旋转画出明显圆圈，说明 GPS/IMU 杠杆臂补偿未确认，停止自动靠泊测试。

## 11. 阶段 G：semantic map 到 Task3 粗中心

只有第 7 节 semantic map API 已通过才执行。

启动公共语义地图：

```bash
ros2 launch robotx_semantic_map semantic_map.launch.py
```

启动 Task3 dry-run：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  platform_id:=0 dry_run:=true
```

观察：

```bash
ros2 topic echo --once /task/semantic_map
ros2 topic echo --once /tongji_mrl/twave/task3/platforms
ros2 topic echo --once /tongji_mrl/twave/task3/coarse_dock
ros2 topic echo /tongji_mrl/twave/task3/state
```

如果没有 UAV 输入，可由测试负责人在独立 ROS domain 使用一条 mock semantic map；不要在
正式比赛 domain 发布 mock。示例仅用于推进器断电的 dry-run：

```bash
ros2 topic pub --once --qos-durability transient_local \
  /task/semantic_map std_msgs/msg/String \
  "{data: '{\"map_epoch\":\"field-dryrun\",\"version\":1,\"objects\":[{\"id\":0,\"type\":\"platform\",\"NED_position\":[5.0,2.0,0.0],\"confidence\":0.9}]}'}"
```

通过标准：只出现一个目标 platform；`coarse_dock` 使用真实 NED 字段、`mock=false`、
`source=semantic_map`；不存在重复 ID。若出现多个同 ID、坐标非有限或 frame 不明，停止。

## 12. 阶段 H：单点 goto 低速实船测试

本阶段第一次允许推进器上电。必须满足：

- 前述所有静态检查通过；
- 遥控安全员已经演示接管；
- 水域开阔，船与岸、人员、dock 至少保持现场负责人规定的安全距离；
- 水泵保持断电；
- 最终控制 topic 只有一个批准的发布者；
- 避障模块的工作方式和失效动作已确认。

测试步骤：

1. 船先保持手动模式，在开阔水面低速验证左右推进方向。
2. 读取当前 `/mavros/local_position/odom`，由控制器负责人确认 goto 使用的 frame、单位和目标格式。
   先执行 `ros2 topic info /safe_passage/goto/target --verbose` 和
   `ros2 topic info /safe_passage/goto/status --verbose`；类型或 topic 不一致时，以已部署控制器
   的源码和负责人确认结果为准。
3. 第一个目标只设为当前位置前方 `0.5 m`，最高速度使用控制器最低安全配置。
4. 发送目标前，观察员确认路径无障碍；安全员口头复述“可接管”。
5. 发送后记录目标、实际轨迹、最大误差、到达时间和 status。
6. 目标到达后立即回到人工/STOP，检查是否仍有非零控制输出。
7. 依次测试 `0.5 m` 直行、低速转向、`1.0 m` 直行；任一失败不增加距离。

goto 接口预期：

- 目标：`/safe_passage/goto/target`，`geometry_msgs/msg/PointStamped`；
- 状态应包含 `DONE` 或 `ABORTED`；
- 具体 `frame_id` 必须以现场控制器源码/负责人确认为准，禁止照抄未知坐标。

本手册不提供一个可直接粘贴的真实运动发布命令。只有控制负责人根据当前位置计算好
0.5 m 目标、复述 frame 后，电脑操作员才使用控制模块自带的测试入口发送；这样可避免把
NED 数值误发到 ENU/local topic。发送前把最终命令复制进 `$TEST_LOG/goto_commands.txt`。

错误处理：

| 现象 | 动作 |
|---|---|
| 船向反方向运动 | 立即人工接管；检查 ENU/NED、X/Y 轴和推进器映射 |
| 无到达反馈 | 人工停止；检查 status topic、距离阈值和控制器进程 |
| `ABORTED` | 不自动重发；保存原因，检查边界、避障和定位 |
| 原地振荡或绕圈 | 人工停止；检查 heading、控制增益、GPS 杠杆臂和速度上限 |
| 网络断开 | 安全员立即接管；不得等待 SSH 自动恢复 |
| 多个 cmd_vel publisher | 立即停止所有自主节点，保留唯一仲裁出口 |

## 13. 阶段 I：dock 周边 LiDAR 数据采集

本阶段的主要目标是采数据，不要求一次识别成功。

建议位置：

1. 正对三个 bay 的中间位置；
2. dock 左前方约 30°；
3. dock 右前方约 30°；
4. 安全圆周上每隔固定角度停车一次；
5. 至少覆盖三个入口可见位置和入口被遮挡位置。

每个位置先停车稳定 5 秒，再录 15–30 秒：

```bash
ros2 bag record -o "$TEST_LOG/dock_view_01" \
  /livox/lidar \
  /tf /tf_static \
  /mavros/local_position/odom \
  /mavros/global_position/global \
  /tongji_mrl/twave/ned_position \
  /tongji_mrl/twave/task3/coarse_dock \
  /tongji_mrl/twave/task3/dock_perception
```

每段 bag 同时记录：船到 dock 的估计距离、船艏朝向、可见入口、天气、浪流、照片编号。
文件录完后执行：

```bash
ros2 bag info "$TEST_LOG/dock_view_01"
du -sh "$TEST_LOG/dock_view_01"
```

入口感知只读检查：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  input_topic:=/livox/lidar input_format:=livox_custom
ros2 topic echo /tongji_mrl/twave/task3/dock_perception
```

上面的 `livox_custom` 对应当前 Orin 实测的
`livox_ros_driver2/msg/CustomMsg`。如果第 9 节现场检查显示输入已经改为
`sensor_msgs/msg/PointCloud2`，把参数改成 `input_format:=pointcloud2`；消息类型与参数不一致
时感知结果无效。

通过标准：输出包含入口中心、朝内 heading、confidence、timestamp 和 valid；无入口时应
输出 invalid/不发布，而不是猜入口。入口方向与照片明显相反时，先查 TF 和 heading，
不要现场盲目调算法阈值。

## 14. 阶段 J：云台、窗口感知和水泵

先只读检查接口：

```bash
ros2 topic info /robotx/perception/sprayer/active_window_3d --verbose
ros2 topic info /vision/target_observation --verbose
ros2 topic info /gimbal/status --verbose
ros2 topic info /pump/status --verbose
ros2 service type /pump/set
ros2 interface show robotx_gimbal_interfaces/srv/SetPump
```

执行顺序：

1. 泵物理断电，只验证窗口识别、云台角度和状态反馈。
2. 用红色目标检查 active window 和目标坐标稳定性。
3. 遮挡目标，确认丢失后不会产生“继续喷水”状态。
4. 由泵负责人确认喷嘴方向、供水、继电器和物理断电后，才允许短时通电。
5. 第一次开泵不超过 1 秒，观察 `/pump/status` 是否与命令一致。
6. 测试停泵，确认状态为 OFF 后等待 10 秒，检查无继电器粘连。
7. 只有停泵可靠后，才测试“红色开泵—绿色立即停泵—绿色连续 5 秒完成”。

不要在本文写死 `SetPump` 请求字段；必须先以 `ros2 interface show` 的现场输出为准。

水泵错误处理：

| 现象 | 动作 |
|---|---|
| `/pump/set` 不存在 | 不接泵电源；检查节点、接口包和 launch |
| 开泵无状态反馈 | 立即物理断泵；不按“可能成功”继续计时 |
| 停泵命令失败 | 重复一次后立即物理断电并结束泵测试 |
| 绿色出现后仍喷水 | 立即断泵；检查窗口消息、控制器状态和唯一命令发布者 |
| 目标丢失仍喷水 | 立即断泵；该版本不得进入自动喷水 |
| 泵/线缆过热或异味 | 立即断电，硬件负责人检查后才能恢复 |

## 15. 阶段 K：通信和颜色序列 dry-run

不启动车辆控制。只启动 Task3 通信 adapter：

```bash
ros2 launch usv_task3_executor task3_communication_adapter.launch.py run_id:=42
```

检查 topic 和订阅者：

```bash
ros2 topic info /tongji_mrl/twave/task_cmd --verbose
ros2 topic info /tongji_mrl/twave/progress --verbose
ros2 topic info /tongji_mrl/twave/result --verbose
ros2 topic info /tongji_mrl/twave/heartbeat --verbose
ros2 topic info /tongji_mrl/tsky/heartbeat --verbose
```

验证内容：

- `begin` 只转发一次，重复命令不会重复执行；
- `end` 转成 SAFE_STOP/ACK，不会被误报为 TASK3_FINISH；
- `tin_color`、`circle_color` 顺序和颜色编码与 UAV/OCS 一致；
- heartbeat 中断超过约 3 秒能检测，恢复后状态可解释；
- docking 与 firefighting 完成必须来自权威条件，不由单帧颜色代替。

无人订阅或没有 ACK 时，只记录“未完成通信闭环”，不能宣布 Task3 完成。

## 16. 现场可选的分段靠泊测试

只有 goto、LiDAR、TF、定位、人工接管和控制权仲裁全部通过后才考虑。当前代码尚未形成
比赛级自动闭环，因此分段测试必须由负责人逐段批准：

1. 仅到安全圆周，停船；
2. 沿圆周走一个短弧，停船；
3. LiDAR 确认入口，但不进 bay；
4. 仅对齐入口，停船；
5. 低速到入口线，停船；
6. 低速进入并人工确认停泊；
7. 靠泊稳定后再单独测试云台和泵；
8. 退出到 staging，再人工返航。

每一步都必须人工口头确认后再发送下一目标。禁止一次发送完整轨迹；禁止在入口附近自动
重试未知故障；入口方向、定位或控制反馈不可信时，立即人工退出。

## 17. 常见错误快速处理表

| 错误/现象 | 最可能原因 | 处理 |
|---|---|---|
| SSH 连接失败 | 不同网段、Wi-Fi/路由器、Orin 未上电 | ping `192.168.206.66`，检查本机 IP 和网线；不要反复重启整船 |
| `ros2` 找不到包 | source 了错误工作区 | 新终端只 source ROS Humble 和正式 `install/setup.bash` |
| duplicate package | 旧源码或备份仍在 `src` | 停止构建，把重复目录移到 `robotx_backups` 后复查 |
| Task3 build 报旧 launch 不存在 | 旧 build/install 清单残留 | 只备份该包的 build/install 产物，再单包构建 |
| `parse_snapshot` ImportError | 公共 semantic map 版本过旧 | 更新公共包、重新构建；不在 Task3 复制实现 |
| executor 启动后一直 IDLE | 没有有效粗中心/当前姿态/开始事件 | 检查 semantic map、NED、heading 和状态机命令 |
| LiDAR topic 不存在 | 驱动/供电/网卡配置 | 检查节点、电源、IP 和启动配置 |
| LiDAR 有数据但无入口 | TF、粗中心、heading、ROI 或视角 | 先录 bag 和照片；核对坐标后再调参数 |
| TF y 仍为 0 | 使用了旧默认参数 | 用 `lidar_y:=0.10` 启动并更新公共感知配置 |
| NED 数据 stale/invalid | GPS、时钟或共享位置节点 | 停止自主测试，检查 GPS、系统时间和节点状态 |
| goto 无响应 | 控制器未启动、frame 错、无控制权 | 人工停止，检查 target/status 和仲裁 owner |
| 船突然转向或振荡 | ENU/NED、heading、增益或推进映射 | 立即人工接管，回到 0.5 m 低速测试 |
| `/cmd_vel` 多发布者 | 多控制器同时在线 | 停止所有自主节点，只保留唯一仲裁输出 |
| 泵无法关闭 | service/继电器/多发布者故障 | 物理断泵电源，结束测试并标记硬件故障 |
| 颜色序列无人接收 | 通信模块/订阅者/协议不一致 | 保存消息和 topic info，不宣称完成 |
| rosbag 写失败 | 磁盘满或权限 | 停止 bag，检查 `df -h`，不要删除唯一原始数据 |
| 节点崩溃 | 依赖、消息类型或版本不一致 | 保留 traceback、commit、topic list；回到上一安全阶段 |

## 18. 测试结束和数据交接

结束顺序：

1. 停止 Task3、goto、感知和通信节点；
2. 确认泵 OFF，并物理断泵电源；
3. 切人工/安全模式，确认推进零输出；
4. 停止 rosbag，执行 `ros2 bag info`；
5. 保存 topic/service 列表、TF、版本、参数、终端日志、照片和视频；
6. 船上岸后再关闭 LiDAR、飞控和 Orin；
7. 不删除失败数据，失败数据对定位接口问题最有价值。

现场结果至少记录：

| 项目 | 结果/数值 |
|---|---|
| 测试日期、地点、人员 |  |
| Task3 commit |  |
| ROS_DOMAIN_ID |  |
| LiDAR topic/type/frequency |  |
| TF x/y/z、RPY |  |
| GPS/RTK 状态 |  |
| NED frame、heading 来源 |  |
| goto 0.5 m/1.0 m 误差 |  |
| 可见 dock/bay 和照片编号 |  |
| bag 路径与大小 |  |
| 泵开/关反馈和最大持续时间 |  |
| 通信 ACK 结果 |  |
| 未通过项、错误日志、处理人 |  |

最终结论只能选择：

- `PASS_FOR_NEXT_STAGE`：仅表示可以进入下一阶段；
- `PASS_FOR_DATA_COLLECTION_ONLY`：只允许采数据，不允许自主控制；
- `FAIL_AND_HOLD`：保持人工/停止状态，修复后重测。

没有完整日志、照片和明确通过标准时，不填写“整链成功”。
