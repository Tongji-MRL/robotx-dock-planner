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
  -> dock 入口感知在线验证
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
| P1 | 完成 Windows 测试电脑、地面端、移动端、Orin 的网络与通信检查 | 四端链路可用 |
| P2 | 完成 LiDAR/TF、入口感知、语义地图和 Task3 dry-run | 对应感知/规划输入可用 |
| P3 | 在开阔水面完成 0.5 m 和 1.0 m 单点 goto | 单点控制接口可用 |
| P4 | 分别完成窗口感知、云台、泵、通信 dry-run | 对应单模块可用 |
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
- 足够磁盘空间保存日志、截图、照片和视频。

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

## 4. Windows 测试电脑和终端安排

### 4.1 三种终端标记

后续每一步都使用以下标记：

| 标记 | 含义 | 是否可以继续输入其他命令 |
|---|---|---|
| **常驻** | 正在运行 launch、节点或持续监视；测试期间必须保持 | 不可以，必须新开终端/tmux 窗口 |
| **监视** | 正在运行 `ping -t`、`topic echo` 等持续命令 | 不可以；需要时用 `Ctrl+C` 结束 |
| **一次性** | 命令执行后自动回到提示符 | 可以复用同一个终端 |

看到命令一直不返回提示符并不代表卡死。`ros2 launch`、`ros2 run`、无 `--once` 的
`ros2 topic echo` 和 Windows 的 `ping -t` 都需要保持运行。不要在这些窗口继续粘贴下一条
命令；应按表中新开窗口。

### 4.2 Windows Terminal 窗口

测试电脑使用 Windows。打开 Windows Terminal/PowerShell，至少建立四个标签页：

| 编号 | 位置 | 类型 | 用途 | 是否一直开着 |
|---|---|---|---|---|
| `W0` | Windows PowerShell | 常驻 | SSH 到 Orin，并进入 tmux | 是 |
| `W1` | Windows PowerShell | 监视 | 持续 ping Orin `192.168.206.66` | 是，所有水上测试期间 |
| `W2` | Windows PowerShell | 监视 | 持续 ping 地面端 `192.168.206.200` | 是，所有通信测试期间 |
| `W3` | Windows PowerShell | 监视 | 持续 ping 移动端 `192.168.206.202` | 是，所有通信测试期间 |

`W1`、`W2`、`W3` 分别执行：

```powershell
ping -t 192.168.206.66
ping -t 192.168.206.200
ping -t 192.168.206.202
```

每条命令各占一个 PowerShell 标签页。使用 `Ctrl+C` 才会停止并显示丢包统计。若测试中
Orin 连续超时 3 次，遥控安全员立即接管；不得等待 SSH 自行恢复。

### 4.3 Orin 上的 tmux 窗口

在 `W0` 中登录：

```powershell
ssh robotx@192.168.206.66
```

登录后先创建记录目录，再创建 tmux。变量必须在启动 tmux 前设置，所有 tmux 窗口才能
继承同一个 `$TEST_LOG`：

```bash
TEST_ID=$(date +%Y%m%d_%H%M%S)
export TEST_LOG=/home/robotx/robotx_logs/task3_field_${TEST_ID}
mkdir -p "$TEST_LOG"
tmux new -s task3_field
```

使用 `Ctrl+b c` 新建窗口，`Ctrl+b ,` 改名，`Ctrl+b n` 切到下一个窗口，
`Ctrl+b d` 安全退出但不停止进程。SSH 断开后重新连接并执行：

```bash
tmux attach -t task3_field
```

按以下名称建立窗口；没有进入对应阶段时保持空闲：

| tmux 编号 | 名称 | 类型 | 用途 |
|---:|---|---|---|
| `T0` | `00_health` | 监视 | 节点、资源、最终控制输出健康状态 |
| `T1` | `01_mavros` | 常驻 | MAVROS/共享位置启动日志 |
| `T2` | `02_lidar` | 常驻 | Mid-360 驱动和 TF |
| `T3` | `03_semantic` | 常驻 | 公共语义地图 |
| `T4` | `04_task3` | 常驻 | Task3 executor/完整 dry-run |
| `T5` | `05_goto` | 常驻 | goto/控制器及状态 |
| `T6` | `06_gimbal_pump` | 常驻 | 窗口、云台和泵节点 |
| `T7` | `07_comm` | 常驻 | Task3 通信 adapter 和 heartbeat |
| `T8` | `08_ops` | 一次性 | 查询、构建、`--once` echo，可反复使用 |
| `T9` | `09_perception` | 常驻 | Task3 LiDAR 入口感知 |

每个新 tmux 窗口第一次使用时都执行：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
```

不要 source `~/livox_ws`、`~/mavros_ws`、历史 staging 或其他旧工作区。除 `T8` 外，启动
进程后不要在同一窗口输入下一阶段命令。

各阶段需要新增和保留的窗口：

| 进入阶段 | 新启动窗口 | 进入下一阶段时继续保留 |
|---|---|---|
| B 网络 | `W1`、`W2`、`W3`、`W0/tmux` | 四个都保留 |
| D 无控制检查 | `T0`；`T4` 启动 executor | `T0` 保留；`T4` 按正文决定停止或保留完整 dry-run |
| E LiDAR | `T2` | `T2` 保留到入口感知结束 |
| F 定位 | `T1` | 所有运动测试期间保留 |
| G 语义地图/Task3 | `T3`、`T4` | Task3 测试期间保留 |
| H goto | `T5` | 所有运动测试期间保留 |
| I 入口感知 | `T9` | 入口/靠泊测试期间保留 |
| J 云台/泵 | `T6` | 仅本阶段和可选靠泊阶段保留 |
| K 通信 | `T7` | 通信/整链测试期间保留 |

`T8` 始终用于查询，命令结束后可以复用。若某个常驻窗口的进程退出，先停止当前阶段并
记录错误，不要在该窗口直接改成运行另一个模块来掩盖问题。

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

### 6.1 Windows 测试电脑到三个设备

在 Windows PowerShell 一次性窗口执行 `ipconfig`，记录测试电脑连接现场网络的 IPv4，记为
`TEST_PC_IP`。它应与其他设备处于同一现场网段，且不能与 `.66`、`.200`、`.202` 冲突。

先做 30 包检查：

```powershell
ipconfig
ping -n 30 192.168.206.66
ping -n 30 192.168.206.200
ping -n 30 192.168.206.202
Test-NetConnection 192.168.206.66 -Port 22
```

随后按第 4 节启动 `W1`、`W2`、`W3` 三个持续 ping 标签页，再从 `W0` SSH 登录。

通过标准：

- 三个地址均回复，IP 没有冲突或周期性掉线；
- Orin 的 30 包检查必须 0 丢包，且不能连续出现明显高延迟；
- 地面端、移动端不能连续丢 3 包；
- Orin 的 TCP 22 端口成功，SSH 不会频繁断开。

### 6.2 Orin 反向检查测试电脑、地面端和移动端

在 Orin `T8` 执行，把第一条中的地址替换为刚才记录的 Windows IPv4：

```bash
ping -c 30 TEST_PC_IP
ping -c 30 192.168.206.200
ping -c 30 192.168.206.202
```

如果 Windows 能 ping Orin，但 Orin 不能 ping Windows，先检查 Windows 当前网络是否被识别
成“公用网络”和 Windows Defender 的 ICMPv4 Echo 入站规则。只放行现场网卡所需规则，
不要关闭整个防火墙。

若地面端或移动端有终端，再分别从它们 ping `192.168.206.66` 和 `TEST_PC_IP`，完成全双向
检查；若没有命令终端，至少确认 Windows、Orin 都能 ping 它们，并在第 15 节用 heartbeat/
ACK 验证应用层的双向通信。单向 ping 成功不能代替应用层 ACK。

### 6.3 Orin 版本和资源

在 Orin `T8` 执行：

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

推进器和泵继续断电。本节全部在 `T8` 执行，均为**一次性命令**；每条结束并重新出现提示符
后才能执行下一条，不需要新开窗口。

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

推进器和泵仍断电。在 `T8` 执行以下一次性检查，并确认所有终端使用同一
`ROS_DOMAIN_ID`：

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

在**新窗口 `T4`** 执行，启动后保持该窗口常驻：

```bash
ros2 run usv_task3_executor task3_executor --ros-args -p dry_run:=true
```

回到 `T8` 检查：

```bash
ros2 node list | grep usv_task3_executor
ros2 topic echo --once /tongji_mrl/twave/task3/state
```

通过标准：节点进入 `IDLE`，没有车辆控制或泵命令发布。该项完成后切回 `T4`，按
`Ctrl+C` 停止 executor，确认提示符已经返回，才能在 `T4` 启动下一条 launch。

公共语义地图 API 修复后，再运行完整 dry-run：

仍在空闲的 `T4` 执行并保持常驻：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  platform_id:=0 dry_run:=true
```

通过标准：executor 和 semantic adapter 都保持运行，无 traceback。

在 `T0` 启动资源和节点监视，之后保持该窗口开启：

```bash
watch -n 2 'date; uptime; free -h | head -n 2; echo ROS_NODES; ros2 node list | sort'
```

## 9. 阶段 E：LiDAR、TF 和点云检查

船仍在岸上支架。在**新窗口 `T2`** 启动 Mid-360；launch 运行期间 `T2` 必须保持常驻：

```bash
ros2 launch robotx_perception_common task3_lidar_bringup.launch.py \
  lidar_x:=0.15 lidar_y:=0.10 lidar_z:=0.22 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

回到 `T8` 执行一次性检查：

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

本阶段仍不发送 setpoint。若 MAVROS/共享位置尚未启动，由飞控负责人在**新窗口 `T1`**
启动，并保持 `T1` 常驻；不要把查询命令粘贴进正在运行 launch 的 `T1`。随后在 `T8` 做
只读检查：

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

在**新窗口 `T3`** 启动公共语义地图，保持 `T3` 常驻：

```bash
ros2 launch robotx_semantic_map semantic_map.launch.py
```

如果 `T4` 里已有第 8 节完整 dry-run，则不要重复启动。否则在**新窗口 `T4`**启动并保持
常驻：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py \
  platform_id:=0 dry_run:=true
```

在 `T8` 依次执行一次性观察；最后一条没有 `--once`，属于持续监视，需要看完后按
`Ctrl+C`，不要直接继续粘贴命令：

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
2. 由控制器负责人在**新窗口 `T5`**启动 goto/控制器并保持常驻；所有接口查询在 `T8`
   执行，不要输入到 `T5` 的运行日志中。
3. 读取当前 `/mavros/local_position/odom`，由控制器负责人确认 goto 使用的 frame、单位和目标格式。
   先执行 `ros2 topic info /safe_passage/goto/target --verbose` 和
   `ros2 topic info /safe_passage/goto/status --verbose`；类型或 topic 不一致时，以已部署控制器
   的源码和负责人确认结果为准。
4. 第一个目标只设为当前位置前方 `0.5 m`，最高速度使用控制器最低安全配置。
5. 发送目标前，观察员确认路径无障碍；安全员口头复述“可接管”。
6. 发送后记录目标、实际轨迹、最大误差、到达时间和 status。
7. 目标到达后立即回到人工/STOP，检查是否仍有非零控制输出。
8. 依次测试 `0.5 m` 直行、低速转向、`1.0 m` 直行；任一失败不增加距离。

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

## 13. 阶段 I：dock 入口感知在线验证

真实 LiDAR 数据已经采集过，本阶段不再录 rosbag。目标只是确认现场实时链路能产生可解释
的入口结果。

`T2` 中的 Mid-360 bringup 必须仍在运行。在**新窗口 `T9`**启动入口感知；该进程运行
期间 `T9` 必须常驻：

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  input_topic:=/livox/lidar input_format:=livox_custom
```

不要覆盖正在运行该 launch 的窗口。切回空闲的 `T8`，执行持续观察：

```bash
ros2 topic echo /tongji_mrl/twave/task3/dock_perception
```

看完后用 `Ctrl+C` 结束 echo；感知 launch 保持运行。上面的 `livox_custom` 对应当前 Orin
实测的 `livox_ros_driver2/msg/CustomMsg`。如果第 9 节现场检查显示输入已经改为
`sensor_msgs/msg/PointCloud2`，把参数改成 `input_format:=pointcloud2`；消息类型与参数不一致
时感知结果无效。

建议在船静止且安全的情况下验证三个视角：正对 dock、左前方、右前方。每个视角只记录
终端截图、船体/dock 照片和以下结果，不需要重新采点云：

- 是否输出 `valid`；
- entrance center 是否位于可见入口附近；
- heading 是否朝向 dock 内部；
- confidence 是否稳定，timestamp 是否持续更新；
- 看不到入口时是否正确 invalid/不发布，而不是猜入口。

入口方向与照片明显相反时，先查 TF、船体 heading、粗中心和输入格式，不要现场盲目调
算法阈值。没有输出时先检查粗中心与 NED pose 是否都在更新。

## 14. 阶段 J：云台、窗口感知和水泵

由云台负责人在**新窗口 `T6`**启动窗口/云台/泵节点并保持常驻。以下只读查询在 `T8`
执行，不要输入正在显示控制日志的 `T6`：

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

不启动车辆控制。在**新窗口 `T7`**启动 Task3 通信 adapter，并保持常驻：

```bash
ros2 launch usv_task3_executor task3_communication_adapter.launch.py run_id:=42
```

在 `T8` 检查 topic 和订阅者：

```bash
ros2 topic info /tongji_mrl/twave/task_cmd --verbose
ros2 topic info /tongji_mrl/twave/progress --verbose
ros2 topic info /tongji_mrl/twave/result --verbose
ros2 topic info /tongji_mrl/twave/heartbeat --verbose
ros2 topic info /tongji_mrl/tsky/heartbeat --verbose
```

验证内容：

- `W1`、`W2`、`W3` 没有连续超时；
- 地面端 `192.168.206.200` 和移动端 `192.168.206.202` 的实际应用均在线；
- `begin` 只转发一次，重复命令不会重复执行；
- `end` 转成 SAFE_STOP/ACK，不会被误报为 TASK3_FINISH；
- `tin_color`、`circle_color` 顺序和颜色编码与 UAV/OCS 一致；
- heartbeat 中断超过约 3 秒能检测，恢复后状态可解释；
- docking 与 firefighting 完成必须来自权威条件，不由单帧颜色代替。

无人订阅、没有 ACK，或只能 ping 通但收不到 heartbeat 时，只记录“网络可达、通信闭环未完成”，
不能宣布 Task3 完成。ping 只证明 IP 可达，不能证明 ROS/应用消息已经互通。

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
| PowerShell 提示找不到 `ssh` | Windows OpenSSH Client 未启用 | 在 Windows“可选功能”启用 OpenSSH Client，或使用已配置的 VS Code Remote SSH；不要临时关闭安全设置 |
| SSH 连接失败 | 不同网段、Wi-Fi/路由器、Orin 未上电 | ping `192.168.206.66`，检查本机 IP 和网线；不要反复重启整船 |
| Windows 能 ping Orin，Orin 不能 ping Windows | Windows 防火墙/公用网络规则 | 只放行现场网卡 ICMPv4 Echo；不要关闭整个防火墙 |
| 地面端或移动端连续丢包 | 现场 AP 覆盖、设备休眠或 IP 冲突 | 暂停通信/运动测试，靠近 AP、关闭省电并检查静态 IP |
| ping 正常但没有 ACK/heartbeat | 应用未启动、端口/DDS/domain 不一致 | 检查应用进程、ROS domain、topic/type；不能按通信成功处理 |
| `ros2` 找不到包 | source 了错误工作区 | 新终端只 source ROS Humble 和正式 `install/setup.bash` |
| duplicate package | 旧源码或备份仍在 `src` | 停止构建，把重复目录移到 `robotx_backups` 后复查 |
| Task3 build 报旧 launch 不存在 | 旧 build/install 清单残留 | 只备份该包的 build/install 产物，再单包构建 |
| `parse_snapshot` ImportError | 公共 semantic map 版本过旧 | 更新公共包、重新构建；不在 Task3 复制实现 |
| executor 启动后一直 IDLE | 没有有效粗中心/当前姿态/开始事件 | 检查 semantic map、NED、heading 和状态机命令 |
| LiDAR topic 不存在 | 驱动/供电/网卡配置 | 检查节点、电源、IP 和启动配置 |
| LiDAR 有数据但无入口 | TF、粗中心、heading、输入格式、ROI 或视角 | 保存终端截图和现场照片；核对坐标后再调参数 |
| TF y 仍为 0 | 使用了旧默认参数 | 用 `lidar_y:=0.10` 启动并更新公共感知配置 |
| NED 数据 stale/invalid | GPS、时钟或共享位置节点 | 停止自主测试，检查 GPS、系统时间和节点状态 |
| goto 无响应 | 控制器未启动、frame 错、无控制权 | 人工停止，检查 target/status 和仲裁 owner |
| 船突然转向或振荡 | ENU/NED、heading、增益或推进映射 | 立即人工接管，回到 0.5 m 低速测试 |
| `/cmd_vel` 多发布者 | 多控制器同时在线 | 停止所有自主节点，只保留唯一仲裁输出 |
| 泵无法关闭 | service/继电器/多发布者故障 | 物理断泵电源，结束测试并标记硬件故障 |
| 颜色序列无人接收 | 通信模块/订阅者/协议不一致 | 保存消息和 topic info，不宣称完成 |
| 节点崩溃 | 依赖、消息类型或版本不一致 | 保留 traceback、commit、topic list；回到上一安全阶段 |

## 18. 测试结束和数据交接

结束顺序：

1. 遥控切人工/安全模式，确认推进零输出；
2. 确认泵 OFF，并物理断泵电源；
3. 在 `T8` 保存各窗口最近输出：

   ```bash
   for w in 00_health 01_mavros 02_lidar 03_semantic 04_task3 05_goto \
            06_gimbal_pump 07_comm 09_perception; do
     tmux capture-pane -p -S -2000 -t "task3_field:$w" \
       > "$TEST_LOG/${w}.txt" 2>/dev/null || true
   done
   ```

4. 依次在 `T5` 停 goto/控制器、`T4` 停 Task3、`T7` 停通信、`T6` 停云台/泵逻辑、
   `T9` 停入口感知、`T3` 停语义地图；每个窗口使用一次 `Ctrl+C` 并等待进程正常退出；
5. 船体已经由人工控制稳定后，再在 `T2` 停 LiDAR、`T1` 停 MAVROS/共享位置、`T0` 停监视；
6. 保存 topic/service 列表、TF、版本、参数、终端截图、照片和视频；
7. 在 `W1`、`W2`、`W3` 分别按 `Ctrl+C`，截图保存三个地址的最终丢包统计；
8. 船上岸后再关闭飞控和 Orin；
9. 不删除失败日志，失败信息对定位接口问题最有价值。

现场结果至少记录：

| 项目 | 结果/数值 |
|---|---|
| 测试日期、地点、人员 |  |
| Windows 测试电脑 IPv4 |  |
| Orin `.66` 丢包/延迟 |  |
| 地面端 `.200` 丢包/延迟、ACK |  |
| 移动端 `.202` 丢包/延迟、heartbeat |  |
| Task3 commit |  |
| ROS_DOMAIN_ID |  |
| LiDAR topic/type/frequency |  |
| TF x/y/z、RPY |  |
| GPS/RTK 状态 |  |
| NED frame、heading 来源 |  |
| goto 0.5 m/1.0 m 误差 |  |
| 可见 dock/bay 和照片编号 |  |
| 泵开/关反馈和最大持续时间 |  |
| 通信 ACK 结果 |  |
| 未通过项、错误日志、处理人 |  |

最终结论只能选择：

- `PASS_FOR_NEXT_STAGE`：仅表示可以进入下一阶段；
- `PASS_FOR_STATIC_CHECKS_ONLY`：网络/接口/感知可检查，不允许自主控制；
- `FAIL_AND_HOLD`：保持人工/停止状态，修复后重测。

没有完整日志、照片和明确通过标准时，不填写“整链成功”。
