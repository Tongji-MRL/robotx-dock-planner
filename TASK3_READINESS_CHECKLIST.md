# RobotX 2026 Task 3 完成度清单

更新时间：2026-10-08

说明：`[x]` 表示已经实现或确认，`[ ]` 表示仍需完成。纯 Python 项目打勾，
只代表逻辑和单元测试完成，不代表 ROS、硬件或实船已经验证。

| 优先级 | 完成 | 项目 | 当前证据 | 剩余工作 / 所需输入 |
|---|---:|---|---|---|
| P0 | [x] | 任务全流程确认 | 粗定位、局部搜索、bay 检查、最终靠泊、喷水和颜色序列流程已整理 | Handbook 更新时同步检查 |
| P0 | [x] | 异常场景系统审计 | `TASK3_ABNORMAL_SCENARIOS.md` 已按任务前中后审计 158 项：52 已覆盖、33 部分覆盖、73 未覆盖 | 按矩阵继续补齐 62 个未覆盖 P0；相似项由统一 safety/watchdog/fault manager 合并解决 |
| P0 | [x] | DockPlanner 最终靠泊几何 | approach、align、entrance、stop 基线测试通过 | 仍需通过适配器交给实船控制器执行 |
| P0 | [x] | UAV 粗中心接口 | 已有 `CoarseDockCenterObservationNED` 和数据时效检查 | Orin ROS adapter 需要读取真实 semantic map 消息 |
| P0 | [x] | UAV-NED 转 USV-NED 纯转换 | 已按 PDF 的 WGS84/ECEF/NED 公式实现；semantic map 可输出现有 coarse dock 中心接口；已确认两套 NED 轴共向 | 当前只需固定平移，无需 yaw 旋转 |
| P0 | [ ] | UAV-NED 转 USV-NED 运行 adapter | 已有纯 Python 转换和 dry run | USV Orin 锁存 UAV 建图起点与 USV 任务起点，订阅状态机通信和 semantic map |
| P0 | [x] | 中心点到 staging/搜索圆弧 | 安全半径已包含 dock、船体包络、粗测余量、粗定位误差与 stand-off；可区分圆外、圆周、圆内、最小包络内和中心重合；圆内生成径向外退目标 | 最小包络内的真实运动必须由 Mid-360 确认局部净空后再交给 goto |
| P0 | [ ] | USV `base_link` 与船体外形 | 水平参考点为中央小盖板几何中心；图纸和粗测得到前 0.820 m、后 0.370 m、左右各 0.405 m；暂定二维包络半径 0.92 m，另留 0.10 m 粗测余量；详见 `USV_BASELINK说明.md` | 确认 Z 基准并统一飞控、GPS、LiDAR 外参；低速水测后再调整误差余量 |
| P0 | [x] | 不同场地可航边界判断 | 同时检查顺/逆时针；切除不安全圆弧；两边均失败时可在明确的最小 stand-off 以上逐级缩小半径；圆内向外到 staging 的路径也检查；随机场地测试通过 | 现场换成真实可航行边界，并根据船体尺寸确定最小 stand-off；接入 executor 前仍不可视为实船闭环 |
| P0 | [x] | 全路径连续边界检查 | approach、搜索圆弧 chord、terminal、退出和返航每段均做多边形求交与最小净距检查；自交边界被拒绝 | 动态障碍和 dock 实体仍需独立障碍层 |
| P1 | [x] | Bay 去重和失败记忆 | 入口中心距离关联；颜色拒绝或 terminal 失败后跳过；三个 bay 全部不可用时结束搜索 | 用真实入口数据标定关联距离，并接 executor 失败事件 |
| P1 | [x] | 入口与颜色置信度门 | flow 分别配置最低 entrance confidence 和 bay color confidence；低于门限保持观察，不进入靠泊 | 用真实数据标定两个门限 |
| P1 | [x] | 安全返航等待点选择 | 合法首选点保持不变；不安全首选点可在场地边界内按岸边净距和 dock 最小距离确定性修复 | 每个正式场地仍需人工给出首选等待区域 |
| P0 | [x] | ArduPilot 官方 ROS2 目标接口审计 | 官方 DDS 使用 `/ap/cmd_gps_pose`；局部位置反馈为 ENU | Task3 内部继续使用 NED，所有转换放在 adapter |
| P0 | [x] | 团队单点 goto 接口确认 | `/safe_passage/goto/target` 接收 `PointStamped`；status 输出 DONE/ABORTED | 源码存在，但尚无实船运行证据 |
| P0 | [x] | Task3 到 goto 的纯坐标转换 | 使用同步的当前 NED 和 odom 数据，无需共用原点即可将 NED 目标转为 ENU；支持状态解析 | 使用真实 topic 验证轴向和时间同步 |
| P0 | [ ] | Task3-to-goto ROS adapter | 纯转换和状态解析已完成 | 订阅当前 NED 与 odom，发布一个 `PointStamped`，消费 status |
| P0 | [x] | goto/terminal 纯控制权仲裁 | 明确 GOTO、TERMINAL、STOP；缺失、过期、错误 frame 或超限命令均回到零速度 | 在 Orin 增加 ROS topic 重映射和唯一输出 publisher |
| P0 | [x] | 公共避障模块接入契约 | `PROCEED/HOLD/REPLAN/ABORT` 带有效期；非 PROCEED、过期或严格模式下缺失均强制零速度 | Task1 公共模块确定 ROS 接口后只编写 adapter，并把 REPLAN/ABORT 接入任务监督器 |
| P0 | [ ] | ROS 速度命令仲裁节点 | goto 和 terminal 当前都面向 `/usv/control/cmd_vel` | 分别重映射输入，只允许仲裁节点发布最终速度 |
| P0 | [x] | MAVROS 工作区定位 | `/home/robotx/mavros_ws/install` 中已有节点和 local-position/setpoint 插件 | source 后做台架验证；当前终端环境尚未加载该 overlay |
| P0 | [x] | Mid-360 驱动 | `/livox/lidar` 为 PointCloud2，`livox_frame`，约 10 Hz、单帧约 19968 点；`/livox/imu` 实时约 200 Hz | 保留启动命令和配置，后续做开机启动评审 |
| P0 | [x] | PointCloud2 纯解析核心 | 已按 Mid-360 实际 26-byte 混合字段布局实现 x/y/z 提取、NaN/Inf 过滤、全零占位点过滤和质量统计 | 使用 dock 现场 rosbag 验证入口识别结果 |
| P0 | [ ] | PointCloud2 ROS adapter | 纯解析核心已直接读取真实 rosbag 的 rclpy PointCloud2 消息；离线抽帧工具已验证 | 在 Orin 订阅 `/livox/lidar`，接入离线检测器并保持 `livox_frame` 边界 |
| P0 | [ ] | Mid-360 安装外参/TF | 实测没有 `livox_frame → base_link` TF | 测量真实安装平移与旋转，建立 URDF 或 static TF，不使用配置文件中的零值冒充 |
| P1 | [x] | Mid-360 测试 rosbag | 已复制到本地并直接读取；12.664 s、62.8 MiB、126 帧点云 | 继续采集包含真实 dock 和多角度观察的 bag |
| P1 | [x] | 真实点云质量统计 | 单帧 19872 点，XYZ 有限比例 100%；754 个全零占位点（约 3.8%）已识别，最大距离 8.840 m | 现场 dock 数据继续记录相同统计，注意原始数据保留全零点 |
| P1 | [x] | 离线 dock 入口几何识别 | synthetic 点云与仿真多视角检测已经完成 | 用录制的 Mid-360 数据调参，不伪造 TF |
| P1 | [ ] | 真实入口观测 | 已有 `DockEntranceObservationNED` 接口 | 确认 LiDAR frame 转换并验证入口中心和方向 |
| P1 | [x] | Dock 对齐接口确认 | `/robotx/perception/docking/alignment`，类型 `DockingStatus`；staging 已验证米/弧度/距离字段解析 | 正式合入前仍需使用现场消息做一次回放验证 |
| P1 | [x] | 安全 bay 接口确认 | `/robotx/perception/docking/safe_bay` 实际为 `std_msgs/String`；另有独立 `safe_bay_indicator` 为 `LEDObservation`；staging 已分别核对 | 现场确认最终启用的 topic，不把两个 topic 混用 |
| P1 | [x] | Bay 颜色有效性规则 | `UNKNOWN`、`NONE`、invalid、过期和 bay mismatch 均不会触发靠泊；staging 已验证 RED/GREEN | 使用现场感知节点做一次回放验证 |
| P1 | [x] | 感知接口 staging adapter | A–I mock 覆盖颜色、对齐、入口 JSON、stale、bay mismatch、resources；13 个单测和隔离 ROS 测试通过 | 尚未合入正式 executor；entrance 仍是 camera optical frame，不能直接生成 terminal pose |
| P1 | [x] | 喷水感知接口确认 | `/robotx/perception/sprayer/active_window_3d` 为 `LEDObservation`；桥接输出 `/vision/target_observation` 为 `TargetObservation`；Orin mock RED/GREEN 已验证 | 现场再确认感知实际频率和丢帧情况 |
| P1 | [x] | 喷水完成判定 | Orin 隔离 domain 已验证首次 GREEN 停泵、连续 GREEN 5 秒完成、中断重新计时、stale/fault 不完成 | 合入正式 executor 后重复 dry run |
| P1 | [ ] | 水泵控制 adapter | staging ROS adapter 已通过 15 个单测和 4 个 mock 场景，只发送 `SetPump(on=false)`；MockPumpBackend 状态经过 TRACKING/LOCKED/SPRAYING/off | 尚未合入正式 Task3；硬件负责人确认后再做 Relay4 无水台架测试 |
| P1 | [x] | 资源颜色序列解析 | 已解析锡罐/resource 颜色和圆圈/destination 颜色；staging 已验证有效、UNKNOWN、缺字段和 `CORE_COMPLETE` 变体 | 在 Orin 订阅正式 topic，交给 USV 状态机 |
| P1 | [x] | 颜色序列接口契约 | staging 已固定 `/tongji_mrl/twave/task3/color_sequence` + `std_msgs/msg/Int32MultiArray`；`data=[tin_color,circle_color]`，6 个颜色编码和 UNKNOWN 规则已测试 | 交给 communication 模块和 UAV/OCS 负责人确认并接收 |
| P1 | [ ] | 向 UAV/OCS 发布颜色序列 | 已有接口契约，但尚未接入正式 communication 模块 | 确认对端订阅同一 topic/type 后再做 ROS publisher |
| P1 | [x] | 重试次数和时间监督 | 最大次数、单次超时和总超时均可配置 | 现场测试后调整参数 |
| P1 | [x] | 退出 dock 与返回几何 | 可生成 dock 外退出点和岸边等待目标点 | 每个场地配置经过确认的安全等待点 |
| P1 | [ ] | 异常恢复接入 executor | 纯 Python 已能输出重试、退出和返回决策 | 接入 executor 状态与 goto DONE/ABORTED 反馈 |
| P2 | [ ] | 复用 Task 1 避障 | 当前版本明确未实现 | 等 Task 1 源码/API 确认后再审计和接入 |
| P0 | [ ] | Orin 代码同步 | 之前只迁移过部分纯接口文件 | 评审后迁移最终文件，避免覆盖其他人的代码 |
| P0 | [ ] | Orin 无控制 dry run | 本地纯 Python 测试通过 | 使用 mock 或 rosbag 通过 ROS adapter 回放，不发送运动命令 |
| P0 | [ ] | 分阶段水上测试 | 尚未执行 | 按安全门依次测试 approach/search、靠泊、水泵和返航 |
| P0 | [ ] | 比赛级整链运行 | 尚未实现 | 完成全部未勾选 P0 项，并保存一次可重复运行记录 |

## 当前本地验证

- 完整 pytest：`175 passed`。
- 独立 LiDAR prototype：`18 tests passed`；200 组随机 synthetic 场景严格成功率 98.5%，
  32 个仿真视角中 5 个可见入口视角全部选对最高候选，覆盖 3 个 bay。
- 真实 Mid-360 bag 抽帧：已验证 `livox_frame`、混合字段解析和全零点过滤。
- `examples.task3_full_dry_run`：粗中心到喷水及颜色序列，运动到达事件使用 mock。
- `examples.task3_safety_recovery_dry_run`：随机边界、安全圆弧、三次尝试、退出 dock 和岸边等待点。
- `examples.task3_control_handoff_dry_run`：STOP、GOTO、TERMINAL 三种速度控制权切换。

## 必须保持可配置的参数

- Dock 搜索半径和岸边安全距离。
- 搜索方向、圆弧长度和 waypoint 间距。
- 最大靠泊尝试次数、单次超时和总超时。
- GREEN 确认时间和感知数据超时。
- 退出 dock 距离和当前场地的岸边等待点。
- goto 与 terminal 的最大前向速度和最大偏航角速度。
