# RobotX USV Task3

本仓库保存部署到 USV Orin 的 Task3 规划、子状态机与 ROS 2 适配代码。仿真生成物、
离线 LiDAR 研究目录和个人 VRX 环境脚本不进入主分支；需要追溯时使用
`archive/task3-research-20261031`。

## 运行链路

```text
UAV 语义地图观测
  -> robotx_semantic_map 合并、确认并发布 /task/semantic_map
  -> task3_semantic_map_adapter 筛选 platform
  -> /tongji_mrl/twave/task3/coarse_dock
  -> Task3FlowManager 生成安全圆周接近与搜索目标
  -> Mid-360 感知输出入口中心/朝向
  -> DockPlanner 生成 terminal docking 路径
  -> 正确泊位识别、喷水验证、颜色序列、退出与返航
  -> USV 总状态机向 UAV/OCS 报告
```

Task3 只发布内部任务数据。对 UAV/OCS 的通信、ACK、重试和当前任务门控由
`usv_state_machine` 与 `robotx_communication` 负责。

## 目录

| 路径 | 内容 |
|---|---|
| `dock_planner/` | 纯 Python 规划、流程、安全边界、重试与接口契约 |
| `usv_task3_executor/` | ROS 2 executor、语义地图、感知和通信适配节点 |
| `ros2/usv_task3_executor/` | ament Python executor、launch、包级测试 |
| `tests/` | 不依赖实船的核心单元测试 |
| `TASK3_ROS_INTERFACE.md` | ROS topic 与消息约定 |
| `TASK3_COMMUNICATION_PROTOCOL.md` | 子状态机与总状态机通信协议 |
| `TASK3_ABNORMAL_SCENARIOS.md` | 异常场景、处理策略与完成情况 |
| `TASK3_READINESS_CHECKLIST.md` | 上船前检查清单 |
| `USV_BASELINK说明.md` | `base_link`、船体包络与待测外参 |

## 与 USV Orin 统一工作区的对应关系

`robotx_workspace/repos/usv_real.repos` 将本仓库固定检出到：

```text
/home/robotx/robotx_ws/src/task3/robotx-dock-planner
```

因此不要把本仓库手工复制成第二份，也不要把其中的包再复制到 `src/perception/`。
本仓库内部的 ROS 包保持如下结构，`colcon` 会递归发现它们：

```text
robotx-dock-planner/
├── dock_planner/                         # 纯 Python Task3 规划与契约
├── usv_task3_executor/                   # 规划层 Python 适配实现
├── ros2/usv_task3_executor/              # ROS 2 executor 包与 launch
```

统一工作区的公共感知、平台 I/O、通信和其他任务仍由各自 manifest 仓库提供；本仓库只
提供 Task3 所需的规划和 executor。Mid-360 公共感知包由统一工作区的
`src/perception/robotx_perception_common` 提供，不能在 Task3 下再放一份。

## Orin 构建

本仓库通常由 `robotx_workspace` 的 manifest 放入：

```text
/home/robotx/robotx_ws/src/task3/robotx-dock-planner
```

在统一工作区执行：

```bash
cd /home/robotx/robotx_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to usv_task3_executor
source install/setup.bash
```

## 启动

共享语义地图节点应先由统一 bringup 启动。Task3 标准入口：

```bash
ros2 launch usv_task3_executor task3_executor.launch.py platform_id:=0 dry_run:=true
```

`dry_run:=false` 只关闭 mock 输入许可，不会自行接管 PX4。导航目标仍需经过工作区中的
运动控制权仲裁和车辆控制适配器。

Mid-360 感知包不在本仓库重复启动；由统一工作区的
`robotx_perception_common` 包负责驱动、静态 TF 和入口感知桥接。当前测得的
`base_link -> livox_frame = (+0.15, +0.10, +0.22) m`，安装角暂按零度处理；移动传感器
或重新安装后，更新统一感知包和 `USV_BASELINK说明.md`。

## 主要输入与输出

| 方向 | Topic | 说明 |
|---|---|---|
| 输入 | `/task/semantic_map` | retained、带 `map_epoch/version` 的公共语义地图 |
| 输入 | `/tongji_mrl/twave/ned_position` | USV 公共 NED 位置；启动圆周搜索还需可靠 heading |
| 输入 | `/tongji_mrl/twave/task3/dock_perception` | 入口中心、入口朝向、置信度和有效期 |
| 输入 | `/tongji_mrl/twave/task3/approach_reached` | 控制器确认到达接近点 |
| 输出 | `/tongji_mrl/twave/task3/platforms` | 给 USV 总状态机的内部平台列表 |
| 输出 | `/tongji_mrl/twave/task3/coarse_dock` | 所选平台的真实 NED 粗中心 |
| 输出 | `/tongji_mrl/twave/task3/state` | Task3 当前状态和诊断信息 |

完整字段见 [TASK3_ROS_INTERFACE.md](TASK3_ROS_INTERFACE.md)。

## 当前边界

- 语义地图只提供 dock/platform 粗中心，不生成假入口、假角点或假 heading。
- Mid-360 驱动由 `livox_ros_driver2` 提供；统一工作区的
  `src/perception/robotx_perception_common` 负责 `CustomMsg`/PointCloud2 转换、
  `livox_frame -> base_link` 和 Task3 感知桥接。
- 飞控 setpoint、避障输出和控制权仲裁属于统一工作区公共包。
- 喷水完成由窗口从红变绿并持续稳定判定；泵本身没有“水已喷入”传感器。
- 真实运行前仍需确认定位消息中的 heading、可航行边界、控制器到达反馈和实测外参。

## 测试

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ros2/usv_task3_executor/test -p 'test_*.py'
```

核心测试不启动 PX4、泵、云台或实船控制。
