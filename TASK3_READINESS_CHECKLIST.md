# RobotX Task3 出发前检查清单

更新时间：2026-10-31

本清单只判断“出发前能否安全带到比赛现场”。没有水上场地时，不把真实靠泊、喷水
和整流程运行写成已完成；这些项目统一列为到场后验证。

现场执行步骤、停止条件、命令和故障处理见 `TASK3_FIELD_TEST_RUNBOOK.md`。

## 一、出发前必须完成

| 完成 | 检查项 | 验收证据 | 当前状态 |
|---:|---|---|---|
| [x] | Task3 源码版本固定 | Orin 的 `/home/robotx/robotx_ws/src/task3` 为发布提交 `a46a96c` | 已完成 |
| [x] | 旧版本可回退 | `/home/robotx/robotx_backups/task3_before_update_20261031_165157` 存在 | 已完成 |
| [x] | Task3 不包含重复感知包 | `src/task3` 中没有 `robotx_perception_common` | 已完成 |
| [x] | 纯 Python 核心测试 | 181 项 unittest 全部通过 | 已完成 |
| [x] | 规划输入校验 | 非法坐标、过期观测、无效入口、错误 bay 不生成危险目标 | 已完成 |
| [x] | 安全圆周规划 | 圆外、圆周、圆内、最小包络内、中心重合均有明确结果 | 已完成 |
| [x] | 可航边界检查 | 入口、搜索弧、退出和返航路径支持边界拒绝 | 已完成 |
| [x] | 重试与退出策略 | 感知失败、靠泊失败、喷水失败可重试，耗尽后退出 | 已完成 |
| [x] | 颜色序列契约 | `tin_color`、`circle_color`、UNKNOWN 规则固定 | 已完成 |
| [x] | 喷水完成逻辑 | 红窗触发、绿窗停泵、连续绿色 5 秒完成 | 纯逻辑已完成 |
| [x] | Orin 只构建 Task3 | `usv_task3_executor` 构建成功，4 个 executable 均可发现；旧 build 产物已备份 | 已完成 |
| [ ] | Orin Task3 dry-run | `task3_executor` 能进入 IDLE，但完整 launch 被公共 `robotx_semantic_map` 缺少 `parse_snapshot` 阻塞 | 待修复公共包版本后重测 |
| [x] | Task3 启动入口检查 | 包 executable 已发现；尚未启动节点 | 静态检查完成 |
| [ ] | 公共接口只读核对 | semantic map、NED position、goto status、dock perception topic/type 正确 | 待执行 |
| [ ] | LiDAR 输入健康检查 | `/livox/lidar`、`livox_frame`、频率、字段和 TF 可读 | 待执行 |
| [ ] | 外参值一致 | `base_link -> livox_frame = (+0.15,+0.10,+0.22) m` 在感知包和文档中一致 | 待核对 |
| [ ] | 感知包单独构建 | `src/perception/robotx_perception_common` 构建成功，不与 Task3 重复 | 待执行 |
| [ ] | 泵/云台 dry-run | 只验证 service/topic 类型和停泵默认值，不接通水泵 | 待执行 |
| [ ] | 通信 dry-run | 颜色序列和 Task3 状态能交给总状态机的 mock 接收端 | 待执行 |
| [ ] | 安全门 | 泵默认关闭、控制权仲裁唯一、异常进入 HOLD/STOP | 待执行 |

当前 dry-run 阻塞记录：Task3 executor 节点本身已启动；`task3_semantic_map_adapter` 在
Orin 上导入 `robotx_semantic_map.parse_snapshot` 失败。该符号属于公共语义地图包，不能
在 Task3 内复制实现；需要统一工作区更新到包含该接口的版本后再重跑 launch。

## 二、出发前有时间再做

| 完成 | 检查项 | 说明 |
|---:|---|---|
| [ ] | 真实 rosbag 回放 | 用已录制 Mid-360 bag 检查点云解析、TF 和入口候选，不控制船 |
| [ ] | 真实接口消息回放 | 回放对齐、safe bay、active window 和颜色序列消息 |
| [ ] | 多次启动/停止 | 检查 launch 重启后没有重复 publisher、残留泵开启或状态卡死 |
| [ ] | 现场参数快照 | 保存 topic、frame、外参、版本和 launch 参数到日志目录 |

## 三、没有场地时明确延期

以下项目不在出发前强行宣称完成：

- 真实 USV 沿安全圆周行驶和动态避障；
- 真实 dock 入口中心、入口朝向和三个 bay 的识别成功率；
- goto/setpoint 的水上到达误差和控制权切换；
- 真实靠泊、喷水、绿窗持续 5 秒和退出返航；
- 复杂水面、通信中断、定位漂移和现场边界的实测参数标定；
- 比赛级完整整链记录。

## 四、出发前停止条件

出现下列任一情况，不进入真实控制或喷水：

1. Task3 或感知包无法从正式工作区构建；
2. `/livox/lidar`、`livox_frame` 或必要定位消息不存在/过期；
3. `base_link -> livox_frame` 外参不一致或 TF 方向未确认；
4. goto、控制权仲裁或通信接口出现多个不一致版本；
5. 泵默认状态不是关闭，或 dry-run 无法证明异常会停泵；
6. Task3 dry-run 不能在无控制发布的情况下完成状态转换。

## 五、推荐执行顺序

```text
版本/备份确认
  -> Task3 单包构建
  -> 感知包单包构建
  -> import/launch 检查
  -> LiDAR/TF/定位只读检查
  -> Task3 mock dry-run
  -> 接口和泵 dry-run
  -> 保存版本与参数快照
```

本清单的 `[x]` 只表示已有代码或离线证据；任何涉及真实运动、泵和水上环境的项目，
仍须在现场按安全流程单独放行。
