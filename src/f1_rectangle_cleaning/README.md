# f1_rectangle_cleaning：矩形水池两条长边任务

独立管理矩形水池任务；复用 `f1_wall_cleaning` 的雷达墙线感知与消息转换，不启动旧单墙控制器。
版本 0.1 是**可运行的无硬件输出开发入口**，尚不能直接用于实机自动清洗。

2026-10-21 用户选择后续采用纯雷达反馈，不引入 IMU 航向控制。本包当前没有 IMU 订阅；方向与速度反馈适配按纯雷达方案继续开发。
Nano 部署、真实墙距/夹角检查、录包和后续开发顺序见 [同步与纯雷达测试步骤](DEPLOY_TEST_NANO.md)。

2026-10-22 实景反馈确认：约 280° 雷达的盲区位于船头，当前前方观测条件不满足。左墙感知可继续验证；前方未知时任务保持拒绝运动。需要先评估实际安装调整或端部距离间接估计方案，不能仅更改坐标变换或绕过前方检查。大角度转向还需独立处理盲区。

## 已实现与边界

- 状态机：`IDLE → APPROACH_A → FOLLOW_A → STOP_A → WAIT_MANUAL_TRANSFER`；人工摆到 B 并确认后执行 `APPROACH_B → FOLLOW_B → STOP_B → FINISHED_SEGMENTS`。
- 地图坐标转换、左墙身份/航向核对、实时墙距侧推需求、前进脉冲、前方短边/地图端点停车。航向超限暂停，尚不自动纠偏。
- 第五路逻辑输出带限幅/增长速率限制，偏近时立即取消推向墙面的输出。主轮和第五路生成同一条 bridge 消息格式的预览；刷盘始终关闭。
- STOP 后依赖连续、不同时间戳的局部速度反馈判定停稳，不能用“已发 STOP”代替。提前发现前方障碍或地图/短墙冲突时暂停，不记作完成长边。
- 新鲜度、时戳偏差、墙线质量、几何边界、定位跳变、感知重启、阶段超时、接收中断检查；异常暂停后不自动恢复。暂停后 `cancel`，重新摆到 A 再 `start`。
- **未实现**实机租约/输出适配、固定地图定位及局部速度估计器、主动制动、自动侧移/180° 掉头、刷盘接触清洗。`output_enabled:=true` 会明确报错。

任务命名为清洗，但本版验证的是**不开刷、不接触墙面的两段运动决策**。完成状态只表示计划段处理结束，`coverage_available=false`，不代表全池或实际清洁覆盖率。

## 文件分工

| 文件 | 用途 |
|---|---|
| `f1_rectangle_cleaning/mission.py` | 不依赖 ROS 的状态机及组合指令字段 |
| `f1_rectangle_cleaning/config.py` | 独立任务参数、矩形坐标变换与验证 |
| `f1_rectangle_cleaning/controller_node.py` | ROS 输入校验、服务、预览和接收监测 |
| `f1_rectangle_cleaning/demo.py` | 合成反馈演示，不是船体动力学仿真 |
| `config/rectangle_preview.yaml` | 当前地图近似矩形与无接触任务配置 |
| `config/perception.yaml` | 本任务独立的雷达感知配置 |
| `launch/rectangle_cleaning.launch.py` | 只启动本任务和一份墙线感知 |

## 当前地图与机器参数

源地图为 `backups/nano_maps/20261021_145054/lidar_20261021_145054.pgm`，
SHA256：`1d29a0090f852fba211221162a5a080f37aa20989995ef8e598233a123d19f3a`。
矩形长轴约 -11.31°，墙带中心间距约 2.88 × 1.52 m。长轴作为池坐标 +x，+y 指向长边 A。
矩形原点为估计的 x 最小、y 最小墙带交点。地图只提供任务几何，不提供厘米级压缩判断。

当前配置使用中心到墙 **0.45 m**、中心到短边 **0.80 m** 提前结束、船头净空 **0.50 m** 停止阈值。
它们是预览用暂定值，不是实测制动距离。前后包络各 0.28 m，保守横向包络各 0.28 m；刷尖 y=0.33 m。
接触目标 0.32 m 留在后续刷洗阶段，本版配置不接受接触目标。

第五推进器已知安装于中心，用户描述“反向将船推向左墙”。ROS 指令的正负号和离墙能力尚未确认，故 `fifth_left_sign=0`。
此时状态中的 `logical_side_left_percent` 显示向左侧推需求，预览指令第五路保持关闭。
只有已核实 ROS 指令正负号后才可设置 +1/-1；**设置正负号也不会开启本版硬件输出**。

## 构建和离线运行

```bash
cd /home/bboss/new-nano/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_rectangle_cleaning
source install/local_setup.bash
ros2 run f1_rectangle_cleaning rectangle_demo
```

演示直接运行纯 Python 状态机，无 ROS 发布者、串口或租约，自动注入一次“人工换边确认”，打印两个段的状态转换。
反馈位置按脚本变化，不能据此判断真实运动、停车或清洁效果。

```bash
colcon test --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --verbose
```

## ROS 预览接口

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py
ros2 topic echo /rectangle_cleaning/status
ros2 topic echo /rectangle_cleaning/command_preview
```

输入必须由已验证的定位/运动估计链路提供；本包不会用零速度或单位协方差伪造定位成功。
目前建图输出尚未适配为这两个任务输入，不能直接满足全部输入合同。后续用雷达定位和连续局部运动估计提供，不要求 IMU。

| 输入话题 | 类型/合同 |
|---|---|
| `/pavo2_scan` | `LaserScan`，由独立感知节点转换成 `/rectangle_cleaning/wall_observation`；保持既有雷达外参 |
| `/rectangle_cleaning/map_pose` | `PoseWithCovarianceStamped`；map 下的 **base_link** 位姿，归一化四元数，有效 x/y/yaw 协方差 |
| `/rectangle_cleaning/local_motion` | `Odometry`；header=odom、child=base_link，twist 为 base_link 中的局部 vx/vy/wz，有效速度协方差；不能填充默认零值 |

map 位姿协方差 x/y 上限 0.01 m²、yaw 上限 0.03 rad²；速度相应上限为 0.01，均要求正值且有限。
这些是接口拒绝阈值，不代表实际估计精度已达到。墙线最大年龄 0.15 s，位姿/速度 0.25 s，三者时戳差不超过 0.10 s。
话题名可通过 `scan_topic`、`map_pose_topic`、`local_motion_topic` launch 参数调整。

```bash
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/cancel std_srvs/srv/Trigger '{}'
# 仅 WAIT_MANUAL_TRANSFER 后，在 B 起点静止、航向反转、左侧确为 B 墙时：
ros2 service call /rectangle_cleaning/confirm_second_wall std_srvs/srv/Trigger '{}'
```

`command_preview` 使用 `f1_interfaces/ActuatorCommand`，token 始终为空，绝不发布到 `/f1/actuator_command`，也不申请 bridge 租约。
不要通过 remap 把预览接到执行器。状态为 JSON，包含 phase/reason、反馈错误、逻辑侧推、完成段及硬件关闭标记。
主轮脉冲使用固定 `direction_until`；脉冲结束后如仍需侧推，必须用新鲜反馈生成新的主轮 STOP + 第五路组合指令。

## 下一步开发顺序

1. 接入保存的 pbstream 固定地图定位与连续局部速度估计，核对 A/B 身份、时间戳和不确定度；先跑无输出实景/回放。
2. 记录第五路 ROS 正负号、向左/向右效果，以及主轮前进 + 侧推组合的实际偏航和停车数据。
3. 在本包增加硬件输出适配层，沿用 `f1_nano_bridge` 的唯一租约、时间有效期、RC 接管和失联停止合同；先验证 A 单段，不开刷。不要并行启动旧 `wall_controller`。
4. 增加 `MOVE_INWARD / TURN_180 / APPROACH_B` 自动换边实现，必须有完整扫掠空间与连续运动反馈；不能把旧感知的小角度转向标志当作掉头许可。
5. 最后接入压缩窗口、刷盘负载下的控制和清洁效果验收。

设计背景与地图示意见工作空间 `docs/long_wall_plan/DESIGN.md`；自动换边设计仍在该文档中，本包当前明确停在人工换边检查点。
