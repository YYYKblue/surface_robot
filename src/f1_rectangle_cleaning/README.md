# f1_rectangle_cleaning：第一版单边沿墙清洗测试

2026-10-23 策略更新：人工将船放在长边 A 附近、左侧为墙且尽量平行；只刷这一边，不换边、不掉头。基于已有矩形地图定位，在船体前缘到前方短墙剩余 **0.80 m** 时关闭输出。雷达保持原安装（船头盲区），第五路 ROS **正号向左**，均按用户确认实现。

| 项目 | 第一版行为 |
|---|---|
| 前进 | 夹角在 ±3° 内时刷盘 15%，主电机 `direction=STOP`；不发 FORWARD/REVERSE，不调速度 |
| 靠墙 | 第五路 +15%；中心到左墙 `d>0.45 m` 开、`d<0.38 m` 关；边界等号和中间区间保持原状态，启动默认关闭；名义目标 0.40 m |
| 航向 | `|ψ|>3°` 才发 25% LEFT/RIGHT，最长 0.1 s，然后主电机中位至少 0.5 s；误差进入 ±3° 或变号时提前结束脉冲 |
| 刷盘门控 | `|ψ|≤3°` 才开 15%；纠偏期间及误差仍超限的中位等待期间关刷；夹角恢复后允许开刷 |
| 停车 | 地图前缘净空 ≤0.80 m，或有效前方观测净空 ≤0.80 m 时，关闭刷盘、第五路和主电机；不反向制动 |
| 转动空间 | 保留雷达小角度扫掠检查；所需方向空间未知或受阻即暂停，人工摆正、cancel 后再 start |

`ψ` 是 `WallObservation.tangent_rad`：左墙切线相对船头的角度；正误差发 LEFT，负误差发 RIGHT。第五路滞回与夹角门控独立；中位等待只约束主电机。停机、异常、取消时全部输出关闭。

## 地图与距离定义

默认矩形来自 `backups/nano_maps/20261021_145054/lidar_20261021_145054.pbstream` 对应地图，长约 2.883 m、宽约 1.520 m、长轴约 -11.31°。这是墙带估计，并非现场测量值。A 为池坐标 +y 侧长墙，船头沿池坐标 +x。

- 左墙距离是 **base_link 中心到墙的垂直距离**，不是刷尖到墙距离。
- 前方距离是固定地图短墙到船体前缘的保守投影：`length - pool_x - (0.28*|cos(yaw_error)| + 0.28*|sin(yaw_error)|)`，单位 m。
- 默认启动读取原时间戳的 `map → base_link` TF；也可设 `use_map_tf:=false`，接入 `/rectangle_cleaning/map_pose`（`PoseWithCovarianceStamped`）。不再依赖未接通的局部速度输入。
- TF 不提供协方差，状态明确给出 `pose_covariance_available=false`；不会填造协方差或零速度。使用 topic 输入时仍检查 x/y/yaw 协方差。
- 检查墙线质量、定位新鲜度、墙/位姿时戳偏差、定位跳变、地图与左墙距离/夹角一致性。定位错误或反馈中断会关闭输出并锁定暂停，不能自动续跑。

**地图停车不能发现船头盲区中的新增障碍。** `front_observed=false` 不再单独阻止本版地图端点任务，但绝不改写为 true；任何实际观测到的刚性碰撞或有效前方障碍仍触发停止。地图本身和起始墙身份需在无输出预览中核对。

`STOPPED` 仅表示本次单边任务已关闭输出。它不证明船体停稳，惯性滑行未测；80 cm 是触发阈值，不是保证最终停在 80 cm。当前刷尖几何为中心左侧 0.33 m，0.40 m 中心墙距通常仍有间隙，本轮检验控制行为和实际接触效果，不宣称清洗覆盖率。

## 构建与测试

```bash
cd /home/bboss/new-nano/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_rectangle_cleaning
source install/local_setup.bash
ros2 run f1_rectangle_cleaning rectangle_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose
```

演示为合成反馈，不是动力学仿真。测试包括纯逻辑、localhost DDS 预览、TF 输入与模拟 bridge 租约，不连接串口或实机。

## 启动与接口

先按 [Nano 测试步骤](DEPLOY_TEST_NANO.md)加载冻结地图定位、核对当前 A 墙及剩余距离，然后启动：

```bash
# 默认预览；不申请租约、不创建执行器发布者。
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py
ros2 topic echo /rectangle_cleaning/status
ros2 topic echo /rectangle_cleaning/command_preview
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/cancel std_srvs/srv/Trigger '{}'
```

状态序列：`IDLE → FOLLOW ↔ CORRECT_HEADING / NEUTRAL_DWELL → STOPPED`；异常为 `PAUSED`。无第二边确认服务。暂停后须 `cancel`、人工重新摆正并确认定位，再 `start`。

实机模式在启动时显式传 `output_enabled:=true`，还需 bridge 的串口/运动许可、主电机方向确认和 `allow_fifth_thruster=true`。本包通过唯一租约发布组合 `ActuatorCommand`；遥控接管、bridge 重启/epoch 变化、命令过期、反馈中断和退出均撤销输出。`use_sim_time=true` 或第五路极性未知时拒绝实机模式。默认仍不开硬件；本次修改未连接或部署 Nano。

`command_preview` token 始终为空，不能 remap 到执行器话题。状态包含实际输出模式、租约、刷盘请求、侧推请求、地图前缘净空、前方观测状态及定位来源。`stopped_verified` 和 `coverage_available` 始终为 false。

## 文件

| 文件 | 用途 |
|---|---|
| `f1_rectangle_cleaning/mission.py` | 单边状态机、滞回、刷盘门控、纠偏脉冲、地图停车 |
| `f1_rectangle_cleaning/config.py` / `config/rectangle_preview.yaml` | 策略阈值、矩形几何和参数检查 |
| `f1_rectangle_cleaning/controller_node.py` | TF/topic 位姿、墙反馈、预览和服务 |
| `f1_rectangle_cleaning/lease.py` | 可选 bridge 租约输出与接管/失联检查 |
| `config/perception.yaml` | 原雷达外参、墙线和小角度扫掠感知 |
| `launch/rectangle_localization.launch.py` / `config/rectangle_localization.lua` | 加载冻结 pbstream 的纯雷达定位入口 |

历史两边任务设计保留于工作空间 `docs/long_wall_plan/DESIGN.md`，不是本版运行流程。
