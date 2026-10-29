# surface_ws：水面清洗机器人 ROS 2 工作空间

更新：2026-10-29，依据当前源码、启动文件和默认配置整理。目标环境为 Ubuntu 22.04 / ROS 2 Humble，机器人端运行于 ARM64 Nano，操作员端为无需安装 ROS 的 Qt App。

项目包含 PAVO2 纯雷达建图、固定地图定位、F103 串口控制、矩形四边清洗、大 L 两段墙清洗和 WebSocket 操作员网关。软件流程已实现，实船定位精度、执行器方向和清洗效果仍需现场验收。

## 项目结构

| 目录 / 功能包 | 职责 |
|---|---|
| [`f1_interfaces`](src/f1_interfaces/README.md) | 统一执行器、控制状态、墙观测、目标边消息与服务 |
| [`f1_nano_bridge`](src/f1_nano_bridge/README.md) | F103 串口协议、CTRL/ACK、控制租约、超时保护和 IMU 遥测 |
| [`f1_mapping_bringup`](src/f1_mapping_bringup/README.md) | Cartographer 2D 纯雷达建图、静态雷达 TF 和扫描检查 |
| [`f1_pool_cleaning`](src/f1_pool_cleaning/README.md) | 固定地图定位、墙感知、沿墙控制、矩形四边任务及覆盖记录 |
| [`f1_l_cleaning`](src/f1_l_cleaning/README.md) | 复用清洗控制器的开放大 L 路线，执行 A → B → C 两段墙 |
| [`f1_operator_gateway`](src/f1_operator_gateway/README.md) | App 的 WebSocket 手动控制、任务服务和遥测入口 |
| [`operator_app/`](operator_app/README.md) | Qt 操作员界面，包含地图、雷达、手动输入和任务按钮 |
| `maps/` | 本地地图资料；被 Git 忽略，克隆仓库和默认部署同步均不包含 |
| `docs/`、`tools/` | 开发验收、历史试验记录和离线路线示意工具 |

仓库共有六个 ROS 功能包。PAVO2 驱动、STM32 固件源码和地图需要单独准备；旧 ZMQ 网关和旧 UI 协议已移除。

## 数据与控制链路

```text
PAVO2 驱动 → /pavo2_scan → Cartographer → /map、map → base_link TF
                 │                            │
                 └──────────┬─────────────────┘
                            ▼
                  墙感知 + 定位门控 + 清洗控制器
                            │ /f1/* 租约与执行器命令
                            ▼
Qt App ←→ WebSocket :8765 ←→ 网关 → f1_nano_bridge ←→ F103
```

建图使用纯雷达，不融合桥发布的 IMU。清洗使用已有 `.pbstream` 冻结地图，操作者先在 RViz 的 `map` 坐标系设置 `2D Pose Estimate`，定位达到 `LOCALIZATION_READY` 后才可开始任务。建图与固定地图定位应择一运行，避免重复发布地图和 TF。

| 任务 | 启动入口 | 任务接口 | 对应地图 |
|---|---|---|---|
| 矩形四边 | `f1_pool_cleaning pool_bringup.launch.py` | `/pool_cleaning/*` | `lidar_20261021_145054.pbstream` |
| 大 L 两段墙 | `f1_l_cleaning l_bringup.launch.py` | `/l_cleaning/*` | `lidar_20261026_170703.pbstream` |

**换地图只需换地图路径。** 两个入口都只接受一个 `map:=` 参数（`.pbstream`，或同目录同名 `.yaml`）：

- 池尺寸、朝向、四边边界和 L 的两段路线由地图自动推导，不再写进 YAML；
- 定位门限使用地图实际空闲水域的范围，不再使用手写矩形；
- 推导失败、地图不是单一封闭池（矩形任务）或不含唯一顺时针墙角（大 L）时启动即报错，不会退回旧地图的数值。

用 `ros2 run f1_pool_cleaning map_profile <地图>` 可离线查看某张地图能推导出什么。船体尺寸、刷盘几何和清洗策略（目标墙距、停车距离、边距）仍按船配置，不随地图变化。

矩形路线为 `edge_0 → edge_1 → edge_2 → edge_3`，三次换边后完成；大 L 为 `edge_0 → edge_1`，一次右转后结束，不返回起点。两种任务都保持墙在船左侧，共享 `/f1/*` 硬件接口；现场一次启动一套任务、一个 bridge 和一套定位。

App 网关默认连接矩形任务，未提供任务类型选择或初始位姿操作。大 L 可通过 ROS 服务操作，或按[网关说明](src/f1_operator_gateway/README.md#连接大-l-任务)显式重映射。App 的 `manual_stop`、断线和输入超时只停止网关持有的手动会话；自动任务应使用自己的 `pause` / `cancel` 服务。

## 构建

在工作空间根目录执行；Nano 的依赖安装和源码同步见[完整部署指南](DEPLOY_NANO.md)。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
colcon list --names-only
```

按需构建可使用 `--packages-up-to f1_l_cleaning f1_operator_gateway`，当前依赖关系下会覆盖六个包。仅构建 `f1_operator_gateway` 不会构建 bridge 或清洗包；只部署 App 手动控制时应显式选择 `--packages-up-to f1_nano_bridge f1_operator_gateway`。

## 启动入口

Nano 每个运行终端先加载同一环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

纯雷达建图需要外部驱动已发布 `/pavo2_scan`：

```bash
ros2 run f1_mapping_bringup scan_check --samples 100
ros2 launch f1_mapping_bringup lidar_mapping.launch.py
```

已有匹配地图时，选择一种清洗任务启动。下面均为预览模式，不启用串口和硬件输出；`start_rviz:=true` 需要图形桌面。

```bash
# 矩形任务；map:= 指向本次要用的地图，其余参数由地图推导。
ros2 launch f1_pool_cleaning pool_bringup.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream \
  start_rviz:=true
```

```bash
# 大 L 任务；与上面的矩形入口择一运行。
ros2 launch f1_l_cleaning l_bringup.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261026_170703.pbstream \
  start_rviz:=true
```

定位就绪后，在另一个已加载环境的终端手动启动；大 L 将下列 `/pool_cleaning/` 换成 `/l_cleaning/`：

```bash
ros2 topic echo /pool_cleaning/localization_status --once
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /pool_cleaning/status --once --full-length
ros2 service call /pool_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /pool_cleaning/cancel std_srvs/srv/Trigger '{}'
```

需要实际输出时，核对现场硬件配置后，在所选 bringup 命令加入 `output_enabled:=true`。若复用已有 bridge，加 `start_bridge:=false` 并在该 bridge 自身配置硬件许可。`pool_nano` 快捷脚本默认开启输出许可，与上述 launch 的默认预览行为不同。

App 网关单独在另一终端运行：

```bash
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

默认监听 `127.0.0.1:8765`；跨机时监听 `0.0.0.0`，App 连接 Nano 的实际 IP。入口当前无认证，使用可信局域网。网关不负责启动 bridge、雷达、定位或清洗节点，Windows 安装与启动见 [App 文档](operator_app/README.md)。

## 离线验证

构建完成后，在单独测试终端加载 ROS 和工作空间环境：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon test --executor sequential --return-code-on-test-failure \
  --ctest-args -E '^firmware(_imu)?$'
colcon test-result --verbose
```

上述命令排除两个依赖外部固件源码的测试组。完整测试需要设置 `F1_FIRMWARE_DIR`，目录准备和补跑命令见[部署指南](DEPLOY_NANO.md#7-nano离线测试)。测试后关闭隔离测试终端；实机终端使用前述运行环境。

本地测试与预览不能证明实船清洗效果。`COMPLETE` 表示配置路线执行结束；覆盖记录统计左刷接触中心线轨迹，不能直接当作池底面积覆盖率。

## 文档导航

| 目的 | 文档 |
|---|---|
| Nano 首次部署、依赖和故障处理 | [DEPLOY_NANO.md](DEPLOY_NANO.md) |
| 日常建图、扫描检查、保存地图 | [START_LIDAR.md](START_LIDAR.md) |
| 矩形清洗部署与实船验收 | [DEPLOY_TEST_NANO.md](src/f1_pool_cleaning/DEPLOY_TEST_NANO.md) |
| 大 L 路线、参数与操作 | [f1_l_cleaning README](src/f1_l_cleaning/README.md) |
| 清洗状态机、开发阶段和验收标准 | [流程图](docs/pool_cleaning/WALL_CLEANING_FLOW.md)、[开发与验收](docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md) |
| 历史工作空间与清洗包升级 | [工作空间迁移](WORKSPACE_MIGRATION.md)、[清洗包迁移](src/f1_pool_cleaning/MIGRATION.md) |
| 历史设计和单边试验 | [长墙设计](docs/long_wall_plan/DESIGN.md)、[2026-10-23 验证](docs/single_side_trial/VALIDATION_20261023.md) |

历史记录保留当时参数与结论；当前启动以各包 README、launch 和 YAML 为准。`tools/preview_long_wall_route.py` 是历史矩形长墙方案的离线示意工具，不生成当前大 L 控制路线。
