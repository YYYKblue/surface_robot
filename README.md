# surface_ws：水面清洗机器人 ROS 2 工作空间

更新：2026-10-31，依据当前源码、启动文件和默认配置整理。目标环境为 Ubuntu 22.04 / ROS 2 Humble，机器人端运行于 ARM64 Nano，操作员端为无需安装 ROS 的 Qt App。

项目包含 PAVO2 纯雷达建图、固定地图定位、F103 串口控制、矩形四边清洗、大 L 两段墙清洗和 WebSocket 操作员网关。软件流程已实现，实船定位精度、执行器方向和清洗效果仍需现场验收。

## 项目结构

| 目录 / 功能包 | 职责 |
|---|---|
| [`f1_interfaces`](src/f1_interfaces/README.md) | 统一执行器、控制状态、墙观测、目标边消息与服务 |
| [`f1_nano_bridge`](src/f1_nano_bridge/README.md) | F103 串口协议、CTRL/ACK、控制租约、超时保护和 IMU 遥测 |
| [`f1_lidar_driver`](src/f1_lidar_driver/README.md) | PAVO2 驱动与 SDK、雷达参数、独立启动及开机服务模板 |
| [`f1_mapping_bringup`](src/f1_mapping_bringup/README.md) | Cartographer 2D 纯雷达建图、静态雷达 TF 和扫描检查 |
| [`f1_localization`](src/f1_localization/README.md) | 固定地图定位、初始位姿、轨迹管理和定位就绪门控 |
| [`f1_pool_cleaning`](src/f1_pool_cleaning/README.md) | 墙感知、关刷导航、沿墙控制、矩形四边任务及覆盖记录 |
| [`f1_l_cleaning`](src/f1_l_cleaning/README.md) | 复用清洗控制器的开放大 L 路线，执行 A → B → C 两段墙 |
| [`f1_operator_gateway`](src/f1_operator_gateway/README.md) | App 的 WebSocket 手动控制、任务服务和遥测入口 |
| [`operator_app/`](operator_app/README.md) | Qt 操作员界面，包含建图/选图、地图、雷达、手动输入和任务按钮 |
| `maps/` | 地图三件套直接存放于此；被 Git 忽略，克隆仓库不包含，当前部署同步命令会传输 |
| `docs/`、`tools/` | 开发验收、历史试验记录和离线路线示意工具 |

仓库共有八个 ROS 功能包。PAVO2 驱动与 SDK 已迁入 `f1_lidar_driver`；STM32 固件源码和地图仍需单独准备。雷达本次完成 PC 端封装，Nano 编译与服务切换按[驱动说明](src/f1_lidar_driver/README.md)由操作者执行。旧 ZMQ 网关和旧 UI 协议已移除。

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

建图使用纯雷达，不融合桥发布的 IMU。清洗使用已有 `.pbstream` 冻结地图，操作者先在 Windows App 地图中发布初始位姿，或在 RViz 的 `map` 坐标系设置 `2D Pose Estimate`，定位达到 `LOCALIZATION_READY` 后才可开始任务。建图与固定地图定位应择一运行，避免重复发布地图和 TF。

| 任务 | 启动入口 | 任务接口 | 示例地图 |
|---|---|---|---|
| 矩形四边 | `f1_pool_cleaning pool_bringup.launch.py` | `/pool_cleaning/*` | `lidar_20261021_145054.pbstream` |
| 大 L 两段墙 | `f1_l_cleaning l_bringup.launch.py` | `/l_cleaning/*` | `lidar_20261026_170703.pbstream` |

**使用默认配置时，换地图只需换地图路径。** 两个入口的地图输入统一为 `map:=`（`.pbstream`，或同目录同名 `.yaml`），并需配齐同名 `.pbstream/.yaml/.pgm`：

- 池尺寸、朝向、四边边界和 L 的两段路线由地图自动推导，不再写进 YAML；
- 定位门限使用地图实际空闲水域的范围，不再使用手写矩形；
- 推导失败、地图不是单一封闭池（矩形任务）或不含唯一顺时针墙角（大 L）时启动即报错，不会退回旧地图的数值。

用 `ros2 run f1_pool_cleaning map_profile <地图>` 可离线查看某张地图能推导出什么。船体尺寸、刷盘几何和清洗策略（目标墙距、停车距离、边距）仍按船配置，不随地图变化。

控制器会将推导结果作为参数默认值；现场 YAML 中显式填写的几何参数仍可覆盖它们。因此更换地图时应检查自定义配置是否残留旧 `boundary_xy`、`route_xy` 或池尺寸。地图推导在读取这些覆盖参数前执行，手工填路线不能绕过不支持的地图校验。

矩形任务包含四条边，大 L 包含两条边且不构造 C→A 清洗墙。默认按当前位姿选择可达起始边，关刷导航到起点，沿墙清洗并按需回补遗漏区间，全部要求区间覆盖后才完成。两种任务都保持墙在船左侧，共享 `/f1/*` 硬件接口；同一机器人仅运行一套 bridge 和定位，网关只允许一个清洗任务活动。组合入口可以让当前地图支持的控制器同时待命。

App 的受管入口支持无地图待机、纯雷达建图、命名保存、地图列表/预览/加载；地图保存到 Nano 工作空间 `maps/`，空名称使用时间戳。App 支持 L／矩形任务选择、地图初始位姿发布、启动条件和完成结果显示。推荐使用[网关组合入口](src/f1_operator_gateway/README.md)启动一套共享定位和当前地图支持的任务；独立大 L bringup 使用网关 `localization_prefix:=/l_cleaning`，不再重映射任务接口。App 的 `manual_stop`、断线和输入超时只停止网关持有的手动会话；自动任务应使用自己的 `pause` / `stop` / `cancel` 服务。

固定地图定位现在由独立的 [`f1_localization`](src/f1_localization/README.md) 提供。可单独构建 `--packages-up-to f1_localization`，并使用 `ros2 launch f1_localization localization.launch.py map:=...` 启动；矩形和大 L 的原 bringup 命令保持不变。旧 `f1_pool_cleaning/pool_localization.launch.py` 保留为转发入口。首次升级需按[定位拆包升级](DEPLOY_NANO.md#定位拆包升级)清理旧安装残留。

## 构建

在工作空间根目录执行；Nano 的依赖安装和源码同步见[完整部署指南](DEPLOY_NANO.md)。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
colcon list --names-only
```

按需构建可使用 `--packages-up-to f1_l_cleaning f1_operator_gateway`，当前依赖关系下会覆盖八个包。网关为完整部署声明了雷达驱动、bridge、定位及清洗包的运行依赖，`--packages-up-to f1_operator_gateway` 会一并构建这些依赖。

## 启动入口

Nano 每个运行终端先加载同一环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

雷达由独立服务常驻运行；迁移后的服务使用本仓库 `f1_lidar_driver`，切换说明见[驱动部署](src/f1_lidar_driver/README.md#nano-由操作者切换现有开机服务)。没有服务实例时可单独执行 `ros2 launch f1_lidar_driver lidar.launch.py`，不要重复启动。

确认驱动已发布 `/pavo2_scan` 后建图：

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

需要实际输出时，核对现场硬件配置后，在所选 bringup 命令加入 `output_enabled:=true`。若复用已有 bridge，加 `start_bridge:=false` 并在该 bridge 自身配置硬件许可。`pool_nano` 快捷脚本和仓库的 `f1-operator.service` 模板默认开启输出许可，与上述 launch 的默认预览行为不同；允许输出不等于自动开始任务。

日常由 App 建图和选图，使用新的无地图受管入口（与其他完整 bringup 择一运行）：

```bash
ros2 launch f1_operator_gateway operator_system.launch.py \
  workspace_root:=/home/nvidia/surface_ws bind_address:=0.0.0.0
```

该入口常驻 bridge 和网关，按 App 操作互斥切换建图与固定地图定位/清洗；默认硬件输出关闭，雷达复用现有常驻驱动。开机服务模板及部署见 [App 建图选图与开机服务](DEPLOY_NANO.md#app-建图选图与开机服务)。

已有节点只需单独连接 App 时，在另一终端运行网关：

```bash
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

默认监听 `127.0.0.1:8765`；跨机时监听 `0.0.0.0`，App 连接 Nano 的实际 IP。入口当前无认证，使用可信局域网。上述单独网关入口不启动其他节点；新增 `operator_bringup.launch.py map:=...` 可启动一套共享定位、bridge 和当前地图支持的清洗任务（雷达仍需另行启动）。Windows 安装与启动见 [App 文档](operator_app/README.md)。

## 离线验证

构建完成后，在单独测试终端加载 ROS 和工作空间环境：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/f1_operator_gateway/test/cyclonedds_localhost.xml"
colcon test --executor sequential --return-code-on-test-failure \
  --ctest-args -E '^firmware(_imu)?$'
colcon test-result --verbose
```

上述命令从工作空间根目录执行，排除两个依赖外部固件源码的测试组。DDS 配置只用于本机测试，避免本地大消息分片接收问题；不要用于实机跨主机通信。完整测试需要设置 `F1_FIRMWARE_DIR`，目录准备和补跑命令见[部署指南](DEPLOY_NANO.md#7-nano离线测试)。Qt App 测试另见 [App 开发验证](operator_app/README.md#开发验证)，不在 colcon 测试中。测试后关闭隔离测试终端；实机终端使用前述运行环境。

本地测试与预览不能证明实船清洗效果。带地图自动任务只有在全部要求接触区间的缺口满足容差后才进入 `COMPLETE`；覆盖记录统计左刷接触中心线轨迹，不能直接当作池底面积覆盖率。

## 文档导航

| 目的 | 文档 |
|---|---|
| 代码职责、入口选择、参数与状态流 | [项目架构与代码导读](docs/ARCHITECTURE.md) |
| Nano 首次部署、依赖和故障处理 | [DEPLOY_NANO.md](DEPLOY_NANO.md) |
| 日常建图、扫描检查、保存地图 | [START_LIDAR.md](START_LIDAR.md) |
| 矩形清洗部署与实船验收 | [DEPLOY_TEST_NANO.md](src/f1_pool_cleaning/DEPLOY_TEST_NANO.md) |
| 大 L 路线、参数与操作 | [f1_l_cleaning README](src/f1_l_cleaning/README.md) |
| 清洗状态机、开发阶段和验收标准 | [流程图](docs/pool_cleaning/WALL_CLEANING_FLOW.md)、[开发与验收](docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md) |
| 历史工作空间与清洗包升级 | [工作空间迁移](WORKSPACE_MIGRATION.md)、[清洗包迁移](src/f1_pool_cleaning/MIGRATION.md) |
| 历史设计和单边试验 | [长墙设计](docs/long_wall_plan/DESIGN.md)、[2026-10-23 验证](docs/single_side_trial/VALIDATION_20261023.md) |

历史记录保留当时参数与结论；当前启动以各包 README、launch 和 YAML 为准。`tools/preview_long_wall_route.py` 是历史矩形长墙方案的离线示意工具，不生成当前大 L 控制路线。
