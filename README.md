# surface_ws ROS 2 工作空间

工作空间原名 `f1_mapping_ws`，现为 `surface_ws`；桥接功能包原名 `nano_bridge`，现为 `f1_nano_bridge`。Nano 旧工作空间已全部删除，当前请按[Nano 从零部署指南](DEPLOY_NANO.md)创建、传输和编译；[改名迁移说明](WORKSPACE_MIGRATION.md)仅供仍保留旧目录的环境参考。

> 本仓库根目录即 ROS 工作空间，包含 `src/`、构建说明和配置，不包含固件、旧网关或桌面 UI。现已在 `f1_nano_bridge` 内提供新版 Windows UI 网关 `f1_ui_gateway`，接入步骤见 [Windows UI 接入](src/f1_nano_bridge/WINDOWS_UI.md)。克隆后直接在仓库根目录运行 colcon。`build/`、`install/`、`log/` 和缓存不纳入版本控制。
>
> 固件互通测试另需 back2 源码，默认查找工作空间同级的 `firmware/`，也可通过 `F1_FIRMWARE_DIR=/absolute/path/to/firmware` 指定。实际运行节点不需要固件源码。旧 ZMQ IMU 链路文档仅供历史参考，其依赖的旧网关不包含在本仓库中。


用于 PAVO2 纯雷达建图的 ROS 2 Humble 工作空间，另含硬件桥与清洁任务功能包。

现已新增原生 ROS F103 硬件桥 [`f1_nano_bridge`](src/f1_nano_bridge/README.md) 和
统一接口包 [`f1_interfaces`](src/f1_interfaces/README.md)，适配 `firmware`（back2，协议 v2），包含二进制控制、租约、CTRL 与 IMU。
新桥默认不打开串口；与占用同一串口的旧网关及旧 ZMQ IMU 发布节点不能同时使用。新版 `f1_ui_gateway` 只调用 ROS 接口，应与原生桥同时运行。
清洗功能统一在 [`f1_pool_cleaning`](src/f1_pool_cleaning/README.md)：合并原 `f1_wall_cleaning` 的感知与原 `f1_rectangle_cleaning` 的当前单边控制，删除旧脉冲控制及其试跑配置。首次升级按 [清洗包迁移说明](src/f1_pool_cleaning/MIGRATION.md) 移走旧包和安装产物。
建图包仅保留纯雷达模式；硬件桥的 IMU 遥测不参与建图。

[f1_pool_cleaning](src/f1_pool_cleaning/README.md) 保持已验证的单边策略：±10° 内滚刷 30% 推进，四主电机从 ±3° 外持续纠偏，第五路以 ±30% 控制中心墙距 38–42 cm，船体前缘到前墙 50 cm 时关闭输出。已接入固定地图定位与可选 bridge 租约输出，无转向空间门控、脉冲等待或自动换边。历史两边设计与地图示意见 [两条长边刷洗设计](docs/long_wall_plan/DESIGN.md)。

清洗任务采用纯雷达反馈，不引入 IMU 航向控制。入口统一为 `pool_*`，话题/服务统一为 `/pool_cleaning/*`。部署与下水步骤见 [Nano 同步与纯雷达测试](src/f1_pool_cleaning/DEPLOY_TEST_NANO.md)。当前仓库包含四个功能包。

建图流程仅使用 **纯雷达模式**：
`ros2 launch f1_mapping_bringup lidar_mapping.launch.py`。
默认输出车体中心 `base_link` 的二维位姿，入口自动发布已确认的雷达平面安装 TF；雷达实际高度仍待测量。
日常操作见 [纯雷达建图启动步骤](START_LIDAR.md)，包含首次部署、启动检查、地图查看、保存与退出。
回波检查、部署和启动步骤见 [纯雷达说明](src/f1_mapping_bringup/README_LIDAR.md)。

`src/f1_mapping_bringup` 配置包仅用于 PAVO2 的 Cartographer 2D 纯雷达建图。
原联合入口、联合 Lua、联合外参及加载代码已删除。
算法讲解见 [结合代码理解 Cartographer](src/f1_mapping_bringup/CARTOGRAPHER_PRINCIPLES.md)。
使用步骤与当前验证边界见 [功能包说明](src/f1_mapping_bringup/README.md)。
`src/f1_nano_bridge` 统一管理桥接功能，内部 `nano_ros_bridge.imu` 保留旧 ZMQ 接入节点和共用的纯 IMU 转换库；原生桥直接从串口发布 IMU。
旧链路历史说明见 [IMU 接入说明](src/f1_nano_bridge/IMU.md)，当前固件启动以原生串口桥说明为准。

## 2026-10-23 清洗包合并验证

四个功能包构建通过；`f1_pool_cleaning` 共 **134 项独立 pytest 用例通过，0 失败、0 错误、0 跳过**：感知与几何参数 28、单边控制 71、反向纠偏回归 3、统一启动 2、ROS 接口与租约 22、合成雷达到实际 bridge Session 链路 8。删除的旧脉冲控制用例不再计入。

合并前后同一组 96 项算法用例均通过；当前控制参数、状态机、租约及墙线算法保持原行为。首次完整测试发现新增链路场景误用了全周合成雷达，改为实机对应的后向 280° 视场后，受影响的 8 项链路测试全部通过，其余组的通过结果沿用。`colcon test-result` 当前显示 135，包含此次单组复测的 1 项 CTest 包装，不是 135 项独立用例。

另已验证新包安装索引、五个可执行入口、Nano 脚本参数入口和离线演示；两个旧包不再出现在安装索引。使用备份 `.pbstream` 启动新的定位入口并查询到冻结轨迹 0，正常退出。ROS 检查仅使用 localhost 隔离域、合成反馈和模拟 bridge；本轮未同步 Nano、未连接串口、未进行水上验证。

## 2026-10-23 简化沿墙控制验证

以下为合并前的历史验证，旧包名、包数量和测试数量保留当时含义。当前仅部署 `f1_pool_cleaning`，不再同步两个旧清洗包。

5 包构建通过；矩形包 53 项逻辑和 19 项 ROS 集成测试通过，感知/单墙控制另 62 项回归通过。使用合成反馈与模拟 bridge，未连接或同步 Nano。详见 [简化版验证记录](docs/single_side_trial/SIMPLIFIED_VALIDATION_20261023.md)。

随后修复大偏角引发的丢墙锁定暂停：矩形任务墙线跟踪范围扩大至 ±70°，增加短暂丢墙等待恢复；状态前部新增厘米制 `left_wall_distance_cm`。本轮两个清洗包共 163 项独立测试通过，仍需同步两个包并在 Nano 重启清洗节点。

后续反向纠偏排查复现了丢墙后误认后墙的问题，已调整最短可见墙段与关联超时，并将四主电机 ±3° 纠偏和滚刷 ±10° 门槛分开。最新两个清洗包 169 项测试通过，另通过 45 种固件编码组合核对。见 [代码复现分析](docs/single_side_trial/HEADING_FEEDBACK_20261023.md)。

## 2026-10-22 本地验证记录

随后补齐 Windows UI 网关，`f1_nano_bridge` 再次构建并完整回归，独立测试由 72 项增至 **121 项全部通过**；同级 Windows UI 的 10 项 Qt/ZMQ 测试也通过。新增网关启动、TCP 状态及退出检查通过，未向 Nano 部署。详情见 [网关验证记录](src/f1_nano_bridge/README.md#2026-10-22-windows-ui-网关验证)。下表保留加入网关前的全工作空间验证记录，本次未重跑其他三个包。

接口合并、桥接包合并及改名完成后，已在 `/home/bboss/new-nano/surface_ws` 全新构建五个功能包，并验证新包索引、安装符号链接和两个桥接 launch 的参数入口。

全工作空间回归后，安装 `python3-zmq` 并补跑此前跳过的 `imu_transport`，结果通过。按各包 pytest XML 汇总，当前保留的测试结果如下（不计 CTest 包装用例）：

| 功能包 | 通过 | 失败/错误 | 跳过 |
| --- | ---: | ---: | ---: |
| `f1_mapping_bringup` | 22 | 0 | 0 |
| `f1_nano_bridge` | 72 | 0 | 0 |
| `f1_wall_cleaning` | 70 | 0 | 0 |
| `f1_rectangle_cleaning` | 36 | 0 | 0 |
| 合计 | 200 | 0 | 0 |

`f1_interfaces` 已构建通过，无独立 pytest 用例。以上结果由完整回归与随后单项 ZMQ 补测组成；`colcon test-result` 还统计 CTest 包装记录，其汇总数会随单项重跑变化，不等同于独立测试数。

本地 ZMQ 环境为 PyZMQ 22.3.0、ZeroMQ 4.3.4。回环测试使用 localhost、随机端口及模拟 IMU 数据，没有连接硬件；本次没有向 Nano 同步或部署。补测命令见 [IMU 说明](src/f1_nano_bridge/IMU.md#补跑-zmq-回环测试)。

## PC 路径

```text
/home/bboss/new-nano/surface_ws
```

在未加载本工作空间安装环境的终端中构建：

从旧版本更新时，先按[接口迁移说明](src/f1_interfaces/README.md#从旧接口包迁移)移走旧接口包源码与生成目录，再重新构建；所有自定义接口统一由 `f1_interfaces` 提供。
原独立 IMU 包也已合并，需按[桥接包迁移说明](src/f1_nano_bridge/README.md#从独立-imu-包迁移)移走旧包；桥接功能统一由 `f1_nano_bridge` 提供。
两个清洗包也已合并，按[清洗包迁移说明](src/f1_pool_cleaning/MIGRATION.md) 更新入口、话题和自定义参数文件。

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
colcon build --symlink-install
```

构建完成后，在另一个终端中加载：

```bash
source /opt/ros/humble/setup.bash
source /home/bboss/new-nano/surface_ws/install/local_setup.bash
```

## Nano 从零部署

Nano 只创建 `/home/nvidia/surface_ws`，系统 ROS 使用 `/opt/ros/humble`。不加载已删除的旧工作空间，也不需要编译空目录。

请按 [DEPLOY_NANO.md](DEPLOY_NANO.md) 依次操作：

1. **Nano**：检查 Ubuntu/ROS，移除终端启动文件中失效的环境加载行，安装 SSH、rsync、colcon、rosdep、Cartographer 和 Cyclone DDS。
2. **PC**：把 `surface_ws` 源码与文档同步到 Nano，排除 `build/`、`install/`、`log/` 等生成目录。
3. **Nano**：用 rosdep 安装包依赖，确认四个项目包，再以单任务方式编译。
4. **Nano**：加载 `/home/nvidia/surface_ws/install/local_setup.bash`，检查新包与启动入口，运行离线测试。
5. **实景使用前**：恢复另行提供的 PAVO2 驱动；它不包含在当前四个包中。若驱动也被删除，须先恢复才有真实雷达输入。

完整固件互通测试需要额外同步只读固件测试源码，指南中提供了单独传输和补测命令。实际运行桥接节点不需要固件源码。
