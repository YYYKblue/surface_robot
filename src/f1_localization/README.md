# f1_localization：固定地图定位

从 `f1_pool_cleaning` 拆出的独立定位包。负责加载 Cartographer 冻结地图、发布栅格地图、处理初始位姿、管理纯定位轨迹，以及检查定位是否可供任务使用。本包不依赖清洗包或 bridge，不发布执行器命令，也不申请控制租约。

## 独立构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_localization \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 launch f1_localization localization.launch.py \
  map:=/path/to/reference.pbstream start_rviz:=true
```

仓库内只需 `f1_interfaces`、`f1_mapping_bringup` 和本包。Cartographer、ROS 等系统依赖通过 rosdep 安装；PAVO2 驱动需另行运行，默认输入 `/pavo2_scan`。

`map:=` 接受 `.pbstream` 或配套 `.yaml` 路径。纯定位需要 `.pbstream`，定位范围计算还需要 YAML 和栅格图；请一起同步同名 `.pbstream`、`.yaml`、`.pgm`，不能只复制状态文件。支持现有 `load_state_filename:=` 兼容参数。

默认不创建新轨迹。在 RViz 的 `map` 坐标系使用 `2D Pose Estimate`，或在 Windows App 地图中设置并发布初始位姿后，管理器调用 Cartographer 的轨迹服务，并检查轨迹状态、TF/扫描新鲜度、地图范围、位姿跳变、有效扫描比例和扫描/地图重合。持续满足稳定窗口后才发布 `LOCALIZATION_READY`。轨迹创建成功本身不表示定位就绪。

## 接口与清洗任务接入

本轮保持原有节点名称 `pool_localization`、`pool_localization_manager`，以兼容现有参数文件和监控脚本。接口默认也保持不变：

| 接口 | 用途 |
|---|---|
| `/initialpose` | RViz 初始位姿输入 |
| `/pool_cleaning/set_initial_pose` | `f1_interfaces/SetInitialPose`，服务方式设置初始位姿 |
| `/pool_cleaning/relocalize` | `std_srvs/Trigger`，结束当前定位轨迹并等待新的初始位姿 |
| `/pool_cleaning/localization_status` | `std_msgs/String`，JSON 定位状态及诊断 |
| `/pool_cleaning/localization_ready` | `std_msgs/Bool`，定位就绪；保留 transient-local QoS |
| `/map`、TF | 栅格地图和 `map → odom → base_link`，供任务与显示使用 |

为大 L 独立启动定位时加 `cleaning_prefix:=l_cleaning`，会将上表中四个 `/pool_cleaning/*` 接口重映射到 `/l_cleaning/*`；`/initialpose`、`/map` 和 TF 不变。这个参数仅控制接口映射，不决定定位算法或任务路线。

```bash
ros2 launch f1_localization localization.launch.py \
  map:=/path/to/reference.pbstream cleaning_prefix:=l_cleaning
# 另一终端加载同一环境后复用定位：
ros2 launch f1_l_cleaning l_bringup.launch.py \
  map:=/path/to/reference.pbstream start_localization:=false
```

矩形与大 L 的 bringup 已直接 include 本包，无需改变原任务启动命令。新增网关 `operator_bringup.launch.py` 将共享定位接口统一映射至 `/f1_localization/*`，两个任务通过 `localization_status_topic` 消费同一状态；定位状态包含规范化地图路径标识 `map_id`，用于核对任务地图。定位与任务必须使用同一张地图；同一机器人只运行一套 Cartographer/定位管理器。建图与固定地图定位择一运行。

常用参数：`scan_topic`、`reference_trajectory_id`、`localization_config`、`publish_lidar_tf`、`lidar_extrinsics_file`、`start_rviz`、`rviz_config`。已有雷达静态 TF 时设置 `publish_lidar_tf:=false`。`localization_config` 中的节点选择器仍使用 `pool_localization_manager`。雷达外参复用 `f1_mapping_bringup` 的配置，不会启动建图。

清洗控制器仍负责检查定位状态的新鲜度和就绪条件，并在定位失效时关闭运动、暂停任务；定位包不负责停止执行器。

## 代码归属

- `localization.py`：定位门控、初始位姿校验、扫描/地图重合与地图范围参数转换。
- `localization_node.py`：ROS 接口、TF/扫描输入、Cartographer 轨迹生命周期。
- `map_io.py`：共用地图文件解析、栅格读取、最大空闲连通域及范围计算。
- `config/localization.lua`、`localization.rviz`：纯雷达定位和显示配置。

矩形四边、大 L 路线推导继续保留在 `f1_pool_cleaning/map_profile.py`，其底层地图读取来自本包。旧 `map_profile` 的公共地图辅助函数保留导入兼容；定位包不会反向导入清洗包。

## 升级兼容与部署

旧入口仍保留为转发，仅有一份定位实现：

```bash
ros2 launch f1_pool_cleaning pool_localization.launch.py map:=/path/to/reference.pbstream
ros2 run f1_pool_cleaning pool_localization --ros-args ...
```

直接运行节点仍需传入原有地图、Cartographer 配置目录等参数，推荐使用 launch。Python 导入路径已迁至 `f1_localization.localization` 和 `f1_localization.localization_node`；Lua/RViz 路径改为本包 `config/localization.lua`、`config/localization.rviz`，外部脚本如写死旧路径需更新。

首次升级应停止旧节点，在保存现场自定义配置后，同步新增定位包和两个清洗包，并传播已迁走文件的删除。不要只覆盖旧清洗目录。重新生成相关包的 build/install，或在全新的构建与安装目录构建，避免残留旧 Python 模块、配置和测试结果。Nano 的操作步骤见[部署指南](../../DEPLOY_NANO.md#定位拆包升级)。

## 验证

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon test --executor sequential --packages-select f1_localization \
  --return-code-on-test-failure
colcon test-result --test-result-base build/f1_localization --verbose
```

测试包括定位门控、独立地图读取、launch 组装，以及使用模拟 Cartographer 服务和合成 TF/扫描的 ROS 定位管理器测试。后者验证初始位姿、就绪、扫描过期、重定位及接口重映射，不启动硬件，也不证明实船定位精度。清洗任务接入另运行 `f1_pool_cleaning` 和 `f1_l_cleaning` 的测试。

### 本次拆包验证记录（2026-10-30）

- 在独立构建/安装目录仅构建本包及两个仓库依赖，确认没有清洗包或 bridge 时定位节点仍可导入；随后完成清洗包构建，并重建当前工作空间的三个受影响包。
- 通过 CTest 执行：定位包 14/14 项通过，矩形清洗 182/183 项通过，大 L 21/22 项通过。两个未通过用例均为组合启动后未收到任务状态；用拆分前提交 `2528c72` 的控制器、地图工具和启动文件在同一环境中也复现了同类超时，因此本次不能报告完整回归全通过。超时根因尚未确定。
- 使用本地真实地图检查新定位入口、旧定位兼容入口、矩形 bringup 和大 L bringup：四个入口均能发布地图与定位状态，每次仅有一套定位，bridge 和硬件输出关闭。该检查没有真实扫描输入，不验证定位精度。
- 两张本地地图的任务几何及定位范围与拆分前逐项相同；定位门控函数、Lua 和 RViz 配置内容保持原行为；`git diff --check` 通过。
