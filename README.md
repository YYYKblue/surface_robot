# F1 Cartographer 2D 工作空间

> 本仓库根目录即 ROS 工作空间，包含 `src/`、构建说明和配置，不包含固件、旧网关或桌面 UI。克隆后直接在仓库根目录运行 colcon。`build/`、`install/`、`log/` 和缓存不纳入版本控制。
>
> 固件互通测试另需 back2 源码，默认查找工作空间同级的 `firmware/`，也可通过 `F1_FIRMWARE_DIR=/absolute/path/to/firmware` 指定。实际运行节点不需要固件源码。旧 ZMQ IMU 链路文档仅供历史参考，其依赖的旧网关不包含在本仓库中。


用于 PAVO2 雷达与 F1 IMU 的 ROS 2 Humble 建图项目。

现已新增原生 ROS F103 硬件桥 [`nano_bridge`](src/nano_bridge/README.md) 和
`f1_bridge_interfaces`，适配 `firmware`（back2，协议 v2），包含二进制控制、租约、CTRL 与 IMU。
新桥默认不打开串口；与旧网关及旧 ZMQ IMU 发布节点不能同时使用。
现已新增 [`f1_wall_cleaning`](src/f1_wall_cleaning/README.md) 和 `f1_wall_interfaces`，实现左侧单墙感知、back2 互斥方向脉冲控制和租约任务接口。默认仅观测/预览，未知几何参数阻止启动运动。固定地图定位、完整池边路线、自动过角和 UI→ROS 适配仍按清洁方案后续实施。
下文原有建图/旧 IMU 链路步骤保留，新桥构建及同步范围以新包说明为准。

基于已保存地图和第五路横向侧推的新思路，新增独立功能包 [f1_rectangle_cleaning](src/f1_rectangle_cleaning/README.md)，负责矩形水池两条长边任务。当前提供无输出状态机、侧推组合指令预览、停车反馈检查及人工换边入口；固定地图定位输入链路、实机输出与自动换边尚未实现。设计与地图示意见 [两条长边刷洗设计](docs/long_wall_plan/DESIGN.md)。

矩形任务后续采用纯雷达反馈，不引入 IMU 航向控制。部署和第一轮实景验证见 [Nano 同步与纯雷达测试](src/f1_rectangle_cleaning/DEPLOY_TEST_NANO.md)。

现有建图流程优先使用 **纯雷达模式**：
`ros2 launch f1_mapping_bringup lidar_mapping.launch.py`。
默认输出车体中心 `base_link` 的二维位姿，入口自动发布已确认的雷达平面安装 TF；雷达实际高度仍待测量。
日常操作见 [纯雷达建图启动步骤](START_LIDAR.md)，包含首次部署、启动检查、地图查看、保存与退出。
回波检查、部署和启动步骤见 [纯雷达说明](src/f1_mapping_bringup/README_LIDAR.md)。

已加入 `src/f1_mapping_bringup` 配置包，用于雷达与 F1 IMU 的 Cartographer 2D 建图。
使用步骤与当前验证边界见 [功能包说明](src/f1_mapping_bringup/README.md)。
`src/f1_imu_bridge` 保留旧 ZMQ 接入节点和共用的纯 IMU 转换库；新桥复用转换库，直接从串口发布 IMU。
旧链路历史说明见 [IMU 接入说明](src/f1_imu_bridge/README.md)，当前固件启动以新桥说明为准。

## PC 路径

```text
/home/bboss/new-nano/f1_mapping_ws
```

在未加载本工作空间安装环境的终端中构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/f1_mapping_ws
colcon build --symlink-install
```

构建完成后，在另一个终端中加载：

```bash
source /opt/ros/humble/setup.bash
source /home/bboss/new-nano/f1_mapping_ws/install/local_setup.bash
```

## 在 Nano 创建相同结构

在 Nano 的新终端执行：

```bash
source /opt/ros/humble/setup.bash
command -v colcon
mkdir -p /home/nvidia/f1_mapping_ws/src
cd /home/nvidia/f1_mapping_ws
touch src/.gitkeep
printf '/build/\n/install/\n/log/\n' > .gitignore
colcon build --symlink-install
```

仅创建空目录时，构建显示 `0 packages finished`。
同步 PC 的七个功能包后再次构建，应包含 `f1_mapping_bringup`、`f1_imu_bridge`、`f1_bridge_interfaces`、`nano_bridge`、`f1_wall_interfaces`、`f1_wall_cleaning` 和 `f1_rectangle_cleaning`。

构建完成后，在 Nano 的另一个终端中加载：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

PC 与 Nano 分别构建，后续同步源码时排除 `build/`、`install/` 和 `log/`。
依赖旧工作空间内的驱动时，需在构建前先加载其环境；具体依赖在加入功能包时确认。

## 将功能包同步到 Nano

确认 Nano 已创建 `/home/nvidia/f1_mapping_ws/src` 后，在 PC 终端执行：

```bash
scp -r /home/bboss/new-nano/f1_mapping_ws/src/f1_mapping_bringup \
  /home/bboss/new-nano/f1_mapping_ws/src/f1_imu_bridge \
  /home/bboss/new-nano/f1_mapping_ws/src/f1_bridge_interfaces \
  /home/bboss/new-nano/f1_mapping_ws/src/nano_bridge \
  /home/bboss/new-nano/f1_mapping_ws/src/f1_wall_interfaces \
  /home/bboss/new-nano/f1_mapping_ws/src/f1_wall_cleaning \
  /home/bboss/new-nano/f1_mapping_ws/src/f1_rectangle_cleaning \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/src/
```

在 Nano 的构建终端中执行：

```bash
source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/ros2_ws/install/setup.bash ]; then
  source /home/nvidia/ros2_ws/install/setup.bash
fi
cd /home/nvidia/f1_mapping_ws
colcon build --symlink-install --packages-up-to f1_mapping_bringup nano_bridge f1_rectangle_cleaning
colcon test --packages-select f1_mapping_bringup f1_imu_bridge f1_wall_cleaning f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --verbose
```

新桥的完整离线测试额外执行 `colcon test --packages-select nano_bridge --return-code-on-test-failure`，
其中真实 C 互通测试需要 `firmware`（back2，协议 v2） 的只读源码；Nano 仅同步功能包时，按
[新桥测试说明](src/nano_bridge/README.md) 提供 `F1_FIRMWARE_DIR` 后再运行该测试。

在 Nano 的另一个终端中查看参数（不启动传感器或建图进程）：

```bash
source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/ros2_ws/install/setup.bash ]; then
  source /home/nvidia/ros2_ws/install/setup.bash
fi
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
ros2 launch f1_mapping_bringup mapping.launch.py --show-args
```
