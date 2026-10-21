# 纯雷达建图启动步骤

本文对应 `f1_mapping_ws` 当前的 PAVO2 + Cartographer 2D 配置，默认在 Nano 上运行建图，在 PC 上按需查看地图。
首次部署先完成第 1 节；已部署并构建过的设备，每次从第 2 节开始。

| 项目 | 当前使用值 |
| --- | --- |
| Nano 登录地址 | `nvidia@192.168.1.188`，若地址已变更请替换 |
| Nano 工作空间 | `/home/nvidia/f1_mapping_ws` |
| PC 工作空间 | `/home/bboss/new-nano/f1_mapping_ws` |
| ROS | ROS 2 Humble |
| ROS 通信环境 | `ROS_DOMAIN_ID=33`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| 雷达输入 | `/pavo2_scan`，类型 `sensor_msgs/msg/LaserScan` |
| 雷达帧 | `pavo2s_frame` |
| 车体中心输出帧 | `base_link`（二维平面位姿） |
| 启动入口 | `lidar_mapping.launch.py` |
| 栅格分辨率 | 默认 `0.05 m/格` |

本模式不需要启动 `f1_imu_bridge`，不使用轮速里程计，也不需要填写 `sensors.yaml` 或启动 `sensor_tf.launch.py`。
入口自动根据 `config/lidar_planar.yaml` 发布雷达安装 TF，平面参数为 x=-0.2143195 m、y=0、yaw=π。
实际高度未知，`base_link` 在此定义为车体中心在雷达扫描平面的投影；TF 的 z/roll/pitch=0 是二维约定，不是实测值。
`mapping.launch.py` 是雷达与 IMU 联合入口，本次只使用 `lidar_mapping.launch.py`。
建图程序本身不发送运动指令；如需操纵设备，使用已有控制链路。

## 1. 首次部署或源码更新后：同步与构建

### 1.1 PC：同步建图包

在 **PC 终端**执行。只同步纯雷达所需的配置包，不复制 PC 的 `build/`、`install/`、`log/`：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/f1_mapping_ws/src'
scp -r /home/bboss/new-nano/f1_mapping_ws/src/f1_mapping_bringup \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/src/
```

### 1.2 Nano：检查依赖并构建

登录 Nano：

```bash
ssh nvidia@192.168.1.188
```

在 **Nano 的新终端**中构建，避免先加载本工作空间旧的安装环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
ros2 pkg executables cartographer_ros
ros2 pkg prefix rmw_cyclonedds_cpp
command -v colcon
```

依赖检查应找到 `cartographer_node`，以及 `cartographer_occupancy_grid_node` 或 `occupancy_grid_node`。
`/home/nvidia/ros2_ws` 是项目现有依赖工作空间；若目标机路径不同，替换为实际安装 Cartographer 的环境。
任一步提示文件、包或命令不存在，先解决依赖或路径问题再继续。

```bash
cd /home/nvidia/f1_mapping_ws
colcon build --symlink-install --packages-select f1_mapping_bringup
```

确认 `f1_mapping_bringup` 构建成功。日常启动不必重复同步和构建。

## 2. 每次启动：加载 Nano 环境

后文所有 **Nano 运行终端**都先执行这一段，包括另开的检查和保存地图终端：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 3. Nano 终端 A：确认雷达可用

### 3.1 检查雷达服务

雷达驱动由已有系统服务 `pavo2-scan.service` 提供，建图入口不会启动驱动。

```bash
systemctl status pavo2-scan.service --no-pager
```

应显示 `active (running)`。如果服务已安装但未运行，启动后再检查：

```bash
sudo systemctl start pavo2-scan.service
systemctl status pavo2-scan.service --no-pager
```

若提示找不到服务，先核实目标机的驱动部署，不要另开重复驱动。
服务运行失败时查看日志：

```bash
journalctl -u pavo2-scan.service -n 80 --no-pager
```

### 3.2 检查实际扫描数据

保持雷达静止，执行：

```bash
ros2 topic info /pavo2_scan --verbose
ros2 run f1_mapping_bringup scan_check --samples 100
```

重点查看：

- 话题类型为 `sensor_msgs/msg/LaserScan`，且存在发布者。
- `scan_check` 的 `frames` 为 `["pavo2s_frame"]`，`issues` 为 `[]`。
- 存在有效回波，静止时墙面距离与现场大致一致。

现有记录中的雷达输入约为 50 Hz，Lua 按 `0.4` 比例采样，算法接收约 20 Hz；
这些是参考值，不是每次必须精确达到的指标。`issues=[]` 只代表消息检查通过，不代表物理测距精度已验收。
若超时、帧名不符或时间戳异常，先处理再启动建图。

### 3.3 确认没有重复建图进程

```bash
ros2 node list
```

首次启动前若已出现 `/cartographer_node` 或 `/cartographer_occupancy_grid_node`，
先确认是否已经有一套建图在运行。需要重开时，先退出原启动终端中的建图。
若是旧服务自动启动的实例，先确认对应服务再停止；旧链路切换见[项目说明](../README.md#旧链路切换与回滚)。
避免多套节点同时发布相同地图和 TF。

## 4. Nano 终端 A：启动纯雷达建图

在已完成第 2、3 节的终端执行：

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py
```

保持这个终端运行。默认不打开 RViz，适合无图形界面的 SSH 会话，实机建图默认 `use_sim_time=false`。

如果在 **Nano 本地图形桌面**操作，希望同时打开 RViz，则使用下面的命令启动同一套建图；两条命令选一条：

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py use_rviz:=true
```

初始阶段保持雷达静止，等待轨迹、子图和地图建立，再缓慢平移、转弯，保持相邻扫描有足够环境结构重叠。
雷达应基本水平，运动基本处于同一平面；尽量返回起点，观察闭环后墙面是否对齐、有无双层墙或扭曲。

## 5. Nano 终端 B：检查地图与位姿

另开 Nano 终端，先执行第 2 节的环境命令，再执行：

```bash
ros2 topic echo /map --once --no-arr --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

第一条应输出地图信息，包括 `resolution: 0.05` 和非零的宽高。
第二条会持续输出车体中心相对地图的平面位姿，检查后按 `Ctrl+C` 结束查询，不影响终端 A 的建图。
若提示 TF 暂不存在，可等待初始扫描处理；持续无地图或 TF 时回到终端 A 查看错误。

本模式的 TF 关系是：

```text
map -> odom -> base_link -> pavo2s_frame
```

对外输出的是**车体中心** `base_link` 的二维位姿，内部仍以雷达帧跟踪。
地图原点不会因此自动变成车体初始中心。检查安装关系可另行运行：

```bash
ros2 run tf2_ros tf2_echo base_link pavo2s_frame
```

应看到平移约 `[-0.2143195, 0, 0]`、绕 z 轴约 ±180°。
若已有外部节点发布相同的安装 TF，建图启动时加 `publish_lidar_tf:=false`，避免重复发布。

### 可选：在 PC 的 RViz 中看地图

PC 与 Nano 应处于能够进行 ROS 2 DDS 通信的网络中。PC 已构建安装 `f1_mapping_bringup` 并安装 RViz 时，
在 **PC 图形桌面终端**执行：

```bash
source /opt/ros/humble/setup.bash
source /home/bboss/new-nano/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2 -d "$(ros2 pkg prefix --share f1_mapping_bringup)/rviz/mapping.rviz"
```

PC 只运行 RViz，建图仍由 Nano 上的终端 A 运行。PC 环境也需要可用的 `rmw_cyclonedds_cpp`。
配置中的 Fixed Frame 为 `map`，地图话题为 `/map`，扫描话题为 `/pavo2_scan`。
若 Nano 本机可见地图而 PC 不可见，先在 PC 用 `ros2 topic list` 检查发现情况，再核对域编号和网络配置。

## 6. Nano 终端 B：保存地图

保持终端 A 的建图运行，在已加载第 2 节环境的 **Nano 终端 B**执行。
以下保存和导出命令在同一个终端中连续操作，保留 `mapping_snapshot` 变量。

### 6.1 保存 Cartographer 状态

```bash
mkdir -p /home/nvidia/f1_mapping_ws/maps
mapping_snapshot="/home/nvidia/f1_mapping_ws/maps/lidar_$(date +%Y%m%d_%H%M%S)"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${mapping_snapshot}.pbstream', include_unfinished_submaps: true}"
ls -lh "${mapping_snapshot}.pbstream"
```

确认响应中的 `status.code` 为 `0`，且 `.pbstream` 文件存在、大小非零，再继续。
保存路径由 Nano 上的 Cartographer 写入，因此这里使用 Nano 的绝对路径。
此操作包含当前未完成的子图，不结束轨迹，保存后仍可继续建图。

### 6.2 导出栅格地图

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "${mapping_snapshot}.pbstream" \
  -map_filestem "${mapping_snapshot}" \
  -resolution 0.05
ls -lh "${mapping_snapshot}.pbstream" "${mapping_snapshot}.pgm" "${mapping_snapshot}.yaml"
```

| 文件 | 用途 |
| --- | --- |
| `.pbstream` | Cartographer 状态快照 |
| `.pgm` | 栅格地图图像 |
| `.yaml` | 栅格地图的分辨率、原点等信息 |

如果提示找不到转换程序，用 `ros2 pkg executables cartographer_ros` 查看安装情况；
先保留已成功保存的 `.pbstream`，待补齐转换工具后再导出。
这是当前时刻的地图快照；若区域探索或闭环尚未完成，后续地图仍可能变化。

## 7. 结束本次建图

1. 停止设备运动，等待最后一批扫描处理，并按第 6 节保存所需地图、确认文件存在。
2. 回到 Nano 终端 A，按 `Ctrl+C` 退出建图。
3. 关闭单独打开的 RViz 或 TF 查询终端。雷达服务可继续保留，供下次建图或现有界面使用。

下次启动仍从第 2 节开始。当前启动命令会新建一张地图，不会自动读取上次的 `.pbstream`。

## 常见问题速查

| 现象 | 优先检查 |
| --- | --- |
| 找不到 `f1_mapping_bringup` | 是否在 Nano 构建成功，是否加载了新工作空间的 `install/local_setup.bash` |
| 找不到 Cartographer 或相关可执行文件 | 旧依赖工作空间是否正确加载，`ros2 pkg executables cartographer_ros` 是否有输出 |
| `scan_check` 超时或话题没有发布者 | 雷达服务、域编号、DDS 环境；查看驱动日志 |
| 提示缺少 IMU 或外参 | 是否误用了 `mapping.launch.py`；纯雷达使用 `lidar_mapping.launch.py` |
| 帧名不符或 TF 持续失败 | 消息实际帧是否为 `pavo2s_frame`，`base_to_lidar_planar` 是否运行，是否存在旧 SLAM 或其他 TF 发布者冲突 |
| SSH 启动 RViz 提示显示错误 | 使用默认不带 RViz 的建图命令，在图形桌面单独运行 RViz |
| 有扫描但地图扭曲、双层墙明显 | 安装是否水平、运动是否过快、环境结构是否足够、时间戳是否正常，以及 Nano 是否处理积压 |

算法配置和已有实机反馈见[纯雷达配置说明](src/f1_mapping_bringup/README_LIDAR.md)。
本文根据仓库当前配置整理，不代表已重新在 Nano 上完成整套启动验收。
