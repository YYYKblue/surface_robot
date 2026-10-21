# 纯雷达 Cartographer 2D

当前优先使用此模式。仅订阅 `/pavo2_scan`，无需启动 `f1_imu_bridge` 或修改 F1 固件。
已有 `mapping.launch.py` 仍是雷达与 IMU 联合模式，纯雷达请使用 `lidar_mapping.launch.py`。
已有网关的 IMU 转发开关可保留，它不参与本模式。

## 当前坐标和输入

```text
map -> odom -> base_link -> pavo2s_frame
```

内部仍以 `pavo2s_frame` 跟踪扫描，对外 TF 输出为车体中心 `base_link` 的二维位姿。
启动入口默认根据 `config/lidar_planar.yaml` 发布 `base_link -> pavo2s_frame` 静态 TF：
用户已确认 x=-0.2143195 m、y=0、yaw=π。该文件仅描述平面安装关系，不读取 IMU 外参。
雷达实际高度仍未知；此模式将 `base_link` 定义为车体中心在扫描平面的投影，
TF 的 z/roll/pitch=0 是二维建模约定，不代表实测高度或倾角。真实三维外参仍保留在 `sensors.yaml` 中待测。
地图仍在固定的 `map` 坐标系中，修改输出参考点不会自动将地图原点设为车体初始位置。
内部轨迹/子图依然基于雷达跟踪帧；车体中心位姿请查询 `map -> base_link`，不能直接将内部雷达轨迹当作车体轨迹。
要求雷达基本水平、运动基本在平面内。只有固定的二维扫描面才能直接对应同一张平面地图。
运行时只保留一套 Cartographer，避免两套节点同时发布相同 map/odom 关系。
已有外部节点发布同一雷达安装 TF 时，启动加 `publish_lidar_tf:=false`，并确保其安装参数一致。
不要同时启动联合模式的 `sensor_tf.launch.py`。外部平面参数文件可用
`lidar_extrinsics_file:=/absolute/path/lidar_planar.yaml` 指定。

实际输入已观测为约 50.1 Hz、3500 点、280°、0.08°/点。
纯雷达初版开启在线相关性扫描匹配，采样比例 0.4，算法处理约 20 Hz；
雷达驱动仍保持 50 Hz。根据 Nano 的 CPU 使用和实际轨迹效果再调采样比例。
距离下限使用已观测消息中的 0.1 m，上限保留安装版 Cartographer 默认值。
消息声明的 100 m 量程不能视为实际测距验收结果。

## 同步与构建

PC：

```bash
scp -r /home/bboss/new-nano/f1_mapping_ws/src/f1_mapping_bringup \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/src/
```

Nano 构建终端：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
cd /home/nvidia/f1_mapping_ws
colcon build --symlink-install --packages-select f1_mapping_bringup
colcon test --packages-select f1_mapping_bringup --return-code-on-test-failure
colcon test-result --verbose
```

另开 Nano 终端，加载运行环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 先检查实际回波

雷达静止时运行：

```bash
ros2 run f1_mapping_bringup scan_check --samples 100
```

工具仅订阅原始扫描，不需要 TF 或 Cartographer。
输出有效距离比例、NaN/无穷大/越界点、扫描几何、时间异常和左右/正前方向的距离中位数。
方向以 `pavo2s_frame` 为准，不是机体方向；距离使用各方向附近 ±1° 内的有效点。
`issues=[]` 表示未发现这些消息问题，不代表测距精度已验收。
开阔环境中较多无穷大回波可能正常，不设置通用“有效比例必须大于多少”的阈值。

用已知距离的墙或物体核对距离，并在 RViz 中观察：静止时墙面是否稳定、形状是否对应实物、
左右方向是否与雷达坐标一致，移动时是否持续有足够结构重叠。

## 启动纯雷达建图

雷达服务保持现有 `pavo2-scan.service`；本入口只启动 SLAM，不重复启动雷达驱动。

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py
```

有图形桌面时可用 `use_rviz:=true` 同时打开地图；无图形的 SSH 终端保持默认 false。
检查结果：

```bash
ros2 topic echo /map --once --no-arr --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
# 上一条持续输出；按 Ctrl+C 结束查询后，再检查安装关系。
ros2 run tf2_ros tf2_echo base_link pavo2s_frame
```

启动时先保持雷达静止，观察初始子图，再缓慢平移、转弯，尽量返回起点检查闭环。
关注同一堵墙是否出现双层、直角是否扭曲、返回起点时结构是否对齐。
这些是地图效果检查，不是绝对定位精度证明。
录包重放可加 `use_sim_time:=true`，并由 rosbag 提供 `/clock`。
点云帧名当前固定为实测的 `pavo2s_frame`；若更换驱动帧名，应同步修改 Lua。

## 验证边界

PC 当前没有 Cartographer，本地已验证包构建、消息检查逻辑和启动动作组成。
2026-10-20 用户在 Nano 实机反馈（车体中心 TF 改动前）：纯雷达 Lua 已成功加载，轨迹 0 和子图 (0, 0) 已创建，
算法接收约 20.16 Hz 扫描，并发布 0.05 m/格的地图；用户反馈整体建图较稳定。
这些结果确认了基本运行链路，闭环误差、绝对尺寸及完整区域覆盖仍需现场验收。
车体中心输出改动需同步、重新构建并重启后，在 Nano 核对静态 TF 和 `map -> base_link`；不能沿用上述反馈视为该改动已实机验收。

## 保存当前地图快照

保持建图节点运行，在已加载同样 ROS 环境的 Nano 终端执行：

```bash
mkdir -p /home/nvidia/f1_mapping_ws/maps
mapping_snapshot="/home/nvidia/f1_mapping_ws/maps/lidar_$(date +%Y%m%d_%H%M%S)"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${mapping_snapshot}.pbstream', include_unfinished_submaps: true}"
```

确认服务响应 `status.code=0`，并且文件存在且非空：

```bash
ls -lh "${mapping_snapshot}.pbstream"
```

这一步包含当前未完成的子图，不结束轨迹；之后仍可继续建图。
导出同一快照的 5 cm 栅格图：

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "${mapping_snapshot}.pbstream" \
  -map_filestem "${mapping_snapshot}" \
  -resolution 0.05
ls -lh "${mapping_snapshot}.pbstream" "${mapping_snapshot}.pgm" "${mapping_snapshot}.yaml"
```

`.pbstream` 保留 SLAM 状态；`.pgm` 和 `.yaml` 是栅格图与坐标信息。
这是阶段备份，尚未完成区域探索或闭环时，不代表最终优化后的地图。
如果目标机缺少转换可执行文件，先保存 pbstream，再查看
`ros2 pkg executables cartographer_ros`，按目标机安装情况处理导出。
