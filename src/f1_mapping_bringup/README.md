# F1 雷达与 IMU 建图配置包

**当前优先路线为纯雷达建图，使用 `lidar_mapping.launch.py`，见 [纯雷达说明](README_LIDAR.md)。**
下文的 `mapping.launch.py` 保留为雷达与 IMU 联合模式。

输入：`/pavo2_scan` (`sensor_msgs/msg/LaserScan`)、
`/f1/imu/data_raw` (`sensor_msgs/msg/Imu`)。
默认启动 Cartographer 2D 与栅格地图节点；RViz 可选。
雷达驱动与 F1 串口网关由外部运行，本包不打开串口、不发送运动指令。

当前 `firmware`（back2，协议 v2） 固件使用同一工作空间的 [ROS nano_bridge](../nano_bridge/README.md) 直接发布 F1 IMU。原 `f1_imu_bridge/imu.launch.py` 属于旧 ZMQ 网关链路，不应与新桥同时启动。两者输出均为 `/f1/imu/data_raw`、`imu_link`；IMU 安装外参仍需按下文实测配置。
地图快照保存步骤见 [纯雷达说明](README_LIDAR.md)；录包工具尚未封装。
启用 IMU 后必须持续提供有效 IMU 数据，只有雷达数据不足以运行本配置。
`package.xml` 中维护者邮箱为占位值，许可证尚未指定；对外发布前应填写实际信息。

## 外参与坐标

`config/sensors.yaml` 的 `null` 必须替换为实际安装测量值。
雷达 x/y/yaw 沿用现有工程；z 未知，roll/pitch 暂按水平安装，均需实测核对。
IMU 位姿全部待测。不得为了启动成功将未知值直接设为零。

本配置固定使用 `base_link`、`pavo2s_frame` 和 `imu_link`。
传感器消息的 `header.frame_id` 应与对应传感器帧一致。
IMU 的 tracking_frame 为 `imu_link`，输出机器人帧为 `base_link`。

```text
map -> odom -> base_link -> pavo2s_frame
                       -> imu_link
```

前两段由 Cartographer 发布，安装关系由本包或现有 TF 发布者提供。
已有相同安装 TF 时，使用 `publish_sensor_tf:=false`，避免重复发布。

## 构建与检查

```bash
source /opt/ros/humble/setup.bash
# 若 Cartographer 仅存在旧工作空间，先 source 对应 install/setup.bash。
cd ~/f1_mapping_ws
colcon build --symlink-install --packages-select f1_mapping_bringup
colcon test --packages-select f1_mapping_bringup
colcon test-result --verbose
```

另开终端，加载依赖环境及新工作空间后，可无设备查看启动参数：

```bash
source /opt/ros/humble/setup.bash
source ~/f1_mapping_ws/install/local_setup.bash
ros2 launch f1_mapping_bringup mapping.launch.py --show-args
```

填好外参且传感器持续发布后启动：

```bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1_mapping_bringup mapping.launch.py
```

启动 RViz 使用 `use_rviz:=true`。Nano 无显示器时保持默认 false。
独立检查安装 TF：

```bash
ros2 launch f1_mapping_bringup sensor_tf.launch.py
```

外部参数文件可用 `sensors_file:=/absolute/path/sensors.yaml` 指定。
可覆盖 `scan_topic`、`imu_topic`、`resolution`。
RViz 扫描话题仍需与修改后的 scan_topic 对应。
录包重放时设置 `use_sim_time:=true`，并通过 `ros2 bag play ... --clock` 提供时钟；
录包已含安装 TF 时关闭本包的安装 TF 发布。

## 依赖和验证边界

PC 开发环境可在缺少 Cartographer 时构建本配置包、运行配置测试和查看参数；
执行完整 SLAM 仍须安装或加载 `cartographer_ros`。
Nano 上先用 `ros2 pkg executables cartographer_ros` 确认存在
`cartographer_node` 与 `cartographer_occupancy_grid_node`。

Lua include 的配置与 Cartographer 算法默认值取自目标机安装版本。
PC 尚无 Cartographer，不能据配置包编译成功认定 Lua 已被算法加载或地图已生成。
IMU 发布必须核对单位、坐标轴、含重力的加速度和与雷达一致的时间基准。
