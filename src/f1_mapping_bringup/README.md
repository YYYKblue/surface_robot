# F1 激光雷达与 GNSS 建图

Cartographer 以 PAVO2 的 `/pavo2_scan` 做局部扫描匹配。默认启用 WTRTK-982 主天线的水平位置约束。没有合格卫星观测时，激光仍可独立建图。

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py
# 有图形桌面时可同时打开 RViz：
ros2 launch f1_mapping_bringup lidar_mapping.launch.py use_rviz:=true
```

入口启动 Cartographer、栅格地图节点及雷达平面安装 TF；雷达驱动由项目内 [`f1_lidar_driver`](../f1_lidar_driver/README.md) 独立运行，默认由 `pavo2-scan.service` 管理。构建依赖会包含驱动，建图入口不会重复启动它。
默认输出 `map -> odom -> base_link -> pavo2s_frame`，栅格分辨率为 0.05 m。
安装参数由 `config/lidar_planar.yaml` 提供；z/roll/pitch=0 是二维建模约定，
实际雷达须基本水平，机器人运动须基本位于平面内。

- [部署、输入检查、启动与保存地图](README_LIDAR.md)
- [结合本包代码理解 Cartographer 建图原理](CARTOGRAPHER_PRINCIPLES.md)
- [工作空间日常启动步骤](../../START_LIDAR.md)

## 参数

| 参数 | 默认值 / 作用 |
| --- | --- |
| `scan_topic` | `/pavo2_scan` |
| `configuration_directory` | 本包的 `config/` |
| `use_rtk` | 默认 `true`。设为 `false` 使用纯雷达基线 |
| `configuration_basename` | 自动选择 `rtk_mapping.lua` 或 `pavo2_2d.lua`，也可指定自定义配置 |
| `lidar_extrinsics_file` | 本包的 `config/lidar_planar.yaml` |
| `publish_lidar_tf` | `true`；已有安装 TF 时设为 `false` |
| `resolution` | `0.05`，输出栅格的米/格 |
| `use_sim_time` | `false`；重放录包且提供 `/clock` 时设为 `true` |
| `use_rviz` | `false` |
| `rviz_config` | 本包的 `rviz/mapping.rviz` |

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py --show-args
```

## 无回波区域

建图使用普通扫描 `/pavo2_scan`。真实回波为射线经过的空间和回波终点提供栅格证据。原始零距离、无效值及超出雷达消息量程的读数跳过。某处从未被有效射线观测时保持未知，后续无回波也不会擦除先前已观测的栅格。整圈无有效回波时，即便 ANT1 的位置持续更新，也不会沿船的航迹生成自由空间。旧环境变量 `F1_MAPPING_BACKEND` 和 `F1_NO_ECHO_RADIUS_M` 已退出建图入口。已有地图的栅格保留当时的观测结果。

## RTK 位置融合

驱动在 `/f1/rtk/fix` 发布 ANT1 的 WGS84 位置。融合节点读取 `/f1/rtk/raw` 中的 GGA，以同一条报文的 UTC、解质量、卫星数和 HDOP 检查位置。默认接受质量码 1、2、4、5，至少 5 颗卫星，HDOP 在 (0, 3]。观测超过 0.5 秒、历元重复或连续位置跳变时会被过滤。连续三个合格位置通过检查后，以最高 2 Hz 发布水平位置约束。

`base_link → gnss_ant1` 的平移为 x = −0.191 m、y = +0.213 m。Cartographer 以 ANT1 为跟踪原点，船体姿态来自雷达扫描匹配。后端平移权重默认 0.5，旋转权重为零。千寻差分可提高 GGA 定位质量。硬件配置和质量判定见 [RTK 驱动](../f1_rtk_driver/README.md)。

新地图用首个采纳的 GGA 定义局部东、北坐标原点，再用同一时刻的天线雷达轨迹和 GGA 轨迹配准坐标系。默认两条轨迹均需达到 1 m 均方根展布，配准残差不超过 0.45 m。原地静止通常只有雷达建图，移动并满足条件后才开始写位置约束。配准得到的是场地坐标变换。船转向时仍由雷达估计船头方向。[融合参数](config/gnss_fusion.yaml)

```bash
ros2 topic echo --once /f1/rtk/position_constraint
ros2 topic echo --once /landmark_poses_list
ros2 bag record -o lidar_gga /pavo2_scan /f1/rtk/raw /f1/rtk/fix /f1/rtk/status /f1/rtk/corrections
```

第一条命令检查已筛选的位置，第二条可查看完成配准的地理原点标记。正常保存地图会将地理参考和轨迹存入同一份 `.pbstream`，栅格由子图导出。重放时播放扫描和原始 GGA，使用 `use_sim_time:=true` 和 rosbag 的 `--clock`。纯雷达对照可传 `use_rtk:=false`。[固定地图定位](../f1_localization/README.md)

## WT102 航向辅助

先加载工作空间，确保 PAVO2 雷达和 RTK 驱动运行。启用后默认打开板载 UART 并接收 WT102 的 Z 轴角速度。GGA 仍只约束天线水平位置。下列命令用于独立建图和录制输入，分两个终端执行。

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py use_wt102_heading:=true
ros2 bag record -o wt102_survey /pavo2_scan /f1/rtk/raw /f1/wt102/sample /tf_static
```

`use_wt102_heading` 默认关闭。若 UART 已由独立采集节点打开，建图时加 `start_wt102_driver:=false`。串口可通过 `wt102_port:=/dev/ttyTHS1` 指定。固定地图定位继续使用现有启动流程。App 受管建图在启动 `operator_system.launch.py` 时传同一个 `use_wt102_heading:=true`。

二维前端以上一次雷达匹配位姿为起点，按测量时间积分机体 Z 角速度，用于扫描内点的姿态外推及下一次扫描匹配。预测需要同一数据流内的连续样本，间隔超过 50 ms 切断积分，末次测量最多外推 20 ms，历史窗口 2 s。失去覆盖时自动使用 Cartographer 原有角速度预测，恢复后从新的匹配位姿起算。原生 IMU 开关保持关闭。水平安装时绕 Z 的转角与传感器在船上的平移位置无关。横摇或纵摇会影响此近似。模块相对上电角度用于录包对照。静置测得的 Z 角速度均值可填入 `gnss_fusion.yaml` 的 `wt102_bias_rad_s`，默认 0 rad/s。

本包为建图可执行文件编入 ROS 2 Cartographer 2.0.9004 的外推器源文件，使用系统静态库中的其他组件。该文件保留上游许可证。其他 Cartographer 版本可构建原始模式，启用 WT102 时会报告版本要求。[上游源码](https://github.com/ros2/cartographer/blob/1489fc933b6e0b1da936ca08af462abe559dd02e/cartographer/mapping/pose_extrapolator.cc)

同一录包分别启用和关闭 `use_wt102_heading`，每次重新启动建图，播放相同的三个输入话题。使用仿真时钟时不会启动串口驱动。安装 TF 由建图入口发布。

```bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py use_sim_time:=true use_wt102_heading:=true
ros2 bag play wt102_survey --clock 100 --topics /pavo2_scan /f1/rtk/raw /f1/wt102/sample
```

对比同一已知转角、回到起点时的朝向，以及往返岸线的重影宽度。现场接线、单独串口采集和测量类型见[WT102 驱动](../f1_wt102_driver/README.md)。

## 更新旧安装

旧版全量 IMU 联合入口和对应外参已经移除，当前 WT102 航向辅助通过标准建图入口启用。
从旧版本更新时，源码同步必须传播删除；仅用 `scp -r` 覆盖会留下旧文件。
使用 [部署说明](README_LIDAR.md) 中限定在本包目录内的 `rsync --delete`，
并在构建前清理本包的旧 `build/`、`install/` 子目录。

`f1_nano_bridge` 内部的 `nano_ros_bridge.imu` 仅提供原生 IMU 转换库，
硬件桥直接从串口接收并发布遥测；纯雷达建图不需要启动硬件桥。
Lua 中的 `imu_sampling_ratio` 是上游配置解析器必读字段，
`TRAJECTORY_BUILDER_2D.use_imu_data = false` 才是禁用 IMU 输入的开关。

## 验证边界

本机可进行配置包构建、启动参数检查和现有离线测试。
完整 SLAM 需要目标机的 `cartographer_ros`、真实扫描和 TF；
配置包构建成功不能代替 Lua 加载、闭环效果或实机精度验收。
