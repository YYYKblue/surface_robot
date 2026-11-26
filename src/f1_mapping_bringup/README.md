# F1 激光雷达与 GNSS 建图

Cartographer 以 PAVO2 的 `/pavo2_scan` 做局部扫描匹配。默认启用 WTRTK-982 单点定位与双天线定向的后端约束。没有合格卫星观测时，激光仍可独立建图。

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

## 湖心整圈无回波建图

在开阔水域选用 `open_water`。船中心的 RTK 固定位置与双天线固定航向决定扫描在地图上的位置。雷达持续发帧时，即使整圈都没有测距回波，沿航迹经过的水域也能逐步画入地图。此模式可从岸边启动后驶向湖心，在同一张图中记录岸边回波与湖心观测。

```bash
source install/setup.bash
ros2 launch f1_mapping_bringup lidar_mapping.launch.py mapping_backend:=open_water no_echo_radius_m:=3.0
```

独立命令与 App 管理的建图进程择一运行。通过 App 建图时，让启动网关的进程继承 `F1_MAPPING_BACKEND=open_water`，再按原来的“开始建图”“保存地图”操作。Nano 的 `f1-operator.service` 可通过 systemd drop-in 的 `[Service]` 下设置 `Environment=F1_MAPPING_BACKEND=open_water`。可选的 `F1_NO_ECHO_RADIUS_M` 控制无回波距离。首次部署需同时更新并运行本版雷达驱动、建图包和 App 网关。默认模式为 `cartographer`。

| 现场观测 | 写入地图的结果 |
| --- | --- |
| 同一方向连续两帧采到原始零距离，间隔不超过 0.1 秒 | 从雷达沿该方向标记推定空闲，默认最远 3 米，碰到已有占据格即停止 |
| 有有效回波 | 沿束标记自由空间，30 米内的回波终点写为占据，超过 30 米只记录截断的自由段 |
| 无效或被滤掉的读数、扫描中断 | 相应方向没有新增观测 |
| GGA 失去固定解、航向无效，或扫描与位姿时间相差超过 0.12 秒 | 暂停写入，条件恢复后沿用原地理原点继续 |

`/pavo2_scan/water` 的 `+inf` 表示设备原始零距离，`NaN` 表示无效。经现有卫星数、HDOP、时效与连续性筛选的 GGA 质量 4 和 HPR 质量 4 才会进入此模式专用的 `/f1/rtk/open_water_landmarks`。合格位姿最高 5 Hz，实时地图有更新时以 1 Hz 发布。复盘这一路径需额外录制 `/pavo2_scan/water`，普通扫描中的零值不能区分上述情况。

空闲是针对水平扫描面的推定，水下障碍、低矮浮物、遮挡或弱反射都可能漏检。3 米是可调的初始推定距离，应以本船实测选择。超过已观测范围的栅格保持未知。栅格按航迹的包围范围扩展，内部分配上限为 16777216 格、单边最多 16384 格。大范围测区可在开始前调大 `resolution`。达到上限的新观测会跳过并记录错误，已建部分仍可保存。

`/write_state` 将这张栅格、船的位姿序列和地理原点一起保存为 `.pbstream`，现有导出工具从同一快照生成 PGM 和 YAML。保存后可重新加载显示；湖心全无回波时的固定地图重定位和自主航行属于独立能力。

## 单点位置与双天线航向融合

Nano 上先核对两路采集服务，再用 App 建图。独立调试可用下列建图命令。建图与固定地图定位各占用一套 Cartographer，运行其中一种模式。

```bash
systemctl is-active pavo2-scan.service f1-rtk.service
# 仅在独立建图时执行
ros2 launch f1_mapping_bringup lidar_mapping.launch.py
```

`/f1/rtk/base_fix` 给出船中心位置，`/f1/rtk/bow_heading` 已含 42° 安装补偿。`/f1/rtk/raw` 的 GGA 用于把船中心消息的接收时间戳关联到航向 UTC。三话题齐全且消息类型为当前 `f1_interfaces` 的录包可直接回放。适配器接受 GGA 质量 1、2、4、5，要求卫星数至少 5、HDOP 在 (0, 3]，且 HPR 质量为 4、航向有效。超过 0.5 秒的观测丢弃。连续三对观测通过相邻位置和转角检查后开始提供约束。

首个采用的船中心经纬度定义本场地局部东、北坐标的原点。每次观测给出船体相对于该原点的位置以及从东逆时针量起的船头角。适配器将逆变换作为一个带方向的地标发布到 `/f1/rtk/landmarks`。Cartographer 按消息的 `base_link` 及雷达安装 TF 换算到跟踪坐标系，并将其与激光轨迹一起优化。船中心高度 NaN 不参与平面计算。激光队列独立于地标队列。

接收机按 20 Hz 输出时，适配器逐对筛选，每秒最多采纳 2 个约束，避免反复计入相关的单点误差。位置和旋转权重分别为 0.5、8.0，是单点定位阶段的初始值。建图和定位共用 [gnss_fusion.yaml](config/gnss_fusion.yaml)。独立启动入口可传 `gnss_config:=/绝对路径/参数.yaml`，纯雷达对照使用 `use_rtk:=false`。

保存地图前，在以下两处确认已产生观测及优化后的非空地标。地理原点及其相对于地图的位姿随正常 `/write_state` 写入同一个 `.pbstream`。刚启动、尚未完成首次优化的地图可能只含雷达结果。

```bash
ros2 topic echo --once /f1/rtk/landmarks
ros2 topic echo --once /landmark_poses_list
```

现场验证时同步录制 `/pavo2_scan`、`/tf`、`/tf_static` 及 RTK 六个采集话题。用同一段静止、直行、转向、回到标记点的数据分别运行融合与纯雷达，比较闭合误差、墙线重影及转向时的位姿延迟。回放时只播放采集话题，让新启动的 Cartographer 生成自己的地图 TF；若运行中已发布安装 TF，避免重复播放该 TF。接收机失锁时检查新地标停止、雷达位姿继续更新。固定地图启动和就绪条件见[定位说明](../f1_localization/README.md)。

## 更新旧安装

本次已删除雷达与 IMU 联合入口、联合 Lua、联合传感器外参和对应加载代码。
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
