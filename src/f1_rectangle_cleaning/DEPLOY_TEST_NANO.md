# Nano：简化单边沿墙测试

更新：2026-10-23。±10° 内开滚刷，范围外四个主电机持续纠偏；第五路双向控制中心墙距 38–42 cm。取消转向空间等几何门控和脉冲等待。雷达保持原外参，前墙 80 cm 仍由已有地图定位计算，按当前船头方向选择前方短墙。

## 1. 同步和构建

源码传输与基础依赖按 [Nano 从零部署指南](../../DEPLOY_NANO.md)。不复制 PC 的 build/install/log。已有 Nano 工作空间可直接同步本次控制包和此前实测修复过的感知包：

```bash
rsync -av /home/bboss/new-nano/surface_ws/src/f1_rectangle_cleaning/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/src/f1_rectangle_cleaning/
rsync -av /home/bboss/new-nano/surface_ws/src/f1_wall_cleaning/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/src/f1_wall_cleaning/
```

依赖 `tf2_ros`、`cartographer_ros`，正常运行 rosdep 后构建即可。

地图副本路径约定为 `/home/nvidia/surface_ws/maps/20261021_145054/lidar_20261021_145054.pbstream`。PC 同步示例：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/maps/20261021_145054'
rsync -av --ignore-existing \
  /home/bboss/new-nano/backups/nano_maps/20261021_145054/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/20261021_145054/
```

Nano 构建、离线测试：

```bash
cd /home/nvidia/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_rectangle_cleaning
source install/local_setup.bash
ros2 run f1_rectangle_cleaning rectangle_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose
```

每个实景终端加载：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

不要同时运行旧 `wall_controller`、第二个任务控制器或另一份建图/map TF 发布者。雷达沿用已有 `/pavo2_scan` 驱动，无需转动安装或更改 yaw。

## 2. 加载冻结地图定位

终端 A：

```bash
ros2 launch f1_rectangle_cleaning rectangle_localization.launch.py \
  load_state_filename:=/home/nvidia/surface_ws/maps/20261021_145054/lidar_20261021_145054.pbstream
```

此入口加载冻结地图，配置纯雷达定位，跟踪/发布坐标系均为 `base_link`，不启动清洗或串口。默认暂停在“尚未创建新的定位轨迹”；不会用任意初始坐标假装定位成功。若已有同一份 `base_link → pavo2s_frame` 静态 TF，增加 `publish_lidar_tf:=false` 避免重复。

终端 B 创建定位轨迹（此命令只启动定位）：

```bash
ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory \
  "{configuration_directory: '/home/nvidia/surface_ws/install/f1_rectangle_cleaning/share/f1_rectangle_cleaning/config', configuration_basename: 'rectangle_localization.lua', use_initial_pose: false, initial_pose: {orientation: {w: 1.0}}, relative_to_trajectory_id: 0}"
ros2 run tf2_ros tf2_echo map base_link
```

`use_initial_pose=false` 依赖 Cartographer 对冻结地图的重定位，矩形环境可能匹配错误或需要时间收敛。不能把新轨迹建立成功当作定位正确。通过已知 A 墙、实际船头方向和尺量距离核对地图位姿，必要时用 RViz 叠加 scan/map。若使用手工初值服务，`use_initial_pose=true` 的位姿是相对指定参考轨迹的变换，不能直接把任意 map 坐标塞入；需按所用 Cartographer 版本和参考轨迹换算。未能建立正确定位时先处理定位，不启动清洗。

源地图和参数矩形必须匹配。当前长宽是约 2.883 × 1.520 m 的墙带估计；换地图后需重新给出 origin/axis/length/width。可沿任意一侧长墙、朝池坐标 +x 或 -x 行驶，按船头方向选前方短墙，左墙仍以实时雷达观测为准。

## 3. 无输出预览验收

终端 C：

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py
```

终端 D：

```bash
ros2 topic echo /rectangle_cleaning/wall_observation --qos-reliability best_effort
# 另一个终端观察：
ros2 topic echo /rectangle_cleaning/status
ros2 topic echo /rectangle_cleaning/command_preview
```

人工放船与左墙平行，中心距左墙约 0.40 m，地图前缘净空大于 0.80 m，核对：

- 左墙 `valid=true`，距离与夹角符合尺量；`front_observed=false` 是现有安装的预期，不修改该标记。
- `feedback_error` 为空，地图 `map_front_clearance_m` 与实际船头到短墙距离相符。TF 不含协方差，因此 `pose_covariance_available=false` 是明确的能力限制。
- `output_enabled=false`、`lease_held=false`，预览 token 为空。

调用 `start` 后人工小幅移动/摆角观察（不连接执行器）：

```bash
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
```

| 测试 | 预期 |
|---|---|
| 平行 | STOP + 刷盘 15%，主轮不发前进 |
| d 从 0.40 增到 >0.42，再回到 0.40，再降到 <0.38 | 第五路依次关、+15%、关、-15%；38–42 cm 范围内均关闭 |
| 夹角超过 +10° / -10° | 刷盘关；四个主电机 LEFT / RIGHT 25%、BOTH 持续纠偏，没有脉冲/中位等待 |
| 误差恢复到 ±10° 内，包括等号 | 主轮 STOP，当帧恢复刷盘 15% |
| 纠偏方向的空间标志为 false | 仍按夹角纠偏，不暂停 |
| 地图前缘净空达到 ≤0.80 m | STOPPED、主轮/刷盘/第五路全部关闭；不出现第二边任务 |
| 停止雷达或定位 | PAUSED；反馈恢复不自动续跑 |
| 地图位姿更新或跳变 | 接受更新，重新计算前墙距离；不做额外地图墙身份验证 |

本版没有 0.36 m 最小墙距暂停门槛；墙距偏小时由第五路向外纠正。超出 ±10° 可以启动任务，先纠偏再开刷。暂停后执行 `cancel` 和 `start`。

## 4. 单边实机测试

预览核对后，退出预览控制器，保持定位与感知来源一致。地图无法发现盲区新增障碍；本轮需要人工观察前方并保留遥控接管。0.80 m 只是关闭输出阈值，需记录之后的惯性滑行。

单独终端启动唯一 bridge（串口路径按已有 bridge 配置；direction_confirmed 表示主轮 LEFT/RIGHT 硬件映射已经核对）：

```bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true
```

另一个终端启动实机模式，启动节点本身不启动任务：

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py output_enabled:=true
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
```

start 返回 acquire requested 仅表示已请求租约；看 status 的 `lease_held` 与 phase 确认实际启动。第五路按墙距输出 +15% / 0 / -15%，刷盘默认 15%。默认 `run_timeout_sec=0`，没有固定时长上限；如需限时试验可另设正值。`brush_percent` 改变推进效果也改变刷洗强度，本版不闭环控制速度。

使用本次默认配置；不要套用历史 `diagnostics/single_side_remote_20261023/rectangle_current_side.yaml`，该文件仍保存旧 ±3° 与 8 秒限制。

停止：

```bash
ros2 service call /rectangle_cleaning/pause std_srvs/srv/Trigger '{}'
# 重新摆放、确认定位后：
ros2 service call /rectangle_cleaning/cancel std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
```

遥控接管、bridge 租约撤销、串口状态失效或进程退出会停止本控制器输出。Nano 的 STOP 不能覆盖已经接管的遥控输入。

录制一次完整单边：

```bash
ros2 bag record -o single_side_trial \
  /pavo2_scan /tf /tf_static /rectangle_cleaning/wall_observation \
  /rectangle_cleaning/status /rectangle_cleaning/command_preview /f1/control_state
```

记录实际墙距、夹角、15% 刷盘是否能推进、第五路双向纠偏是否振荡、纠偏次数、停车触发位置和最终滑行距离。刷尖原几何距中心 0.33 m，因此中心墙距 0.40 m 不保证刷毛接触；本版不自动向接触压缩目标推进。
