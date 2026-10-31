# Nano：阶段 4 矩形四边自动清洁一周部署与验收

文档核对：2026-10-31。本页适用于矩形任务；大 L 使用[独立任务入口与地图](../f1_l_cleaning/README.md)。通过 Qt App 操作时，自动任务使用“停止任务”“暂停任务”或“取消任务”；顶部红色“停止手动输出”只释放网关的手动租约。App 建图、选图和共享定位使用[受管入口](../f1_operator_gateway/README.md)，与下述完整 bringup 择一运行。

当前任务使用冻结地图定位，默认 `start_policy=left_ray`：沿船体 +y 射线选墙并锁定最近投影点，关刷旋转、前后移动和侧移进入旧版起始范围；不做路径搜索或环境碰撞检查。随后复用旧版沿墙/换边。矩形仅补齐起始边前段，L 形到终点停且不返回、不补洗。`COMPLETE` 表示路线执行完成，覆盖缺口单独记录。

船体按长 50 cm、宽 45 cm 建模，左刷稳定中心墙距为 32–36 cm，目标 34 cm。相邻换边仍使用中心距前墙 30 cm 停止、0.5 s 后退右推和地图航向反馈右转，转向容差 ±20°，转向和整段换边时间超时默认关闭。矩形仅通过相邻转弯返回起始边补前段，L 形不返回、不补洗。本文覆盖软件预览和现场记录要求；软件完成不等于已归档实船清洁效果证据。

## 1. 同步和构建

两个清洗包已合并为 `f1_pool_cleaning`。Nano 第一次升级必须先按 [迁移说明](MIGRATION.md) 停止旧节点、移走两个旧源码包和对应 build/install 目录；普通 rsync 不会自动删除它们。

源码传输与基础依赖按 [Nano 从零部署指南](../../DEPLOY_NANO.md)。2026-10-30 定位已拆至 `f1_localization`，首次升级必须按[定位拆包升级](../../DEPLOY_NANO.md#定位拆包升级)同步新增包、传播旧文件删除并重新生成安装目录，不能只同步本清洗包。

统一启动入口依赖 `f1_localization` 和 `f1_nano_bridge`；`--packages-up-to f1_pool_cleaning` 会一起构建。依赖通过 rosdep 安装。

地图路径为 `/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream`，同名 YAML 和 PGM 也直接放在 `maps/` 下；YAML 的 `image` 使用同目录 PGM 文件名。PC 单独同步该地图的示例：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/maps'
rsync -av --ignore-existing \
  /home/bboss/new-nano/surface_ws/maps/lidar_20261021_145054.{pbstream,yaml,pgm} \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/
```

Nano 构建、离线测试：

```bash
cd /home/nvidia/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_pool_cleaning
source install/local_setup.bash
ros2 run f1_pool_cleaning pool_demo
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/f1_pool_cleaning/test/cyclonedds_localhost.xml"
colcon test --executor sequential \
  --packages-select f1_localization f1_pool_cleaning --return-code-on-test-failure
colcon test-result --verbose
```

关闭测试终端后，每个实景终端加载（不要继承上述测试专用 DDS 配置）：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

不要同时运行旧 `wall_controller` / `rectangle_controller`、第二个任务控制器或另一份建图/map TF 发布者。雷达沿用已有 `/pavo2_scan` 驱动，无需转动安装或更改 yaw。

## 2. 推荐：统一启动

完成同步和构建后，下面的快捷脚本在每个终端自动加载 ROS 与当前工作空间环境，域编号默认 33，RMW 默认 CycloneDDS；已设置的环境值会保留。雷达 `/pavo2_scan` 仍使用已有驱动。

终端 A：

```bash
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano
```

这条命令统一启动冻结地图、定位管理器、bridge、墙面感知和清洗控制器。定位启动时不会在没有初始位姿时创建轨迹；默认地图为 `maps/lidar_20261021_145054.pbstream`。它启用串口、运动、主电机方向确认和第五路许可，但清洗任务保持 `IDLE`，等待 RViz2 标定定位并手动 `start`。

脚本不自动判断已有节点。若定位和 bridge 已经运行，改用：

```bash
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano bringup \
  start_localization:=false start_bridge:=false
```

只有其中一个已运行时，只传对应的一个 `false` 参数。跳过定位时复用现有定位轨迹，不创建第二条轨迹；跳过 bridge 时沿用现有 bridge 的输出许可配置。启动前应退出旧清洗控制器。若仅已有雷达静态 TF，传 `publish_lidar_tf:=false`。

若需要自动打开 RViz2 配置，在终端 A 的命令后加 `bringup start_rviz:=true`。在 RViz2 中确认 Fixed Frame 为 `map`，显示 Map、LaserScan、TF，使用 `2D Pose Estimate` 标定初始位置和船头方向。确认 `/pool_cleaning/localization_status` 的 `state` 为 `LOCALIZATION_READY` 后，终端 B 再查看状态并启动任务：

```bash
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano status
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano start
```

没有 `LOCALIZATION_READY` 时 `/pool_cleaning/start` 会拒绝申请任务租约。定位跳变、TF/雷达过期或轨迹结束会关闭输出并进入 `PAUSED`；检查原因后调用 `cancel`，重新标定初始位姿，再次 `start`。

`start` 返回后检查 `lease_held=true`、`start_edge_id` 和实际动作阶段。初始阶段为 `APPROACH_START → ALIGN_START / MOVE_START_ALONG / MOVE_START_SIDE → ACQUIRE_WALL`，随后进入旧版 `APPROACH_WALL/FOLLOW/CORRECT_HEADING`。已在范围内时不离墙。相邻换边仍为 `EDGE_STOP → MOVE_INWARD → TURN_TO_NEXT_EDGE → APPROACH_NEXT_EDGE`。L 形终点不返回；矩形只补起始边前段。`coverage_complete` 独立于路线 `COMPLETE`，缺口只显示不追加补洗。

暂停、取消仍在终端 B 执行；快捷脚本尚无 `stop` 子命令，停止任务使用 ROS 服务：

```bash
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano pause
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano cancel
# 已加载运行环境的终端：
ros2 service call /pool_cleaning/stop std_srvs/srv/Trigger '{}'
```

终端 A 按 `Ctrl+C` 会退出本次统一入口启动的节点；通过跳过参数复用的外部节点继续运行。

已加载环境时，也可直接使用 ROS 启动入口；它默认无硬件输出，实机显式加 `output_enabled:=true`：

```bash
ros2 launch f1_pool_cleaning pool_bringup.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream output_enabled:=true
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
```

下列各节保留分开启动和预览检查的用法。使用统一入口后，无需再重复启动这些节点或创建定位轨迹。

## 3. 单独加载冻结地图定位

终端 A：

```bash
ros2 launch f1_localization localization.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream
```

此入口加载冻结地图、静态雷达 TF、`/map` 栅格地图节点和定位管理器，跟踪/发布坐标系均为 `base_link`，不启动清洗或串口。默认不会创建新的定位轨迹；在 RViz2 发布 `/initialpose` 后，管理器调用 Cartographer 的 `/start_trajectory`，使用 `reference_trajectory_id`（默认 0）作为冻结参考轨迹。若已有同一份 `base_link → pavo2s_frame` 静态 TF，增加 `publish_lidar_tf:=false` 避免重复。

也可以通过服务提交同一个带 `map` frame 的初始位姿：

```bash
ros2 service call /pool_cleaning/set_initial_pose f1_interfaces/srv/SetInitialPose \
  "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}}"
```

`/pool_cleaning/localization_status` 会区分轨迹已创建和定位真正就绪；轨迹创建成功本身不代表定位正确。通过已知 A 墙、实际船头方向和尺量距离核对地图位姿，必要时在 RViz2 叠加 scan/map。错误朝向、地图外位姿、过期 TF、雷达断流和跳变都必须停留在 `LOCALIZATION_CHECK` 或 `PAUSED`，不能启动清洗。

定位和控制器必须传入同一 `map:=`。默认配置从地图推导 origin/axis/length/width、四边边界和定位范围，不需要手填旧地图尺寸；使用现场配置时应移除不再适用的显式几何覆盖。默认自动选边无需预先放在 `edge_0`；如需固定起始边，启动时加 `start_policy:=specified default_edge_id:=edge_0`。指定边仍会规划接近路径，不可达时暂停，不会自动改选其他边。

## 4. 无输出预览与矩形一周验收

终端 C：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream
```

终端 D：

```bash
ros2 topic echo /pool_cleaning/wall_observation --qos-reliability best_effort
# 另一个终端观察：
ros2 topic echo /pool_cleaning/status
ros2 topic echo /pool_cleaning/command_preview
```

人工将船放在地图中的可达空闲水域，使用 RViz2 发布初始位姿，待 `LOCALIZATION_READY` 后核对：

- `can_start=true`、`start_blockers=[]`，定位、地图位姿和原始扫描均新鲜。启动前 `current_edge_id` 可为空；规划后才检查所选边与 `edge_target`，清洗时墙必须在左侧。
- 左墙线在导航完成后重新获取；有效时距离与夹角应符合尺量。`front_observed=false` 不阻止自动任务开始，不手工修改该标记。
- `localization_state=LOCALIZATION_READY`。TF 不含协方差时，`pose_covariance_available=false` 是明确的能力限制。
- `output_enabled=false`、`lease_held=false`，预览 token 为空。

调用 `start` 后先检查规划、导航与找墙状态；预览不会驱动船体，需要合成反馈或人工移动才能推进流程。以下沿墙输出预期适用于已进入贴墙清洗阶段（不连接执行器）：

```bash
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
```

| 测试 | 预期 |
|---|---|
| 已进入 32–36 cm 且夹角在 ±10° 内 | 刷盘 30%，主轮按航向误差纠偏；第五路关闭 |
| d 从 0.34 增到 >0.36，再回到 0.34，再降到 <0.32 | 第五路依次关、+30%、关、-30%；32–36 cm 范围内均关闭 |
| 夹角超过 +10° / -10° | 刷盘关；四个主电机 LEFT / RIGHT 25%、BOTH 持续纠偏，没有脉冲/中位等待 |
| 误差在 3° 到 10° 之间 | 继续 LEFT / RIGHT 纠偏，同时开启刷盘 30% |
| 误差恢复到 ±3° 内，包括等号 | 主轮 STOP，刷盘保持 30% |
| 纠偏方向的空间标志为 false | 仍按夹角纠偏，不暂停 |
| 墙线在 ±70° 内、相邻有效观测角度变化 ≤30° | 拟合质量满足条件时继续跟踪；刷盘仍只在 ±10° 内开启 |
| 新鲜扫描暂时未拟合到左墙 | `WAIT_WALL`、全部输出关闭，最多等待 1.5 秒；恢复后自动继续纠偏/刷洗 |
| 丢墙超过 1.5 秒 | `PAUSED`，`reason` 包含感知原因，需 cancel/start |
| 可见左墙被遮挡，仅剩后方短墙 | 在恢复窗口内保持墙关联约束，禁止将短墙重标为左墙继续纠偏 |
| 当前边投影达到 `edge_stop_m` | `EDGE_STOP`，随后执行一个停止指令周期、后退右推和右转；状态切换到下一条边 |
| 执行四条边及起始边前段 | 路线执行结束进入 `COMPLETE`，所有输出关闭；`coverage_complete` 和 `remaining_contact_intervals` 单独显示覆盖缺口，不追加补洗 |
| 停止雷达或定位 | PAUSED；反馈恢复不自动续跑 |
| 地图位姿正常更新 | 更新边进度和前墙距离，并通过 `EdgeTarget` 维持当前目标墙关联 |
| 地图位姿发生超阈值跳变 | 定位管理器锁存 `PAUSED`，控制器关闭输出；反馈恢复后不自动续跑 |

本版没有 0.36 m 最小墙距暂停门槛；墙距偏小时由第五路向外纠正。超出 ±10° 可以启动任务，先纠偏再开刷。暂停后执行 `cancel` 和 `start`。

诊断单次启动后暂停：`ros2 topic echo /pool_cleaning/status --once --full-length`，查看 `phase`、`reason`、`wall_valid`、`wall_reason`。状态前部的 `left_wall_distance_cm` 是船中心到左墙的距离（厘米）；无效或过期时显示 `null`。`WAIT_WALL` 可以自动恢复；`PAUSED` 不会自行恢复。带地图自动任务初次启动不要求有效墙线，但必须有新鲜原始扫描；找墙和正式清洗阶段仍须核对墙线。

状态前部同时显示 `wall_angle_deg` 和 `direction`：0=STOP、3=LEFT、4=RIGHT。船头向左墙偏转时，应观察到负墙角和 RIGHT。遇到疑似反向，完整保存 `/pool_cleaning/status` 与 `/f1/control_state`，核对感知符号、请求方向和固件目标，不能只凭 `CORRECT_HEADING` 阶段名称推断方向。

当前默认 `association_timeout_sec=2.0` 长于 `wall_recovery_sec=1.5`。自定义参数时，应保持墙关联有效期大于恢复窗口与扫描时效之和，避免任务仍在等待恢复时墙身份先被清除。

## 5. 矩形一周实机测试

预览核对后，退出预览控制器，保持定位与感知来源一致。地图无法发现盲区新增障碍；本轮需要人工观察前方并保留遥控接管。0.30 m 是船中心到前方墙的换边触发阈值，需记录每个转移的漂移、0.5 s 后退右推位移和最终 `COMPLETE` 状态。当前直边推进依赖滚刷，配置不支持 `brush_percent=0`；如需无刷动作验收，应另行制定现场测试步骤，不能把禁用输出预览当作实船无刷完整一周。

单独终端启动唯一 bridge（串口路径按已有 bridge 配置；direction_confirmed 表示主轮 LEFT/RIGHT 硬件映射已经核对）：

```bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true
```

另一个终端启动实机模式，启动节点本身不启动任务；在检查终端另行调用 start：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream output_enabled:=true
# 另一个已加载环境的终端：
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
```

start 返回 acquire requested 仅表示已请求租约；看 status 的 `lease_held` 与 phase 确认实际启动。第五路按墙距输出 +30% / 0 / -30%，刷盘默认 30%。默认 `run_timeout_sec=0`，没有固定时长上限；靠墙、入口、边端点和换边均由反馈与地图进度决定。`brush_percent` 改变推进效果也改变刷洗强度，本版不闭环控制速度。

使用本次默认配置；不要套用历史 `diagnostics/single_side_remote_20261023/rectangle_current_side.yaml`，该文件仍保存旧 ±3° 与 8 秒限制。

停止：

```bash
ros2 service call /pool_cleaning/pause std_srvs/srv/Trigger '{}'
# 重新摆放、确认定位后：
ros2 service call /pool_cleaning/cancel std_srvs/srv/Trigger '{}'
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
```

遥控接管、bridge 租约撤销、串口状态失效或进程退出会停止本控制器输出。Nano 的 STOP 不能覆盖已经接管的遥控输入。

录制一次完整矩形一周：

```bash
ros2 bag record -o rectangle_one_loop_trial \
  /pavo2_scan /tf /tf_static /pool_cleaning/wall_observation \
  /pool_cleaning/localization_status /pool_cleaning/edge_target \
  /pool_cleaning/status /pool_cleaning/command_preview /f1/control_state
```

记录 `edge_0` 到 `edge_3` 的实际墙距、夹角、靠墙和入口调整时间、30% 刷盘是否能推进、第五路双向纠偏是否振荡、每次 `EDGE_STOP` 触发位置、后退右推位移、转向误差、每条边的 `cleaned_intervals`、最终 `completed_edges` 和未覆盖原因。刷尖约距中心 0.33 m，34 cm 作为名义中心值；实际接触压缩仍需实船测量。

一周任务的现场记录至少应包含：起始初始位姿、地图版本、实际选中起始边与 `run_id`，四条边及起始边补齐段的轨迹、简单靠近和相邻换边的停止原因与漂移，暂停/停止/取消/遥控接管记录，以及最终状态中的 `phase`、`completed_edges`、要求区间、已覆盖区间和剩余缺口。角部留白和短边有效清洗长度应单独标注，不能只凭最终 `COMPLETE` 声明全池面积已覆盖。
