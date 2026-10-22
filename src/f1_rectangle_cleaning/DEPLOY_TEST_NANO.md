# Nano 同步与纯雷达验证步骤

更新：2026-10-21。按用户选择，本阶段使用纯雷达反馈，不引入 IMU 航向控制。
本包当前没有 IMU 订阅，无需为此修改运行参数；保留其他包现有 IMU 功能。

本轮目标：部署预览节点，验证真实墙距、墙面夹角和前方净空，并保存可回放数据。
本轮不启动自动运动；固定地图定位与局部速度输入尚未接通，任务 start 被拒绝属于预期。
以下命令由用户在对应终端执行；没有自动连接或部署到 Nano。

**2026-10-22 实景更新：用户确认约 280° 雷达视场的盲区位于船头。** 当前安装下，前方观测可能持续为 false，不能按本文原先的前方净空测试预期直接通过。
本轮优先验证左墙距离/夹角与实际视场。`front_observed=false` 时 `front_clearance_m=0` 不代表障碍贴在船头；前方未知，不允许前进。
若安装可调整，优先让前进方向进入实际扫描范围，再按真实安装更新外参并验证定位。若安装不可调整，后续需评估可见短墙片段/固定地图定位给出的端部距离估计；该估计不能作为前方盲区没有新障碍的证明，不能简单把 front_observed 改成 true。
盲区移到船尾也不等于完整转向空间已检查；掉头必须单独验证。当前保留原前方未知与转向空间不足时的拒绝逻辑。

用户已确认安装朝向可以调整，当前优先采用实际调整安装的方案；尚未收到已完成调整的反馈，因此代码中的外参暂不改变。

### 先调整实际视场，再继续部署测试

1. 停止当前感知/建图进程，关闭设备电源后调整雷达安装。目标是盲区朝船尾、船头和左墙在有效扫描范围内。若当前朝向确实为已记录的 yaw=pi，水平旋转雷达 180° 对应新 yaw 约为 0；不是将整船转 180°。
2. 尽量保持雷达测量原点的位置不变，并记录实际转动角度及光心相对 base_link 的新 x/y。仅在光心位置确实未变时，才能保留原 x=-0.2143195 m、y=0。
3. 按实物更新 `src/f1_mapping_bringup/config/lidar_planar.yaml` 的 x_m/y_m/yaw_rad，以及 `src/f1_rectangle_cleaning/config/perception.yaml` 的 lidar_x_m/lidar_y_m/lidar_yaw_rad。重新同步构建并重启相应节点。不要先改软件角度假装盲区已移动；其他旧单墙入口如再使用，也需要核对各自的雷达外参。
4. 静止摆在前方墙面可见的位置，检查墙距/夹角以及 front_observed。期待前方有效回波覆盖足够时 front_observed=true；若仍为 false，继续检查真实遮挡、无效回波和变换，不取消未知区域保护。原始扫描角度范围属于传感器坐标，调整安装后它可能完全不变，不能单凭 angle_min/angle_max 判断身体前方已可见。
5. 新外参下重新核对已有地图定位。原地图先保留，是否需要重建以匹配验证为准。船尾盲区仍可能导致左右转向空间标志 false，这需要后续独立处理，不能因船头恢复可见就放行掉头。

## 1. PC：同步源码

Nano 开机并接入同一网络；以下使用 `nvidia@192.168.1.188`，地址变化时替换。
更新前结束旧的清洁节点和本工作空间的构建/测试，避免运行中加载新旧混合代码。
此同步会更新五个包的源码，不会重启任何服务、启动 bridge、刷写固件或改变已运行 bridge 的参数。
IMU 转换库已并入 `nano_bridge` 的内部模块，不代表矩形任务使用 IMU。

在 PC 执行：

```bash
cd /home/bboss/new-nano/f1_mapping_ws
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/f1_mapping_ws/src'
rsync -av --itemize-changes --backup \
  --backup-dir="/home/nvidia/f1_mapping_ws/deploy_backups/$(date +%Y%m%d_%H%M%S)" \
  --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
  src/ nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/src/
```

仅传 `src/`，不传 PC 的 `build/ install/ log/`。没有使用 `--delete`；被覆盖的远端源码会放入带时间的备份目录。
PC 和 Nano 都需要 `rsync`。遇到连接、同步失败先处理错误，不继续构建旧源码。

本轮感知测试不用地图，不必重新建图或覆盖 Nano 原地图。后续定位使用已保存的 pbstream；如需在 Nano 留一份明确路径的副本，可在 PC 执行：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/f1_mapping_ws/maps/20261021_145054'
rsync -av --ignore-existing \
  ../backups/nano_maps/20261021_145054/ \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/maps/20261021_145054/
```

后续加载状态文件路径为 `/home/nvidia/f1_mapping_ws/maps/20261021_145054/lidar_20261021_145054.pbstream`。
原 YAML 的 `image` 是旧绝对路径；将来加载该副本栅格时需要改为同目录 PGM 的相对文件名。本轮不加载该 YAML。

## 2. Nano：构建与离线测试

登录 Nano，在新的构建终端执行；不要先 source 本工作空间旧的 install：

首次从旧接口包升级时，先在 Nano 工作空间根目录按[统一接口迁移说明](../f1_interfaces/README.md#从旧接口包迁移)备份并移走旧接口包源码、build 和 install，再构建。上面的 rsync 不会删除遗留旧包；所有使用自定义接口的 PC/Nano 节点均需同步升级。
原独立 IMU 包也需按[桥接包迁移说明](../nano_bridge/README.md#从独立-imu-包迁移)移走，之后只构建统一的 `nano_bridge`。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
cd /home/nvidia/f1_mapping_ws
colcon list --names-only
colcon build --symlink-install --parallel-workers 1 \
  --packages-up-to f1_mapping_bringup nano_bridge f1_rectangle_cleaning \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

应能识别 `f1_interfaces`、`nano_bridge` 和 `f1_rectangle_cleaning`，共五个包。依赖工作空间沿用现有 `/home/nvidia/ros2_ws`；只有构建成功后才继续。

```bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run f1_rectangle_cleaning rectangle_demo
colcon test --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose
```

demo 应打印 A 接近、前进、停车、等待人工换边、B 接近、前进、停车、结束的序列。
它只是合成反馈的状态机演示，不代表船体动力学仿真。当前测试包含 31 项纯逻辑、5 项 ROS 测试；colcon 汇总会加上两项 CTest 包装，显示 38 项。以零失败/零错误为准。
ROS 测试使用独立 localhost 域和合成输入，不连接执行器。

## 3. Nano：每个实景测试终端加载环境

以下为各终端公共前置命令：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

终端 A 检查既有雷达服务：

```bash
systemctl status pavo2-scan.service --no-pager
ros2 run f1_mapping_bringup scan_check --samples 100
ros2 node list
```

若服务已安装但未启动，执行 `sudo systemctl start pavo2-scan.service` 后再检查。
应有有效 `/pavo2_scan`，frame 为 `pavo2s_frame`。不要另外启动第二份雷达驱动或旧 `wall_controller`。
本轮不用新开建图或 bridge；雷达感知直接使用包内已确认的平面外参。

随后在终端 A 保持运行：

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py
```

## 4. Nano：查看真实感知结果

终端 B 加载公共环境后：

```bash
ros2 topic echo /rectangle_cleaning/wall_observation --qos-reliability best_effort
```

重点字段：

| 字段 | 含义与检查方法 |
|---|---|
| `valid` / `reason` | 左墙线是否拟合成功；先看此项再解释距离和角度 |
| `distance_m` | base_link 到左墙线的垂直距离，单位 m，不是雷达到墙或刷头到墙距离 |
| `tangent_rad` | 墙线相对船体前向的夹角，单位 rad；平行时接近 0，0.087 rad 约为 5° |
| `residual_m` / `support_m` | 拟合残差和有效墙段长度，帮助区分稳定墙线与杂乱回波 |
| `front_observed` | 前方走廊回波是否足以用于净空判断；false 时不能解释成前方空闲 |
| `front_clearance_m` | 船头前方走廊净空的保守估计，当前观察范围上限约 1 m，不一定等于完整短墙距离 |
| `rigid_collision` | 雷达是否发现点进入配置的刚性危险区域 |

第一项检查为扫描覆盖。可用 `ros2 topic echo /pavo2_scan --once --no-arr` 查看 angle_min/angle_max 等原始元数据；这些角度属于雷达坐标系，要结合真实安装朝向解释。
即使 `valid=true`，也只表示左墙拟合成功，不表示前方或完整船体周围已观测。
`rigid_collision=false` 同样不能证明盲区内没有危险；新修复仅保证前方没有扫描束时，仍检查已看见的侧面/后方回波是否进入刚性危险区域。

依次做下面几组测试，刷盘不启动，保持船体与墙面有间隙：

1. **静止、平行**：人工摆在长墙左侧可见的位置，中心到墙约 45 cm，记录约 30 秒。用尺核对中心距墙，观察角度均值、距离偏差和波动。
2. **改变距离**：保持近似平行，将中心距墙缓慢改到约 55 cm，再回到 45 cm。应看到约 10 cm 的对应变化，角度基本不变。
3. **改变角度**：在留有空间的位置，缓慢向两边各偏转约 5°，再摆正。角度应向相反符号变化，摆正后回到零附近；此时墙距以实际中心位置为准。
4. **前方净空（仅当前方实际可见时）**：静止核对船头到短墙距离，验证 `front_observed` 与净空值。当前用户确认的船头盲区安装不具备此项前提，先记录缺失情况。不要为触发阈值让船自主冲向墙面。

这些是感知检查，不需要下发任何自动执行器命令。测试姿态/位置大幅改变后如果墙线关联丢失，可在任务未运行时重置：

```bash
ros2 service call /rectangle_cleaning/reset_observation std_srvs/srv/Trigger '{}'
```

重置不保证回波质量合格，应继续看 `valid/reason`。不要通过放宽有效性检查掩盖持续无效或测距明显错误。

另开终端查看任务状态：

```bash
ros2 topic echo /rectangle_cleaning/status
```

当前预期为 IDLE、`output_enabled=false`、`lease_held=false`；会提示缺少定位/运动反馈。
**本轮无需调用 start**。即使墙线有效，缺少 `/rectangle_cleaning/map_pose` 或 `/rectangle_cleaning/local_motion` 仍不能开始任务；这并非部署失败。
仅在 idle 感知测试时 `command_preview` 没有持续输出也属正常，不要期待有执行器动作。

## 5. 保存原始雷达数据供开发

终端 C 加载公共环境，在上述摆放测试之前开始录制：

```bash
mkdir -p /home/nvidia/f1_mapping_ws/test_bags
ros2 bag record \
  -o "/home/nvidia/f1_mapping_ws/test_bags/rectangle_lidar_$(date +%Y%m%d_%H%M%S)" \
  /pavo2_scan /tf /tf_static \
  /rectangle_cleaning/wall_observation /rectangle_cleaning/status
```

部分 TF 话题本轮没有发布者是正常的；必须确认原始 `/pavo2_scan` 和墙线观测被记录。
测试结束按 Ctrl+C 正常关闭录包，记下生成目录，并用 `ros2 bag info <实际录包目录>` 检查消息数。
另记每段动作的顺序、尺量距离、摆放角度及大致时刻；只录消息而没有对应实测值难以评估误差。

在 PC 下载已完成录制的数据：

```bash
mkdir -p /home/bboss/new-nano/backups/nano_bags
rsync -av \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/test_bags/ \
  /home/bboss/new-nano/backups/nano_bags/
```

回放重算感知时应只播放原始扫描等输入，避免把录制的墙线话题与新节点输出混在一起；回放节点需要 `use_sim_time:=true` 和 bag 的 `/clock`。
当前墙线时效预算为 150 ms，需要记录实际处理耗时；Nano 跟不上时先检查负载和频率，不直接取消过期保护。

## 6. 后续开发顺序（纯雷达）

| 阶段 | 软件工作 | 验证完成的依据 |
|---|---|---|
| 感知基线 | 修正必要的外参/墙线选择，统计距离与夹角误差、延迟和有效率 | 45/55 cm、正负偏转和前方净空检查与实测一致，数据可回放复现 |
| 纯雷达反馈接入 | 加载 pbstream 做固定地图定位；从连续局部雷达位姿估计 vx/vy/wz，建立时间戳、误差和质量检查 | 静止不报明显运动，已知位移/旋转方向正确，失配时反馈无效；提供新包要求的两个输入 |
| 单长边闭环 | 雷达夹角纠偏、第五路墙距控制、租约输出；先验证 A 单段且不开刷 | 航向和墙距受控，端部停车有余量，数据中断/RC 接管后停止输出 |
| 自动换边 | 离墙侧移、停稳、连续转角跟踪 180°、靠近 B；检查整个运动范围 | 离墙和掉头不越过边界，匹配丢失时暂停，最后正确识别 B 墙 |
| 接触清洗 | 逐步引入刷头接触目标和刷盘负载 | 刚性间隙、贴墙稳定性、停车和实际清洁效果均经实机验证 |

单墙线可以提供墙距和相对航向，但仅凭一条直墙难以可靠判断沿墙方向的位移；停车/掉头需要利用前方墙、其他可见结构或多帧扫描匹配。
大角度转向不能继续要求“始终看到同一条左墙”，需要独立的定位/匹配与空间检查。纯雷达方案是否能覆盖整次转向，要用现场数据验证。
速度应来自连续局部运动估计，不能把地图定位跳变差分成真实速度；也不能把缺少的速度和协方差用常数伪装为有效反馈。
此计划不要求先完成完整动力学模型，但接通执行器前仍要明确第五路命令正负号、双向横移效果和实际停车余量。
