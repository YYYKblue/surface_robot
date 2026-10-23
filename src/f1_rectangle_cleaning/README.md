# f1_rectangle_cleaning：简化单边沿墙清洗

2026-10-23 更新：只沿当前左墙清洗，不换边、不掉头。滚刷负责推进，四个主电机负责航向纠偏，第五电机负责中心到左墙约 40 cm 的距离控制。

| 条件 | 输出 |
|---|---|
| 左墙夹角在 **[-10°, +10°]**，包括边界 | 滚刷 **15%** 开启推进；四个主电机中位 |
| 夹角 **>+10°** | 关滚刷，四个主电机 **LEFT 25%、BOTH** 持续纠偏 |
| 夹角 **<-10°** | 关滚刷，四个主电机 **RIGHT 25%、BOTH** 持续纠偏 |
| 夹角回到 ±10° 内 | 当帧停止转向，恢复滚刷；没有脉冲间隔或中位等待 |
| 中心到左墙 **>42 cm** | 第五电机向左推，默认 **+15%** |
| 中心到左墙 **<38 cm** | 第五电机向右推，默认 **-15%** |
| 中心墙距 **38–42 cm**，包括边界 | 第五电机关闭；不保留上一次推力 |
| 船体前缘到前墙 **≤80 cm** | 滚刷、主电机、第五电机全部关闭，进入 `STOPPED` |

`ψ = WallObservation.tangent_rad` 是左墙切线相对船头的夹角；正误差发 LEFT，负误差发 RIGHT。每个新墙观测都重新计算航向与侧推，纠偏时第五电机仍工作。超出 ±10° 也可以启动任务，先纠偏再开滚刷。

已移除：转向空间验证、刷毛接触/压缩门控、刚性最小墙距门控、人工摆放距离范围、地图与左墙身份/夹角一致性、重复墙线质量门控、定位协方差门槛、定位跳变暂停，以及纠偏脉冲/中位等待。墙线提取仍输出实际有效性，控制器要求有效、新鲜、数值有限的墙观测和位姿。保留通信超时、执行器命令期限、遥控接管、取消/暂停关闭输出。

默认无固定运行时长上限，直到前墙阈值或手动停止；可通过 `run_timeout_sec > 0` 另设试验时长。状态序列为 `IDLE → FOLLOW ↔ CORRECT_HEADING → STOPPED`；数据或通信异常进入 `PAUSED`。暂停后 `cancel` 再 `start`。

## 前墙距离

现有雷达安装有船头盲区，因此仍用已有冻结地图的 `map → base_link` 位姿计算前墙距离；这版没有改雷达外参。默认地图矩形长约 2.883 m、宽约 1.520 m，参数来自 `20261021_145054` 地图。

按船头在矩形长轴上的朝向选择前方短墙，支持沿长轴正向或反向行驶，无需手动切换“当前边”配置。令池坐标为 `x`、船头与长轴夹角为 `yaw`：

```text
船体前缘投影 = 0.28*|cos(yaw)| + 0.28*|sin(yaw)|
朝长轴正向：前墙净空 = length - x - 船体前缘投影
朝长轴反向：前墙净空 = x - 船体前缘投影
```

船体位姿随矩形地图一致旋转 180° 时，计算出的前墙净空相同。有效前方雷达观测若已显示净空 ≤80 cm，也会关闭输出；`front_observed=false` 不阻止启动，不会把未知距离当成零。

80 cm 指 **船体前缘到墙**，不是船中心到墙；这是关闭输出的触发点，实际滑行距离仍需下水测量。地图距离仍依赖真实定位精度，不能发现雷达盲区新增障碍。40 cm 指 **base_link 中心到左墙**；现有刷尖距中心 33 cm，40 cm 墙距不保证刷毛接触。

## 构建与离线验证

```bash
cd /home/bboss/new-nano/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_rectangle_cleaning
source install/local_setup.bash
ros2 run f1_rectangle_cleaning rectangle_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose
```

演示和测试使用合成反馈、localhost DDS 和模拟 bridge，不连接串口或实机。

## 启动

Nano 源码同步、地图定位和 bridge 启动见 [下水测试步骤](DEPLOY_TEST_NANO.md)。默认预览不申请租约：

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py
ros2 topic echo /rectangle_cleaning/status
ros2 topic echo /rectangle_cleaning/command_preview
```

实机使用同一个入口，显式开启输出，再启动任务：

```bash
ros2 launch f1_rectangle_cleaning rectangle_cleaning.launch.py output_enabled:=true
# 另一终端：
ros2 service call /rectangle_cleaning/start std_srvs/srv/Trigger '{}'
# 暂停或取消：
ros2 service call /rectangle_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /rectangle_cleaning/cancel std_srvs/srv/Trigger '{}'
```

实机需要已有 bridge 的串口/运动许可、主电机方向确认和 `allow_fifth_thruster=true`。`mode=BOTH` 由固件混控四个主电机；这版不发主电机 FORWARD/REVERSE，推进仍由滚刷提供。保持第五路既有“正号向左”的映射。

状态包含 `wall_angle_deg`、`wall_distance_m`、`heading_band_deg`、`map_front_clearance_m`、刷盘/侧推请求及租约状态。预览 token 始终为空。`STOPPED` 只表示输出已关闭，不代表船体已经停稳。

请使用本包更新后的默认配置。历史诊断目录中的 `rectangle_current_side.yaml` 仍保存当时的 ±3°、8 秒限制，不是本版测试参数。
