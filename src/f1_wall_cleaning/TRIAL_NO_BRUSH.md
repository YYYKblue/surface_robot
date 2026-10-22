# 关闭刷盘的首次自动沿墙试跑

用户于 2026-10-21 明确选择先跳过完整运动标定，关闭刷盘观察自动沿墙效果。
此模式使用已提供的静态尺寸，以及明确未标定的运动初值，不作为贴墙清洁验收。

## 试跑行为

- 25 档、BOTH；前进脉冲最长 0.10 s、转向脉冲最长 0.05 s，STOP 后等待 1 s。
- 目标为车体中心到左墙 0.45 m；按当前尺寸，名义刚性间距 0.17 m、未压缩刷尖间距 0.12 m。目标是隔开刷毛试走，不主动压刷。
- 最长运行 10 s，到期暂停；重新试跑需显式 resume，取消后可重新 start。
- 配置 brush_allowed=false，实际命令的 brush_enabled=false、brush_percent=0；第五路始终关闭。
- 在目标距离/角度窗口稳定后进入 FOLLOW_WALL；不进入 CLEAN_WALL，不累计开刷时间。
- 暂用前方停车阈值 0.50 m、缩短脉冲区 1.0 m、转向上界 0.20 rad。这些是试跑假设，不是实测保证，物理惯性和漂移仍可能超出假设。
- 保留扫描 150 ms、命令 100 ms、bridge 发送间隔 80 ms 的限制，以及墙端、前方未知、碰撞、转向盲区、RC/epoch/连接变化后的暂停行为。
- supervised_trial=true 是独立的显式试验入口，pulse_response_calibrated 仍为 false；正常模式仍要求标定。验证会拒绝开启刷盘、加长运行/脉冲或降低该模式的指定余量。

## 同步和构建

PC 上同步源码（Nano 地址变化时替换）：

```bash
scp -r /home/bboss/new-nano/f1_mapping_ws/src/f1_wall_cleaning \
  nvidia@192.168.1.188:/home/nvidia/f1_mapping_ws/src/
```

Nano 的新终端构建：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
cd /home/nvidia/f1_mapping_ws
colcon build --symlink-install --packages-up-to f1_wall_cleaning \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 启动

后续每个 Nano 终端先加载：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

使用水中平直、无突出障碍的长墙段，人工将船放置到左墙旁，中心距墙约 45～55 cm、船头近似平行，前方留足空地。保持现场遥控器可用于接管。
无需运行建图；保留现有雷达驱动。确认不再运行手动控制脚本、旧串口服务或另一套 bridge/沿墙节点。

终端 A：

```bash
ros2 launch nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true
```

终端 B：

```bash
ros2 launch f1_wall_cleaning wall_cleaning.launch.py \
  config:=/home/nvidia/f1_mapping_ws/src/f1_wall_cleaning/config/supervised_no_brush_trial.yaml \
  output_enabled:=true
```

启动不会自动 acquire 或运动。终端 C 可先查看状态：

```bash
ros2 topic echo /wall_cleaning/status
```

状态检查后退出 echo，显式开始一次试跑：

```bash
ros2 service call /wall_cleaning/start std_srvs/srv/Trigger '{}'
```

start 返回 acquire requested 仅表示申请已提交，实际阶段及暂停原因看 status。
需要提前暂停或停止时分别使用：

```bash
ros2 service call /wall_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /f1/stop std_srvs/srv/Trigger '{}'
```

暂停或到期后保持停止，不能自动恢复；检查现场和暂停原因后才由操作人显式 resume。
/f1/stop 请求停止 Nano 控制，不能覆盖 RC，服务应答也不代表船体已无惯性。

## 观察与记录

观察是否识别正确左墙、能否保持约 45 cm 中心墙距、动作是否足够可见、是否振荡/漂移，以及首次暂停原因。
约 280° 雷达可能使转向被拒绝；若出现 turn sweep unknown/blocked，保留原检查并记录观测，不关闭盲区保护。

可在开始前另外录包（含控制租约的原始数据仅用于分析，不用于向实机回放）：

```bash
ros2 bag record -o wall_trial_no_brush_01 \
  /pavo2_scan /wall_cleaning/wall_observation /wall_cleaning/effort_preview \
  /wall_cleaning/status /f1/control_state /f1/imu/data_raw /diagnostics
```

## 本次验证记录

2026-10-21：本机 ROS Humble、CycloneDDS 下完整沿墙包 pytest 为 67 项通过、0 失败，包含 16 项感知、40 项控制/配置和 11 项 ROS 测试。
新增 ROS 用例使用合成雷达、实际感知/控制节点和模拟 bridge Session，验证显式 start 后进入 FOLLOW_WALL、前进命令被接受、刷盘与第五路指令始终关闭、pause 后释放租约且不再发命令。
控制测试另验证 10 s 到期暂停、默认模式仍拒绝未标定输出、试跑不能开启刷盘或放宽指定约束，以及前方/转向保护仍生效。
首次 ROS 运行受沙箱网络接口访问限制；允许本机测试网络后，发现预览模式等待定时回调可能导致命令过期，已改为新观测到达时立即更新，与实机路径一致，未放宽时限。修正后完整 67 项通过。
用户确认 Nano 尚未开机，因此本次未同步 Nano、未打开实机串口、未执行运动。本机测试不代表 Nano 或水上实测已完成。
