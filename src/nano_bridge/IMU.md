# nano_bridge 的旧 ZMQ IMU 接入

本功能已从 `f1_imu_bridge` 合并到 `nano_bridge`，实现位于 `nano_ros_bridge.imu`。本文描述旧网关接入链路；当前 back2 固件使用 [原生串口桥](README.md)，它已直接发布 IMU，不应同时启动本节点。

链路：F1 串口 → 现有 nano-bridge → ZMQ `f1_imu` → 本节点 → `/f1/imu/data_raw`。
串口始终由旧网关持有。本节点只建立 SUB 连接，不发送控制、不打开串口。
节点与网关应运行在同一台 Nano 上，采集使用系统时间；录包重放时不运行本节点。

## 数据和时间

网关在每个校验通过的 IMU 帧后转发，不从约 5 Hz 的 UI 状态快照提取。
使用已有状态端口（默认 5555）；ZMQ 主题为 `f1_imu`。
需要网关新版 `runtime.py` 和 `config.py`，以及 `f1.imu_stream_enabled=true`。
旧配置缺少开关时默认关闭转发。UI 状态中的 `capabilities.f1_imu_stream` 可确认开关。

协议 `f1-imu-v1` 包含网关/串口会话 ID、主机接收 Unix/单调时间（整数纳秒）以及
原始 IMU 解析字段。节点检查格式、数值、序号、状态掩码和过期情况。
利用 STM32 毫秒增量和最低观测接收延迟重建时间戳，处理 32 位回绕和双计数器重启。
这是近似对时，不能补偿固件、串口积压的固定延迟，也不等于雷达与 IMU 硬件同步。
时间校正后不能递增的样本会丢弃；系统时间明显跳变后停止发布，需重启节点与建图轨迹。
网关 PUB 有界且非阻塞，过载可能丢包；通过序号缺口诊断，不承诺无损转发。

默认单位按现有协议资料假定为 m/s²、rad/s，必须核对已烧录固件。
配置提供 `acceleration_scale` 和 `angular_velocity_scale` 用于单位换算。
加速度应包含重力，静止时模长应约为 9.81 m/s²；方向依实际传感器坐标轴而定。
`required_status_mask=0` 仅代表未检查固件状态位，不能据此认定 IMU 已完成初始化；
取得实际状态位定义后再设置掩码。
固件四元数约定未核实，ROS 消息标记 orientation 不可用，仅发布加速度与角速度。
两者协方差全零表示未知。`imu_link` 必须表达这两组向量实际使用的坐标系。

输出 QoS 为 reliable、keep-last 100，可匹配 reliable 或 best-effort 订阅者。
诊断话题 `/f1/imu/diagnostics` 包含频率、数据年龄、缺帧、重启及丢弃计数。
参数在启动时读取且为只读，修改配置后重启节点。

## 构建与无外设检查

依赖 `python3-zmq`、ROS Humble 的 rclpy/sensor_msgs/diagnostic_msgs。
若缺少 ZMQ，可在 Nano 安装：

```bash
sudo apt-get install python3-zmq
```

```bash
source /opt/ros/humble/setup.bash
cd ~/f1_mapping_ws
colcon build --symlink-install --packages-up-to nano_bridge f1_mapping_bringup
source install/local_setup.bash
colcon test --packages-select nano_bridge --return-code-on-test-failure --ctest-args -R '^imu'
colcon test-result --verbose
```

测试包含真实 ZMQ→ROS 回环：使用随机端口与独立测试话题，只生成明确的模拟数据，
不打开串口、不连接生产 5555 端口、不发布到 `/f1/imu/data_raw`。
先验证无输入时只报告等待，再验证模拟报文转换。
PC 若缺少 `python3-zmq`，此项显示 skipped；Nano 安装依赖后应实际执行通过。

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/f1_mapping_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch nano_bridge imu.launch.py
```

未连接 F1 或未升级网关时，节点等待数据，诊断报告 WARN，不会发布伪造 IMU。
在相同 ROS 环境的另一终端检查：

```bash
ros2 topic echo /f1/imu/diagnostics --once
```

接入 F1 并启用新版网关后，再验证实际频率、静止值及转动方向：

```bash
ros2 topic hz /f1/imu/data_raw
ros2 topic echo /f1/imu/data_raw --once
```

外部配置文件可通过 `parameters_file:=/absolute/path/imu.yaml` 指定。
本节点只提供独立 IMU 遥测。建图包现仅保留 `lidar_mapping.launch.py` 纯雷达入口，不使用本话题。

## 历史参考：更新旧 Nano 网关

以下依赖本仓库未包含的旧网关源码，仅记录历史步骤；不适用于当前 back2 固件部署。当前部署使用 [nano_bridge 说明](README.md)。

Nano 已确认的服务工作目录为 `/home/nvidia/Desktop/wbcNewRos-19`。
可用 `systemctl show nano-bridge.service -p WorkingDirectory -p ExecStart --no-pager` 核实。
只更新两个网关源码文件，保留 Nano 自己的串口、相机和网络配置。
可以先把文件传到临时目录，等准备好重启网关时再替换：

PC：

```bash
scp /home/bboss/new-nano/nano_bridge/runtime.py \
    /home/bboss/new-nano/nano_bridge/config.py nvidia@192.168.1.188:/tmp/
```

Nano（本步骤会重启原通信服务，连接实物时应在机器人停止后操作）：

```bash
cd /home/nvidia/Desktop/wbcNewRos-19
sudo systemctl stop nano-bridge.service
cp -an nano_bridge/runtime.py nano_bridge/runtime.py.before-imu
cp -an nano_bridge/config.py nano_bridge/config.py.before-imu
cp -an config.json config.json.before-imu
cp /tmp/runtime.py nano_bridge/runtime.py
cp /tmp/config.py nano_bridge/config.py
python3 - <<'PY'
import json
from pathlib import Path
path = Path('config.json')
config = json.loads(path.read_text())
config['f1']['imu_stream_enabled'] = True
path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
PY
python3 -B -m nano_bridge --check
sudo systemctl start nano-bridge.service
journalctl -u nano-bridge.service -n 30 --no-pager
```

这一步只增加 F1 原始数据转发，不改变设备运动指令、界面状态刷新频率或控制协议。
回滚时停止网关，恢复三个 `.before-imu` 文件，再启动服务。
维护者邮箱为占位值，许可证尚未指定，对外发布前应填写实际信息。
