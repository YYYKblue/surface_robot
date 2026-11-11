# f1_nano_bridge

本包负责 F103 串口控制、CTRL/ACK、IMU 发布和 ROS 执行器接口。Windows App 不再直接连接本包，也不再使用旧 ZMQ 网关；新版 App 通过同级功能包 `f1_operator_gateway` 接入。

## F103 v3 连续输出（2026-11-11）

当前对接仓库内 `under/F103firmware`，配置 `firmware_profile: f1_direction_percent_v3`。下行仍为 39 字节、460800 baud、每 20 ms 一帧，版本字节为 3；CTRL 为 `CTRL,3,...`。桥接不兼容旧 v1/v2 固件；收到校验正确但版本不支持的 CTRL 时撤销租约、停止发送动作，状态报告 `INCOMPATIBLE` 及实际/预期版本。恢复 v3 后仍需中性握手和新的控制申请，不恢复旧动作。

`ActuatorCommand.speed_percent` 接受整数 0–100，CH06 编码为 `1050 + 9*p`，主电机 PWM 偏移为 `4*p` μs。`0` 或 `STOP` 只令四主电机中位，独立滚刷、第五电推仍按各自使能和百分比执行。完整停止需要同时关闭辅助输出；中性握手使用零主输出、STOP 和辅助关闭。CH07 保持 1500，遥控器 H 自动前进锁存不用于 Nano。

| 输出数字 | 旧三档 / 当前物理遥控器 PWM 偏移 | v3 Nano PWM 偏移 |
|---|---:|---:|
| 25 | 125 μs | 100 μs |
| 50 | 250 μs | 200 μs |
| 75 | 375 μs | 300 μs |
| 100 | 不适用 | 400 μs |

这些是 PWM 幅度，不能推导实际转速、推力或船速。自主任务第一阶段仍固定使用新标度的 25%，需要重新验证纠偏、接近、换边和前进辅助。固件负责四电机混控，当前前进极性按 BL/BR/FL/FR 为 `+,+,-,+`；升级后需逐路核对实际方向。

桥接分别解析 CTRL、IMU 与 `RC,2,...,CH1..CH14`，正常 RC 报告不再计为无效行。`ControlState` 新增固件版本/兼容状态及 RC 原始通道、flags、sequence、接收年龄和有效性；重复 RC 时间戳不能刷新年龄，超过 0.4 s 或报文标明失联时 `rc_valid=false`。RC 通道仅用于诊断，执行状态和控制源以 CTRL 为准。兼容标志不代表遥测新鲜、握手完成或已取得控制权。

CTRL 的四路 PWM 是目标值，ACK 表示命令接受，不表示电机已达到目标或船已停稳。当前软件斜坡输出、UART/DMA 故障计数和仲裁重置原因尚未上报。原有命令 TTL、CTRL/ACK 超时、发送间隔保护和 RC 接管规则保持不变。

## 入口

| 入口 | 职责 |
|---|---|
| `f1_bridge` | 串口、固件协议、控制状态和 IMU |
| `f1_operator_gateway` | 新 Qt App 的 WebSocket 网关，位于独立功能包 |

App 的启动方式和协议见 [`f1_operator_gateway`](../f1_operator_gateway/README.md) 与仓库根目录的 [`operator_app`](../../operator_app/README.md)。本包不再安装 Windows UI、ZMQ 或旧 gateway 脚本。

## ROS 接口

| 接口 | 类型与用途 |
|---|---|
| `/f1/actuator_command` | `f1_interfaces/ActuatorCommand`，方向、0–100 主输出、电机组、辅助目标及期限 |
| `/f1/acquire_control` | `f1_interfaces/AcquireControl`，由网关手动会话或清洗控制器申请单一控制租约 |
| `/f1/release_control` | `f1_interfaces/ReleaseControl`，持有租约的网关或清洗控制器释放控制权 |
| `/f1/stop` | `std_srvs/srv/Trigger`，全局停止请求，不能覆盖 RC |
| `/f1/control_state` | `f1_interfaces/ControlState`，设备、ACK、控制源和故障状态 |
| `/f1/imu/data_raw` | `sensor_msgs/msg/Imu` |
| `/diagnostics` | 串口、控制会话与 IMU 诊断 |

内部控制租约仍由硬件桥使用，以拒绝旧命令、区分控制来源并保证断线停止；它不再暴露给 App 操作员。

## 启动

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
colcon build --symlink-install --packages-up-to f1_nano_bridge f1_operator_gateway
source install/local_setup.bash

ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true
# 另一终端加载同一 ROS/工作空间环境后启动网关：
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

网关默认监听 `127.0.0.1:8765`。跨机使用时监听地址设置为 `0.0.0.0`，Windows App 填 Nano 的实际局域网地址。当前入口没有登录认证，只应放在受信任局域网。

## 控制安全边界

- App 发送最新手动输入，不发送 token，也不调用 ROS 服务。
- 网关只保留一个活动操作端；其他连接可接收遥测，但不能发送手动或任务操作。
- `allow_fifth_thruster` 默认在 `config/bridge.yaml` 中启用，因此 Qt App 可使用第五电推的 `-100%` 到 `100%` 输入；如需硬件锁定可改为 `false`。
- App 停止发送输入、WebSocket 断开或网关退出时，网关释放自己的手动会话。自动任务的租约由任务控制器管理，需通过其 `pause/stop/cancel` 停止；App 的 `manual_stop` 不调用 `/f1/stop`。
- 重连后不会恢复上一次动作；必须重新产生新输入。
- RC 接管、epoch 改变、串口错误、CTRL/ACK 超时和原始命令过期仍会停止输出。
- 系统时间相对启动基准偏移超过 50 ms，或 IMU 检测到时间跳变时，先撤销租约并锁定故障，串口线程尽力发送中性帧、关闭串口，主进程以 75 退出。只有时间故障请求这种恢复，其他故障不会被它自动清除。`f1-operator.service` 会重新检查时间并重启整套 operator 为待机；直接运行本包没有自动重启。恢复不沿用旧 token、命令或任务，必须重新选图和定位。详见[三层恢复策略](../../DEPLOY_NANO.md#app-建图选图与开机服务)。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
colcon build --symlink-install --packages-up-to f1_nano_bridge f1_operator_gateway
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
colcon test --packages-select f1_nano_bridge f1_operator_gateway \
  --return-code-on-test-failure
colcon test-result --verbose
```

固件测试默认使用工作空间 `under/F103firmware`，也可通过 `F1_FIRMWARE_DIR` 显式指定同版本源码。`under/` 被 Git 忽略，新克隆需另行准备完整固件副本。测试库在临时目录编译，覆盖真实 C 解码/仲裁及 Python 桥接的映射，不烧录固件、不连接真实串口。完整环境准备见[部署文档](../../DEPLOY_NANO.md#7-nano离线测试)。跨机 WebSocket、实际 Wi-Fi 延迟、雷达坐标和执行器方向仍需现场验收。
