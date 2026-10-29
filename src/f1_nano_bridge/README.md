# f1_nano_bridge

本包负责 F103 串口控制、CTRL/ACK、IMU 发布和 ROS 执行器接口。Windows App 不再直接连接本包，也不再使用旧 ZMQ 网关；新版 App 通过同级功能包 `f1_operator_gateway` 接入。

## 入口

| 入口 | 职责 |
|---|---|
| `f1_bridge` | 串口、固件协议、控制状态和 IMU |
| `f1_operator_gateway` | 新 Qt App 的 WebSocket 网关，位于独立功能包 |

App 的启动方式和协议见 [`f1_operator_gateway`](../f1_operator_gateway/README.md) 与仓库根目录的 [`operator_app`](../../operator_app/README.md)。本包不再安装 Windows UI、ZMQ 或旧 gateway 脚本。

## ROS 接口

| 接口 | 类型与用途 |
|---|---|
| `/f1/actuator_command` | `f1_interfaces/ActuatorCommand`，方向、档位、轮组、辅助目标及期限 |
| `/f1/acquire_control` | `f1_interfaces/AcquireControl`，由 Nano 网关内部自动使用 |
| `/f1/release_control` | `f1_interfaces/ReleaseControl`，由 Nano 网关在停止、断线和超时时使用 |
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
- App 停止发送输入、WebSocket 断开或网关退出时，网关释放自己的手动会话。自动任务的租约由任务控制器管理，需通过其 `pause/cancel` 停止；App 的 `manual_stop` 不调用 `/f1/stop`。
- 重连后不会恢复上一次动作；必须重新产生新输入。
- RC 接管、epoch 改变、串口错误、CTRL/ACK 超时和原始命令过期仍会停止输出。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
colcon build --symlink-install --packages-up-to f1_nano_bridge f1_operator_gateway
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
colcon test --packages-select f1_nano_bridge f1_operator_gateway \
  --return-code-on-test-failure --ctest-args -E '^firmware(_imu)?$'
colcon test-result --verbose
```

上述命令跳过需要外部固件源码的 `firmware` 和 `firmware_imu` 测试组；完整测试的 `F1_FIRMWARE_DIR` 准备步骤见[部署文档](../../DEPLOY_NANO.md#7-nano离线测试)。测试不会连接真实串口或驱动电机。跨机 WebSocket、实际 Wi-Fi 延迟、雷达坐标和执行器方向仍需现场验收。
