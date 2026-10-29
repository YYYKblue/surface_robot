# f1_operator_gateway

更新：2026-10-29。Qt 操作员 App 的 Nano 侧 WebSocket 入口。App 发送当前输入，网关通过 `/f1/acquire_control`、`/f1/release_control` 管理手动租约，并向 `/f1/actuator_command` 发布有期限的执行器指令。App 不需要 ROS，也不处理 token。

网关只提供控制和遥测转发，不启动 bridge、雷达、Cartographer 或清洗节点。

## 构建与启动

只部署手动控制链路时，同时构建 bridge 和网关：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
colcon build --symlink-install --packages-up-to f1_nano_bridge f1_operator_gateway
source install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

网关依赖 `python3-websockets`，由 rosdep 安装，也可使用 `sudo apt install python3-websockets`。每个运行终端加载上述环境。若尚未运行 bridge，在终端 A 启动；硬件方向已核对时才启用对应许可：

```bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true
```

在终端 B 启动网关；已有清洗 bringup 提供 bridge 时，直接复用：

```bash
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

`0.0.0.0` 是监听地址，Windows App 应连接 `ws://Nano实际IP:8765`。当前无 TLS 或登录认证，使用可信局域网。App 安装与启动见 [operator_app](../../operator_app/README.md)。

## 参数和接口

| 参数 | 默认值 | 设置方式 |
|---|---|---|
| `bind_address` | `127.0.0.1` | launch 参数；跨机设为 `0.0.0.0` |
| `port` | `8765` | launch 参数 |
| `input_timeout_sec` | `0.25` | launch 参数；有效范围为 0.1–1.0 秒 |
| `telemetry_hz` | `10.0` | 节点参数；当前 launch 未暴露，可用 `ros2 run ... --ros-args -p telemetry_hz:=10.0` |

当前命令期限固定为 250 ms；增加输入超时不会同步延长执行器命令期限。App 每 50 ms 发送一次最新手动输入。

| ROS 接口 | 用途 |
|---|---|
| `/f1/actuator_command` | 发布手动执行器命令 |
| `/f1/acquire_control`、`/f1/release_control` | 申请与释放手动租约 |
| `/f1/control_state` | 设备状态、控制源及第五路许可 |
| `/map`、TF `map → base_link`、`/pavo2_scan` | 地图、船体位姿和雷达遥测 |
| `/pool_cleaning/status` | 默认订阅的矩形任务状态 |
| `/pool_cleaning/start`、`pause`、`cancel` | 默认调用的矩形任务服务 |

协议名为 `f1-operator-v1`，消息使用 JSON 文本：

| App 操作 | 字段与含义 |
|---|---|
| `hello` | `client`、可选 `version`；获取协议回应及已有地图 |
| `manual_input` | `seq`、`direction`、`speed_percent`、`mode`，以及 `brush_enabled/brush_percent`、`thrust_enabled/thrust_percent` |
| `manual_stop` | 可选 `reason`；释放网关手动会话 |
| `task` | `action` 为 `start`、`pause` 或 `cancel` |

方向可取 `STOP/FORWARD/REVERSE/LEFT/RIGHT`，档位为 25、50、75，轮组为 `FRONT/BOTH/BACK`；两个辅助执行器百分比范围均为 -100 到 100。服务端发送 `hello`、`map`、`telemetry`、`reply` 或 `error`。地图在内容变化及客户端 `hello` 时发送，遥测包含 `state/pose/scan/task`。

## 控制行为与边界

- 可以有多个 WebSocket 连接接收遥测；首次发送控制操作的客户端成为活动操作端，其他客户端的手动和任务操作会被拒绝。活动端断开后才解除该绑定，手动停止本身不会让出操作端身份。
- 第五路未获 bridge 确认启用时，网关屏蔽第五路字段并回复原因，仍保留主驱动输入。
- 手动输入超时、活动端断开或网关退出时，网关释放自身手动租约。后续运动仍需新的输入和有效授权。
- 任务操作先释放网关手动会话，再调用所选任务服务。执行器互斥由 bridge 的单租约机制承担；当前网关没有根据任务状态独立拒绝手动输入的判断。
- **`manual_stop` 不调用 `/f1/stop` 或任务 `pause/cancel`。** App 红色“停止全部”按钮和空格键发送的是 `manual_stop`，不能据此认定自动清洗已停止；自动清洗应点“暂停任务”或“取消任务”，并确认任务状态。App 断线也不会主动取消自动任务。

这些行为对应 `operator_gateway/node.py`；硬件桥仍独立处理 RC 接管、通信错误、租约及命令超时。

## 连接大 L 任务

网关默认端点固定为 `/pool_cleaning/*`，没有 `cleaning_prefix` launch 参数。先按[大 L 文档](../f1_l_cleaning/README.md)启动任务，再用以下命令替代普通网关启动，将状态及三个服务重映射到 `/l_cleaning/*`：

```bash
ros2 run f1_operator_gateway f1_operator_gateway --ros-args \
  -p bind_address:=0.0.0.0 \
  -r /pool_cleaning/status:=/l_cleaning/status \
  -r /pool_cleaning/start:=/l_cleaning/start \
  -r /pool_cleaning/pause:=/l_cleaning/pause \
  -r /pool_cleaning/cancel:=/l_cleaning/cancel
```

一次只运行一个网关实例；上述命令与普通 launch 不同时使用。App 尚无任务类型选择，界面静态“单边清洗”文字不代表实际路线；实际执行内容由所连接控制器决定。初始位姿仍通过 RViz 或任务的 `set_initial_pose` 服务设置。

## 验证

构建后可在隔离测试终端运行：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
colcon test --packages-select f1_operator_gateway --return-code-on-test-failure
colcon test-result --test-result-base build/f1_operator_gateway --verbose
```

现有测试覆盖协议解析及部分输入处理，不等同于跨机 WebSocket、Wi-Fi 断连、自动任务停止或显示坐标的实机验收。
