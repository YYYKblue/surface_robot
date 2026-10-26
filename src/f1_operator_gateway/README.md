# f1_operator_gateway

新版 Qt 操作员 App 的 Nano 侧入口。它不是 ROS 的控制权标准，而是一个应用网关：App 只发送当前输入，网关自动使用现有 `/f1/acquire_control`、`/f1/release_control` 服务，将命令发布到 `/f1/actuator_command`，并在输入停止、WebSocket 断开或网关退出时释放内部会话。

网关同时订阅 `/map`、`/tf` 中的 `map → base_link`、`/pavo2_scan`、`/f1/control_state` 和 `/pool_cleaning/status`，通过 WebSocket 提供给 Qt App。地图只在更新时发送，雷达和状态按遥测频率发送；发送队列不保存历史手动输入。

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

`0.0.0.0` 只代表监听地址，不是 Windows 连接地址。当前 WebSocket 入口没有登录认证，应该只放在可信局域网；需要跨网或多用户时，应在此处增加 TLS/认证层。

依赖 `python3-websockets`（Nano 上可通过 `sudo apt install python3-websockets` 安装）。第五电推是否可用由 Nano 的 `allow_fifth_thruster` 参数决定；未启用时网关只屏蔽第五电推字段，不会让主驱动失效。控制 App 不需要安装 ROS，也不接触 token、ROS QoS 或执行器协议。当前网关只允许一个操作端连接；第二个连接不能发送手动指令。
