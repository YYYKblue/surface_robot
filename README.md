# surface_ws ROS 2 工作空间

本工作空间运行 PAVO2 纯雷达建图、F103 串口桥、清洗任务和新版 Qt 操作员 App 网关。目标环境是 ROS 2 Humble / Ubuntu 22.04 / ARM64 Nano。

## 功能包

| 功能包 | 职责 |
|---|---|
| `f1_interfaces` | 执行器、状态、感知和清洗任务接口 |
| `f1_nano_bridge` | F103 串口、控制状态、ACK、IMU 和 ROS 执行器接口 |
| `f1_operator_gateway` | Qt App 的 WebSocket 控制与遥测网关 |
| `f1_mapping_bringup` | Cartographer 2D 纯雷达建图 |
| `f1_pool_cleaning` | 当前单边沿墙清洗任务 |

旧 Windows UI、旧 ZMQ 网关、`f1-ui-v2` 协议及其测试已经删除。硬件桥仍保留内部 ROS 控制会话和固件超时保护，但这些机制不暴露给 App。

清洗任务的后续开发、阶段门槛和实船验收顺序见[沿边清洗开发与验收文档](docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)。

## 新 App 链路

```text
Windows Qt operator_app
        │ WebSocket :8765
        ▼
f1_operator_gateway
        │ ROS 2
        ├── f1_nano_bridge → F103
        ├── /map、TF、/pavo2_scan → 地图与雷达显示
        └── /pool_cleaning/* → 清洗任务
```

App 只发送最新手动输入、停止和任务操作。网关自动使用现有控制服务，并负责输入超时、断线停止、单操作端和任务互斥。地图只在更新时发送，雷达、位姿和状态按遥测频率发送。

## PC 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to f1_operator_gateway
source install/local_setup.bash
```

## Nano 启动

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash

ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

网关默认 `127.0.0.1:8765`。跨机时使用 `bind_address:=0.0.0.0`，App 连接 Nano 实际 IP。入口当前没有认证，只放在可信局域网。

## 验证边界

```bash
colcon test --packages-select f1_nano_bridge f1_operator_gateway \
  --return-code-on-test-failure
```

桌面测试不替代真实 Wi-Fi、跨机 WebSocket、雷达 TF 对齐、执行器方向和水上验收。完整 Nano 部署步骤仍见 [`DEPLOY_NANO.md`](DEPLOY_NANO.md)。
