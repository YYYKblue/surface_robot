# Windows F1 UI 接入

链路：Windows `F1Console.cmd` → ZeroMQ DEALER/ROUTER、TCP 5556、`f1-ui-v2` → Nano `f1_ui_gateway` → ROS `f1_nano_bridge` → F103。

网关代码已包含在本包中，无需旧工作空间或旧接口包。网关使用 `f1_interfaces`，不打开串口。Windows 只安装 UI 的 Python 依赖，不安装 ROS、不设置 DDS 或 `ROS_DOMAIN_ID`。

## 1. 在 Nano 构建

在 Nano 的 `/home/nvidia/surface_ws` 同步当前源码后：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 1 \
  --packages-up-to f1_nano_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 pkg executables f1_nano_bridge
ros2 launch f1_nano_bridge gateway.launch.py --show-args
```

应包含 `f1_bridge`、`imu_bridge`、`f1_ui_gateway`。运行依赖已在 `package.xml` 声明，包含 `python3-zmq`。

## 2. 启动原生桥和网关

Nano 两个终端都先执行以下环境设置。域 33 是示例；两个节点取值必须相同，且使用相同 RMW。Cyclone DDS 需已安装（见工作空间部署指南）。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

终端 1，首次只启用串口、完成停止握手，观察状态：

```bash
ros2 launch f1_nano_bridge bridge.launch.py serial_enabled:=true
```

确认实机方向后，结束上面的桥进程，再显式启用运动：

```bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true
```

终端 2，启动网关，允许 Windows 跨机访问：

```bash
ros2 launch f1_nano_bridge gateway.launch.py bind_address:=0.0.0.0
```

默认 `127.0.0.1` 只允许 Nano 本机连接。`0.0.0.0` 监听所有 IPv4 接口；Windows 应填写 Nano 的实际局域网 IP，不能填写 `0.0.0.0`。

第五路默认禁用。独立启动时，bridge 和网关两条命令均增加 `allow_fifth_thruster:=true`。bridge 省略该参数时保留 YAML 值，网关会同时核对 bridge 发布的能力，避免界面显示可用而 bridge 拒绝。

也可使用统一入口代替上述两个进程，同时开启第五路：

```bash
ros2 launch f1_nano_bridge ui_control.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true \
  allow_fifth_thruster:=true bind_address:=0.0.0.0
```

统一入口所有使能参数默认仍为 false。更新后应重新构建 `f1_interfaces` 与 `f1_nano_bridge` 并重启使用 `ControlState` 的节点；该消息新增 `allow_fifth_thruster` 和 `last_stop_reason`。

## 3. Windows 连接和网络检查

Windows 与 Nano 接入相互可达的局域网，双击 `F1Console.cmd`，输入 Nano 的 IP，例如 `192.168.1.188`。UI 不会自动启动远程桥或网关。

Nano 检查 IP、监听及 ROS 状态：

```bash
hostname -I
ss -ltnp 'sport = :5556'
ros2 topic echo /f1/control_state
```

运行 `ros2` 检查的终端也应加载相同的工作空间、域和 RMW。Windows PowerShell 检查 TCP 可达性（替换实际 IP）：

```powershell
Test-NetConnection 192.168.1.188 -Port 5556
```

若失败，依次核对 IP、网关监听地址、Nano 防火墙的 TCP 5556 入站规则、WiFi 客户端隔离或访客网络限制。同一 WiFi 名称不保证终端间可通信。

| 现象 | 检查 |
| --- | --- |
| TCP 连接失败 | 监听是否为 `0.0.0.0:5556`；地址、路由、防火墙及 AP 隔离 |
| TCP 可达但无桥状态 | 两节点的 `ROS_DOMAIN_ID`、RMW、工作空间及 `f1_interfaces` 是否一致 |
| 状态 MONITOR、无法申请控制 | 桥尚未启用运动或方向确认 |
| 状态 RC / HANDSHAKE / FAULT | 遥控来源、串口握手、固件及 CTRL/ACK 状态 |
| 控制失效后恢复连接仍不运动 | 需重新明确申请控制；不自动恢复动作 |

TCP 5556 没有用户身份认证，应只在受信任局域网使用，不暴露公网。UI 的停止操作仅释放本连接持有的控制令牌，不调用可影响其他 ROS 控制者的全局 `/f1/stop`。

## 4. 协议与验证边界

沿用 `f1-ui-v2` 协议，命令新增可选 `input_age_ns`。网关每 20 ms 发送状态和一次性票据，票据在 Nano 创建后 150 ms 失效，与 bridge 原有最大 TTL 一致。GUI 每 20 ms 采样，独立网络线程收发，状态显示每 100 ms 刷新。网关以“票据原始创建时间 − 输入快照年龄 + 150 ms”确定命令截止时间，网络往返和 ROS 等待均消耗该预算，转发不刷新时效；Windows 与 Nano 无需同步时钟。旧客户端字段仍可解析，但应一起更新 Windows UI 与 Nano 源码，获得线程隔离与旧快照保护。

命令须完整包含方向、档位、轮组、辅助开关及比例；拒绝非法类型、重复序号、重复/过期票据和未启用的第五路。同一租约允许换轮组；Qt 切换时仅清除主轮方向，辅助保持。鼠标移出、下拉菜单和失焦不触发停止或释放。GUI 150 ms 没有新采样时本地释放；网关 250 ms 无命令、桥状态过期、RC 接管或连接/epoch 变化后仍撤销本连接控制；迟到的申请结果也会释放。桥仍独立执行其 150 ms 最大 TTL 和串口故障规则。

时间跳变超过 50 ms 后网关锁存拒绝控制，需排除时钟问题并重启；时钟采样前后被调度暂停超过 5 ms 的样本不用于判断跳变。Qt 优先处理同批最新票据，再处理申请回复；旧租约的指令失败回复不影响新租约。界面保留最近控制中断原因。网络发送成功、ROS 服务成功和固件 ACK 都不等于机械停机；PWM 状态是目标值。

本地验证可运行：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select f1_nano_bridge --return-code-on-test-failure
colcon test-result --test-result-base build/f1_nano_bridge --verbose
```

`test_ui_protocol.py` 验证纯协议，`test_ui_gateway.py` 验证真实本机 TCP ↔ ROS（串口关闭或模拟 ROS 后端）。仓库同级 `UI/tests/test_stability.py` 另验证现有 Qt 客户端和真实 ZMQ 往返。这些测试不替代 Windows ↔ Nano 网络、实机方向和执行器验收。

2026-10-22 最新本地结果：UI、bridge 全套及固件 control_mux / host_uart 两模块合计 222 项通过，接口包与 bridge 构建通过；CTest 注册 10 组含 ui_timing。新增真实 Qt 网络线程、输入冻结、PTY 假写超时 / 背压及 Gateway → Session → C 固件时序回归。详细证据、固件只读审查、串口诊断和部署步骤见 [链路稳定性报告](../../../docs/CONTROL_LINK_STABILITY_2026-10-22.md)。未部署到 Nano，未连接实际硬件，本轮不刷固件。
