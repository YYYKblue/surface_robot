# F1 统一桥接包 f1_nano_bridge

原 ROS 包 `nano_bridge` 已更名为 `f1_nano_bridge`，工作空间更名为 `surface_ws`。当前 Nano 旧工作空间已删除，请按[从零部署指南](../../DEPLOY_NANO.md)传输源码、安装依赖并编译；[改名迁移说明](../../WORKSPACE_MIGRATION.md)只适用于仍保留旧目录的其他环境。Python 模块仍为 `nano_ros_bridge`，原生桥节点 `/nano_bridge` 与 YAML 顶层键 `nano_bridge` 对应。

修订：2026-10-22。只适配 `firmware`（back2，协议 v2） 的版本2协议，固件不修改。实现独占串口、独立50 Hz发送、控制租约、停止屏障、CTRL状态及同串口原生IMU。Python模块为 `nano_ros_bridge`，与根目录旧 `nano_bridge` 区分。

2026-10-22：原 `f1_imu_bridge` 已并入本包，工作空间只保留一个桥接功能包 `f1_nano_bridge`。本包负责设备通信、协议转换与 ROS 桥接；消息/服务定义统一在 `f1_interfaces`，清洁任务仍由对应业务包实现。

| 模块或入口 | 职责 |
| --- | --- |
| `nano_ros_bridge.protocol/session/transport` | 串口协议、控制租约与传输 |
| `nano_ros_bridge.node`、`bridge.launch.py` | 原生串口控制、CTRL 与 IMU 遥测 |
| `nano_ros_bridge.ui_protocol/ui_gateway`、`gateway.launch.py` | Windows UI 的 `f1-ui-v2` TCP 5556 网关；只调用 ROS，不打开串口 |
| `nano_ros_bridge.imu.conversion` | 两条 IMU 接入链路共用的数据校验、单位与时间转换 |
| `nano_ros_bridge.imu.node`、`imu.launch.py` | 旧网关 ZMQ → ROS IMU 接入，详见 [IMU 说明](IMU.md) |

Windows UI 使用 `bridge.launch.py` + `gateway.launch.py`，完整网络设置及启动命令见 [Windows UI 接入](WINDOWS_UI.md)。原生串口桥已包含 IMU 发布，无需额外启动 `imu.launch.py`。旧 ZMQ 节点的 ROS 名称仍为 `f1_imu_bridge`，保留原 YAML 参数匹配，但它已不是独立功能包。原生串口入口与 Python 模块的既有使用方式不变。

## 从独立 IMU 包迁移

此节只供旧包目录仍存在的环境使用。已清空的 Nano 直接执行从零部署流程。

先结束本工作空间的桥接节点与构建/测试，同步新源码。在未加载旧 `install` 的新终端进入工作空间根目录，移走遗留的旧包源码和生成目录，再重新构建整个工作空间：

```bash
bridge_backup="$(mktemp -d /tmp/f1_bridge_migration.XXXXXX)"
for tree in src build install; do
  if [ -e "$tree/f1_imu_bridge" ]; then
    mkdir -p "$bridge_backup/$tree"
    mv "$tree/f1_imu_bridge" "$bridge_backup/$tree/"
  fi
done
echo "旧 IMU 包备份：$bridge_backup"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

仅覆盖同步源码会留下旧包。上述命令适用于本项目默认的隔离安装布局；当前新部署使用系统 ROS 作为基础环境，项目包统一在 `surface_ws` 内构建。

旧入口 `ros2 launch f1_imu_bridge imu.launch.py` 改为 `ros2 launch f1_nano_bridge imu.launch.py`，旧 `ros2 run f1_imu_bridge imu_bridge` 改为 `ros2 run f1_nano_bridge imu_bridge`。Python 导入改用 `nano_ros_bridge.imu.conversion` 或 `nano_ros_bridge.imu.node`。外部包若仍声明依赖 `f1_imu_bridge`，需改为 `f1_nano_bridge`。

## 与上一版的变化

- 主轮从四电机独立百分比改为一个整车方向、三档PWM和轮组；不再由Nano混控或做前电机反向。
- 外层帧版本改为02；CTRL严格要求版本2、20字段，增加speed_percent、direction和DIR_CONFLICT。
- `ActuatorCommand`移除四个电机字段，新增direction、speed_percent和可选direction_until。所有消息使用者必须重建并重启，不能混用旧install。
- 固件配置固定 `f1_direction_gear_v2`；旧profile拒绝启动。会话、原始期限、RC接管后不恢复、默认禁用等机制保留。

旧 AMBT 或 PUB/SUB UI 不兼容 back2；当前 Windows F1 UI 通过本包的 `f1-ui-v2` DEALER/ROUTER 网关接入，不迁移 H7/相机。不能同时启动占用 F103 串口的旧 `nano-bridge.service`、`usv-motor.service` 或其他 IMU 串口节点，也不能将旧桥作为 back2 回退。

## 构建与默认运行

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
colcon build --symlink-install --packages-up-to f1_wall_cleaning \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 launch f1_nano_bridge bridge.launch.py
```

默认serial_enabled=false、motion_enabled=false，不开串口，不发硬件字节。参数只读且拒绝仿真时间。默认配置在 `config/bridge.yaml`，通过 `config:=/absolute/path/bridge.yaml` 使用运行副本。

```bash
ros2 topic echo /f1/control_state
ros2 interface show f1_interfaces/msg/ActuatorCommand
ros2 service call /f1/stop std_srvs/srv/Trigger '{}'
```

确认板上back2且其他串口消费者已停用后，`serial_enabled:=true` 可建立会话。此模式会发送STOP/辅助关闭并完成300 ms握手，仍不能申请运动租约；它不是被动监听。实机方向确认后才显式使用：

```bash
ros2 launch f1_nano_bridge bridge.launch.py \
  serial_enabled:=true motion_enabled:=true direction_confirmed:=true
```

三个启动开关均默认false并覆盖YAML同名值。本次没有部署服务、连接实际串口、烧录或发送真实运动。

## ROS接口

| 接口 | 类型与用途 |
| --- | --- |
| `/f1/actuator_command` | `f1_interfaces/ActuatorCommand`，方向、档位、轮组、辅助目标及期限 |
| `/f1/acquire_control` | `AcquireControl`，READY后授予唯一令牌 |
| `/f1/release_control` | `ReleaseControl`，仅持有者令牌可撤销自己的控制 |
| `/f1/stop` | `std_srvs/Trigger`，撤销Nano授权并请求STOP，不能覆盖RC |
| `/f1/control_state` | `ControlState`，授权、源/epoch/ACK、方向/档位、PWM目标与数据年龄 |
| `/f1/imu/data_raw` | `sensor_msgs/Imu`，ACC/GYRO mask=6、复用纯时间转换函数 |
| `/diagnostics` | 会话与IMU诊断 |

指令定义：

| 字段 | 合同 |
| --- | --- |
| token、sequence | acquire返回的令牌；uint64在单租约内严格递增 |
| computed_at、valid_until | 同机系统时钟的原始计算时间/期限，0<TTL≤150 ms，最多容忍5 ms未来计算时间；转发不得刷新 |
| direction | STOP=0、FORWARD=1、REVERSE=2、LEFT=3、RIGHT=4，互斥 |
| speed_percent | 25/50/75；PWM档位，不是实测速度/推力，默认25 |
| mode | FRONT=0、BOTH=1、BACK=2；允许同租约切换，固件停用被排除的轮组 |
| direction_until | 可选绝对脉冲结束时间，零表示使用valid_until；设置时须在computed_at之后且不超过500 ms |
| brush_enabled/percent | 滚刷明确开关、±100%辅助速度 |
| thrust_enabled/percent | 第五路，默认不允许启用 |

等待READY → acquire →150 ms内发首个结果 → 每20～50 ms根据新输入计算并更新。重复结果不能更改原始时间。指令QoS为BEST_EFFORT/VOLATILE/KEEP_LAST(1)。无租约、未知profile、非法方向/档位、原始结果过期均不能输出运动。

脉冲绝对期限只截断动作和辅助输出，不撤销仍新鲜的租约；上层随后发送新计算的STOP，等待后再开始下一脉冲。50 Hz重发不会延长脉冲。若上层停止计算，valid_until到期仍会撤销令牌、等待STOP确认；不能依靠持续发序号让旧动作续命。

不订阅 `/cmd_vel`，不接受旧逐电机字段，不推断未标定动力学。服务成功不是机械停止证明。DDS租约不是身份认证，应在可信ROS网络使用，不把录包中的令牌用于实机。

## 线协议

USART1，460800、8N1。39字节：`A5 5A 02 19` + epoch LE32 + seq LE32 + SBUS25 + CRC16 LE。CRC为CCITT-FALSE（初值FFFF、多项式1021），覆盖偏移2～36。

| 通道 | 编码 |
| --- | --- |
| CH01/02 | 固定1500，不是电机输出 |
| CH03/04 | STOP=1500/1500，前进=1950/1500，后退=1050/1500，左转=1500/1050，右转=1500/1950 |
| CH05 | FRONT/BOTH/BACK=1050/1500/1950 |
| CH06 | 25/50/75=1050/1500/1950 |
| CH08/09 | 辅助有符号速度，保留原死区反算 |
| CH10/11 | 1050关闭、1950开启 |
| 其他 | 1500 |

每通道raw=`(8*servo-6990+2)//5`，16×11bit低位优先。正常flags/footer=0。默认STOP取BOTH、25%、辅助关闭；对应固件参考帧见协议测试。

固件按方向表处理安装符号。BOTH、25%、前进的CTRL目标BL/BR/FL/FR=`1625/1625/1375/1375`；LEFT=`1375/1625/1625/1375`；50/75档偏移250/375 μs。Nano不再做前轮反向。

CTRL为版本2共20字段：末尾新增speed_percent、direction，结果增加DIR_CONFLICT。旧版18字段拒绝。DIR_CONFLICT主轮停止但辅助独立；活动租约收到非OK时被撤销，后续完整STOP关闭辅助。

## 故障与时序

- 串口打开等v2 CTRL，再以其epoch发送STOP。累计ACK确认、armed/OK且辅助关闭后进入READY；固件回中至少300 ms。
- RC接管、epoch变化、设备时钟重置或重连立即丢弃任务；RC期间不发送Nano帧。交回后重新握手，等待明确acquire，不恢复旧动作。
- CTRL≥400 ms陈旧不再发送；ACK≥400 ms不前进撤销；发帧间隔≥80 ms撤销，不追赶补发运动。
- 原始命令到期、stop或release均撤销、清空目标；STOP屏障累计ACK后才可重新申请。
- `ControlState.last_stop_reason` 保留本次租约失效原因，避免停止握手覆盖原因；新授权清空该字段。`allow_fifth_thruster` 报告 bridge 的第五路配置，网关结合自己的配置决定 UI 能力。
- 短写、串口错误和接收积压关闭链路；重连不保存运动。系统时钟相对monotonic变化>50 ms锁存FAULT。
- 独立线程20 ms调度，Linux/Python非硬实时。状态记录最大发送间隔。串口采用exclusive、描述符检查和TIOCEXCL，仍需停用已知旧消费者。

固件100 ms失联、RC五秒静止交回逻辑保持不变。CH03/04端点及SW2变化现在参与接管，CH01/02/H不参与。`/f1/stop`不能覆盖RC，也不是独立硬件急停。PWM字段是目标值，不是当前斜坡、实测转速或推力。

IMU格式未改，要求mask=6，orientation仍标记不可用；时间为主机与设备毫秒近似对齐，不代表硬件同步。不要同时启动旧ZMQ IMU节点。纯雷达建图入口不变。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
source install/local_setup.bash
colcon test --packages-select f1_nano_bridge f1_wall_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_nano_bridge --verbose
colcon test-result --test-result-base build/f1_wall_cleaning --verbose
```

需要cc、pytest、pyserial、ROS Humble及只读back2源码；`F1_FIRMWARE_DIR`可覆盖默认固件目录。测试库编译到临时目录。实际运行无需携带固件源码。

合并后 `f1_nano_bridge` 测试还包含 IMU 转换和旧 ZMQ→ROS 回环。旧 ZMQ 入口及回环测试需要 `python3-zmq`；缺少此依赖时回环测试会 skipped，原生串口桥与 IMU 转换模块可独立导入。

测试覆盖v2参考帧、方向/档位/轮组、非法输入、CTRL、实际固件C的45种组合、v1拒绝、冲突辅助语义、26项固件原有回归、绝对脉冲截止、期限/停止屏障/RC、PTY及ROS默认禁用。当前和历史结果分别记录如下。不构建目标板HEX，不替代船体方向、最低档动力学、跨机DDS或水上验收。

### 2026-10-22 Windows UI 网关验证

- 新写 `f1-ui-v2` 网关并完成本地构建；`ros2 pkg executables` 包含 `f1_ui_gateway`，`gateway.launch.py --show-args` 正常。
- 本包完整回归 **121 项通过、0 失败、0 错误、0 跳过**，包含原有 72 项、纯 UI 协议 47 项及真实本机 TCP ↔ ROS 2 项。CTest 另统计 9 个包装用例，`colcon test-result` 汇总 130，不能当作 130 项独立测试。
- 仓库同级现有 `UI/tests/test_stability.py` 的 10 项 Qt/UI 测试通过，包含真实 ZeroMQ 客户端往返；网关实际 launch、TCP hello/state 和进程组 SIGINT 干净退出另行检查通过。
- 测试未连接实际设备、未向 Nano 部署，Windows ↔ Nano 网络仍待现场验证。运行步骤见 [Windows UI 接入](WINDOWS_UI.md)。

### 2026-10-22 合并、改名与 ZMQ 补测记录（加入 UI 网关前）

- `surface_ws` 五个包全新构建通过，`f1_nano_bridge` 的两个启动入口和安装路径检查通过。
- 当时本包 pytest 结果为 **72 项通过、0 失败、0 错误、0 跳过**：协议23、会话15、固件互通6、PTY/时钟3、ROS5、IMU转换19、ZMQ回环1。
- 完整回归时 ZMQ 回环因缺少依赖跳过；安装 PyZMQ 22.3.0 / ZeroMQ 4.3.4 后已单独补跑通过，模拟数据正确转换为 ROS IMU，无输入时不发布伪造样本。
- 结果来自本机测试，没有连接实际硬件或部署到 Nano。全工作空间结果见[工作空间验证记录](../../README.md#2026-10-22-本地验证记录)，复现命令见 [ZMQ 补测说明](IMU.md#补跑-zmq-回环测试)。

### 2026-10-21 back2 验证记录

- 相关5包完成构建；最终4个变更包再次构建完成。
- bridge实际pytest：协议23、会话15、固件互通6、PTY/时钟3、ROS5，合计 **52项通过，0失败、0跳过**。
- 固件互通中另调用back2原有 **26项控制回归，全部通过**；45种方向/档位/轮组组合也与实际C输出一致。
- 沿墙包 **49项通过**，包括15项感知、26项控制、8项ROS联调；其中1条服务应答超时警告保留记录，不能据此认定跨机DDS已验收。
- 修正无输入节点在组SIGINT与launch转发信号下的退出等待；默认bridge和两个沿墙节点正常退出。串口和运动默认关闭。
- `colcon test-result`分别报告57和52，包含5和3个CTest包装用例；实际pytest合计 **101项**，不能把109计为独立算法测试。
- 验证前后back2目录540个文件内容哈希一致。固件源码、HEX及工程文件未修改；没有实机运动或部署。

本次依据的固件SHA-256：

```text
29fd93cd28b759a06c5348622b48dec52d2775f464fcf90ed3e8a78a900fb60c USER/host_protocol.c
fc79e6ccb37595acd9aa0bff86a8731ea499b7f58a575eee1f20ebdb3cce4f46 USER/control_mux.c
6531538aacc6c361463cf91d281b3866a2fa6d1063f8e5e4f6b1c30f671ad981 USER/rc_control.c
81d428fa6bb8f78133317eb914bdcb75ed927ce64a8d61c587bc29e024ceef37 USER/imu_report.c
```
