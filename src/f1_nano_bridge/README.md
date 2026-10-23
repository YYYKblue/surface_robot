# F1 统一桥接包 f1_nano_bridge

原 ROS 包 `nano_bridge` 已更名为 `f1_nano_bridge`，工作空间更名为 `surface_ws`。当前 Nano 旧工作空间已删除，请按[从零部署指南](../../DEPLOY_NANO.md)传输源码、安装依赖并编译；[改名迁移说明](../../WORKSPACE_MIGRATION.md)只适用于仍保留旧目录的其他环境。Python 模块仍为 `nano_ros_bridge`，原生桥节点 `/nano_bridge` 与 YAML 顶层键 `nano_bridge` 对应。

修订：2026-10-24。只适配 `firmware`（back2，协议 v2） 的版本2协议。实现独占串口、独立50 Hz发送、控制租约、停止屏障、CTRL状态及同串口原生IMU。Python模块为 `nano_ros_bridge`。

本包负责设备通信、协议转换与 ROS 桥接；消息/服务定义统一在 `f1_interfaces`，清洁任务仍由对应业务包实现。IMU 只保留 F103 串口 → 原生桥 → `/f1/imu/data_raw` 接入，旧 ZMQ 接收节点及相关入口已删除。

| 模块或入口 | 职责 |
| --- | --- |
| `nano_ros_bridge.protocol/session/transport` | 串口协议、控制租约与传输 |
| `nano_ros_bridge.node`、`bridge.launch.py` | 原生串口控制、CTRL 与 IMU 遥测 |
| `nano_ros_bridge.ui_protocol/ui_gateway`、`gateway.launch.py` | Windows UI 的 `f1-ui-v2` TCP 5556 网关；只调用 ROS，不打开串口 |
| `nano_ros_bridge.imu.conversion` | 原生串口 IMU 样本校验、单位与时间转换；不提供独立节点或网络入口 |

Windows UI 使用 `ui_control.launch.py`，或分别启动 `bridge.launch.py` 与 `gateway.launch.py`；完整网络设置及启动命令见 [Windows UI 接入](WINDOWS_UI.md)。原生串口桥已包含 IMU 发布，包内只有 `f1_bridge`、`f1_ui_gateway` 两个可执行入口。ZeroMQ 依赖用于 TCP 5556 UI 网关。

## 更新现有安装

源码更新必须传播文件删除，不能只覆盖新增文件。结束本工作空间的桥接节点与构建/测试，在未加载旧 `install` 的终端中进入工作空间根目录。若存在已废弃的独立 `f1_imu_bridge` 包，先移走；再删除本包的旧生成目录，避免安装空间继续暴露已移除的入口：

```bash
bridge_backup="$(mktemp -d /tmp/f1_bridge_migration.XXXXXX)"
for tree in src build install; do
  if [ -e "$tree/f1_imu_bridge" ]; then
    mkdir -p "$bridge_backup/$tree"
    mv "$tree/f1_imu_bridge" "$bridge_backup/$tree/"
  fi
done
echo "旧 IMU 包备份：$bridge_backup"
rm -rf build/f1_nano_bridge install/f1_nano_bridge
rm -rf src/f1_nano_bridge/nano_ros_bridge/imu/__pycache__
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

上述命令适用于本项目默认的隔离安装布局。源码同步时还需删除本包遗留的旧节点、脚本、launch、YAML；推荐对源码包目录使用 `rsync --delete --exclude='__pycache__/' --exclude='*.pyc'`，再执行上述缓存与生成目录清理。当前新部署使用系统 ROS 作为基础环境，项目包统一在 `surface_ws` 内构建。

重新构建并加载环境后，`ros2 pkg executables f1_nano_bridge` 应只列出 `f1_bridge`、`f1_ui_gateway`。IMU 随 `bridge.launch.py` 启动；外部包若仍声明依赖 `f1_imu_bridge`，需改为 `f1_nano_bridge`，并移除独立 IMU 节点启动配置。

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
colcon build --symlink-install --packages-up-to f1_pool_cleaning \
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
| `/f1/imu/data_raw` | `sensor_msgs/Imu`，同串口 ACC/GYRO mask=6、纯时间转换函数 |
| `/diagnostics` | 会话与IMU诊断 |

指令定义：

| 字段 | 合同 |
| --- | --- |
| token、sequence | acquire返回的令牌；uint64在单租约内严格递增 |
| computed_at、valid_until | 同机系统时钟的原始计算时间/期限，0<TTL≤250 ms，最多容忍5 ms未来计算时间；转发不得刷新 |
| direction | STOP=0、FORWARD=1、REVERSE=2、LEFT=3、RIGHT=4，互斥 |
| speed_percent | 25/50/75；PWM档位，不是实测速度/推力，默认25 |
| mode | FRONT=0、BOTH=1、BACK=2；允许同租约切换，固件停用被排除的轮组 |
| direction_until | 可选绝对脉冲结束时间，零表示使用valid_until；设置时须在computed_at之后且不超过500 ms |
| brush_enabled/percent | 滚刷明确开关、±100%辅助速度 |
| thrust_enabled/percent | 第五路，默认不允许启用 |

等待READY → acquire →250 ms内发首个结果 → 每20～50 ms根据新输入计算并更新。重复结果不能更改原始时间。指令QoS为BEST_EFFORT/VOLATILE/KEEP_LAST(1)。无租约、未知profile、非法方向/档位、原始结果过期均不能输出运动。

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

IMU串口格式未改，要求mask=6，orientation仍标记不可用；时间为主机与设备毫秒近似对齐，不代表硬件同步。原生桥直接转换已解析的串口样本，不经过 JSON 或 ZMQ。纯雷达建图入口不变。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
source install/local_setup.bash
colcon test --packages-select f1_nano_bridge f1_pool_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_nano_bridge --verbose
colcon test-result --test-result-base build/f1_pool_cleaning --verbose
```

需要cc、pytest、pyserial、ROS Humble及只读back2源码；`F1_FIRMWARE_DIR`可覆盖默认固件目录。测试库编译到临时目录。实际运行无需携带固件源码。

IMU 测试覆盖原生样本转换、时间与有效位校验、实际固件 C 格式化器到原生解析器及 ROS 消息的转换。固件格式化测试已迁入本包，不再依赖已删除的旧解析模块；`python3-zmq` 仍用于 UI 网关及其测试。

测试覆盖v2参考帧、方向/档位/轮组、非法输入、CTRL、实际固件C的45种组合、v1拒绝、冲突辅助语义、26项固件原有回归、绝对脉冲截止、期限/停止屏障/RC、PTY及ROS默认禁用。当前和历史结果分别记录如下。不构建目标板HEX，不替代船体方向、最低档动力学、跨机DDS或水上验收。

### 2026-10-24 移除旧 IMU 接入验证

- 删除旧 ZMQ IMU 节点、可执行脚本、launch、YAML、专用说明及回环测试；原生桥直接转换串口样本，删除旧 JSON 包装。
- 原固件目录中依赖旧解析器的 IMU 测试与 C 适配器迁入本包，使用当前解析器，自动编译真实固件格式化器；保留单位、状态有效期、计数器回绕、分帧和偏航边界检查。
- 清理本机本包 `build/`、`install/` 及 Python 缓存后重新构建成功。安装索引只含 `f1_bridge`、`f1_ui_gateway`，旧 IMU 节点模块与安装文件均不存在。
- 本包 **172 项独立 pytest 测试通过，0 失败、0 错误、0 跳过**。首次 ROS/UI 两组测试因沙箱禁止枚举 UDP 网络接口失败，放行本机网络后仅补跑这两组并通过；其余组沿用本轮已通过结果。此数量不含 CTest 包装用例。
- 文档相对文件链接及 `git diff --check` 通过。本轮未向 Nano 部署，未连接设备串口或修改生产固件。

### 2026-10-22 Windows UI 网关验证

- 新写 `f1-ui-v2` 网关并完成本地构建；`ros2 pkg executables` 包含 `f1_ui_gateway`，`gateway.launch.py --show-args` 正常。
- 本包完整回归 **121 项通过、0 失败、0 错误、0 跳过**，包含原有 72 项、纯 UI 协议 47 项及真实本机 TCP ↔ ROS 2 项。CTest 另统计 9 个包装用例，`colcon test-result` 汇总 130，不能当作 130 项独立测试。
- 仓库同级现有 `UI/tests/test_stability.py` 的 10 项 Qt/UI 测试通过，包含真实 ZeroMQ 客户端往返；网关实际 launch、TCP hello/state 和进程组 SIGINT 干净退出另行检查通过。
- 测试未连接实际设备、未向 Nano 部署，Windows ↔ Nano 网络仍待现场验证。运行步骤见 [Windows UI 接入](WINDOWS_UI.md)。

### 2026-10-22 合并、改名与 ZMQ 补测记录（加入 UI 网关前）

以下保留当时测试结果；旧 ZMQ IMU 节点及其回环测试已于 2026-10-24 删除。

- `surface_ws` 五个包全新构建通过，`f1_nano_bridge` 的两个启动入口和安装路径检查通过。
- 当时本包 pytest 结果为 **72 项通过、0 失败、0 错误、0 跳过**：协议23、会话15、固件互通6、PTY/时钟3、ROS5、IMU转换19、ZMQ回环1。
- 完整回归时 ZMQ 回环因缺少依赖跳过；安装 PyZMQ 22.3.0 / ZeroMQ 4.3.4 后已单独补跑通过，模拟数据正确转换为 ROS IMU，无输入时不发布伪造样本。
- 结果来自本机测试，没有连接实际硬件或部署到 Nano。全工作空间结果见[工作空间验证记录](../../README.md#2026-10-22-本地验证记录)。

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
