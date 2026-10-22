# F103 原生 ROS 2 bridge（back2 / v2）

修订：2026-10-21。只适配 `firmware`（back2，协议 v2） 的版本2协议，固件不修改。实现独占串口、独立50 Hz发送、控制租约、停止屏障、CTRL状态及同串口原生IMU。Python模块为 `nano_ros_bridge`，与根目录旧 `nano_bridge` 区分。

## 与上一版的变化

- 主轮从四电机独立百分比改为一个整车方向、三档PWM和轮组；不再由Nano混控或做前电机反向。
- 外层帧版本改为02；CTRL严格要求版本2、20字段，增加speed_percent、direction和DIR_CONFLICT。
- `ActuatorCommand`移除四个电机字段，新增direction、speed_percent和可选direction_until。所有消息使用者必须重建并重启，不能混用旧install。
- 固件配置固定 `f1_direction_gear_v2`；旧profile拒绝启动。会话、原始期限、RC接管后不恢复、默认禁用等机制保留。

旧根目录桥和旧UI仍采用AMBT/ZMQ，不兼容back2。本包不接收旧UI控制，不迁移H7/相机。不能同时启动占用F103串口的旧 `nano-bridge.service`、`usv-motor.service` 或其他IMU串口节点，也不能将旧桥作为back2回退。

## 构建与默认运行

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/f1_mapping_ws
colcon build --symlink-install --packages-up-to f1_wall_cleaning \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 launch nano_bridge bridge.launch.py
```

默认serial_enabled=false、motion_enabled=false，不开串口，不发硬件字节。参数只读且拒绝仿真时间。默认配置在 `config/bridge.yaml`，通过 `config:=/absolute/path/bridge.yaml` 使用运行副本。

```bash
ros2 topic echo /f1/control_state
ros2 interface show f1_interfaces/msg/ActuatorCommand
ros2 service call /f1/stop std_srvs/srv/Trigger '{}'
```

确认板上back2且其他串口消费者已停用后，`serial_enabled:=true` 可建立会话。此模式会发送STOP/辅助关闭并完成300 ms握手，仍不能申请运动租约；它不是被动监听。实机方向确认后才显式使用：

```bash
ros2 launch nano_bridge bridge.launch.py \
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
| mode | FRONT=0、BOTH=1、BACK=2；同租约改变轮组会撤销授权 |
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
- 短写、串口错误和接收积压关闭链路；重连不保存运动。系统时钟相对monotonic变化>50 ms锁存FAULT。
- 独立线程20 ms调度，Linux/Python非硬实时。状态记录最大发送间隔。串口采用exclusive、描述符检查和TIOCEXCL，仍需停用已知旧消费者。

固件100 ms失联、RC五秒静止交回逻辑保持不变。CH03/04端点及SW2变化现在参与接管，CH01/02/H不参与。`/f1/stop`不能覆盖RC，也不是独立硬件急停。PWM字段是目标值，不是当前斜坡、实测转速或推力。

IMU格式未改，要求mask=6，orientation仍标记不可用；时间为主机与设备毫秒近似对齐，不代表硬件同步。不要同时启动旧ZMQ IMU节点。纯雷达建图入口不变。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/f1_mapping_ws
source install/local_setup.bash
colcon test --packages-select nano_bridge f1_wall_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/nano_bridge --verbose
colcon test-result --test-result-base build/f1_wall_cleaning --verbose
```

需要cc、pytest、pyserial、ROS Humble及只读back2源码；`F1_FIRMWARE_DIR`可覆盖默认固件目录。测试库编译到临时目录。实际运行无需携带固件源码。

本轮覆盖v2参考帧、方向/档位/轮组、非法输入、CTRL、实际固件C的45种组合、v1拒绝、冲突辅助语义、26项固件原有回归、绝对脉冲截止、期限/停止屏障/RC、PTY及ROS默认禁用。最终结果见本文件末尾验证记录。不构建目标板HEX，不替代船体方向、最低档动力学、跨机DDS或水上验收。

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
