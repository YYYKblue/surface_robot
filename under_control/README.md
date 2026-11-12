# H7 水下手动控制服务

Windows App 同时连接水面 `:8765` 与水下 `:8766`。本服务在同一台 Nano 上运行，集成水下网络网关、控制会话和 RS485 桥接，不依赖 ROS，也不占用 F103 串口。

2026-11-12 已部署至 `192.168.1.188`，新服务开机启用，旧 `nano-h7-bridge.service` 已停止并禁用。H7 适配器缺席时稳定待机，详细状态、验证范围及备份路径见[部署记录](../docs/UNDER_CONTROL_NANO_DEPLOYMENT.md)。新增缺席启动/模拟拔插测试后，水下服务测试为 35 项通过。

硬件契约为 `under/H7firmware/Communication/PROTOCOL.md` 的原电机适配版 V6：115200/8N1，16 字节下发、17 字段 `UNDER,6,...` 回包。USB 转 RS485 连接 H7 UART8 对应的板载 RS485 接口，不接内部 TTL 引脚。

## 启动与部署

H7 不要求每次开机都连接。systemd 服务常驻并监听 8766：适配器缺席时为 OFFLINE，每秒检查一次设备，仅在状态变化时记录日志，不反复重启服务。插入后自动发现和停止握手；拔掉后撤销控制，重新插入仍须新的方向操作。不要为 unit 增加串口设备的 Requires/BindsTo/ConditionPathExists，否则会妨碍这一行为。设备路径应使用实际适配器的 `/dev/serial/by-id/`，不要自动选择任意 ttyUSB/ttyACM。

在 Nano 工作空间根目录安装依赖并启动无串口服务（此时 App 始终禁止水下运动）：

```bash
python3 -m pip install -r under_control/requirements.txt
python3 -m under_control --bind 0.0.0.0
```

核对实际设备身份、已有 H7 服务和进程占用后使用稳定设备路径。只指定串口时会发送发现和停止握手，运动仍禁用；它不是被动监听，会停止 Nano 控制源下的已有动作。以下设备路径需替换：

```bash
python3 -m under_control --bind 0.0.0.0 --serial-port /dev/serial/by-id/实际H7适配器
# 允许新的人工操作；启动或重连不会自动运动。
python3 -m under_control --bind 0.0.0.0 --serial-port /dev/serial/by-id/实际H7适配器 --motion-enabled
```

开机服务模板使用 `nvidia` 和 `/home/nvidia/surface_ws`；现场不同需修改 unit。编辑 `deploy/under-control.env.example` 的设备路径，将它安装为 `/etc/under-control.env`，将 `deploy/under-control.service` 安装到 `/etc/systemd/system/`。确认旧进程已释放同一串口后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now under-control.service
journalctl -u under-control.service -n 100 --no-pager
```

模板默认 `UNDER_MOTION_ENABLED=false`。核验停止握手与板端反馈后，设置为 `true` 并重启此服务才允许运动。2026-11-12 实际部署的配置已为 `true`，但当时 H7 缺席，尚未完成实机握手与运动验收；不要用模板覆盖现有配置。首次安装、升级和排查命令见 [Nano 部署指南](../DEPLOY_NANO.md#水下-h7独立服务与可选设备自启动)。停止并禁用 `under-control.service` 即可撤销部署，水面 `f1-operator.service` 不依赖它；恢复旧 H7 服务前先核对串口已释放。网络用于现有可信局域网，未增加登录或 TLS。

## App 操作

```powershell
python operator_app/main.py --url ws://Nano地址:8765
# 水下默认同一地址的 8766，也可指定：
python operator_app/main.py --url ws://Nano地址:8765 --under-url ws://Nano地址:8766
python operator_app/main.py --mock
python operator_app/main.py --url ws://Nano地址:8765 --surface-only
```

- 水面 W/A/S/D；水下 ↑/↓/←/→ 和箭头按钮，按住运动、松开停止。←/→ 是转向。同台同时按多个方向键输出 STOP；两台按键独立。
- 两台各有百分比和停止按钮。驾驶时按空格、顶部“两台手动停止”、窗口失焦、输入框获得焦点、触摸取消、日志导出或退出均撤销两台手动意图。编辑文本或数值时，方向键和空格交给输入控件，不触发驾驶；进入编辑前已清除两台意图。
- 水面自动任务仅锁定水面手动控制；水下仍可操作。一台断线、过期或控制源变化只清除该台旧操作。
- “两台手动停止”不取消水面自动任务，需使用任务暂停/取消；也不能覆盖 H7 遥控接管，需遥控器回中停车。
- 水下前进带动前滚刷，所有运动带动顶升目标 40%；停止或 0% 停止全部水下输出。前滚刷无独立启停；换向轮可能先刹车等待 1 秒。调到 0 再调高需重新按方向。
- 百分比不是实测速度，回包兼容值不是实际 PWM 脉宽。Mock 只模拟界面状态，不模拟机械运动。

## 会话与时序

串口只允许一个请求等待回包，正常目标周期 50 ms，回包完成后至少留 3 ms 总线空闲。发现用 epoch=0、STOP/0；之后使用板端 epoch 和递增 uint32 序号完成至少 300 ms 停止握手，并等待 H7 的 3 秒上电准备期。经匹配 ACK 确认中性、armed=1、result=OK 后才允许运动。

App 保留独立收发和 250 ms GUI 意图期限。服务串口线程独立检查 250 ms App 输入期限，网络线程停顿不能无限续发旧动作。150 ms 无完整串口回包进入断线恢复；反馈新鲜度门限为 250 ms。H7 自身 300 ms 无新有效控制即停车。这些局部期限不是任意端到端故障的统一停车时间。

断线、超时、RC 接管/归还、epoch 改变会撤销操作并更新 `control_generation`。App 输入必须带当前代次和递增 `seq`；旧代次、重复或倒退序号不续期。H7 重启可能重新从 epoch=1 开始，因此另检查板端 ms 倒退，允许正常 uint32 回绕。重连只发送停止，不重放运动。

RC 回包 ACK 仍属于 Nano 请求，不能作为遥控动作确认。RC 期间仅查询和显示，H7 保持遥控优先；归还后重新停止握手。

WebSocket 请求必须带 `protocol="under-operator-v1"`、`device="h7_under_original_v6"`。首包 `hello` 带 `version=1`；只有 `manual_input/manual_stop`，没有任务或地图操作。例如：

```json
{"protocol":"under-operator-v1","device":"h7_under_original_v6","op":"manual_input","seq":42,"control_generation":8,"direction":"LEFT","speed_percent":25}
```

代次必须读取当前遥测，示例不能重复发送。`hello` 返回会话、客户端 ID 与能力；10 Hz `telemetry` 返回串口身份、控制代次、来源、owner、armed/result、回包年龄、板端方向/百分比、停止确认和诊断计数。界面将 GUI 排队时间计入回包年龄。

## 验证

```bash
python3 -m pytest under_control/test -q
QT_QPA_PLATFORM=offscreen python3 -m pytest operator_app/test -q
```

测试包含协议示例、握手/超时/ACK/代次，真实 H7 `nano_comm.c` 和 `motor.c` 的本机 C 编译，以及 WebSocket→pyserial PTY→H7 C 端到端测试，不打开实际串口。C 联测需要 gcc 和本地 H7 源码；源目录不存在会明确跳过，跳过不代表固件已验证。

2026-11-12 分阶段验证记录：

- 初始双机版本：服务相关 34 项与 App 110 项在 Linux 合计 144 项通过、无跳过；真实 Windows 的 App 110 项通过。exe 完成原生 mock 启动和正常退出，此阶段尚未部署 Nano。
- 增加 H7 缺席启动与模拟拔插处理后：单独重跑水下服务测试，35 项通过；此阶段未重跑 App 联合测试。
- 随后已完成 Nano 部署及约 12 秒、123 条无 H7 遥测检查，旧 H7 桥已停用，详见[部署记录](../docs/UNDER_CONTROL_NANO_DEPLOYMENT.md)。没有连接实际 H7 串口或驱动电机，也未通过重启 Nano 验证开机过程。

受限沙箱无法正常完成本机 asyncio/socket 测试，上述网络与 PTY 测试在允许本机套接字的执行环境完成。Windows 分发包中的构建信息是打包时的快照，不表示后续 Nano 部署状态。

实机仍需验收方向、联动、按住/松开、换向等待、拔线、App 关闭、遥控接管/归还及两台同时运行。固件描述的遥控上游是水面 H7→CN6，而当前水面 ROS 对接 F103；现场应核对实际转发板卡与接线。
