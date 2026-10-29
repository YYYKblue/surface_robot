# F1 操作员 Qt App

更新：2026-10-29。这是 Windows 操作员端的 Qt 实现。它只负责输入和显示，不直接运行 ROS，也不管理控制 token：

```text
Qt App → WebSocket → f1_operator_gateway → ROS → f1_nano_bridge / 清洗任务
```

当前实现包含：

- 地图、机器人位姿、雷达点和控制状态显示；
- 手动方向、PWM 档位、轮组、滚刷和侧推输入；
- 滚刷、第五电推支持手动输入 `-100%` 到 `100%`，并独立控制启用状态；
- 清洗任务开始、暂停和取消；
- 固定的手动停止按钮（界面文字为“停止全部”，实际范围见下文）；
- 摄像头预留面板；
- 断线时不自动恢复运动。

代码按职责拆分：`main.py` 只负责启动，`window.py` 负责窗口和操作，`map_canvas.py` 负责地图/雷达绘制，`transport.py` 负责 WebSocket 和模拟遥测。

App 不申请或释放 ROS 控制租约，也不发送 ROS 消息。Nano 网关管理手动会话和输入超时，硬件桥通过单租约约束执行器控制来源。

## Windows 开发

先按[网关说明](../src/f1_operator_gateway/README.md)启动 Nano 的 bridge 和网关。地图显示需要已有建图或定位节点，任务按钮需要对应清洗控制器；仅启动 App 和网关不会创建这些节点。

```powershell
cd operator_app
python -m pip install -r requirements.txt
python main.py --url ws://192.168.1.188:8765
```

支持 PySide6；当前开发机若只有 PyQt5，代码会自动回退到 PyQt5。没有连接机器人时可以启动界面查看布局：

```powershell
cd operator_app
python main.py --mock
```

键盘 W/A/S/D 与方向按钮按住运行、松开停止主驱动；已独立开启的滚刷和第五路不会因松开方向键而关闭。`Space` 或窗口顶部红色按钮清除当前手动输出并发送 `manual_stop`。

**红色按钮不是自动任务的全局停止入口。** 网关处理 `manual_stop` 时只释放手动租约，自动任务需要使用“暂停任务”或“取消任务”。App 断线不会主动暂停自动清洗。暂停后的任务需先 cancel，再检查起始条件并 start。

默认任务接口是 `/pool_cleaning/*`，对应矩形四边任务；大 L 任务需要使用[网关重映射命令](../src/f1_operator_gateway/README.md#连接大-l-任务)。界面的“当前任务：单边清洗”是静态文字，不能用它判断已连接的任务类型。当前也没有初始位姿操作入口，定位需先在 RViz 设置。

## Windows 打包

在 `operator_app` 目录，可额外安装 PyInstaller 并生成目录分发版本：

```powershell
python -m pip install pyinstaller
pyinstaller --noconsole --name F1Operator main.py
```

分发完整的 `dist/F1Operator/` 目录，运行其中的 `F1Operator.exe`。如确实需要单文件版本，在命令中增加 `--onefile`，产物为 `dist/F1Operator.exe`；Windows 产物需要在 Windows 上构建。Nano 侧仍运行 ROS 网关及相关节点。

## 协议边界

App 发送 `hello`、`manual_input`、`manual_stop` 和 `task` 四类消息；协议为 `f1-operator-v1`。手动输入每 50 ms 发送一次，App 不重试旧命令；网关断开或输入超时会停止手动会话。遥测使用同一 WebSocket 的服务端推送，地图在变化及连接握手时发送，雷达按网关设定频率发送（默认 10 Hz）。

`--mock` 仅提供模拟遥测用于界面检查，不验证真实 WebSocket、任务控制或硬件输出。当前地图画布含固定的 π 航向显示补偿，网关发送原始扫描而非经 TF 转换的地图点云；显示对齐仍需与 RViz 和实际安装核对。
