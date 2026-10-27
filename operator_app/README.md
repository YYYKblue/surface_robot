# F1 操作员 Qt App

这是新版 Windows 操作员端的第一版骨架。它只负责输入和显示，不直接运行 ROS，也不管理控制 token：

```text
Qt App → WebSocket → f1_operator_gateway → ROS → f1_nano_bridge / 清洗任务
```

当前实现包含：

- 地图、机器人位姿、雷达点和控制状态显示；
- 手动方向、PWM 档位、轮组、滚刷和侧推输入；
- 滚刷、第五电推支持手动输入 `-100%` 到 `100%`，并独立控制启用状态；
- 清洗任务开始、暂停和取消；
- 固定的停止按钮；
- 摄像头预留面板；
- 断线时不自动恢复运动。

代码按职责拆分：`main.py` 只负责启动，`window.py` 负责窗口和操作，`map_canvas.py` 负责地图/雷达绘制，`transport.py` 负责 WebSocket 和模拟遥测。

App 不申请或释放 ROS 控制租约，也不发送 ROS 消息。Nano 网关负责控制会话、过期、互斥和停止。

## Windows 开发

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

键盘 W/A/S/D 与界面按钮保持按住运行、松开停止。`Space` 或窗口顶部的红色按钮发送 STOP。

需要发布 Windows 单文件版本时，可在 Windows 开发机额外安装 PyInstaller，再执行：

```powershell
python -m pip install pyinstaller
pyinstaller --noconsole --name F1Operator main.py
```

生成的 `dist/F1Operator` 目录随 App 一起分发；Nano 侧仍只部署 ROS 网关。

## 协议边界

App 发送 `hello`、`manual_input`、`manual_stop` 和 `task` 四类消息。手动输入采用最新值，App 不重试旧命令；网关断开或输入超时会停止输出。遥测使用同一 WebSocket 的服务端推送，地图只在变化时发送，雷达按网关设定频率发送。
