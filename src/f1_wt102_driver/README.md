# WT102 板载 UART 采集

此驱动面向 P3768-0000 配置的 Jetson Orin Nano。模块水平安装，Z 轴向上，从上方看左转为正。默认端口为 `/dev/ttyTHS1`，115200、8N1。下表为断电接线时使用的物理脚号。WT102 引脚从有黑色器件的一面辨认。

| Nano 40 针排针 | WT102 | 用途 |
|---|---|---|
| 1（3.3 V） | 2 或 11（VCC） | 供电 |
| 6（GND） | 5 或 8（GND） | 共地 |
| 10（UART RX） | 4（TX） | 航向数据 |
| 8（UART TX） | 3（RX） | 配置时可接，当前驱动只读 |

## 启动与采集

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to f1_wt102_driver
source install/setup.bash
ros2 run f1_wt102_driver wt102_uart
```

在第二个终端加载同一工作空间。只看角速度可执行：

```bash
ros2 topic echo /f1/wt102/sample --filter 'm.kind == 1' --qos-reliability best_effort
```

`/f1/wt102/sample` 是带 Nano ROS 接收时刻的 `f1_interfaces/msg/Wt102Sample`。类型 1 是机体 Z 轴校准角速度（rad/s），类型 2 是相对上电方向的角度（rad）。同一次串口读取中，每种类型发布最新值，时间戳精度受主机调度和串口传输影响。驱动重连、时钟回退或轮询停顿超过 50 ms 会切分数据流。板载 UART 拔掉传感器时串口设备通常仍存在，驱动持续等待后续有效帧。默认 SensorDataQoS。

先水平静置记录零偏，再用左右各 90° 和回到起点核对单位、方向和漂移。现场应分别记录推进器和滚刷运行时的数据。启动建图入口会按开关管理串口，单独采集时请保持一个串口读者。同步录包、回放与建图参数见[建图说明](../f1_mapping_bringup/README.md#wt102-航向辅助)。
