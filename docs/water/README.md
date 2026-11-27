# Nano 共线水质检测器

四支探头的 A/B 并接在同一条 RS-485 总线上，Nano 接一个 USB 转换器。采集服务独占这个串口，依次读 pH、EC、余氯、浊度。水面网关订阅 `/f1/water_quality`，通过已有的 8765 WebSocket 把数据发到 Windows App 的「水质」页。开机后即可采集，无需加载地图。

## 首次接线与地址确认

1. 确认检测器供电。按四本手册，pH、EC、余氯支持 7–30 V DC，浊度支持 10–30 V DC。若用同一电源，四者的共同范围为 **10–30 V DC**，还应遵守检测器整机铭牌。USB 转换器负责通信。探头棕线为电源正、黑线为电源负，黄（或绿）为 A、蓝为 B。
2. 向装配方取得「探头 → 从站地址」对照及总线波特率。四种探头的出厂参数都是地址 1、4800、8N1。若仍是出厂地址，按各手册用厂家配置工具**一次只接一支**改址，记录地址再并回总线。例如可以把 pH、EC、余氯、浊度分别设为 1、2、3、4。已经分配过的地址可直接沿用。pH 与浊度的测量寄存器布局相同，有效回包本身无法证明探头类型。采集程序只发送功能码 03。
3. 在 Nano 插入转换器前后分别运行以下命令，比较新增设备。当前 CH340 转换器使用 `port: auto`，可换 USB 插孔。采集程序只尝试独占打开空闲的 1a86:7523 设备，先以 115200 波特监听 1.5 秒，遇到主动输出数据的设备立即释放。对静默设备以配置波特率读取 0x07D0，至少两个已配置的不同探头必须分别返回自己的地址并通过 CRC 校验。确认后持有同一串口，拔出或连续两轮整条总线无有效测量时释放并重新枚举。首次识别需至少两支探头通电；已识别后可继续采集单支在线探头。其他型号转换器可以显式填写已核对的设备路径。

   ```bash
   python3 -m serial.tools.list_ports -v
   ls -l /dev/serial/by-id/ /dev/serial/by-path/
   ```

4. 本机已确认 EC 电极常数为 **K=10**，浊度量程为 **0–1000 NTU**。配置模板已填入这两项。

## 配置和启动

同步 `src/f1_interfaces/`、`src/f1_operator_gateway/` 至 Nano，更新 Windows 端 `operator_app/`。以下命令适用于仓库使用的 ROS Humble、用户 nvidia 和复制安装布局。在无作业的维护时段停止操作员服务后构建，并在完成后重新启动它以加载新网关。

```bash
cd /home/nvidia/surface_ws
sudo systemctl stop f1-operator.service
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --base-paths src --executor sequential \
  --packages-select f1_interfaces f1_operator_gateway \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/local_setup.bash
ros2 interface show f1_interfaces/msg/WaterQualityReading
sudo systemctl start f1-operator.service
```

首次配置时复制模板，然后编辑 `/etc/f1-water-quality.yaml`。后续升级保留已经核对的本机配置。

```bash
sudo cp -n src/f1_operator_gateway/config/water_quality.yaml /etc/f1-water-quality.yaml
sudoedit /etc/f1-water-quality.yaml
```

本机配置为 `port: auto`、4800 波特，pH、EC、余氯、浊度地址分别为 1、2、3、4。地址 0 表示该探头待配置。轮询时跳过未配置及重复配置的地址。若已有旧版配置，请在原文件中将 `ec_k` 更新为 10、`turbidity_range_ntu` 更新为 1000。服务每秒发起一轮采样，单站读超时为 250 ms。

```bash
sudo install -m 644 src/f1_operator_gateway/deploy/f1-water-quality.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now f1-water-quality.service
export ROS_DOMAIN_ID=33 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /f1/water_quality
```

后续修改配置只需 `sudo systemctl restart f1-water-quality.service`。Windows 从仓库根目录执行 `python operator_app/main.py --url ws://<Nano_IP>:8765`，打开「水质」。本地界面预览使用 `python operator_app/main.py --mock`，显示模拟值。

## 数据与现场核对

| 探头 | 读取寄存器 | 工程值 |
| --- | --- | --- |
| pH | 0、1 | pH 除以 100，温度为有符号值除以 10 |
| EC | 0 至 3 | 本机 K=10，电导率取原值，单位 μS/cm。温度除以 10，盐度与 TDS 直接取 ppm |
| 余氯 | 0 | 浓度除以 100，单位 mg/L |
| 浊度 | 0、1 | 本机 1000 NTU 量程，测量寄存器原值除以 10，单位 NTU。温度为有符号值除以 10 |

pH 手册示例数据 `01 03 04 03 16 01 09` 的标准 CRC 为 `DA 25`（低字节在前），示例印作 `B8 BE`。驱动按标准校验。

ROS 消息含探头名、采样或错误的时间、状态、测量值及辅助读数。ROS 中缺失值为 NaN，网关在 JSON 中省略对应字段，真实零值保留。单个探头无应答或 CRC 错误时应仅清除该路数值；采集服务停止超过 3 秒时网关标记采样过期，App 遥测中断后也会清除读数。接线后逐路与厂家工具或参考读数比对，核对探头归属、量程、单位和断开恢复。使用厂家工具时先停止水质服务，使总线上同时只有一个主站。

## 2026-11-27 Nano 部署记录

在 `192.168.1.188` 上，将水质变更合入该机已有的 `f1_interfaces`、`f1_operator_gateway` 并编译成功。备份为 `/home/nvidia/surface_ws_backups/water-before-20261127-194622.tar.gz`。操作员网关已恢复运行，连续 20 帧含 `water_quality` 字段。Nano 上 78 项协议、网关及伪串口测试通过。

本次接入只枚举到一个无唯一序列号的 CH340，物理路径为 `/dev/serial/by-path/platform-3610000.usb-usb-0:3.2:1.0-port0`。水下服务的自动发现也会打开此口，日志显示 H7 握手超时。已安装水质服务与 `/etc/f1-water-quality.yaml`，服务处于 disabled/inactive，待明确与 H7 的串口归属后再启用。

在独占该口期间，以 4800、8N1 读取地址 1–254 的从站地址寄存器，只有地址 1 收到字节。随后按一、二、四个测量寄存器分别读取地址 1 各 12 次，36 次均未得到校验有效的完整应答；重复读取通信配置寄存器则偶有有效应答。该表现与并联探头同址冲突相符。此次仅执行功能码 03，没有更改实物地址。目标为 pH 1、EC 2、余氯 3、浊度 4；现场配置中 pH 暂为 0，避免误认地址 1 的应答。EC 电极常数为 10，浊度量程上限为 1000 NTU。

随后现场仅给 EC 探头供电。Nano 在地址 1 连续两次读到电极常数 10.0（寄存器 0x0054–0x0055）及四个测量寄存器，确认地址 2 无应答后，向 0x07D0 写入 2 一次并收到完整回显。新地址 2 连续两次回读成功，旧地址 1 连续两次无应答。改址后读到电导率 0 μS/cm、温度 17.8 °C、盐度 0 ppm、TDS 0 ppm。本次验证覆盖在线改址和回读，断电重启后的地址保持待核对。

余氯探头通电后，在地址 1 两次读到清洗参数（4 小时、40 秒、模式 1）和浓度寄存器。确认地址 3 无应答后只写一次地址寄存器，收到完整回显；新地址 3 连续两次回读相同清洗参数和 0.00 mg/L，原地址 1 连续两次无应答。同时抽查 EC 地址 2 一次无应答，其当时供电状态未确认。余氯地址断电保持待核对。

浊度探头通电后，地址 4 两次无应答，地址 1 两次返回测量寄存器原值 10000、188。向 0x07D0 写入地址 4 一次并收到完整回显，地址 4 两次回读相同，原地址 1 两次无应答。按已确认的 1000 NTU 量程，现场读数为 1000.0 NTU（量程上限）、18.8 °C。此次只验证通信、改址和换算；探头当时的测量环境及断电后的地址保持待核对。

四支同时供电后，1–4 号分别回报自己的从站地址；连续五轮共 20 次测量读取成功。Nano 配置已将 pH 地址设为 1，串口已切换为自动发现模式。Nano 上编译成功、79 项测试通过，含模拟串口编号变化后重新发现。现场水质服务在 H7、RTK 服务运行期间自动识别总线，日志记录本次分配为 `/dev/ttyUSB0`。随后验证 35 帧连续网关遥测四路均为 ok，已启用水质开机自启。电脑经局域网读取真实 8765 遥测并调用 App 的 Qt 水质显示组件，读到 pH 6.31（17.8 °C）、电导率 0.0 μS/cm（17.8 °C、盐度和 TDS 各 0 ppm）、余氯 0.00 mg/L、浊度 1000.00 NTU（18.9 °C）。现场未实际换 USB 插孔或重启整机；浊度值达到满量程，未用标准液验证。
