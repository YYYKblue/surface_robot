# WTRTK-982 单点定位与双天线定向

该 ROS 2 Humble 包独立读取 WTRTK-982 的 USB 串口。记录船体轨迹时订阅 `/f1/rtk/base_fix`（船中心水平位置）和 `/f1/rtk/bow_heading`（船头真北航向）。两者的参考点或方向轴均属于 `base_link`。驱动同时保留主天线位置与双天线基线航向，可独立运行，无需启动雷达、TF 或 Cartographer。

## 准备接收机

1. 将两根天线装在开阔处，连接模块，用 Type-C 数据线接电脑。在维特上位机打开实际串口，出厂参数为 115200、8N1。
2. 在实际连接的串口发送 `UNILOGLIST`，查出该端口已启用的 GGA 和 HPR 日志名。沿用原有日志名调整周期，每行以 CR/LF 结束。`0.05` 表示每 50 ms 输出一次。本机查到 `COM3` 上原有 `GNGGA` 和 `GPHPR`，实测生效的命令如下。如果同一端口同时启用了 `GNGGA` 和 `GPGGA`，它们可能都输出 `$GNGGA`。确认保留 `GNGGA` 后，用 `UNLOG COM3 GPGGA` 关闭重复项。[和芯星通 N4 指令手册，第 1、7、8 章](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf)

   ```text
   GNGGA COM3 0.05
   GPHPR 0.05
   ```

3. 核对命令回执和 `UNILOGLIST`，再实测两类原始报文各约 20 Hz。取得有效解后，观察两类报文各自的 UTC 是否以 0.05 秒递增、两路是否同历元配对。发送 `SAVECONFIG` 并核对回执，下一次断电重启后复查。需要恢复 5 Hz 时将两条命令的周期都设为 `0.2`，复核后保存。
4. 关闭上位机串口，将 RTK 接到任一可用 USB 数据口。该型号使用 CH340（`1a86:7523`），驱动被动读取报文。运行账号需要串口读写权限，Ubuntu 通常由 dialout 组提供。

[维特上手教程](https://wit-motion.yuque.com/wumwnr/bf4d0f/qpalh34ulfouy3p2) · [天线安装与规格](https://wit-motion.yuque.com/wumwnr/docs/sl3yxrnbigoi432o)

## 独立构建和启动

驱动使用 `f1_interfaces` 生成的消息。以下命令用于 PC 开发调试。Nano 日常采集由 `f1-rtk.service` 开机启动。手动串口配置或前台调试前先暂停服务，完成后恢复，用 `systemctl status f1-rtk.service` 和 `/f1/rtk/status` 检查。首次从旧版升级时，先按下节处理旧驱动的生成目录。[Nano 复制安装、服务与串口检查](../../DEPLOY_NANO.md#独立-rtk-部署)

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to f1_rtk_driver --symlink-install
source install/local_setup.bash
ros2 launch f1_rtk_driver rtk.launch.py
```

默认 `port: auto`。节点只枚举 USB 标识为 `1a86:7523` 的 CH340，独占候选串口，在 115200 波特率下被动监听最多 1.5 秒。同一设备同时出现校验通过、结构有效的 GGA 和 HPR 后才认作 RTK，沿用同一个打开的串口。室内无定位解也能识别。占用、拔除或识别失败时重试，重插后重新枚举。识别过程不写串口，换插口或设备号无需改配置。接收机需按上节持续输出两类报文（出厂配置不同的模块应先配置输出）。

启动参数 `config:=/绝对路径/rtk.yaml` 选择配置文件。如果要使用不同型号的适配器，可用 `port:=/dev/serial/by-id/设备唯一标识` 显式选择已确认设备，此模式直接打开给定路径。自动模式下日志 `Opened RTK ...` 显示实际设备名，`/f1/rtk/status.connected` 表示串口已通过识别。

### 从驱动内置消息升级

`Heading`、`RtkStatus`、`RawSentence` 的类型名前缀已由 `f1_rtk_driver/msg/` 迁至 `f1_interfaces/msg/`。外部 ROS 订阅者需更新类型并重新启动，编译型订阅者还需重建。旧 rosbag 的这三种类型仍带原包名，可在旧环境中回放，或转换类型后供新订阅者使用。两个位置话题沿用 `sensor_msgs/msg/NavSatFix`。

项目默认使用 colcon 隔离安装。首次升级有旧类型的 PC 或 Nano 工作空间时，在工作区根目录将旧驱动的生成目录移出，再执行对应平台的构建命令。这样 ROS 接口索引会与当前源码一致。

```bash
if [ -f install/f1_rtk_driver/share/ament_index/resource_index/rosidl_interfaces/f1_rtk_driver ]; then
  rtk_previous=$(mktemp -d /tmp/f1-rtk-previous.XXXXXX)
  for tree in build install; do
    if [ -d "$tree/f1_rtk_driver" ]; then
      mv "$tree/f1_rtk_driver" "$rtk_previous/$tree"
    fi
  done
fi
```

## 数据约定

| 话题 | 类型 | 含义 |
|---|---|---|
| `/f1/rtk/base_fix` | `sensor_msgs/msg/NavSatFix` | 船中心的 WGS84 经纬度，默认 `frame_id=base_link`。有效位置沿用当次 GGA 定位级别，高度为 NaN，位置协方差未知。 |
| `/f1/rtk/bow_heading` | `f1_interfaces/msg/Heading` | 船头（`base_link` 的 +x）相对真北的顺时针角，范围 [0°, 360°)。与同次原始航向共享时间戳、UTC、质量码和有效标志。 |
| `/f1/rtk/fix` | `sensor_msgs/msg/NavSatFix` | 主天线的 WGS84 经纬度。椭球高取 GGA 海拔加大地水准面差距，缺少任一项时为 NaN。仅靠 GGA 无法确定位置协方差和实际使用的星座，标为未知。 |
| `/f1/rtk/heading` | `f1_interfaces/msg/Heading` | M→S 基线从真北顺时针计量的角度，东为 90°。仅 HPR 质量码 4 且角度有效时 `valid=true`，其余角度输出 NaN。 |
| `/f1/rtk/status` | `f1_interfaces/msg/RtkStatus` | 默认 20 Hz 状态。分别报告串口连接、GGA/HPR 质量码、有效性和距上次有效观测的秒数。包含 GGA 的 UTC、卫星数、HDOP 和累计拒收行数。串口断开后质量码回到 -1。 |
| `/f1/rtk/raw` | `f1_interfaces/msg/RawSentence` | 每条完整串口文本行及接收时间，包括校验失败的报文。去掉行尾 CR/LF，超出 1024 字节的行丢弃并计数。 |

当 GGA、HPR 各以 20 Hz 输出时，`fix`、`heading` 和 `bow_heading` 各随报文以 20 Hz 更新。两路每个历元都有效且配对成功时，有效 `base_fix` 也是 20 Hz。仅输出这两类报文时 `raw` 约为 40 条/秒。启动参数文件中的 `status_rate_hz` 只控制状态心跳，设备离线时仍按该频率报告。

所有 `header.stamp` 使用 ROS 接收时间。报文自带的 UTC 为日内时间，保留在 `Heading.utc`、`RtkStatus.fix_utc` 和原文中。超时判断使用单调时钟。默认 1.5 秒未收到该类有效报文时，分别撤销有效标志，对原先有效的话题发布一次无效观测。失联时同样撤销。状态中的年龄从上次有效接收起继续增长，从未收到过时为 -1。

GGA 质量码 1 表示单点，4 表示 RTK 固定，5 表示 RTK 浮点。首阶段在室外期望看到 `fix_quality=1` 和 `heading_quality=4`。标准 NavSatFix 不能区分 RTK 固定和浮点，两者均归入 GBAS，精确类别看状态话题的原始码。

室内无解时，UM982 可以输出质量码为 0 的 GGA 和字段全空的 HPR。后者会发布无效航向，质量码为 -1，角度为 NaN。可从 `raw` 与 `heading` 的连续更新确认定向报文已经到达。

## 船体安装与坐标换算

2026-11-21 在船上静止采集了两组各 420 帧的雷达扫描。操作者确认目标 3 是左后方主天线 M、目标 17 是右前方从天线 S。按约 5 cm 圆柱外径估算轴心，两次结果如下。坐标原点为船中心在雷达扫描平面的投影，+x 向船头，+y 向左舷。

| 扫描 | M 的 (x, y)，m | S 的 (x, y)，m | 基线长度 | 从船头顺时针至 M→S |
|---|---|---|---|---|
| 第一次 | (−0.1910, +0.2117) | (+0.2244, −0.1600) | 0.5575 m | 41.82° |
| 第二次 | (−0.1910, +0.2137) | (+0.2244, −0.1600) | 0.5588 m | 41.98° |

驱动默认参数取两次测量的近似值。更换支架、移动天线或用卷尺复核轴心后，修改配置并重启 RTK 节点。雷达可见横宽约 2 cm，轴心估算依赖所取外壳直径。天线相位中心及其垂直高度尚未实测。[安装参数](config/rtk.yaml) · [船体和雷达坐标约定](../f1_mapping_bringup/config/lidar_planar.yaml)

| 参数 | 默认值 | 含义 |
|---|---|---|
| `base_frame` | `base_link` | 船中心及其向前的 +x 轴 |
| `antenna_x_m` | −0.191 | 从船中心到 M 的纵向距离，向前为正 |
| `antenna_y_m` | +0.213 | 从船中心到 M 的横向距离，向左为正 |
| `baseline_clockwise_from_bow_deg` | +42.0 | 从船头顺时针转到 M→S 的安装角，假定接收机输出原始基线方向 |

1. 船头真北航向 `H = (HPR航向 − 安装角) mod 360°`。例如原始 HPR 为 42° 时，船头朝正北。原始 HPR 为 0° 时，船头指向 318°。若以后需要 ROS 的 ENU yaw，角度为 `90° − H`，再按所需区间回绕。
2. 用同一 GNSS UTC 历元的有效 GGA 和固定解 HPR 求船中心。两条报文可先后任意到达，配对时精确匹配 UTC。令 `x, y` 为上表中 M 的坐标，从 M 移到船中心的偏移为 `东 = −x sin(H) + y cos(H)`、`北 = −x cos(H) − y sin(H)`。将这个米制偏移加到 WGS84 局部切平面后换回经纬度，消息沿用 GGA 的 ROS 接收时间。例如船头朝北时，船中心应位于 M 以东 21.3 cm、以北 19.1 cm。[经纬度与地心坐标换算公式](https://epsg.io/9602-method)
3. 该换算采用船体近似水平的二维模型。船中心的 `altitude` 为 NaN，主天线的原始椭球高仍可从 `fix` 读取。安装补偿只校正参考点和固定角差，单点定位误差、航向抖动及船体横摇和纵摇仍会影响结果。此前约 50 cm 基线静置测试的 1,500 个固定解样本，减去安装角前后的航向跨度均为 8.58°。

启动后获得同一历元的有效观测才会输出有效 `base_fix`。GGA 无解、HPR 失去固定解或串口断开时，已有的船中心解立即失效，发布 `STATUS_NO_FIX` 和 NaN 经纬度。若两路都在输出、却持续没有新配对，也会在 `stale_timeout_sec` 后撤销上次有效位置。船体位置的可用性应直接检查 `base_fix.status.status`。`status.fix_valid` 单独反映主天线 GGA，有效的主天线单点位置仍可单独使用。

## 台架验收

在已加载工作区环境的终端查看话题。持续监听的命令用 Ctrl+C 结束，或另开终端录包。每次录制使用新的输出目录。

```bash
ros2 topic echo --once /f1/rtk/status
ros2 topic hz /f1/rtk/fix
ros2 topic echo --once /f1/rtk/base_fix
ros2 topic echo --once /f1/rtk/bow_heading
ros2 bag record -o rtk_standalone_01 /f1/rtk/raw /f1/rtk/fix /f1/rtk/heading /f1/rtk/base_fix /f1/rtk/bow_heading /f1/rtk/status
```

1. 在室外开阔处静置，等 GGA 有效且 HPR 质量码为 4 后，连续录制至少 60 秒。在同一段 60 秒窗口内，从 `raw` 分别统计 GGA、HPR 各自的唯一 UTC 数、重复及缺失历元。预期每类约 1,200 个历元，相邻 UTC 间隔 50 ms。核对有效 `base_fix` 的数量及时间戳与可配对的 GGA 一致，记录 `status.rejected_lines` 在窗口内的增量。室内无解时仍可核对串口报文频率。拔掉接收机后可核对状态心跳。
2. 对比相同接收时间的 `fix` 与 `base_fix`，两点水平距离应约 0.286 m。这个距离用于核对杆臂换算，单点定位的绝对精度需另用已知点验证。
3. 将船头沿已知真北方位的参考线停放，比较 `bow_heading` 与参考方位。再将整船顺时针转约 90°停稳，两次船头航向之差按 360°回绕后应约为 +90°。分别记录平均偏差和静止波动，以区分安装角误差与定向噪声。
4. 岸上拔插 RTK 的 USB。确认状态变为失联，两类有效标志撤销，接回同一路径后无需重启节点即可恢复。

运行包内测试可用 `colcon test --packages-select f1_rtk_driver --event-handlers console_direct+`。串口集成测试在本机创建伪终端并通过实际 launch 和 ROS 话题交换数据。
