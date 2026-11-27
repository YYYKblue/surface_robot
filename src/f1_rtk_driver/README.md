# WTRTK-982 主天线定位与可选 NTRIP 差分

该 ROS 2 Humble 包独立读取 WTRTK-982 的 USB 串口，并在 `/f1/rtk/fix` 发布主天线 ANT1 的 WGS84 位置。GGA 即可完成设备识别和位置输出。接收机原有的 HPR 输出只会出现在串口原文中。

## Nano 直接接入千寻差分

差分功能默认关闭。启用后，Nano 的网络线程向 NTRIP caster 上报接收机有效 GGA、接收 RTCM3；现有驱动持有的同一串口负责写入 RTCM 和读取 GGA。接收机解算后仍通过原有话题发布位置，无需把 RTCM 交给 Cartographer。自动发现期间保持被动监听，不向尚未确认的 CH340 写入。显式指定串口时，操作者必须确认它属于 RTK。

### 前提与参数

- Nano 已开机且应用可以访问互联网。4G 模块注册成功或模块内部 ping 成功，不能代替 Nano 本机 TCP 联网检查。
- 接收机处于流动站工作状态，USB 对应端口允许 RTCM 输入；本包不自动改接收机工作模式、端口配置或保存固件参数。当前 USB 连接在既有部署中对应接收机 COM3，换设备后重新核实。
- 天线位于室外开阔处且有有效 GGA；没有有效位置时显示 `waiting_gga`，不伪造位置向服务商申请差分。
- 账号在有效期和服务覆盖范围内；同一个千寻账号只运行一个客户端。

| 参数 | 本项目首次接入设置 |
|---|---|
| 服务器 | `rtk.ntrip.qxwz.com`（优先域名；卖家提供的 IP 为 `203.107.45.154`） |
| 端口 | `8002`，千寻 WGS84 服务，匹配当前坐标处理约定 |
| 挂载点 | `RTCM32_GGB`，沿用所购账号参数 |
| 协议 | NTRIP v1；支持 ICY/HTTP 成功响应，HTTP chunked 暂不支持 |
| GGA 上报 | 最新的有效主天线原始 GGA，默认每 1 秒发送一次 |
| 重连 / 无 RTCM 超时 | 默认 5 秒 / 10 秒 |

`8003` 是 CGCS2000，不能在当前 WGS84 数据链中随意替换。是否升级到 `AUTO` 挂载点，应依据账号规格和接收机兼容性确认。服务端 GGA 上报频率与本地 GGA 20 Hz 输出独立，不需要降低接收机输出频率。[千寻接入与排障](https://help.qxwz.com/400015) · [和芯星通 NTRIP 接入原理](https://m.unicorecomm.com/assets/Html/N4/N4-NTRIP.html)

### 安装与启用

先按 [Nano 部署说明](../../DEPLOY_NANO.md#独立-rtk-部署)同步、构建驱动。以下命令在 Nano 的 `nvidia` 账号下执行。配置程序交互输入账号、隐藏输入密码，原子写入权限为 `0600` 的文件；不把凭据写入源码、ROS 参数或 shell 命令历史。

```bash
cd /home/nvidia/surface_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run f1_rtk_driver configure_ntrip
```

默认文件为 `/home/nvidia/.config/f1_rtk_driver/ntrip.json`。非敏感结构参见 [配置示例](deploy/ntrip.example.json)。账号到期后再次运行配置程序，并重启 RTK 服务加载新账号。

更新服务模板并添加独立环境文件。已有 `/etc/f1-rtk.env` 时先检查、保留其他设置，只新增或修改同名变量。停止任何使用同一账号的 Windows 客户端。确认机器人处于待机，再重启独立 RTK 服务；会短暂中断 RTK 话题，其他服务无需重启。

```bash
sudo install -m 0644 install/f1_rtk_driver/share/f1_rtk_driver/deploy/f1-rtk.service /etc/systemd/system/f1-rtk.service
sudoedit /etc/f1-rtk.env
```

环境文件内容（只有路径，没有账号密码）：

```text
F1_RTK_NTRIP_CONFIG=/home/nvidia/.config/f1_rtk_driver/ntrip.json
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart f1-rtk.service
systemctl is-enabled f1-rtk.service
systemctl is-active f1-rtk.service
export ROS_DOMAIN_ID=33 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo --once /f1/rtk/corrections
ros2 topic echo --once /f1/rtk/status
```

前台调试需先释放现有服务的串口，再使用 `F1_RTK_NTRIP_CONFIG=文件路径 ros2 launch f1_rtk_driver rtk.launch.py`。也可用自定义 YAML 中的 `ntrip_config_file` 指定路径；该参数优先于环境变量。默认 YAML 不设置此项，避免覆盖环境变量。

### 状态与验收

`/f1/rtk/corrections` 使用 `diagnostic_msgs/msg/DiagnosticArray`，每秒发布一次，无账号密码。原有 `RtkStatus` 消息定义不变。

| 字段 / 状态 | 含义 |
|---|---|
| `disabled` / `config_error` | 未配置差分 / 私密配置格式、路径或权限错误；位置采集继续运行 |
| `waiting_receiver` / `waiting_gga` | 等待已确认的串口 / 等待有效且新鲜的 GGA |
| `connecting` / `connected` | 正在连接 / caster 已接受请求，尚未收到有效 RTCM 帧 |
| `streaming` / `retrying` | 已收到 CRC 校验通过的 RTCM / 失败后等待重连 |
| `received_bytes`、`received_frames` | 本次驱动进程累计接收的有效 RTCM 字节、帧数 |
| `written_bytes`、`write_age_sec` | 累计交给串口的字节数、距最近写入的秒数；不能单独证明接收机已采用差分 |
| `rtcm_age_sec` | 距最近有效 RTCM 帧接收时间；从未收到时为 -1 |
| `differential_age_sec`、`reference_station_id` | 最近有效 GGA 的差分龄期与站号；接收机没提供时为 NaN / 空字符串，过时与否结合 `fix_valid` 判断 |
| `fix_quality`、`fix_valid` | 接收机最新 GGA 质量码及位置有效性 |

诊断级别 OK 表示差分流和串口写入新鲜，不保证 RTK 固定。**固定位置必须同时检查 `/f1/rtk/status.fix_valid=true` 和 `fix_quality=4`。** `fix_age_sec` 是位置观测年龄，不是差分龄期。

网络线程与串口采集分开；串口写入不阻塞，每轮最多 4096 字节。只有 CRC24Q 校验通过的完整 RTCM3 帧入队，队列最多 16 KiB，排队超过 2 秒的数据丢弃。串口重连、网络会话变化时清空旧数据。无效或过时 GGA 会停止网络会话；配置错误、鉴权失败或网络故障不停止原有串口采集。连接重试不会延长已购买账号的有效期。

验收时先静置采集 10–15 分钟，统计固定解占比、首次固定时间、位置散布，再主动断开并恢复差分网络，确认网络恢复、数据重获及定位质量变化。接收机在差分断开后可能暂时保持固定标志，应结合差分龄期和写入状态判断。录包建议：

```bash
ros2 bag record -o rtk_ntrip_01 /f1/rtk/raw /f1/rtk/fix /f1/rtk/status /f1/rtk/corrections
```

位置约束使用配置中的平移权重。记录固定解建立、退化和恢复时的位置跳变，以便现场核对地图配准效果。

回退为独立采集：从 `/etc/f1-rtk.env` 删除或清空 `F1_RTK_NTRIP_CONFIG`，确保自定义 YAML 未指定该路径，再重启 `f1-rtk.service`。确认差分诊断为 `disabled`，主天线位置仍正常输出。无需删除私密配置或停用 RTK 开机服务。

## 准备接收机

1. 将两根天线装在开阔处，连接模块，用 Type-C 数据线接电脑。在维特上位机打开实际串口，出厂参数为 115200、8N1。
2. 在实际连接的串口发送 `UNILOGLIST`，查出该端口已启用的 GGA 日志名。沿用原有日志名调整周期，每行以 CR/LF 结束。`0.05` 表示每 50 ms 输出一次。本机查到 `COM3` 上原有 `GNGGA`，实测生效的命令如下。如果同一端口同时启用了 `GNGGA` 和 `GPGGA`，它们可能都输出 `$GNGGA`。确认保留 `GNGGA` 后，用 `UNLOG COM3 GPGGA` 关闭重复项。[和芯星通 N4 指令手册，第 1、7、8 章](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf)

   ```text
   GNGGA COM3 0.05
   ```

3. 核对命令回执和 `UNILOGLIST`。验证 GGA 约 20 Hz，UTC 以 0.05 秒递增，再发送 `SAVECONFIG` 并在冷启动后复查。若 GGA 已按预期输出，可直接使用现有设置。
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

默认 `port: auto`。节点只枚举 USB 标识为 `1a86:7523` 的 CH340，独占候选串口，在 115200 波特率下被动监听最多 1.5 秒。同一设备收到两条校验通过、结构有效的 GGA 后才认作 RTK，沿用同一个打开的串口。室内无定位解也能识别。占用、拔除或识别失败时重试，重插后重新枚举。识别过程不写串口，换插口或设备号无需改配置。接收机需按上节持续输出 GGA（出厂配置不同的模块应先配置输出）。

启动参数 `config:=/绝对路径/rtk.yaml` 选择配置文件。如果要使用不同型号的适配器，可用 `port:=/dev/serial/by-id/设备唯一标识` 显式选择已确认设备，此模式直接打开给定路径。自动模式下日志 `Opened RTK ...` 显示实际设备名，`/f1/rtk/status.connected` 表示串口已通过识别。

### 从驱动内置消息升级

`Heading`、`RtkStatus`、`RawSentence` 的类型名前缀已由 `f1_rtk_driver/msg/` 迁至 `f1_interfaces/msg/`。外部 ROS 订阅者需更新类型并重新启动，编译型订阅者还需重建。旧 rosbag 的这三种类型仍带原包名，可在旧环境中回放，或转换类型后供新订阅者使用。主天线位置话题沿用 `sensor_msgs/msg/NavSatFix`。

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
| `/f1/rtk/fix` | `sensor_msgs/msg/NavSatFix` | ANT1 的 WGS84 经纬度，默认帧为 `gnss_ant1`。椭球高取 GGA 海拔加大地水准面差距，缺项时为 NaN。协方差未知。 |
| `/f1/rtk/status` | `f1_interfaces/msg/RtkStatus` | 串口连接、GGA 质量、有效性、观测年龄、UTC、卫星数、HDOP 和拒收计数。 |
| `/f1/rtk/raw` | `f1_interfaces/msg/RawSentence` | 每条完整串口文本及其 ROS 接收时间，含其他报文及校验失败的行。超过 1024 字节的行丢弃并计数。 |
| `/f1/rtk/corrections` | `diagnostic_msgs/msg/DiagnosticArray` | 千寻连接、RTCM 接收和串口写入状态，1 Hz。 |

`fix` 随 GGA 更新。`status_rate_hz` 默认 20 Hz，设备离线时仍发送心跳。GGA 质量码 0 表示无解，1 为单点，4 为固定解，5 为浮点解。无解时位置为 NaN，状态为 `STATUS_NO_FIX`。串口断开或上次有效位置超过 `stale_timeout_sec`（默认 1.5 秒）时撤销有效位置。历史消息中的航向字段保留类型兼容，当前驱动固定输出 `heading_valid=false`、`heading_quality=-1` 和 `heading_age_sec=-1`。

## 主天线安装

2026-11-21 在船上静止采集了两组各 420 帧的雷达扫描。操作者确认目标 3 是左后方主天线 M、目标 17 是右前方从天线 S。按约 5 cm 圆柱外径估算轴心，两次结果如下。坐标原点为船中心在雷达扫描平面的投影，+x 向船头，+y 向左舷。

| 扫描 | M 的 (x, y)，m | S 的 (x, y)，m | 基线长度 | 从船头顺时针至 M→S |
|---|---|---|---|---|
| 第一次 | (−0.1910, +0.2117) | (+0.2244, −0.1600) | 0.5575 m | 41.82° |
| 第二次 | (−0.1910, +0.2137) | (+0.2244, −0.1600) | 0.5588 m | 41.98° |

建图和定位使用 ANT1 相对船中心的二维安装偏移，x = −0.191 m、y = +0.213 m。移动主天线后在融合配置中更新这两个距离。驱动输出始终以 ANT1 为参考点。[融合配置](../f1_mapping_bringup/config/gnss_fusion.yaml)

## 台架验收

```bash
ros2 topic echo --once /f1/rtk/status
ros2 topic hz /f1/rtk/fix
ros2 bag record -o rtk_standalone_01 /f1/rtk/raw /f1/rtk/fix /f1/rtk/status /f1/rtk/corrections
```

室外静置 60 秒，在同一窗口统计 GGA 的唯一 UTC、缺失历元、拒收数和位置散布。20 Hz 接收机应产生约 1,200 个历元。验证绝对精度需使用已知点。岸上拔插 USB，确认连接及位置有效标志撤销，重连后随新 GGA 恢复。

运行包内测试可用 `colcon test --packages-select f1_rtk_driver --event-handlers console_direct+`。串口集成测试通过伪终端、实际 launch 和 ROS 话题交换数据。
