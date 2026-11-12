# Nano 从零部署 surface_ws

更新：2026-11-12。适用于 Nano 首次部署或已有 `surface_ws` 升级。包含水面 ROS 系统及独立水下 H7 服务。仅部署水下控制时直接阅读[水下服务章节](#水下-h7独立服务与可选设备自启动)，无需构建 ROS。PC 与 Nano 分别构建；下面每节标明执行设备。2026-11-09 控制器配置加载修复已部署到 Nano 并验证，见[故障排查](#定位正常但开放任务不能启动)。这项修复不涉及雷达驱动迁移；雷达旧服务迁移见第 8 节，旧清洗包升级见[清洗包迁移说明](src/f1_pool_cleaning/MIGRATION.md)。

| 项目 | 统一约定 |
| --- | --- |
| PC 工作空间 | `/home/bboss/new-nano/surface_ws` |
| Nano 工作空间 | `/home/nvidia/surface_ws` |
| Nano 登录 | `nvidia@192.168.1.188`，地址变化时替换 |
| 系统 ROS | `/opt/ros/humble` |
| 雷达驱动包 | `f1_lidar_driver` |
| 桥接功能包 | `f1_nano_bridge` |
| 自定义接口包 | `f1_interfaces` |
| 沿边清洗功能包 | `f1_pool_cleaning` |
| 大 L 清洗功能包 | `f1_l_cleaning` |
| 操作员网关 | `f1_operator_gateway` |
| 水下独立服务 | 根目录 `under_control/`，systemd unit 为 `under-control.service` |

PC 和 Nano 分别编译，不传 PC 的生成目录。部署状态以各项验证记录为准，文档中的操作命令不表示全部步骤均已在目标设备执行。

## 水下 H7：独立服务与可选设备自启动

Windows App 同时连接水面 `ws://Nano地址:8765` 与水下 `ws://Nano地址:8766`。水下服务自行承担网关、会话和 USB 转 RS485 桥接，不依赖 ROS，不加入水面 launch，也无需 colcon。箭头控制、协议及软件测试见 [under_control README](under_control/README.md)。

**2026-11-12 已部署到 `192.168.1.188`**：新服务 enabled/active，旧 `nano-h7-bridge.service` disabled/inactive，水面与雷达服务未重启。当时 H7 适配器缺席，仅验证了稳定待机和网络遥测，尚未实测 H7 握手、物理拔插、运动或整机重启自启动。完整现场快照、备份和回退见[部署记录](docs/UNDER_CONTROL_NANO_DEPLOYMENT.md)。

### 首次安装到另一台 Nano

先在 PC 同步服务；工作目录固定为 `/home/nvidia/surface_ws`，不同用户名或路径需同步修改 unit：

```bash
cd /home/bboss/new-nano/surface_ws
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws'
rsync -av --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' --exclude='test/' \
  under_control nvidia@192.168.1.188:/home/nvidia/surface_ws/
```

以下在 Nano 以 `nvidia` 用户执行。依赖必须能被 unit 使用的 `/usr/bin/python3` 导入；无需安装 Qt 或传输固件源码。若缺少 pip，先 `sudo apt-get install python3-pip`。首次安装依赖使用：

```bash
cd /home/nvidia/surface_ws
/usr/bin/python3 -m pip install --user -r under_control/requirements.txt
/usr/bin/python3 -c 'import serial, websockets; print(serial.__version__, websockets.__version__)'
if [ ! -e /etc/under-control.env ]; then
  sudo install -m 0644 under_control/deploy/under-control.env.example /etc/under-control.env
fi
sudoedit /etc/under-control.env
sudo install -m 0644 under_control/deploy/under-control.service /etc/systemd/system/under-control.service
```

将 `UNDER_SERIAL_PORT` 改为实际 H7 适配器的 `/dev/serial/by-id/...`，不能使用水面 F103 的端口，也不要自动选取任意 ttyUSB/ttyACM。模板的运动许可为 `false`；只会发现和停止握手，核验板端反馈后再改为 `true` 并重启服务。只要配置了串口，停止握手就会写串口，不是被动监听。现有 `192.168.1.188` 的现场配置已为 `true`，设备仍须就绪且收到新的人工输入才会运动；升级时保留现场 env，不用模板覆盖。

核对已有服务和端口占用后释放同一 H7 串口。如果确认旧服务确实是被替换的独立 H7 桥，先 `sudo systemctl disable --now nano-h7-bridge.service`；不停止水面 `f1-operator.service` 或雷达服务。然后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now under-control.service
systemctl is-enabled under-control.service
systemctl status under-control.service --no-pager
journalctl -u under-control.service -n 80 --no-pager
```

### 缺席、接入与故障处理

- H7 可不随 Nano 开机连接。服务保持运行并监听 8766；缺席时每秒检查路径，日志仅在状态变化时记录。App 水下区显示 OFFLINE/不可控制，水面链路独立运行。
- 插入设备后自动发现并完成停止握手；掉线清除旧输入，重连后必须重新按方向。USB 适配器仍在但 H7 未上电、接线异常或无有效回包时，也不能控制。
- 不给 unit 增加设备 `Requires`、`BindsTo` 或 `ConditionPathExists`。设备等待由程序处理；进程异常才由 systemd 每隔 2 秒尝试恢复，60 秒内最多启动 5 次。
- `active` 仅表示进程在运行，不代表 H7 已就绪。结合日志与 App 的串口连接、协议、控制源、armed、停止确认和新鲜反馈核验。设备仍缺席时不能验收运动。
- 同一适配器再次插入无需重启服务。更换适配器后核对身份，修改 `/etc/under-control.env` 并 `sudo systemctl restart under-control.service`。因配置错误达到启动限额时，修正后执行 `sudo systemctl reset-failed under-control.service`，再 start。

### 升级与回退

水下停机后先停止 `under-control.service`，备份现有 `under_control/`、unit 和 `/etc/under-control.env`，再同步服务源码；更新 unit 后 daemon-reload 并启动。无需重建 ROS，修改 App 则需另行分发 Windows 包。使用本页第 3 节整套同步时，也应先停止正在更新的水下服务，避免其重启时加载不完整的文件。

回退旧桥时先 `sudo systemctl disable --now under-control.service`，确认串口释放，再 `sudo systemctl enable --now nano-h7-bridge.service`。旧桥依赖原程序与配置，不能删除备份或同时启动两个桥。实际备份位置见[部署记录](docs/UNDER_CONTROL_NANO_DEPLOYMENT.md)。

## 2026-11-11 F103 v3 升级

此版桥接仅支持仓库 `under/F103firmware` 的 v3 协议。部署前核对实机已烧录的版本，收到旧版 CTRL 时会显示 `INCOMPATIBLE` 并拒绝控制。自定义 bridge YAML 的 `firmware_profile` 应为 `f1_direction_percent_v3`。先停止现有 operator 节点，更新源码并重建 `f1_interfaces` 及其依赖节点，完成检查后一同重启；旧生成消息不能与新增协议/RC字段混用。Windows 同步更新整个 `operator_app`，exe 在 Windows 重新打包。此节是升级步骤，编译或启动上位机服务不会自动烧录下位机。

主输出为整数 0–100；0 仅停止主电机，辅助独立。新 25% 为 PWM 偏移 100 μs，旧最低档为 125 μs，默认自主动作仍用新 25%。检查四电机当前极性 BL/BR/FL/FR=`+,+,-,+`、零主输出时辅助独立、手动全停、断线、RC接管与归还后重新握手；再验证纠偏、固定时间后退换边和前进辅助。CTRL/ACK/PWM 目标不能证明船体达到速度或停止。完整契约见[桥接说明](src/f1_nano_bridge/README.md#f103-v3-连续输出2026-11-11)。

### 2026-11-11 Nano 实际执行记录

- `192.168.1.188` 全量构建成功：8 个包，耗时约 1 分 28 秒；全部 CMake 安装模式统一为 `OFF`（复制安装）。
- `backups/` 与 `deploy_backups/` 已移到 `/home/nvidia/surface_ws_backups/pre-v3-20261111-105419/`。同目录保存构建前安装快照 `install-before-build.tar.gz`、源码快照及服务配置。快照保存当时磁盘文件；此前 Python 安装软链接已指向同步后的 v3 源码，因此它不等同于完整旧 v2 可运行版本。
- Nano 的 `under/`、`operator_app/` 已删除，PC 原件保留；地图和运行日志保留。默认 `colcon list` 也仅发现 8 个项目包。
- 已启动新版 `f1-operator.service`；雷达服务保持运行，PID 未变化。12 秒内收到 121 条遥测，均为 `INCOMPATIBLE`：实际报告固件 v2，要求 v3，`motion_authorized=false`、`drive_output.available=false`。因此当前不可通过新版 App 控制，需另行烧录配套 v3 固件。本次未烧录、未发送运动或清洗任务命令。
- 精简部署测试排除两组固件 C 测试及一个依赖该 C 库的 PTY 集成用例。首次运行暴露 PTY 源码依赖，修正测试选择后补跑传输组通过；最终 xUnit 为 770 个用例、0 错误、0 失败，`colcon test-result` 含 CTest 汇总计为 808 tests。该结果不表示新版固件已通过实机验收。

## 1. Nano：检查系统与 ROS

在 Nano 本机终端执行：

```bash
cat /etc/os-release
uname -m
ls /opt/ros/humble/setup.bash
```

以下 apt 安装流程针对 **Ubuntu 22.04 + ROS 2 Humble**，支持 ARM64。删除用户工作空间通常不会删除 `/opt/ros/humble`。若该文件仍存在，就使用现有系统 ROS；若不存在，先按 [ROS Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)恢复 ROS 及软件源，再继续。系统若不是 Ubuntu 22.04，不要套用 Jammy 的安装源，应先确认原 ROS 所在系统或容器。

检查终端启动文件中是否仍自动加载已删除的目录：

```bash
grep -nE 'source .*setup\.(bash|sh)|\. .*setup\.(bash|sh)' ~/.bashrc ~/.profile
```

把指向已删除工作空间的加载行注释掉；首次编译前也不要自动加载 `surface_ws/install`。保留 `/opt/ros/humble/setup.bash`。修改后关闭终端并重新登录，避免旧环境变量残留。无匹配时 grep 不输出属于正常情况。

## 2. Nano：安装工具和基础依赖

在 Nano 新终端执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y \
  openssh-server rsync build-essential cmake \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-serial python3-websockets python3-yaml \
  python3-opencv python3-numpy \
  ros-humble-cartographer-ros ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins
sudo systemctl enable --now ssh
mkdir -p /home/nvidia/surface_ws/src
```

`python3-opencv` 与 `python3-numpy` 只用于启动时从地图推导清洗几何（`map_profile`），
不参与实时控制回路。Cartographer 使用系统安装的 `ros-humble-cartographer-ros`，无需另外创建依赖工作空间。其他消息包、RViz、构建与测试依赖在传完源码后由 rosdep 安装。Humble 的 Cartographer 发布信息见 [ROS Index](https://index.ros.org/p/cartographer_ros/)。

如果提示找不到 `ros-humble-*`，先检查系统版本、ROS 软件源和 `apt-get update` 的错误，不继续编译。WebSocket 网关依赖 `python3-websockets`，不再安装或使用 ZMQ。

## 3. PC：把源码和文档传到 Nano

在 **PC 终端**执行。PC 如缺少 rsync，先执行 `sudo apt-get install rsync`。

```bash
cd /home/bboss/new-nano/surface_ws
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/src'

rsync -av --progress \
  --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
  src under_control docs tools README.md DEPLOY_NANO.md \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/
```

这里只同步 Nano 使用的 `src/`、`under_control/`、文档和工具；不传 `under/` 下位机工程、`operator_app/` Windows App、历史备份、开发环境配置及 PC 的生成目录。`src/f1_operator_gateway/` 是水面网关；`under_control/` 是独立水下服务，不由后面的 ROS 构建和测试覆盖。仅部署水面时可从同步列表省略 `under_control`。没有使用 `--delete`，远端独有文件保留，同名文件会更新；同步不会自动重启服务或更新 `/etc/under-control.env`。已误传的目录不会被此命令自动删除，见第 10 节。

日常代码同步不覆盖 Nano 地图。首次部署或明确需要更新地图时，另行执行：

```bash
rsync -av --progress /home/bboss/new-nano/surface_ws/maps/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/
```

`maps/` 被 Git 忽略，克隆仓库不会得到地图。每份地图的同名 `.pbstream`、`.yaml`、`.pgm` 直接放在工作空间 `maps/` 下，App 地图列表不递归扫描子目录。YAML 的 `image` 使用同目录 PGM 文件名，例如 `image: lidar_20261021_145054.pgm`，避免绑定 PC 或 Nano 的绝对路径。命令行启动时用 `map:=实际绝对路径` 指定本次地图；只传 `map:=xxx.yaml` 时，同名 `.pbstream` 也必须存在。

不要直接复制 PC 的整个工作空间生成目录到 Nano。PC 的 `build/` 包含本机绝对路径、CMake 缓存和生成索引，`install/` 也包含本机生成的文件与链接。这里的目录选择只限制本次传输，不会清理 Nano 上已经存在的生成目录。若曾误传，或遇到缓存/JSON 索引错误，按[第 10 节](#10-同步后重复包cmake-缓存或-json-索引报错)备份并重新生成。

## 4. Nano：安装源码声明的依赖

从 PC 登录 Nano，或使用 Nano 本机的新终端：

```bash
ssh nvidia@192.168.1.188
```

以下命令都在 **Nano** 执行：

```bash
(
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/nvidia/surface_ws
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro humble
  rosdep install --from-paths src --ignore-src --rosdistro humble -y
  rosdep check --from-paths src --ignore-src --rosdistro humble
  colcon list --base-paths src --names-only
)
```

`rosdep update` 使用普通用户执行。该流程依据[官方 rosdep 使用说明](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)，从 `package.xml` 解析依赖；网络或依赖解析失败时先解决错误，再执行下一步。
括号内启用了失败即停止，避免更新失败后继续使用已有缓存安装依赖。若出现缺失的 `debian.yaml` 或 GitHub 索引超时，按[第 9 节](#9-rosdep-更新报错处理)修复后重试。

当前应列出八个项目包（已包含 PAVO2 驱动）：

```text
f1_interfaces
f1_l_cleaning
f1_lidar_driver
f1_localization
f1_mapping_bringup
f1_nano_bridge
f1_operator_gateway
f1_pool_cleaning
```

若列表为空，检查是否把源码放进了 `/home/nvidia/surface_ws/src`。先传源码，再构建，不需要先编译空工作空间。

## 5. Nano：编译

在 Nano 新构建终端执行，只加载系统 ROS，尚不要 source 本工作空间的 install。已有 operator 服务时，先确认无任务、无手动控制，再执行 `sudo systemctl stop f1-operator.service`，避免运行期间替换消息接口。

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --base-paths src --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

只扫描 `src/`，同时限制包间和包内并行度，减小 Nano 编译时的内存占用。这里统一采用复制安装（`AMENT_CMAKE_SYMLINK_INSTALL=OFF`），避免在已有复制安装缓存上直接切换 `--symlink-install` 造成目录冲突。仅构建本仓库时，成功应看到 `8 packages finished`，且没有 failed/aborted。失败时保留输出，先定位首个报错；只有编译成功后才加载本工作空间环境。

## 6. Nano：检查安装结果

另开一个 Nano 终端，执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 pkg prefix f1_nano_bridge
ros2 pkg executables f1_nano_bridge
ros2 interface show f1_interfaces/msg/ActuatorCommand
ros2 pkg executables cartographer_ros
ros2 pkg executables f1_lidar_driver
ros2 launch f1_lidar_driver lidar.launch.py --show-args
ros2 launch f1_nano_bridge bridge.launch.py --show-args
ros2 pkg executables f1_operator_gateway
ros2 launch f1_operator_gateway operator_gateway.launch.py --show-args
ros2 launch f1_mapping_bringup lidar_mapping.launch.py --show-args
ros2 pkg executables f1_l_cleaning
ros2 launch f1_pool_cleaning pool_bringup.launch.py --show-args
ros2 launch f1_l_cleaning l_bringup.launch.py --show-args
```

`f1_nano_bridge` 前缀应为 `/home/nvidia/surface_ws/install/f1_nano_bridge`，其可执行程序应只有 `f1_bridge`；`f1_operator_gateway` 应提供 `f1_operator_gateway`，`f1_l_cleaning` 应提供 `l_controller`。IMU 随原生串口桥发布，无独立接入节点。Cartographer 应有 `cartographer_node` 及 `cartographer_occupancy_grid_node` 或 `occupancy_grid_node`。`--show-args` 只检查启动入口，不启动硬件。

已有 operator 服务且本节检查通过后，执行：

```bash
sudo systemctl start f1-operator.service
systemctl status f1-operator.service --no-pager
journalctl -u f1-operator.service -n 80 --no-pager
```

`active` 只代表进程存活。还需查看 App/网关遥测中的 `firmware_protocol_version`、`firmware_protocol_compatible` 和 bridge 状态；旧 v2 固件会被新版桥接拒绝，必须另外烧录配套 v3 固件才能恢复控制。不要用放宽协议检查的方法绕过它。

以后运行节点时只加载 `/opt/ros/humble/setup.bash` 和 `/home/nvidia/surface_ws/install/local_setup.bash`。同机查看结果的终端使用相同 ROS 域；PC 参与实时通信时也设置域 33 和相同 RMW，并加载 PC 自己构建的 `surface_ws`。

## 7. Nano：离线测试

Nano 的精简部署不携带固件源码。在单独测试终端运行 ROS 离线用例，排除需要 F103 C 源码的两组测试，以及 `transport` 组内同样依赖固件 C 库的 PTY 集成用例：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/nvidia/surface_ws
export CYCLONEDDS_URI="file://$PWD/src/f1_operator_gateway/test/cyclonedds_localhost.xml"
PYTEST_ADDOPTS="-k 'not test_real_serial_pty_deadline_stop_and_imu'" \
  colcon test --base-paths src --executor sequential --return-code-on-test-failure \
  --ctest-args -E '^firmware(_imu)?$'
colcon test-result --verbose
```

用例包含网关协议、开放墙路线、清洗状态机及本机 ROS 检查，应无失败、无错误。测试使用合成输入和本机通信，不连接执行器。指定的 DDS 配置只供 localhost 测试，不能用于跨机运行。测试后关闭该终端，实景终端按第 6 节加载环境。Qt App 测试在 PC 另行执行，不计入 colcon 结果。

固件真实 C 解码/仲裁测试通常在 PC 完整工作空间运行。如确需在 Nano 执行，可将同版本固件工程单独放到工作空间外，并设置 `F1_FIRMWARE_DIR`；`USER/SYSTEM/HARDWARE/tests` 必须完整，主机 C 测试库在临时目录编译：

```bash
export F1_FIRMWARE_DIR=/home/nvidia/firmware_test_sources/F103firmware
colcon test --packages-select f1_nano_bridge --return-code-on-test-failure \
  --ctest-args -R '^(firmware(_imu)?|transport)$'
colcon test-result --verbose
```

这只在 Nano 编译和运行主机侧 C 测试库，不烧录固件，也不构建目标板 HEX。PC 的通过记录不能代替 Nano 的实际测试结果。

<a id="8-实机雷达单独恢复驱动"></a>

## 8. 实机雷达：切换到项目内驱动

PAVO2 源码及 SDK 已纳入 `src/f1_lidar_driver`，随工作空间构建，不再需要从外部寻找驱动源码。2026-10-31 实机只读检查确认：现有 `pavo2-scan.service` 已开机启用，驱动实际位于 `/home/nvidia/ros2_ws/src/my_robot_stack/pavo2_ros2`；旧目录当前仍存在。不要在新驱动验收前删除它。

PC 已准备配置、启动入口与服务模板；**Nano 的同步、构建和服务切换由你执行**：

1. 按前文同步源码，在 Nano 构建 `f1_lidar_driver`（或整个工作空间）。
2. 核对新包 `config/pavo2.yaml`；保持雷达 `10.10.10.121:2368`、本机有线静态地址 `10.10.10.100/24`、扫描话题 `/pavo2_scan` 和帧名 `pavo2s_frame`。
3. 按[驱动包的服务切换步骤](src/f1_lidar_driver/README.md#nano-由操作者切换现有开机服务)备份并替换同名 `pavo2-scan.service`。新服务仅加载系统 ROS 与 `surface_ws/install/local_setup.bash`，启动 `f1_lidar_driver lidar.launch.py`。不要同时运行旧、新驱动。
4. 雷达供电接线后检查实际扫描，随后再运行建图或定位：

```bash
systemctl show pavo2-scan.service -p UnitFileState -p ActiveState -p ExecStart --no-pager
ros2 topic info /pavo2_scan --verbose
ros2 run f1_mapping_bringup scan_check --samples 100
```

`local_ip` 参数不会替系统设置网卡 IP。既有有线连接已设为自动连接和 `10.10.10.100/24`；没有接线时网口 DOWN、服务连接失败并重试，不代表驱动已完成实机验收。旧 `pavo2s-scan.service` 保持禁用。

App 受管入口继续复用独立雷达服务，切换建图/定位不重启雷达。服务故障时同时检查 `journalctl -u pavo2-scan.service` 和 `/home/nvidia/pavo2_scan_driver.log`。详细配置、回退与验收边界见[驱动说明](src/f1_lidar_driver/README.md)。

当前使用的地图统一放在 PC 和 Nano 各自工作空间的 `maps/` 根目录；历史 `backups/nano_maps` 仅作归档，不作为默认运行路径。

## 9. rosdep 更新报错处理

2026-10-22 Nano 反馈了两处更新错误：

| 报错 | 含义 |
| --- | --- |
| 找不到 `file:///usr/share/python3-rosdep2/debian.yaml` | rosdep 源列表仍引用一个已不存在的本地规则文件；可能是更换 rosdep 安装包后遗留的配置，单凭日志不能确认变更过程 |
| `raw.githubusercontent.com/.../index-v4.yaml` 超时 | ROS 发行版索引下载失败；base/python 等规则已经使用镜像，不代表发行版索引也已换源 |

后续 `All system dependencies have been satisfied` 表示依照当前可用规则，项目声明的依赖已满足，不表示上面的更新成功。`colcon list` 能列出第 4 节的当前包列表，才说明当前源码位置与功能包发现符合预期；历史日志中的四包数量不再适用于当前版本。

在该 Ubuntu Nano 上，仅当所引用文件确实不存在时，备份源列表并注释失效引用：

```bash
if [ ! -f /usr/share/python3-rosdep2/debian.yaml ]; then
  sudo sed -i.bak '\|^[[:space:]]*yaml[[:space:]].*file:///usr/share/python3-rosdep2/debian\.yaml|s|^|# disabled missing source: |' \
    /etc/ros/rosdep/sources.list.d/*.list
fi
```

`.bak` 保存修改前的列表；命令只注释匹配的源行，保留其他规则源。无需为了这条失效引用再同时安装另一套 rosdep。

根据[清华 TUNA 的 rosdistro 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/)，为当前终端指定发行版索引，再重新更新并检查：

```bash
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
(
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/nvidia/surface_ws
  rosdep update --rosdistro humble
  rosdep install --from-paths src --ignore-src --rosdistro humble -y
  rosdep check --from-paths src --ignore-src --rosdistro humble
  colcon list --base-paths src --names-only
)
```

确认没有 `ERROR`，并显示缓存更新成功与依赖检查通过，再继续第 5 节编译。该 `export` 只对当前终端有效；以后新终端执行 rosdep 更新前需重新设置，或手动在 `~/.bashrc` 中保留一行同样的设置。若镜像仍超时，先检查网络，不继续反复初始化或删除源码。

## 10. 同步后重复包、CMake 缓存或 JSON 索引报错

如果 `colcon build` 报 `Duplicate package names`，且两个路径分别位于 `backups/.../src/` 和 `src/`，原因是默认递归扫描把备份也识别成了源码。备份应放在工作空间外；尚需留在工作空间内时，至少在备份根目录添加 `COLCON_IGNORE`。日常构建仍使用第 5 节的 `--base-paths src`。

以下一次性整理命令只在 **Nano** 执行：历史备份移到工作空间外，删除误传的 Windows 与下位机源码，保留 `src/`、`under_control/`、地图和日志。`under/` 是固件源码，`under_control/` 是运行中的水下服务，不能混淆删除。执行前确认这两份工程在 PC 有原件，且 Nano 中没有独有修改。

```bash
set -e
cd /home/nvidia/surface_ws
surface_backup_root="$(mktemp -d /home/nvidia/surface_ws_cleanup.XXXXXX)"
for directory in backups deploy_backups; do
  if [ -d "$directory" ]; then mv -- "$directory" "$surface_backup_root/"; fi
done
rm -rf -- /home/nvidia/surface_ws/under /home/nvidia/surface_ws/operator_app
source /opt/ros/humble/setup.bash
colcon list --names-only
```

下面处理生成目录本身损坏的情况；仅重复包报错不需要清空 `build/` 或 `install/`。


2026-10-22 Nano 构建反馈：`f1_interfaces` 成功，`f1_nano_bridge` 随即提示缓存没有 `CMAKE_PROJECT_NAME`，并在 `_read_codemodel()` 的 `json.load()` 报 `JSONDecodeError: Expecting value: line 1 column 1`。

这表明 colcon 读到的 CMake 缓存和生成索引无效；此处读取的是 `build/f1_nano_bridge/.cmake/api/v1/reply/index-*.json`，不是 UI 网络 JSON。可能来自误传 PC 生成目录、构建中断或文件写入异常，尚不能仅凭日志确定根因。仓库的桥接包 `CMakeLists.txt` 已包含 `project(f1_nano_bridge)`，不需要为这个错误再添加一次 `project()`。

先停止本工作空间节点及其他构建进程。在 Nano 新终端只加载系统 ROS，不自动加载待重建的工作空间；若 `~/.bashrc` 自动 source 该工作空间，先注释该行再重新打开终端。

在 **Nano** 执行以下命令。命令把三个生成目录移到工作空间外的备份目录，不改动 `src/`、地图或配置；如果之前是从 PC 复制整个工作空间，重新生成所有包可避免后续包继续使用异机缓存。

```bash
(
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/nvidia/surface_ws
  test -s src/f1_nano_bridge/CMakeLists.txt
  sed -n '1,12p' src/f1_nano_bridge/CMakeLists.txt
  df -h .
  df -i .

  surface_cache_backup="$(mktemp -d /home/nvidia/surface_ws_build_backup.XXXXXX)"
  for generated_dir in build install log; do
    if [ -e "$generated_dir" ] || [ -L "$generated_dir" ]; then
      mv -- "$generated_dir" "$surface_cache_backup/"
    fi
  done
  printf '旧生成目录已备份到：%s\n' "$surface_cache_backup"

  CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
    colcon build --base-paths src --executor sequential \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
```

开头应能看到 `project(f1_nano_bridge)`，磁盘空间和 inode 不应耗尽。若 `test -s` 失败或文件内容不完整，先按第 3 节重新同步源码；不要在不完整源码上继续构建。

只有构建成功后，在运行终端加载并检查：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
ros2 pkg executables f1_nano_bridge
ros2 launch f1_operator_gateway operator_gateway.launch.py --show-args
```

[`--cmake-clean-cache`](https://colcon.readthedocs.io/en/main/reference/verb/build.html#cmake-specific-arguments) 只移除 `CMakeCache.txt`，不等于清空 `.cmake/api` 中的生成索引；本次两个位置都出现异常，因此采用备份整个生成目录的方法。

如果重新生成后仍出现空 JSON，不要反复覆盖或清空缓存。保留新失败日志及上面的备份，检查 `df -h`、`df -i`，并查看 `log/latest_build/f1_nano_bridge/stderr.log` 和 `stdout_stderr.log`；必要时用 `journalctl -k -n 100 --no-pager` 检查 OOM 或文件系统 I/O 错误。Nano 重新构建的结果仍需实际执行确认，本说明未远程操作设备。

## 定位拆包升级

2026-10-30 起，定位实现位于 `f1_localization`，当时源码包含七个项目包（现已新增雷达驱动，为八包）。矩形和大 L 的任务启动命令不变，旧定位 launch 与可执行入口保留转发。详见[定位包说明](src/f1_localization/README.md)。

首次升级时，先停止运行中的定位和清洗节点，备份现场修改的参数文件。在 PC 将新定位包及两个清洗包同步到 Nano；下列 `--delete` 仅限这三个包目录，用于移除迁走的旧定位源码和配置：

```bash
for package in f1_localization f1_pool_cleaning f1_l_cleaning; do
  rsync -av --delete --exclude='__pycache__/' --exclude='.pytest_cache/' \
    "/home/bboss/new-nano/surface_ws/src/$package/" \
    "nvidia@192.168.1.188:/home/nvidia/surface_ws/src/$package/"
done
```

在 Nano 的新终端中只加载系统 ROS，将原构建和安装目录移到独立备份目录后重建。不要加载旧 overlay，避免残留模块或旧 CTest 结果掩盖缺失依赖：

```bash
set -e
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
localization_backup=$(mktemp -d /home/nvidia/pre-localization-split.XXXXXX)
touch "$localization_backup/COLCON_IGNORE"
for generated in build install log; do
  if [ -d "$generated" ]; then mv "$generated" "$localization_backup/"; fi
done
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --base-paths src --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon test --executor sequential \
  --packages-select f1_localization f1_pool_cleaning f1_l_cleaning \
  --return-code-on-test-failure
colcon test-result --verbose
```

测试完成后关闭测试终端，实机另开终端加载正常运行环境。单独部署定位可只构建 `--packages-up-to f1_localization`；当前包含接口、雷达驱动、建图配置和定位四包，不会构建清洗包或 bridge，也不会自动启动驱动。地图配套文件仍需独立同步。

## Windows App 清洗功能升级（2026-10-30）

同步并重新构建 `f1_operator_gateway`、`f1_localization`、`f1_pool_cleaning`、`f1_l_cleaning`，同时更新 Windows 的整个 `operator_app`。原手动协议保留，新任务界面要求后端提供启动检查及任务实例标识。

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --base-paths src --executor sequential --packages-up-to f1_operator_gateway \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/local_setup.bash
ros2 launch f1_operator_gateway operator_bringup.launch.py \
  map:="$PWD/maps/现场地图.pbstream" bind_address:=0.0.0.0
```

该入口默认预览，不会自动 start。已有完整 bringup 时不要再运行它；仅启动网关即可，已有 L 定位使用 `localization_prefix:=/l_cleaning`。两类任务接口不再重映射。完整操作与验收边界见[网关](src/f1_operator_gateway/README.md)和 [App](operator_app/README.md)文档。

### 当前版本升级核对（2026-11-09，`9db6798`）

- `l` 任务已扩展为开放多段墙链，App 显示“开放多边清洗”。同步两个清洗包、网关及整个 Windows App；使用 exe 时重新打包。先运行 `ros2 run f1_pool_cleaning map_profile <现场地图> --json` 核对墙链和边编号，原示例大 L 的两条边现为 `edge_3/edge_4`。默认最长链，其他候选通过 `l_config` 的 `route_chain_index` 选择。
- 现场自定义 YAML 不会自动获得新默认值。核对 `wall_target_m=0.33`、墙距区间 `0.32–0.36` m、`front_stop_m=0.40`、`brush_percent=50.0`、`side_percent=50.0` 和独立的 `transfer_side_percent=50.0`；感知配置的 `brush_max_compression_m=0.02`。沿墙默认由滚刷推进，主电机提速需在启动前单独启用，实船预期见[矩形验收步骤](src/f1_pool_cleaning/DEPLOY_TEST_NANO.md)。
- 2026-11-09 显示链路更新需要同步 Nano 网关和整个 Windows `operator_app`，exe 重新打包。App 只显示底图、实时位姿与实际轨迹，网关停止上报显示用雷达点和预期轨迹；地图改为后台压缩，受管定位底图首次加载后冻结，建图默认最多每 2 秒更新一次。实际轨迹与诊断使用独立消息，旧 App 不兼容新版压缩地图。控制器仍使用 `front_stop_m=0.40`、`wall_band_min_m=0.32`、`wall_band_max_m=0.36`；本文更新不表示已执行远端部署。
- 定位 Lua 当前为 `motion_filter.max_time_seconds=0.3`、`optimize_every_n_nodes=5`；更频繁优化不等于定位就绪，需观察 Nano 负载与定位状态，见[定位调优说明](src/f1_localization/README.md)。
- 网关新增可补读的停止诊断历史，App 支持日志导出。更换工作空间内服务模板不会自动更新 `/etc/systemd/system/` 中已安装的副本；采用当前校时与恢复策略时，按下节重新安装模板并 `daemon-reload`。
- 控制器配置加载修复位于 `f1_operator_gateway/launch/operator_bringup.launch.py`。受管模式和直接组合启动均在传入清洗 launch 前解析默认 YAML；无效的显式路径会报错。升级后核对两个控制器的实际参数，不能只看源码 YAML 或 App 的定位提示，详见下节。

发生手动停止或连接异常时，关闭 App 前导出“诊断日志”，并保留同一时段的 `journalctl -u f1-operator.service`。App 内存日志、网关最近 50 条事件、`log/operator_runtime/` 模式子进程日志分别记录不同层次，不会相互替代；时间含义与保留范围见[App 诊断说明](operator_app/README.md#诊断日志)。


## 定位正常但开放任务不能启动

App 的“定位已稳定”“雷达正常、位姿正常”来自共享定位状态；“开始清洗”还取决于所选任务的 `can_start`。矩形与开放任务复用同一套控制器启动检查，但由两个独立节点接收 TF、维护位姿缓存和发布任务状态。切换下拉框只切换查看的任务，不会把矩形控制器的位姿传给开放控制器。

标准固定地图配置应为：

| 检查项 | 矩形任务 | 开放多边任务 |
|---|---|---|
| 控制器 | `/pool_controller` | `/l_controller` |
| 状态话题 | `/pool_cleaning/status` | `/l_cleaning/status` |
| `use_map_tf` | `true` | `true` |
| `require_localization_ready` | `true` | `true` |
| 位姿来源 | `map → base_link` TF，状态为 `pose_source=tf` | 同左 |

先确认当前地图支持哪类任务：受管状态 `runtime.task_kinds` 只列出已支持并启动的类型。如果开放任务原因是 `no open wall route; supported boundaries are closed or degenerate`，则地图未提取出有效开放墙链，管理器不会启动 `/l_controller`。这种情况下没有该节点或参数服务是预期行为，应核对地图几何，与下面的 YAML 漏加载故障不同。

两控制器默认 `pose_max_age_sec=0.25`，分别检查位姿源时间戳和距本节点最近一次接收时间戳前进的新位姿的时间；重复读取旧 TF 不刷新接收计时。共享定位默认允许的位姿年龄为 0.5 秒。因此短时更新停顿也可能出现定位尚正常、任务不可启动，但不能仅凭中文提示认定是延迟。

在 **Nano 终端**只读检查当前实例，环境应与开机服务一致：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 param get /pool_controller use_map_tf
ros2 param get /pool_controller require_localization_ready
ros2 param get /l_controller use_map_tf
ros2 param get /l_controller require_localization_ready
ros2 topic echo /pool_cleaning/status --once
ros2 topic echo /l_cleaning/status --once
```

关注状态 JSON 的 `can_start`、`start_blockers`、`pose_source` 和 `phase`。App 悬停启动条件文字可查看原始原因，常见原因含义如下：

| 原始原因 | 含义 |
|---|---|
| `pose receive stream stopped` | 控制器没有收到有效位姿，或距最近一次收到时间戳前进的新位姿已超时 |
| `fresh fixed-map base_link pose required` | 控制器当前没有位姿对象 |
| `pose stale or future stamped` | 已有位姿，但源时间戳无效、过旧或超前 |

App 将多种 `pose` 原因合并为“位姿数据未就绪”，应结合原始原因区分。预检还包括任务阶段、控制权、定位就绪等条件；自动选墙模式不以墙观测作为 `can_start` 前置条件，真正调用 `start` 时还会选墙和检查可用起始区间。按钮可点击不代表几何选墙或完整清洗一定成功。

### 2026-11-09 空配置路径故障与修复

已确认的故障链为：`operator_mode.launch.py` 的 `rectangle_config`、`l_config` 默认空字符串被嵌套 launch 继承，内层 `DeclareLaunchArgument` 不会用默认路径覆盖已有空值。旧代码将空值继续作为 `config` 传给清洗节点，日志出现 `Parameter file path is not a file: .`，两个清洗 YAML 均被跳过。矩形 launch 另行覆盖了 TF 和定位就绪参数，因而掩盖了问题；开放控制器则使用代码默认的 `false`，等待无人发布的 `/l_cleaning/map_pose`，不能消费已经正常存在的 TF。这次持续不能启动的原因是配置没有加载，不是单纯上报慢。

修复后的 `operator_bringup.launch.py` 在构造当前地图支持的清洗任务前解析配置：

- `rectangle_config` 未指定或为空：使用 `f1_pool_cleaning` 安装目录的 `config/pool_cleaning.yaml`。
- `l_config` 未指定或为空：使用 `f1_l_cleaning` 安装目录的 `config/l_cleaning.yaml`。
- 显式非空路径必须是文件，否则报出参数名和路径并中止该次地图启动；保留有效的自定义 YAML。

这条规则适用于受管模式和直接 `operator_bringup`；独立清洗入口的参数名仍是 `config`。`localization_config` 的空值语义没有改变。源码 YAML 内容正确，不代表安装目录及运行实例已使用它，可用 `ros2 pkg prefix f1_pool_cleaning`、`ros2 pkg prefix f1_l_cleaning` 找到当前环境选中的安装前缀，再检查 `share/<包名>/config/`。

`use_map_tf` 与 `require_localization_ready` 是只读参数，不能用 `ros2 param set` 临时修复。部署启动文件或参数改动后，需要在任务停止时通过 App 重新加载地图并按新会话设置初始位姿，或重启对应启动进程，随后重新查询实际参数。仅同步文档、或源文件已与正在运行的修复版本一致时，无需重启当前正常会话。不要通过关闭定位检查或放宽 250 ms 门限掩盖配置缺失。

日志分别查看 `journalctl -u f1-operator.service -n 100 --no-pager` 和工作空间 `log/operator_runtime/<session_id>.log`。本次修复的测试、Nano 参数核验、备份和操作者反馈见[网关验证记录](src/f1_operator_gateway/README.md#控制器配置加载修复验证2026-11-09)。

## App 建图选图与开机服务

本次新增受管入口，允许没有地图时开机待机，日常在 Windows App 完成建图、保存和选图。先同步源码并重新构建 `--packages-up-to f1_operator_gateway`，Windows 同步整个 `operator_app/` 并重新打包。先以前台预览模式验证：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1_operator_gateway operator_system.launch.py \
  workspace_root:=/home/nvidia/surface_ws bind_address:=0.0.0.0
```

新入口复用现有雷达驱动，不启动、停止或替换 `pavo2-scan.service`。不要与旧的完整建图/定位/清洗 bringup 或另一个网关同时运行。确认现场串口、方向等配置后才指定 `output_enabled:=true`。

仓库提供 `src/f1_operator_gateway/deploy/f1-operator.service` 和 `start-operator.sh`。服务模板按 Nano 用户 `nvidia`、工作空间 `/home/nvidia/surface_ws` 编写；安装前按实际路径修改 `User/WorkingDirectory/F1_WORKSPACE/ExecStart`。当前船体部署按操作者要求设置 `F1_OUTPUT_ENABLED=true`，开启串口、运动及第五路控制许可；单独执行 launch 的默认值仍为 false。仅预览时将服务环境变量改为 false。服务使用 `Restart=always`，也覆盖子节点退出后 ROS launch 返回成功的情况；执行 `systemctl stop` 不会触发自动重启。开机或重启后进入待机，不恢复旧动作。

`ExecStartPre` 运行 `deploy/wait-operator-clock.py --offline-after 20`，采用以下三层恢复策略：

1. **启动**：优先等待 `NTPSynchronized=yes` 和连续 5 秒稳定时间；20 秒后仍未同步则进入离线模式，重新观察完整的 5 秒稳定窗口。离线模式要求系统日期处于 2024-01-01 至 2038-01-19 03:14:08 UTC（不含）之间，但这只是合理范围检查，不证明日期准确。时间偏移相对窗口起点变化超过 20 ms、采样延迟或中断会重新计时。日期无效或时间持续不稳定时仍保持 `activating (start-pre)`，每 30 秒记录原因。NTP 查询最多可能额外占用 3 秒。
2. **运行**：保留桥接相对启动基准的 50 ms 时间偏移保护和 IMU 时间跳变检测。时间故障先锁定输出、撤销租约，再由串口线程尽力发送中性帧并关闭串口；桥接主循环不依赖 ROS 定时器，发现恢复请求后清理退出，退出码为 75。中性帧不能保证设备收到或覆盖 RC，固件看门狗仍是兜底。其他故障不因这条时间恢复路径自动解锁。
3. **恢复**：受管 `operator_system.launch.py` 在桥接退出后关闭网关及受管地图/任务进程；systemd 等待 5 秒，重新执行启动检查并拉起整个 operator 服务。恢复为无地图待机，旧租约、运动指令及任务不恢复。App 自动重连并清空旧手动输入，操作者需要重新选图、定位，再开始任务。未保存的建图进度可能丢失，已经保存的地图保留。

离线模式使用操作系统已经建立的时间，不会运行 `date`/`hwclock`、伪造同步标志或写入 RTC。目标机应通过有后备电源的 RTC，或系统已有的持久化时间恢复机制建立合理日期；没有 RTC 时，上次保存时间只是粗略下界，长时间断电后日志和地图日期可能不准确。部署时必须验证断电后的 `date -Is`，不能仅凭检查通过认定时间准确。日志中的 `offline local time (unverified date)` 表示离线启动。

联网恢复后仍允许系统校时；若因此出现明显时间变化，会安全停止并走上述重启流程，而不是无缝继续作业。当前桥接比较的是相对启动时的累计偏移，长时间的平滑校正也可能超过 50 ms，因此不能把“只平滑调整”当作不会触发恢复的保证。服务限制 600 秒内最多启动 5 次；超过限制后保持失败状态，不自动恢复任务。排除时间反复变化问题后，执行 `sudo systemctl reset-failed f1-operator.service` 再 `sudo systemctl start f1-operator.service`。

脚本单独运行时默认仍严格要求 NTP；`--offline-after 0` 表示立即开始离线判定，`--timeout` 仅用于诊断，超时失败退出，不会强制放行。直接执行 ROS launch 不经过启动检查，也没有 systemd 自动重启；时间异常退出后需要自行启动。

2026-10-31 校时检查部署验证：13 个本地测试通过；Nano 临时 systemd 测试模拟未校时，确认后续进程未启动。正式服务在 15:06:22 开始检查，15:06:27 通过后启动；随后连续 15 秒的 151 条遥测均为 `READY`，CTRL/ACK、IMU 和雷达数据正常，船体保持停止。此次验证没有修改系统时间或重启整机。原服务备份位于 Nano 的 `deploy_backups/clock-gate-20261031_150457/`。

上述记录属于旧的严格 NTP 启动版本，不代表三层恢复策略已通过实机验收。升级须同时同步并构建 `f1_nano_bridge` 与 `f1_operator_gateway`，再安装新 service 模板并执行 `daemon-reload`。仅更新 Python 脚本不会改变已安装服务的严格启动参数。Windows App 协议没有变化，无需为本次恢复策略重新打包。

已有服务升级时，在结束任务、船体停止后执行以下步骤（先将新版源码同步到 Nano）：

```bash
sudo systemctl stop f1-operator.service
cd /home/nvidia/surface_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --base-paths src --executor sequential --packages-up-to f1_operator_gateway \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
sudo install -m 644 src/f1_operator_gateway/deploy/f1-operator.service /etc/systemd/system/f1-operator.service
sudo systemctl daemon-reload
sudo systemctl reset-failed f1-operator.service
sudo systemctl start f1-operator.service
journalctl -u f1-operator.service -n 80 --no-pager
```

检查 `systemctl cat f1-operator.service` 中实际生效的 `ExecStartPre` 包含 `--offline-after 20`，同时核对本机原有 drop-in 是否覆盖了模板。`enable --now` 不会重启已经运行的服务，不能替代上述 stop/start。

三层策略的现场验收：断开路由器 WAN、保持局域网连接后冷启动，确认约 25 秒（不含网络就绪等待）进入离线待机；验证地图和日志日期的实际误差；在无运动输出的条件下验证正向/反向时间跳变，确认服务重启且不恢复旧任务；恢复联网后确认校时和后续重启能够收敛。持续跳变时不得获得运动租约，达到启动限流后应停止重试。健康检查按原有顺序在 operator 服务完成启动后执行，等待期间应直接查看 `journalctl`。

以下为目标机的一次性安装步骤；执行前关闭已占用同端口/同硬件的手工入口：

```bash
sudo install -m 644 /home/nvidia/surface_ws/src/f1_operator_gateway/deploy/f1-operator.service /etc/systemd/system/f1-operator.service
sudo systemctl daemon-reload
sudo systemctl enable --now f1-operator.service
systemctl status f1-operator.service --no-pager
journalctl -u f1-operator.service -n 80 --no-pager
```

服务使用 control-group 停止策略，退出时收回受管子进程。开机无地图时只待机，不自动恢复运动、建图或清洗。地图写入 Nano 的 `F1_WORKSPACE/maps/`。本文同步命令包含地图；仅更新源码时可加入 `--exclude='/maps/'`，避免覆盖 Nano 上的同名地图。

开机健康检查的受管副本位于同一 `deploy/` 目录中的 `usv-boot-health.service` 与 `usv_boot_health.sh`。更新 Nano 现有服务：

```bash
sudo install -m 755 /home/nvidia/surface_ws/src/f1_operator_gateway/deploy/usv_boot_health.sh /usr/local/sbin/usv_boot_health.sh
sudo install -m 644 /home/nvidia/surface_ws/src/f1_operator_gateway/deploy/usv-boot-health.service /etc/systemd/system/usv-boot-health.service
sudo systemctl daemon-reload
sudo systemctl enable usv-boot-health.service
sudo systemctl start usv-boot-health.service
cat /var/log/usv_boot_health.log
```

该服务在雷达和 App 服务启动后采集一次快照，记录系统时间、网络、服务状态、8765 端口、地图文件、配置中的雷达地址与串口设备，以及 8 秒 ROS 扫描/控制状态。它只订阅状态，不申请控制权、不打开串口、不发送运动命令。日志保留最新快照；`inactive (dead)` 表示一次性脚本已退出，检查 `Result` 与日志确认执行情况。

2026-10-31 Nano 部署验证：App 服务及健康检查均已启用自启动；串口实际为稳定 by-id 路径对应的 `/dev/ttyACM0`，桥接回报 `READY`、CTRL/ACK 和 IMU 数据持续更新，WebSocket 地图列表返回 3 份有效地图。对 App 服务主进程发送 SIGINT 后，进程正常退出，systemd 约 5 秒后自动拉起；该验证没有发送运动指令，也没有重启整机。原配置备份在 Nano 的 `deploy_backups/operator-20261031_134505/`。

本地集成测试只能使用隔离域和 localhost DDS 配置；不能让模拟雷达进入实机域。新增 `runtime_ros` 测试用合成扫描驱动真实 Cartographer，验证 WebSocket 建图、断线重连、三件套保存和重新加载，硬件输出始终关闭。
