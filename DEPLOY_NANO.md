# Nano 从零部署 surface_ws

更新：2026-10-31。适用于 Nano 首次部署或已有 `surface_ws` 升级。当前包含 PAVO2 驱动、建图与定位、硬件桥、两种清洗任务及 Qt App 网关。PC 与 Nano 分别构建；本文命令由操作者在目标设备执行。本次雷达封装没有在 Nano 编译、部署或切换服务。已有旧清洗包升级见[清洗包迁移说明](src/f1_pool_cleaning/MIGRATION.md)，雷达旧服务迁移见第 8 节。

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

下面每节标明命令在哪台设备执行。PC 和 Nano 分别编译，不传 PC 的生成目录。本说明没有自动连接或修改 Nano。

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
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/src'
rsync -av --progress \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' \
  --exclude='.git/' --exclude='.venv/' \
  --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
  --exclude='/maps/' --exclude='/test_bags/' --exclude='/deploy_backups/' \
  /home/bboss/new-nano/surface_ws/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/
```

源目录末尾的 `/` 表示传工作空间的目录内容，目标是 `/home/nvidia/surface_ws/`，源码会落入其 `src/`，不会变成 `surface_ws/surface_ws/src`。本命令同时传输 README、App 和部署文档；没有使用 `--delete`。本轮新建目录无需删除远端文件。

`maps/` 被 Git 忽略且被上述同步命令排除，需要单独提供地图。启动时用 `map:=实际绝对路径` 指定本次要用的地图，池尺寸、边界和 L 路线都会由它推导，不必再改 YAML；只传 `map:=xxx.yaml` 时，同名 `.pbstream` 必须存在，因为固定地图定位需要 Cartographer 状态而不只是栅格图。地图传输示例见各任务文档。

不要直接复制 PC 的整个工作空间生成目录到 Nano。PC 的 `build/` 包含本机绝对路径、CMake 缓存和生成索引，`install/` 也包含本机生成的文件与链接。这里的排除规则只阻止本次传输，不会清理 Nano 上已经存在的生成目录。若曾误传，或遇到缓存/JSON 索引错误，按[第 10 节](#10-同步后-cmake-缓存或-json-索引报错)备份并重新生成。

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
  colcon list --names-only
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

仍在刚才的 Nano 构建终端执行，尚不要 source 本工作空间的 install：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

同时限制包间和包内并行度，减小 Nano 编译时的内存占用。仅构建本仓库时，成功应看到 `8 packages finished`，且没有 failed/aborted。失败时保留输出，先定位首个报错；只有编译成功后才加载本工作空间环境。

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

以后运行节点时只加载 `/opt/ros/humble/setup.bash` 和 `/home/nvidia/surface_ws/install/local_setup.bash`。同机查看结果的终端使用相同 ROS 域；PC 参与实时通信时也设置域 33 和相同 RMW，并加载 PC 自己构建的 `surface_ws`。

## 7. Nano：离线测试

先在单独测试终端运行无需固件源码的用例：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/nvidia/surface_ws
colcon test --executor sequential --return-code-on-test-failure \
  --ctest-args -E '^firmware(_imu)?$'
colcon test-result --verbose
```

此命令排除 `firmware` 和 `firmware_imu` 两组测试，两者都需要仓库外的 STM32 固件源码；其余用例包含网关协议、大 L 路线、清洗状态机和本机 ROS 检查，应无失败、无错误。测试使用合成输入和本机通信，不连接执行器。测试后关闭该终端，实景终端按第 6 节加载环境。

如要运行完整测试，在 PC 单独同步只读测试所需的固件目录：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_firmware'
rsync -av --exclude='__pycache__/' --exclude='*.pyc' \
  /home/bboss/new-nano/firmware/USER \
  /home/bboss/new-nano/firmware/SYSTEM \
  /home/bboss/new-nano/firmware/HARDWARE \
  /home/bboss/new-nano/firmware/tests \
  nvidia@192.168.1.188:/home/nvidia/surface_firmware/
```

`surface_firmware` 是测试资料目录，不是另一个 ROS 工作空间。在 Nano 测试终端补跑：

```bash
export F1_FIRMWARE_DIR=/home/nvidia/surface_firmware
colcon test --packages-select f1_nano_bridge --return-code-on-test-failure \
  --ctest-args -R '^firmware(_imu)?$'
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

历史地图备份约定为 PC 的 `/home/bboss/new-nano/backups/nano_maps`；使用前确认实际文件存在，或从本地 `maps/` 单独同步。

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
  colcon list --names-only
)
```

确认没有 `ERROR`，并显示缓存更新成功与依赖检查通过，再继续第 5 节编译。该 `export` 只对当前终端有效；以后新终端执行 rosdep 更新前需重新设置，或手动在 `~/.bashrc` 中保留一行同样的设置。若镜像仍超时，先检查网络，不继续反复初始化或删除源码。

## 10. 同步后 CMake 缓存或 JSON 索引报错

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
    colcon build --symlink-install --parallel-workers 1 \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
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
localization_backup=$(mktemp -d "$PWD/pre-localization-split.XXXXXX")
touch "$localization_backup/COLCON_IGNORE"
for generated in build install log; do
  if [ -d "$generated" ]; then mv "$generated" "$localization_backup/"; fi
done
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon test --executor sequential \
  --packages-select f1_localization f1_pool_cleaning f1_l_cleaning \
  --return-code-on-test-failure
colcon test-result --verbose
```

测试完成后关闭测试终端，实机另开终端加载正常运行环境。单独部署定位可只构建 `--packages-up-to f1_localization`；不会构建清洗包或 bridge。地图配套文件仍需独立同步。

## Windows App 清洗功能升级（2026-10-30）

同步并重新构建 `f1_operator_gateway`、`f1_localization`、`f1_pool_cleaning`、`f1_l_cleaning`，同时更新 Windows 的整个 `operator_app`。原手动协议保留，新任务界面要求后端提供启动检查及任务实例标识。

```bash
colcon build --symlink-install --packages-up-to f1_operator_gateway
source install/local_setup.bash
ros2 launch f1_operator_gateway operator_bringup.launch.py \
  map:="$PWD/maps/现场地图.pbstream" bind_address:=0.0.0.0
```

该入口默认预览，不会自动 start。已有完整 bringup 时不要再运行它；仅启动网关即可，已有 L 定位使用 `localization_prefix:=/l_cleaning`。两类任务接口不再重映射。完整操作与验收边界见[网关](src/f1_operator_gateway/README.md)和 [App](operator_app/README.md)文档。


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

仓库提供 `src/f1_operator_gateway/deploy/f1-operator.service` 和 `start-operator.sh`。服务模板按 Nano 用户 `nvidia`、工作空间 `/home/nvidia/surface_ws` 编写；安装前按实际路径修改 `User/WorkingDirectory/F1_WORKSPACE/ExecStart`。模板默认 `F1_OUTPUT_ENABLED=false`，实机需要在核对硬件许可后改为 `true`。

以下为目标机的一次性安装步骤；执行前关闭已占用同端口/同硬件的手工入口：

```bash
sudo install -m 644 /home/nvidia/surface_ws/src/f1_operator_gateway/deploy/f1-operator.service /etc/systemd/system/f1-operator.service
sudo systemctl daemon-reload
sudo systemctl enable --now f1-operator.service
systemctl status f1-operator.service --no-pager
journalctl -u f1-operator.service -n 80 --no-pager
```

服务使用 control-group 停止策略，退出时收回受管子进程。开机无地图时只待机，不自动恢复运动、建图或清洗。地图写入 Nano `maps/`，该目录仍被默认同步排除，不会因源码更新而覆盖已有地图。

本地集成测试只能使用隔离域和 localhost DDS 配置；不能让模拟雷达进入实机域。新增 `runtime_ros` 测试用合成扫描驱动真实 Cartographer，验证 WebSocket 建图、断线重连、三件套保存和重新加载，硬件输出始终关闭。
