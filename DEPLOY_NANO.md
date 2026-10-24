# Nano 从零部署 surface_ws

更新：2026-10-24。适用于 Nano 上旧 ROS 工作空间已全部删除的情况。直接创建新目录，不执行旧目录改名、迁移或清理命令。若已有旧清洗包，先按 [清洗包合并说明](src/f1_pool_cleaning/MIGRATION.md) 升级，再使用本文当前包列表。

| 项目 | 统一约定 |
| --- | --- |
| PC 工作空间 | `/home/bboss/new-nano/surface_ws` |
| Nano 工作空间 | `/home/nvidia/surface_ws` |
| Nano 登录 | `nvidia@192.168.1.188`，地址变化时替换 |
| 系统 ROS | `/opt/ros/humble` |
| 桥接功能包 | `f1_nano_bridge` |
| 自定义接口包 | `f1_interfaces` |
| 沿边清洗功能包 | `f1_pool_cleaning` |
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
  ros-humble-cartographer-ros ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins
sudo systemctl enable --now ssh
mkdir -p /home/nvidia/surface_ws/src
```

Cartographer 使用系统安装的 `ros-humble-cartographer-ros`，无需另外创建依赖工作空间。其他消息包、RViz、构建与测试依赖在传完源码后由 rosdep 安装。Humble 的 Cartographer 发布信息见 [ROS Index](https://index.ros.org/p/cartographer_ros/)。

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

源目录末尾的 `/` 表示传目录内容，目标应是 `surface_ws/src`，不会变成 `surface_ws/surface_ws/src`。本命令同时传输 README 和部署文档；没有使用 `--delete`。本轮新建目录无需删除远端文件。

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

当前应列出恰好五个项目包：

```text
f1_interfaces
f1_mapping_bringup
f1_nano_bridge
f1_operator_gateway
f1_pool_cleaning
```

若列表为空，检查是否把源码放进了 `/home/nvidia/surface_ws/src`。先传源码，再构建，不需要先编译空工作空间。

## 5. Nano：编译

仍在刚才的 Nano 构建终端执行，尚不要 source 本工作空间的 install：

```bash
cd /home/nvidia/surface_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

同时限制包间和包内并行度，减小 Nano 编译时的内存占用。成功时应看到 `4 packages finished`，且没有 failed/aborted。失败时保留输出，先定位首个报错；只有编译成功后才加载本工作空间环境。

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
ros2 launch f1_nano_bridge bridge.launch.py --show-args
ros2 pkg executables f1_operator_gateway
ros2 launch f1_operator_gateway operator_gateway.launch.py --show-args
ros2 launch f1_mapping_bringup lidar_mapping.launch.py --show-args
```

`f1_nano_bridge` 前缀应为 `/home/nvidia/surface_ws/install/f1_nano_bridge`，其可执行程序应只有 `f1_bridge`；`f1_operator_gateway` 应提供 `f1_operator_gateway`。IMU 随原生串口桥发布，无独立接入节点。Cartographer 应有 `cartographer_node` 及 `cartographer_occupancy_grid_node` 或 `occupancy_grid_node`。`--show-args` 只检查启动入口，不启动硬件。

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
  --ctest-args -E '^firmware$'
colcon test-result --verbose
```

此命令明确排除了 `firmware` 测试，因为 ROS 工作空间不包含它所需的 STM32 固件源码；其余用例包含网关协议和本机 ROS 检查，应无失败、无错误。测试使用合成输入和本机通信，不连接执行器。测试后关闭该终端，实景终端按第 6 节加载环境。

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
  --ctest-args -R '^firmware$'
colcon test-result --verbose
```

这只在 Nano 编译和运行主机侧 C 测试库，不烧录固件，也不构建目标板 HEX。PC 的通过记录不能代替 Nano 的实际测试结果。

## 8. 实机雷达：单独恢复驱动

**当前四个包不包含 PAVO2 雷达驱动。** 如果驱动曾放在被删除的工作空间里，它也需要从原源码/备份恢复。上述四包编译成功不代表 `/pavo2_scan` 已恢复。

先检查旧服务是否存在、指向哪里：

```bash
systemctl cat pavo2-scan.service
systemctl show pavo2-scan.service -p ExecStart -p WorkingDirectory --no-pager
```

服务文件仍存在不等于驱动存在。若入口指向已删除目录，需要恢复对应 ROS 2 驱动源码，并更新服务引用的路径和环境。后续驱动源码也放在 `/home/nvidia/surface_ws/src/` 中，安装依赖并在同一工作空间构建；加入驱动后包数量会多于四个。仓库尚未包含该驱动及服务定义，因此这里不假设其仓库地址、包名或可执行文件。

驱动恢复后，在运行终端检查：

```bash
ros2 topic info /pavo2_scan --verbose
ros2 run f1_mapping_bringup scan_check --samples 100
```

确认扫描有效后，再按[纯雷达建图步骤](START_LIDAR.md)启动 SLAM，或按[沿边清洗实景验证](src/f1_pool_cleaning/DEPLOY_TEST_NANO.md)继续。已有地图备份仍在 PC 的 `backups/nano_maps`，需要时再单独恢复。

## 9. rosdep 更新报错处理

2026-10-22 Nano 反馈了两处更新错误：

| 报错 | 含义 |
| --- | --- |
| 找不到 `file:///usr/share/python3-rosdep2/debian.yaml` | rosdep 源列表仍引用一个已不存在的本地规则文件；可能是更换 rosdep 安装包后遗留的配置，单凭日志不能确认变更过程 |
| `raw.githubusercontent.com/.../index-v4.yaml` 超时 | ROS 发行版索引下载失败；base/python 等规则已经使用镜像，不代表发行版索引也已换源 |

后续 `All system dependencies have been satisfied` 表示依照当前可用规则，项目声明的依赖已满足，不表示上面的更新成功。四个包已列出则说明源码位置与功能包发现正常，无需重新传输源码。

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
