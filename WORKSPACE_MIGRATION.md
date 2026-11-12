# 工作空间与桥接包改名

**目标 Nano 已使用 `/home/nvidia/surface_ws`，不适用下面的旧目录改名命令。请使用[Nano 部署指南](DEPLOY_NANO.md)。** 本文仅保留给其他仍有旧 ROS 工作空间的环境作为历史迁移参考；不表示设备上所有旧程序或旧 H7 桥备份都已删除。

| 项目 | 原名称 | 新名称 |
| --- | --- | --- |
| PC 工作空间 | `/home/bboss/new-nano/f1_mapping_ws` | `/home/bboss/new-nano/surface_ws` |
| Nano 工作空间 | `/home/nvidia/f1_mapping_ws` | `/home/nvidia/surface_ws` |
| ROS 桥接包 | `nano_bridge` | `f1_nano_bridge` |

Python 模块仍为 `nano_ros_bridge`，原生桥节点仍为 `/nano_bridge`，配置文件顶层键仍为 `nano_bridge`。话题、服务、参数与执行器协议保持原样。

以上改名仅指水面 ROS 桥。2026-11-12 新增的水下 [`under_control/`](under_control/README.md) 是工作空间根目录下的独立 Python 服务，不属于这八个 ROS 包，不由 colcon 构建。双机部署需保留并另行同步该目录；移动工作空间时还需更新 `under-control.service` 的工作目录与启动脚本路径。旧 `nano-h7-bridge.service` 是另一套历史 H7 服务，不能按本页的 ROS 包改名替换；其停用、备份和回退见[水下部署记录](docs/UNDER_CONTROL_NANO_DEPLOYMENT.md)。

PC 源码目录已改名。下面步骤仅适用于仍保留旧目录、已按[从零部署指南](DEPLOY_NANO.md)准备好系统 ROS 和工具的 Nano；不依赖其他已删除的用户工作空间。

## 1. Nano：移动旧工作空间

先结束该工作空间的节点与构建/测试，打开没有加载旧工作空间安装环境的新终端。以下移动步骤仅用于旧目录存在、新目录尚未创建的情况；若新旧目录都已存在，先确认要保留的工作空间，不要把两个目录直接嵌套或覆盖。

```bash
(
  set -e
  test -d /home/nvidia/f1_mapping_ws
  test ! -e /home/nvidia/surface_ws
  mv /home/nvidia/f1_mapping_ws /home/nvidia/surface_ws
)
```

进入新工作空间，将遗留旧包及带旧绝对路径的生成目录备份移走。即使已经手动改过目录名，也需要处理旧生成目录。

```bash
(
  set -e
  cd /home/nvidia/surface_ws
  migration_backup="$(mktemp -d /tmp/surface_ws_migration.XXXXXX)"
  mkdir -p "$migration_backup/src"
  for package in nano_bridge f1_imu_bridge f1_bridge_interfaces f1_wall_interfaces; do
    if [ -e "src/$package" ]; then
      mv "src/$package" "$migration_backup/src/"
    fi
  done
  for tree in build install log; do
    if [ -e "$tree" ]; then
      mv "$tree" "$migration_backup/"
    fi
  done
  echo "旧源码与生成目录备份：$migration_backup"
)
```

## 2. PC：同步新源码

完成上面的 Nano 迁移后，在 PC 执行：

```bash
rsync -av --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
  /home/bboss/new-nano/surface_ws/src/ \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/src/
```

## 3. Nano：重新构建并加载

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
colcon list --names-only
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

当前源码应列出八个包：`f1_interfaces`、`f1_nano_bridge`、`f1_lidar_driver`、`f1_mapping_bringup`、`f1_localization`、`f1_pool_cleaning`、`f1_l_cleaning` 和 `f1_operator_gateway`。PAVO2 驱动已纳入仓库；既有雷达服务按[驱动说明](src/f1_lidar_driver/README.md)由操作者在 Nano 编译后切换。构建成功后，在运行终端加载新的 `install/local_setup.bash`。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
ros2 launch f1_nano_bridge bridge.launch.py --show-args
ros2 launch f1_operator_gateway operator_gateway.launch.py --show-args
```

原 `ros2 launch/run nano_bridge ...` 改为 `ros2 launch/run f1_nano_bridge ...`；外部包的 `package.xml` 依赖也应改用 `f1_nano_bridge`。外部终端初始化文件、脚本或服务中若保存了旧工作空间路径，需要对应更新并重新加载环境。

移动目录也会影响实际地图 YAML 中的绝对 `image` 路径。使用前应将其改为同目录 PGM 文件名或新路径；归档地图备份保留原始内容。

两个旧清洗包的迁移、话题和命令变更见 [清洗包合并说明](src/f1_pool_cleaning/MIGRATION.md)。
