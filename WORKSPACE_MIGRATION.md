# 工作空间与桥接包改名

**当前 Nano 已删除全部旧工作空间，不适用下面的目录迁移命令。请直接使用[Nano 从零部署指南](DEPLOY_NANO.md)。** 本文仅保留给其他仍有旧目录的环境作为历史迁移参考。新部署统一只使用 `surface_ws`。

| 项目 | 原名称 | 新名称 |
| --- | --- | --- |
| PC 工作空间 | `/home/bboss/new-nano/f1_mapping_ws` | `/home/bboss/new-nano/surface_ws` |
| Nano 工作空间 | `/home/nvidia/f1_mapping_ws` | `/home/nvidia/surface_ws` |
| ROS 桥接包 | `nano_bridge` | `f1_nano_bridge` |

Python 模块仍为 `nano_ros_bridge`，原生桥节点仍为 `/nano_bridge`，配置文件顶层键仍为 `nano_bridge`。话题、服务、参数与执行器协议保持原样。

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

当前源码应列出七个包：`f1_interfaces`、`f1_nano_bridge`、`f1_mapping_bringup`、`f1_localization`、`f1_pool_cleaning`、`f1_l_cleaning` 和 `f1_operator_gateway`，额外雷达驱动另计。构建成功后，在运行终端加载新的 `install/local_setup.bash`。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
ros2 launch f1_nano_bridge bridge.launch.py --show-args
ros2 launch f1_operator_gateway operator_gateway.launch.py --show-args
```

原 `ros2 launch/run nano_bridge ...` 改为 `ros2 launch/run f1_nano_bridge ...`；外部包的 `package.xml` 依赖也应改用 `f1_nano_bridge`。外部终端初始化文件、脚本或服务中若保存了旧工作空间路径，需要对应更新并重新加载环境。

移动目录也会影响实际地图 YAML 中的绝对 `image` 路径。使用前应将其改为同目录 PGM 文件名或新路径；归档地图备份保留原始内容。

两个旧清洗包的迁移、话题和命令变更见 [清洗包合并说明](src/f1_pool_cleaning/MIGRATION.md)。
