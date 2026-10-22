# 工作空间与桥接包改名

| 项目 | 原名称 | 新名称 |
| --- | --- | --- |
| PC 工作空间 | `/home/bboss/new-nano/f1_mapping_ws` | `/home/bboss/new-nano/surface_ws` |
| Nano 工作空间 | `/home/nvidia/f1_mapping_ws` | `/home/nvidia/surface_ws` |
| ROS 桥接包 | `nano_bridge` | `f1_nano_bridge` |

Python 模块仍为 `nano_ros_bridge`，原生桥节点仍为 `/nano_bridge`，配置文件顶层键仍为 `nano_bridge`。话题、服务、参数与执行器协议保持原样。

PC 源码目录已改名。Nano 需要单独迁移和同步，下面命令供在对应设备上执行。

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
if [ -f /home/nvidia/ros2_ws/install/setup.bash ]; then
  source /home/nvidia/ros2_ws/install/setup.bash
fi
cd /home/nvidia/surface_ws
colcon list --names-only
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

应列出五个包：`f1_interfaces`、`f1_nano_bridge`、`f1_mapping_bringup`、`f1_wall_cleaning` 和 `f1_rectangle_cleaning`。构建成功后，在运行终端加载新的 `install/local_setup.bash`。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/surface_ws/install/local_setup.bash
ros2 launch f1_nano_bridge bridge.launch.py --show-args
ros2 launch f1_nano_bridge imu.launch.py --show-args
```

原 `ros2 launch/run nano_bridge ...` 改为 `ros2 launch/run f1_nano_bridge ...`；外部包的 `package.xml` 依赖也应改用 `f1_nano_bridge`。外部终端初始化文件、脚本或服务中若保存了旧工作空间路径，需要对应更新并重新加载环境。

移动目录也会影响实际地图 YAML 中的绝对 `image` 路径。使用前应将其改为同目录 PGM 文件名或新路径；归档地图备份保留原始内容。
