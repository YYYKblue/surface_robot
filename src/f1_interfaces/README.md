# F1 统一接口包

本工作空间的所有自定义 ROS 消息和服务统一放在 `f1_interfaces`，业务节点在各自功能包中实现。后续新增自定义接口继续加入本包，并在 `CMakeLists.txt` 中注册；ROS 标准接口仍使用原有标准包。

| 类型 | 接口 | 用途 |
|---|---|---|
| msg | `ActuatorCommand` | 执行器指令与租约期限 |
| msg | `ControlState` | 硬件桥控制状态 |
| msg | `WallObservation` | 墙面与周边空间观测 |
| msg | `EffortPreview` | 旧脉冲控制预览接口；保留类型兼容，当前清洗包不发布 |
| msg | `CleaningStatus` | 旧脉冲控制状态接口；保留类型兼容，当前清洗包不发布 |
| srv | `AcquireControl` | 申请控制租约 |
| srv | `ReleaseControl` | 释放控制租约 |

Python 使用 `from f1_interfaces.msg import ActuatorCommand` 或 `from f1_interfaces.srv import AcquireControl`；依赖包在 `package.xml` 中声明 `f1_interfaces`。

2026-10-22 控制链修复：`ControlState` 新增 `allow_fifth_thruster`（bridge 第五路配置）及 `last_stop_reason`（最近撤权原因，保留到下一次成功申请）。更新时重新构建本包和依赖节点，并同步重启，不能混用旧生成消息。

## 从旧接口包迁移

Nano 已清空旧工作空间时不执行本节，按[从零部署指南](../../DEPLOY_NANO.md)直接创建并编译 `surface_ws`。以下仅供其他保留旧接口包的环境参考。

原 `f1_bridge_interfaces` 和 `f1_wall_interfaces` 已合并到本包。消息字段、服务定义、话题名称和服务名称不变，但 ROS 类型名称变为 `f1_interfaces/msg/...` 和 `f1_interfaces/srv/...`。PC、Nano 及其他使用这些自定义接口的节点必须同步更新并重新构建，旧类型不能与新类型直接通信。历史 rosbag 中的旧类型也不会自动转换。

首次更新旧工作空间时，先结束本工作空间的节点和构建/测试。在没有加载本工作空间旧 `install` 的新终端中，进入工作空间根目录，执行以下命令备份遗留源码与旧生成目录（适用于本项目默认的隔离安装布局）：

```bash
migration_backup="$(mktemp -d /tmp/f1_interfaces_migration.XXXXXX)"
for package in f1_bridge_interfaces f1_wall_interfaces; do
  for tree in src build install; do
    if [ -e "$tree/$package" ]; then
      mkdir -p "$migration_backup/$tree"
      mv "$tree/$package" "$migration_backup/$tree/"
    fi
  done
done
echo "旧接口备份：$migration_backup"
```

源码同步命令不会自动删除 Nano 上的旧包，因此 PC 和 Nano 均需检查。随后加载 ROS 及必要的驱动工作空间环境，重新构建整个工作空间：

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 interface show f1_interfaces/msg/ActuatorCommand
ros2 interface show f1_interfaces/srv/AcquireControl
```

`colcon list` 应只包含一个自定义接口包 `f1_interfaces`，共四个功能包。原独立 IMU 桥也已并入 `f1_nano_bridge`，更新旧版本时同时参考[桥接包迁移说明](../f1_nano_bridge/README.md#从独立-imu-包迁移)。部署入口与测试命令见[工作空间说明](../../README.md)。
