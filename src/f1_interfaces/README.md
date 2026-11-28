# F1 统一接口包

`f1_interfaces` 提供水面控制、清洗与定位共用的消息和服务。RTK 驱动通过本包发布接收机状态和串口原文，通过标准 `sensor_msgs/msg/NavSatFix` 发布位置。[RTK 话题](../f1_rtk_driver/README.md#数据约定)

| 类型 | 接口 | 用途 |
|---|---|---|
| msg | `ActuatorCommand` | 执行器指令与租约期限 |
| msg | `ControlState` | 硬件桥控制状态 |
| msg | `WallObservation` | 墙面与周边空间观测 |
| msg | `EdgeTarget` | 固定地图位姿生成的目标边/左墙上下文 |
| msg | `SurveyStatus` | 跨湖建图与定位的轨迹编号、地理配准、位置与航向有效性以及当前位姿来源 |
| msg | `Wt102Sample` | WT102 串口接收时间、数据流编号、测量类型和 SI 值 |
| msg | `Heading` | 供历史航向录包解码 |
| msg | `RtkStatus` | 接收机串口状态、定位质量、有效性和观测年龄，保留历史航向字段 |
| msg | `RawSentence` | 带 ROS 接收时间的单行串口原文 |
| msg | `EffortPreview` | 旧脉冲控制预览接口；保留类型兼容，当前清洗包不发布 |
| msg | `CleaningStatus` | 旧脉冲控制状态接口；保留类型兼容，当前清洗包不发布 |
| srv | `AcquireControl` | 申请控制租约 |
| srv | `ReleaseControl` | 释放控制租约 |
| srv | `SetInitialPose` | 向泳池定位管理器提交 RViz2 使用的 `map` 坐标初始位姿 |
| srv | `SetLaneSpacing` | 设置下次弓字形巡游的航线间距（米） |
| srv | `SetEdge` | 在任务空闲时为下一次矩形或大 L 任务指定地图起始边 |
| srv | `SetCleaningSpeed` | 启动前设置清洗速度原始整数值 0–100，由控制器映射为前进辅助输出 0–50 |

Python 使用 `from f1_interfaces.msg import ActuatorCommand` 或 `from f1_interfaces.srv import AcquireControl`；依赖包在 `package.xml` 中声明 `f1_interfaces`。

2026-11-23 起，RTK 的三个自定义消息使用本包的类型名。旧订阅者和历史 rosbag 的处理见 [RTK 接口迁移](../f1_rtk_driver/README.md#从驱动内置消息升级)。

2026-11-16 清洗速度：`SetCleaningSpeed` 请求字段为 `int32 value`，控制器只接受整数 0–100；响应为 `bool success` 和 `string message`。映射公式为 `(value + 1) // 2`，只用于沿墙前进辅助。新增服务后需重新构建接口包、清洗控制器和网关；本次没有修改 `ActuatorCommand` 或 F103 v3 协议。Nano 上已完成该构建并核对生成接口，详见[部署验证记录](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md#2026-11-16-nano-部署验证)。

2026-11-11 F103 v3：`ActuatorCommand.speed_percent` 为整数 0–100 的主电机 PWM 输出；0 仅停止主电机，滚刷和第五电推独立。新 25% 对应 100 μs 偏移，旧 25 档为 125 μs。`ControlState` 新增 `firmware_protocol_version`（0 表示尚未观察到 CTRL）、`firmware_protocol_compatible` 和 `rc_valid/rc_flags/rc_sequence/rc_age_sec/rc_channels[14]`。RC 为原始输入诊断；CTRL PWM 仍为目标值。必须重建本接口包及所有依赖节点并一同重启，不能混用旧生成消息。详细编码和兼容策略见[桥接说明](../f1_nano_bridge/README.md#f103-v3-连续输出2026-11-11)。

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

源码同步命令不会自动删除 Nano 上的旧包，因此 PC 和 Nano 均需检查。随后加载 ROS 及必要的驱动工作空间环境，重新构建整个工作空间。以下是 PC 开发机的符号链接安装示例；Nano 使用 [部署指南](../../DEPLOY_NANO.md#5-nano编译) 中的复制安装方式，不直接使用下面的 `--symlink-install`：

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 interface show f1_interfaces/msg/ActuatorCommand
ros2 interface show f1_interfaces/srv/AcquireControl
```

迁移完成后，`colcon list --base-paths src` 应能发现 `f1_interfaces` 和独立的 `f1_rtk_driver`。当前完整包列表见[工作空间说明](../../README.md)。IMU 由 `f1_nano_bridge` 原生串口桥直接发布，更新旧版本时同时参考[工作空间迁移说明](../../WORKSPACE_MIGRATION.md)，清理旧独立包与接入入口。
