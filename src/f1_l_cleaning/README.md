# f1_l_cleaning：大 L 两段墙清洗

文档核对：2026-10-31。项目构建、部署和任务入口总览见[工作空间 README](../../README.md)。

示例地图是 `maps/lidar_20261026_170703.pgm`，其左下方大 L 用于早期路线验证。当前路线由传入的地图推导，开放路线为
`A → B → C`，只有 `edge_0`、`edge_1`，墙始终在船左侧。
本包负责路线、配置和启动编排；沿墙、右转、租约及硬件输出复用
`f1_pool_cleaning`，定位由独立的 [`f1_localization`](../f1_localization/README.md) 提供。默认只发布预览，不产生硬件输出，也不自动 start。

## 路线与动作

| 点 | map x / m | map y / m | 含义 |
|---|---:|---:|---|
| A | -1.801431 | 0.681948 | 下方直墙右端 |
| B | -10.462431 | -0.786552 | 左下角 |
| C | -11.917931 | 10.962448 | 左侧直墙上端 |

上表为该示例地图的历史拟合点：AB 约 8.78 m，BC 约 11.84 m，右转约 92.56°。
分辨率 0.05 m，原点 `[-29.576431, -11.162552, 0]`，不是实测控制点或当前配置中的固定值。
当前默认配置不填写路线坐标。显式调整使用 `config/l_cleaning.yaml` 的 `route_xy`；不能填 `boundary_xy`，
不能追加 C→A。参数必须是三个有限点且只包含一次 45°–135°右转。

动作顺序：第一段距终点约 0.30 m 关刷停车 → 同时后退、右推 0.50 s
→ 停车 → 根据地图航向右转 → 进入第二段方向 ±20°后停车
→ 重新建立第二面墙关联、靠墙和纠偏 → 清洗第二段 → 距 C 约 0.30 m
关刷停车并进入 `COMPLETE`。从 AB 中间起步只清后续 AB 和 BC；从 BC 起步只清后续 BC。
不返回 A，不补起始前段或中途缺口。`COMPLETE` 表示本次剩余路线执行结束，覆盖记录单独显示。

“重新找墙”使用现有感知和近墙控制：新鲜但无有效墙线时保持零输出，
最多等待 1.5 s；持续丢墙或数据过期进入 `PAUSED`。不会在无墙反馈时盲目推进。
转角的停车是停止命令周期，不代表已通过速度传感器确认船体静止。
默认转移超时仍沿用原包的 0（关闭），完成转向依赖航向反馈。

## 构建和启动

在工作区根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_l_cleaning
source install/local_setup.bash
ros2 launch f1_l_cleaning l_bringup.launch.py \
  map:="$PWD/maps/lidar_20261026_170703.pbstream" \
  start_rviz:=true
```

`map:=` 是本任务唯一的地图输入：`route_xy`、定位门限和控制器使用的地图范围
都由它推导，换池子只换这一个路径。推导逻辑见
[`f1_pool_cleaning/map_profile.py`](../f1_pool_cleaning/f1_pool_cleaning/map_profile.py)；
地图若没有唯一一个"墙在左、水在右"的顺时针墙角，启动会直接报错，不会猜测。
可以先离线检查：

```bash
ros2 run f1_pool_cleaning map_profile "$PWD/maps/lidar_20261026_170703.pbstream"
```

需要在现场覆盖已成功推导的路线时，仍可在 `config/l_cleaning.yaml` 写
`route_xy: [ax, ay, bx, by, cx, cy]`，显式参数优先于推导默认值。
但控制器先执行 `_map_geometry()`，再读取 `route_xy`；地图路线推导失败会先退出，
不能通过该参数强行启用被拒绝的地图。组合入口还会先按地图几何判断任务是否可用。

地图文件由现场提供，不打包进源码。
`maps/` 被 Git 忽略；当前根部署命令会同步它。每份地图的同名 `.pbstream`、`.yaml`、`.pgm` 直接放在 `maps/` 下，YAML 的 `image` 使用同目录 PGM 文件名。若需要单独传输大 L 地图，在 PC 确认三份文件存在后执行：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/maps'
rsync -av /home/bboss/new-nano/surface_ws/maps/lidar_20261026_170703.{pbstream,yaml,pgm} \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/
```

雷达驱动需已运行，默认扫描话题 `/pavo2_scan`。
在 RViz 的 `2D Pose Estimate` 标记初始位姿。默认 `start_policy=left_ray` 从船中心沿 +y 命中第一面任务墙；公共角点按清洗方向与船头夹角选择。未命中有限 AB/BC 时拒绝启动，需调整朝向；不向无限延长线或最近墙回退。
两种任务共用[简单靠近规则](../f1_pool_cleaning/README.md)：锁定墙和最近投影点，关刷旋转、前后移动和侧移进入旧版启动范围，再确认左墙。该阶段不做路径搜索或环境碰撞检查。也可用 `start_policy:=specified` 配合 `default_edge_id`，或通过 `set_edge` 指定起始段。定位状态变为
`LOCALIZATION_READY` 后手动启动：

```bash
ros2 topic echo /l_cleaning/localization_status
ros2 service call /l_cleaning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /l_cleaning/status
ros2 topic echo /l_cleaning/command_preview
ros2 service call /l_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /l_cleaning/cancel std_srvs/srv/Trigger '{}'
```

暂停后重跑需先 cancel；完成后可 start 新任务。若需要重新清洗整条 L，请先遥控回到合适位置。
从 `edge_0` 启动执行其剩余段和 `edge_1`；从 `edge_1` 启动只执行其剩余段。始终在 C 前约 30 cm 停止，不生成 C→A 转场，不补洗。相邻边仍使用原后退/右推和闭环右转。

现有节点复用：

- 已有适配本地图的定位与 `/l_cleaning/localization_status`：
  `start_localization:=false`。
- 已有 bridge：`start_bridge:=false`；复用共享 `/f1/*` 租约与命令接口。
- 只启动定位：`start_cleaning:=false start_bridge:=false`。
- 已有雷达静态 TF：`publish_lidar_tf:=false`。
- 可通过 `config`、`perception_config`、`localization_config` 指定现场参数文件。

不要同时启动两套 Cartographer/定位管理器或两个 bridge。定位范围现在由 `map:=`
的空闲水域推导；这是定位粗范围检查。扫描与占用栅格重合、新鲜度、跳变检查仍然保留。
普通 `l_cleaning.launch.py` 只启动感知和控制器，也必须传入同一 `map:=`；
默认配置不含独立运行所需的固定几何值，不能省略地图。生产配置仍要求定位就绪。

需要实际输出时，在完成现场检查后将同一启动命令加入 `output_enabled:=true`。
该参数会为新启动的 bridge 开启串口与运动权限；如果复用已有 bridge，其权限
必须在该 bridge 自身配置。预览用 `output_enabled:=false`；`brush_percent=0`
不受现有控制配置支持，不能用它表示无输出模式。

## Qt App 接入

App 已支持任务类型选择、初始位姿发布、定位门控与任务完成显示。已有大 L bringup 时，网关设置 `localization_prefix:=/l_cleaning`；不再重映射任务的 `status/start/pause/stop/cancel`。也可以使用[网关组合入口](../f1_operator_gateway/README.md)统一启动，定位共享 `/f1_localization/*`，本任务消费同一定位状态。

自动任务使用“停止任务”“取消任务”或“暂停任务”。App 顶部“停止手动输出”按钮只释放手动会话。暂停后需先取消再重新开始；带地图的自动任务完成后可直接开始新任务。

## 验证边界

```bash
colcon test --packages-select f1_localization f1_pool_cleaning f1_l_cleaning --event-handlers console_direct+
colcon test-result --verbose
```

软件测试覆盖开放路线校验、一次后退右推和右转、第二墙重关联、第二段完成停车、
默认禁用硬件输出和定位门控。现场仍需确认拟合端点、0.50 s 动作位移、
实际余转、角部净空、定位稳定性及刷盘接触效果；进入 `COMPLETE` 表示本次剩余路线执行结束，
覆盖记录由 `coverage_complete` 单独报告，不证明实际去污效果。原控制器的盲区限制仍然存在。
射线选墙、简单靠近参数和覆盖状态字段见[共享实现说明](../f1_pool_cleaning/README.md)。

## App 轨迹、停止与墙距扩展（2026-10-31）

本包复用矩形控制器的 `expected_track`、`center_wall_distance_m` 与墙观测新鲜度字段，但目标轨迹只有两条独立清洗段，保持开放的 L 路线，不增加回到起点的闭合边。历史轨迹按 L 任务的 map_id/run_id 单独记录，不与矩形任务混合。

新增 Trigger `/l_cleaning/stop`，进入 STOPPED、关闭输出、释放租约并保留进度；`/l_cleaning/cancel` 另清除当前边关联。再次开始是新任务，不支持断点续跑。界面、历史保留范围与左墙距离定义见[App 说明](../../operator_app/README.md)。
