# f1_l_cleaning：大 L 两段墙清洗

文档核对：2026-10-30。项目构建、部署和任务入口总览见[工作空间 README](../../README.md)。

目标是 `maps/lidar_20261026_170703.pgm` 左下方的大 L。开放路线为
`A → B → C`，只有 `edge_0`、`edge_1`，墙始终在船左侧。
本包负责路线、配置和启动编排；沿墙、右转、租约及硬件输出复用
`f1_pool_cleaning`，定位由独立的 [`f1_localization`](../f1_localization/README.md) 提供。默认只发布预览，不产生硬件输出，也不自动 start。

## 路线与动作

| 点 | map x / m | map y / m | 含义 |
|---|---:|---:|---|
| A | -1.801431 | 0.681948 | 下方直墙右端 |
| B | -10.462431 | -0.786552 | 左下角 |
| C | -11.917931 | 10.962448 | 左侧直墙上端 |

AB 约 8.78 m，BC 约 11.84 m，右转约 92.56°。坐标来自栅格墙线拟合，
分辨率 0.05 m，原点 `[-29.576431, -11.162552, 0]`，不是实测控制点。
修改路线使用 `config/l_cleaning.yaml` 的 `route_xy`；不能填 `boundary_xy`，
不能追加 C→A。参数必须是三个有限点且只包含一次 45°–135°右转。

动作顺序：第一段距终点约 0.30 m 关刷停车 → 同时后退、右推 0.50 s
→ 停车 → 根据地图航向右转 → 进入第二段方向 ±20°后停车
→ 重新建立第二面墙关联、靠墙和纠偏 → 清洗第二段 → 距 C 约 0.30 m
关刷停车。以上是相邻两边的沿墙动作；自动任务还会检查起始段及转弯后遗漏区间，
必要时关刷导航回补，所有要求区间补齐后才进入 `COMPLETE`。

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

需要在现场覆盖推导结果时，仍可在 `config/l_cleaning.yaml` 写 `route_xy`；
参数优先于地图推导。

地图文件由现场提供，不打包进源码。
`maps/` 被 Git 忽略；当前根部署命令会同步它。每份地图的同名 `.pbstream`、`.yaml`、`.pgm` 直接放在 `maps/` 下，YAML 的 `image` 使用同目录 PGM 文件名。若需要单独传输大 L 地图，在 PC 确认三份文件存在后执行：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/maps'
rsync -av /home/bboss/new-nano/surface_ws/maps/lidar_20261026_170703.{pbstream,yaml,pgm} \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/
```

雷达驱动需已运行，默认扫描话题 `/pavo2_scan`。
在 RViz 的 `2D Pose Estimate` 标记初始位姿，船可位于任一地图墙边附近；
默认 `start_policy=nearest_reachable` 优先选择当前船体左侧可达的任务墙边，
左侧均不可达或没有左侧墙时再尝试其他墙；每组按墙段距离、航向调整量排序。
左侧根据墙段最近点在船体坐标系中的位置判定，随当前船头方向变化。
两种任务共用[自动选边规则](../f1_pool_cleaning/README.md)，
关刷导航到接近点后重新确认左墙。也可用 `start_policy:=specified` 配合
`default_edge_id`，或在空闲时通过 `set_edge` 指定任务起始段，指定边优先于自动选边且不可达时不回退。定位状态变为
`LOCALIZATION_READY` 后手动启动：

```bash
ros2 topic echo /l_cleaning/localization_status
ros2 service call /l_cleaning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /l_cleaning/status
ros2 topic echo /l_cleaning/command_preview
ros2 service call /l_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /l_cleaning/cancel std_srvs/srv/Trigger '{}'
```

暂停后重跑需先 cancel，再 start；带地图的自动任务完成后可直接 start 新任务。自动模式可从 `edge_0` 或 `edge_1`
开始；清洗完当前段后沿开放路线经过另一段，最后关刷导航回补起始段未清洗部分。
L 路线的末端到起始段之间不执行清洗，也不构造虚假的闭合墙面。相邻边转弯继续
使用现有后退/右推和闭环转向逻辑。

现有节点复用：

- 已有适配本地图的定位与 `/l_cleaning/localization_status`：
  `start_localization:=false`。
- 已有 bridge：`start_bridge:=false`；复用共享 `/f1/*` 租约与命令接口。
- 只启动定位：`start_cleaning:=false start_bridge:=false`。
- 已有雷达静态 TF：`publish_lidar_tf:=false`。
- 可通过 `config`、`perception_config`、`localization_config` 指定现场参数文件。

不要同时启动两套 Cartographer/定位管理器或两个 bridge。定位范围现在由 `map:=`
的空闲水域推导；这是定位粗范围检查。扫描与占用栅格重合、新鲜度、跳变检查仍然保留。
普通 `l_cleaning.launch.py` 只启动感知和控制器，生产配置仍要求定位就绪。

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
实际余转、角部净空、定位稳定性及刷盘接触效果；进入 `COMPLETE` 表示软件记录的
要求接触区间已覆盖，不证明实际去污效果。原控制器的盲区限制仍然存在。
自动选边、导航参数和覆盖状态字段见[共享实现说明](../f1_pool_cleaning/README.md)。

## App 轨迹、停止与墙距扩展（2026-10-31）

本包复用矩形控制器的 `expected_track`、`center_wall_distance_m` 与墙观测新鲜度字段，但目标轨迹只有两条独立清洗段，保持开放的 L 路线，不增加回到起点的闭合边。历史轨迹按 L 任务的 map_id/run_id 单独记录，不与矩形任务混合。

新增 Trigger `/l_cleaning/stop`，进入 STOPPED、关闭输出、释放租约并保留进度；`/l_cleaning/cancel` 另清除当前边关联。再次开始是新任务，不支持断点续跑。界面、历史保留范围与左墙距离定义见[App 说明](../../operator_app/README.md)。
