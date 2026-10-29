# f1_l_cleaning：大 L 两段墙清洗

文档核对：2026-10-29。项目构建、部署和任务入口总览见[工作空间 README](../../README.md)。

目标是 `maps/lidar_20261026_170703.pgm` 左下方的大 L。开放路线为
`A → B → C`，只有 `edge_0`、`edge_1`，墙始终在船左侧。
本包负责路线、配置和启动编排；沿墙、右转、定位、租约及硬件输出复用
`f1_pool_cleaning`。默认只发布预览，不产生硬件输出，也不自动 start。

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
关刷停车、进入 `COMPLETE`。全程最多一次转移，没有返回起点的动作。

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
  load_state_filename:="$PWD/maps/lidar_20261026_170703.pbstream" \
  start_rviz:=true
```

地图文件由现场提供，不打包进源码。请显式指定本机 pbstream 路径；
默认路径是 `~/surface_ws/maps/lidar_20261026_170703.pbstream`。
`maps/` 被 Git 忽略，根部署命令也排除了它；即使本地已有地图，Nano 仍需单独同步。在 PC 确认源文件存在后执行：

```bash
ssh nvidia@192.168.1.188 'mkdir -p /home/nvidia/surface_ws/maps'
rsync -av /home/bboss/new-nano/surface_ws/maps/lidar_20261026_170703.pbstream \
  nvidia@192.168.1.188:/home/nvidia/surface_ws/maps/
```

雷达驱动需已运行，默认扫描话题 `/pavo2_scan`。
在 RViz 的 `2D Pose Estimate` 标记初始位姿，初始船应位于 AB 附近、
朝向 B，墙在船左侧。定位状态变为 `LOCALIZATION_READY` 后手动启动：

```bash
ros2 topic echo /l_cleaning/localization_status
ros2 service call /l_cleaning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /l_cleaning/status
ros2 topic echo /l_cleaning/command_preview
ros2 service call /l_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /l_cleaning/cancel std_srvs/srv/Trigger '{}'
```

暂停、完成后的重跑需先 cancel，并将船重新放到第一段合法起始位置后 start。
`set_edge` 只接受 `edge_0`；第二段由状态机自动进入，不能跳过第一段。

现有节点复用：

- 已有适配本地图的定位与 `/l_cleaning/localization_status`：
  `start_localization:=false`。
- 已有 bridge：`start_bridge:=false`；复用共享 `/f1/*` 租约与命令接口。
- 只启动定位：`start_cleaning:=false start_bridge:=false`。
- 已有雷达静态 TF：`publish_lidar_tf:=false`。
- 可通过 `config`、`perception_config`、`localization_config` 指定现场参数文件。

不要同时启动两套 Cartographer/定位管理器或两个 bridge。旧矩形配置的定位范围
不适合此地图；新定位 YAML 的 `[-14, 1] × [-2.5, 13.5] m` 是粗范围检查，
不是可行驶区域或障碍物地图。扫描与占用栅格重合、新鲜度、跳变检查仍然保留。
普通 `l_cleaning.launch.py` 只启动感知和控制器，生产配置仍要求定位就绪。

需要实际输出时，在完成现场检查后将同一启动命令加入 `output_enabled:=true`。
该参数会为新启动的 bridge 开启串口与运动权限；如果复用已有 bridge，其权限
必须在该 bridge 自身配置。预览用 `output_enabled:=false`；`brush_percent=0`
不受现有控制配置支持，不能用它表示无输出模式。

## Qt App 接入

App 网关默认订阅和调用 `/pool_cleaning/*`，不会自动连接本包。使用 App 操作大 L 时，按[网关说明](../f1_operator_gateway/README.md#连接大-l-任务)重映射 `status/start/pause/cancel` 四个接口；共享 `/f1/*`、`/map` 和 TF 不变。网关需要另行构建和启动，`--packages-up-to f1_l_cleaning` 不包含网关。

自动任务停止使用 `/l_cleaning/pause` 或 `/l_cleaning/cancel`。App 的红色“停止全部”按钮实际发送 `manual_stop`，仅释放网关手动会话，不会暂停本任务。

## 验证边界

```bash
colcon test --packages-select f1_pool_cleaning f1_l_cleaning --event-handlers console_direct+
colcon test-result --verbose
```

软件测试覆盖开放路线校验、一次后退右推和右转、第二墙重关联、第二段完成停车、
默认禁用硬件输出和定位门控。现场仍需确认拟合端点、0.50 s 动作位移、
实际余转、角部净空、定位稳定性及刷盘接触效果；进入 `COMPLETE` 只表示两段
任务流程结束，不证明全部清洗区间有效覆盖。原控制器的盲区和动态避障限制仍然存在。
