# f1_pool_cleaning：泳池沿边清洗

文档核对：2026-11-09，配置加载修复基线 `9db6798`。本页说明矩形四边任务及共享清洗实现；开放多段直墙任务由 [`f1_l_cleaning`](../f1_l_cleaning/README.md) 配置和适配，使用 `/l_cleaning/*` 接口。

2026-11-08：共享换边增加左转分支：前行出墙端 → 左移至下一边入口 → 左转 → 重新关联墙；原右转动作不变。多段墙链提取见 `open_route.py`，本次未增加相邻转角或船体空间检查。

2026-10-30：定位实现、Lua/RViz 配置及定位单元测试迁至 [`f1_localization`](../f1_localization/README.md)。本包保留定位状态消费与停机门控，原定位 launch/脚本仅做兼容转发；矩形与大 L 的 bringup 直接启动新包。

2026-10-31：启动改为 `start_policy=left_ray`：从已定位船体中心沿 +y 发射射线，选第一条命中的有限墙段；命中公共角点时选择与船头 +x 的规定清洗方向夹角较小的墙。锁定墙和船中心最近投影点后，关刷使用旋转、前进/后退和左右平移进入旧版合理起始范围，再交给原沿墙控制。没有路径搜索、栅格足迹或雷达碰撞门控；此阶段不绕障。保留定位、位姿新鲜度、控制租约和停止/取消。矩形顺时针清洗后仅补齐起始边前段；L 形只清洗本次剩余路线，到终点停，不返回、不补洗。

阶段 1 的软件流程已经具备：定位启动默认不创建新轨迹；操作者在 RViz2 的 `map` 坐标系使用 `2D Pose Estimate` 后，管理器通过 Cartographer `/start_trajectory` 创建带初始位姿的纯定位轨迹。启动入口同时运行栅格地图节点发布 `/map`，管理器检查轨迹状态、`map → base_link` TF 新鲜度、地图边界、雷达有效率和扫描/地图重合，并在稳定窗口后发布 `/pool_cleaning/localization_status` 的 `LOCALIZATION_READY`。阶段 2 的边选择、靠墙和入口调整，阶段 3 的自动换边，以及阶段 4 的四边编排和按边覆盖记录均已接入当前包。代码和本地测试已覆盖完整一周流程；实船清洁效果、地图误差和规定次数的现场重复证据仍需按开发验收文档归档。

Windows App 现已支持任务类型选择、初始位姿发布、启动条件和完成结果。推荐使用[网关组合入口](../f1_operator_gateway/README.md)；自动任务状态新增 `task_type/map_id/stamp_ns/run_id/completed_at_ns/can_start/start_blockers`。定位接口可通过控制器 `localization_status_topic` 指向共享管理器。自动任务暂停后需取消再重启，完成后可直接开始新任务。

完整状态机和 ROS 数据流见[沿墙清洗流程图](../../docs/pool_cleaning/WALL_CLEANING_FLOW.md)。

当前执行器参数：滚刷 50%，沿墙及起始点接近阶段第五电机双向推力 ±50%，滚刷角度门控 ±10°，四主电机从偏角超过 ±3° 就开始纠偏，开刷与纠偏可以同时进行。中心墙距 32–36 cm 时第五路关闭并允许滚刷工作；超出该区间时先侧推调整。右转换边时滚刷关闭，`REVERSE` 与向右侧推 50% 同时持续 0.5 s。

第五电推参数 `side_percent=50.0` 用于沿墙调距、起始点接近侧移和左转换边侧移；`transfer_side_percent=50.0` 独立控制换边时的右侧推。启停条件和侧推方向保持原逻辑，墙距在 32–36 cm 时第五路关闭。

默认沿墙清洗前向推进依靠 50% 滚刷，四主电机负责航向纠偏：偏角在 ±3° 内时主电机中位，超过 ±3° 时以 LEFT/RIGHT 25 档、BOTH 纠偏。可在任务开始前开启下述主电机提速功能。25 是 PWM 档位，不是实测船速。带地图任务在墙距超出 32–36 cm 时关闭滚刷；旧单边入口保留原先只按角度开刷的规则。船中心距墙端 0.4 m 时按原规则停车换边。起点接近阶段的前后移动及换边动作保持原逻辑。

**主电机提速（2026-11-08）**：默认关闭，矩形与开放任务各自保存当前控制器会话中的开关。仅 `IDLE/CANCELLED/STOPPED/COMPLETE` 接受设置，运行、获取租约和 `PAUSED` 均拒绝；暂停后需先取消。停止/完成后可以为下一次任务修改，控制器重启或重新加载地图时恢复关闭。

- 启用后，仅在 `FOLLOW`、滚刷开启、墙距 32–36 cm、无需纠偏且反馈有效时输出 `FORWARD + BOTH + 25`。
- 距离采用 `edge.length_m - 船中心沿墙投影 s`；严格大于 0.80 m 才允许提速，等于或小于 0.80 m 即撤销前进辅助，本次经过该墙段不再恢复。换到下一墙段重新判断；短墙可能全段不提速。
- 矩形回到起始边补洗时，同时检查距实际补洗完成位置的剩余沿程距离（包括刷盘偏移及完成容差），同样提前 0.80 m 撤销辅助。
- 超出 ±3° 时纠偏优先；纠偏结束且未进入近端区时可恢复提速。80 cm 内仍可纠偏，初始接近和换边阶段不叠加提速。
- 新位姿进入近端区时，即使没有新墙观测也撤销先前的提速命令，不延长旧命令的有效期。定位、墙观测、租约和超时保护继续生效。

Windows App 的“清洗任务”页提供开关。ROS 接口为 `std_srvs/srv/SetBool`，例如在开始前调用：

```bash
ros2 service call /pool_cleaning/set_forward_boost std_srvs/srv/SetBool '{data: true}'
```

开放任务使用 `/l_cleaning/set_forward_boost`。状态提供 `forward_boost_enabled/active/reason/editable/cutoff_m/cutoff_reached`（每项均带 `forward_boost_` 前缀）；`active` 表示当前有效控制命令中的前进辅助，不是实测推力反馈。预览模式仍不驱动硬件。App 等待服务成功及后续状态确认后才完成设置，不自动重发；断线不改变 Nano 已接受的设置。

大偏角丢墙修复：本任务的墙线识别范围从约 ±34.4° 扩大到 ±70°，相邻有效观测的角度变化容差从约 11.5° 调整为 30°。这是感知识别范围，滚刷门控仍为 ±10°。运行中收到新鲜但未拟合到墙的扫描时进入 `WAIT_WALL`，全部输出关闭，保持任务与租约，最多等待 1.5 秒；墙线恢复后自动继续纠偏或刷洗。超过等待时间进入 `PAUSED`。带地图任务初次 start 要求定位和位姿就绪，进入起始范围后重新获取有效墙线；无地图的旧沿墙入口仍需有效墙线。

反向纠偏复现修复：盲船头下转向会缩短可见左墙段，本任务拟合所需最短墙段从 0.50 m 调整为 0.30 m，其他点数、残差与内点比例条件保留。墙线关联有效期从 1 秒延长至 2 秒，长于本任务 1.5 秒的丢墙恢复窗口，避免等待期间丢弃墙身份、把后方短墙当成左墙并发出相反转向。详见 [代码复现分析](../../docs/single_side_trial/HEADING_FEEDBACK_20261023.md)。

| 条件 | 输出 |
|---|---|
| 左墙夹角在 **[-10°, +10°]**，包括边界，且中心墙距 **32–36 cm** | 滚刷 **50%** 开启推进 |
| 夹角在 **[-3°, +3°]** | 默认主电机 STOP；启用提速且满足上述条件时 FORWARD、25 档、BOTH |
| 夹角 **>+3° / <-3°** | 四主电机 LEFT / RIGHT 25%、BOTH，直到回到 ±3° |
| 夹角 **>+10°** | 关滚刷，四个主电机 **LEFT 25%、BOTH** 持续纠偏 |
| 夹角 **<-10°** | 关滚刷，四个主电机 **RIGHT 25%、BOTH** 持续纠偏 |
| 夹角回到 ±10° 内 | 当帧恢复滚刷，继续纠偏至 ±3°；没有脉冲间隔或中位等待 |
| 中心到左墙 **>36 cm** | 第五电机向左推，默认 **+50%** |
| 中心到左墙 **<32 cm** | 第五电机向右推，默认 **-50%** |
| 中心墙距 **32–36 cm**，包括边界 | 第五电机关闭；不保留上一次推力 |
| 船中心到前墙 **≤40 cm** | 滚刷、主电机、第五电机全部关闭，进入换边停止指令周期（不测量物理停稳） |

`ψ = WallObservation.tangent_rad` 是左墙切线相对船头的夹角；正误差发 LEFT，负误差发 RIGHT。每个新墙观测都重新计算航向与侧推，纠偏时第五电机仍工作。超出 ±10° 也可以启动任务，先纠偏再开滚刷。

船头转向左墙时，正确左墙观测应为负角度，控制器应发 RIGHT（`direction=4`）。`CORRECT_HEADING` 现在可以同时开启滚刷：角度在 3° 到 10° 之间时两者同时工作。左右指令与实船转向的对应关系仍需用现场 CTRL 和实际动作确认，软件方向表本身不能证明电机接线与安装正确。

换边阶段保留转移数据新鲜度、通信超时、执行器命令期限、遥控接管和取消/暂停关闭输出；转角使用连续地图航向反馈，不用固定时间代替 90°完成判断。地图边身份与左墙目标关联由 `EdgeTarget` 恢复，防止换边后误认相邻墙。

默认无固定运行时长上限。定位管理器检查通过后发布 `LOCALIZATION_READY`，控制器在 start 时选墙并进入简单靠近，直边状态序列为 `APPROACH_WALL → APPROACH_EDGE → FOLLOW ↔ CORRECT_HEADING/WAIT_WALL → EDGE_STOP`，矩形右转换边状态序列为 `EDGE_STOP → MOVE_INWARD → TURN_TO_NEXT_EDGE → APPROACH_NEXT_EDGE`；端点和后退右推之间各保留一个停止指令周期，不等待物理停稳。转向目标航向误差放宽为 ±20°，转向和整段换边的时间超时默认关闭。临时丢墙为 `WAIT_WALL`。`wall_recovery_sec` 默认 1.5 秒；持续丢墙、反馈过期、定位退化或通信异常进入 `PAUSED`。手动暂停、取消和过期数据不会因反馈恢复而自动续跑；暂停后必须 `cancel` 再 `start`。

启动不要求预先靠墙，但默认左侧射线必须命中当前任务墙段。L 形未命中时拒绝启动，提示调整朝向；不延长墙段、不回退最近墙。角点默认容差 1 cm，转角相同时按边 ID 排序。`specified` 配合 `default_edge_id` 或空闲时的 `set_edge` 可显式指定墙。旧配置名 `nearest_reachable` 仅作为兼容别名转为 `left_ray`，会输出弃用提示。

参考点使用启动时船体中心到所选有限墙段的最近点，并按入口余量和前墙停车区间修正；不是射线交点。船头对齐后以前后移动修正沿墙位置，再侧移进入范围。默认沿墙容差 10 cm、航向容差 20°、距墙段最多 85 cm，墙在左侧且船在水侧，连续满足 0.2 s 后交接。已在范围内直接交接，不要求退到 60 cm。获取硬件租约期间目标也保持锁定。等待指定墙新鲜观测最多沿用 `wall_recovery_sec`（1.5 s），确认后调用原 `Mission._update_edge()` 调整墙距和纠偏。

## 前墙距离

现有雷达安装有船头盲区，因此仍用已有冻结地图的 `map → base_link` 位姿计算前墙距离；这版没有改雷达外参。模型矩形（原点、朝向、长宽和四边边界）由 `map:=` 指定的地图推导，例如 `20261021_145054` 地图得到约 2.883 m × 1.520 m，长轴相对 map x 约 -11.31°。`config/pool_cleaning.yaml` 不再保存这些数值。

前墙距离统一用于四条边的端点判断；当前边使用 `EdgeSegment` 的地图投影。令池坐标为 `x`、船头与长轴夹角为 `yaw`：

```text
朝长轴正向：船中心到前墙距离 = length - x
朝长轴反向：船中心到前墙距离 = x
```

船体位姿随矩形地图一致旋转 180° 时，计算出的前墙中心距离相同。有效前方雷达观测给出的是船体前缘净空，控制器会加回船长半尺寸后再与 40 cm 中心距离阈值比较；`front_observed=false` 不阻止启动，不会把未知距离当成零。

40 cm 指 **base_link 中心到前方墙**。刷墙目标是 **base_link 中心到左墙 32–36 cm**；左侧刷尖约 33 cm，33 cm 作为名义目标。到达边端点后同时发出 `REVERSE` 和向右侧推 0.5 s，再使用地图航向反馈完成右转。地图距离仍依赖真实定位精度，不能发现雷达盲区新增障碍。

## 包内职责与接口

当前本包执行矩形四条地图边，提供边选择、靠墙/入口调整、沿边控制、自动换边、按边覆盖区间记录和一周结束状态报告。凸多边形与凹多边形仍按[开发与验收文档](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)中的后续阶段执行。

- `perception.py` / `perception_node.py`：墙线拟合、关联和空间观测；`perception_config.py` 只含感知与几何参数。
- `geometry.py`：矩形/有序边界、顺时针左墙边段、投影和边选择。
- `mission.py` / `config.py`：边段状态机、靠墙/入口调整、端点停止、左右换边动作和共享控制参数。
- `map_profile.py` / `open_route.py`：地图几何推导、开放直墙链拟合、候选墙链选择及命令行检查。
- `autonomous.py` / `coverage.py`：射线选墙、简单动作靠近、矩形起始边补齐和覆盖记录；沿墙和相邻换边复用 `mission.py`。
- `controller_node.py` / `lease.py`：任务服务、状态、预览与 bridge 租约输出。
- `ros_support.py` / `runner.py`：公共消息转换、时间戳和节点退出处理。
- [`f1_localization`](../f1_localization/README.md)：独立负责 RViz 初始位姿、Cartographer 轨迹生命周期、冻结地图定位及定位就绪门控。
- `pool_localization.launch.py`：兼容入口，转发到新定位包；Lua/RViz 配置位于新包。

感知和控制仍为两个独立节点。当前接口统一为 `/pool_cleaning/*`：

| 接口 | 类型与用途 |
|---|---|
| `wall_observation` | `f1_interfaces/WallObservation`，感知结果 |
| `wall_marker` | `visualization_msgs/Marker`，墙线显示 |
| `reset_observation` | `std_srvs/Trigger`，重置关联；活动任务会暂停 |
| `map_pose` | `geometry_msgs/PoseWithCovarianceStamped`，仅在 `use_map_tf=false` 时使用 |
| `localization_status` | `std_msgs/String`，JSON 定位状态和诊断；就绪状态为 `LOCALIZATION_READY` |
| `localization_ready` | `std_msgs/Bool`，持久化的定位门控结果 |
| `set_initial_pose` | `f1_interfaces/SetInitialPose`，服务方式提交初始位姿 |
| `relocalize` | `std_srvs/Trigger`，结束当前定位轨迹并要求重新标定 |
| `command_preview` | `f1_interfaces/ActuatorCommand`，无令牌预览 |
| `status` | `std_msgs/String`，JSON 状态 |
| `start`、`pause`、`stop`、`cancel` | `std_srvs/Trigger`，任务控制；停止与取消均不支持断点续跑 |
| `set_edge` | `f1_interfaces/SetEdge`，显式指定当前地图边 |
| `edge_target` | `f1_interfaces/EdgeTarget`，地图位姿生成的目标墙上下文 |

`f1_interfaces`、`f1_nano_bridge`、`f1_mapping_bringup` 继续独立。
旧包升级须先按 [迁移说明](MIGRATION.md) 清理旧源码/安装产物并更新自定义参数文件中的节点名。
实测尺寸与历史雷达视场记录见 [MEASUREMENTS.md](MEASUREMENTS.md)。

## 配置加载与启动预检

标准地图自动任务中，`/pool_controller` 与 `/l_controller` 均应使用 `use_map_tf=true`、`require_localization_ready=true`，按 `map_frame=map`、`base_frame=base_link` 读取 TF，状态中的 `pose_source` 应为 `tf`。两包默认 YAML 都配置了这两个开关；矩形 `pool_cleaning.launch.py` 还以默认值为 `true` 的同名 launch 参数显式传入，开放入口则依赖其 YAML。

| 启动入口 | 控制器配置参数 |
|---|---|
| 独立 `pool_bringup.launch.py` / `pool_cleaning.launch.py` | `config:=/绝对路径/配置.yaml`，节点键为 `pool_controller` |
| 独立 `l_bringup.launch.py` / `l_cleaning.launch.py` | `config:=/绝对路径/配置.yaml`，节点键为 `l_controller` |
| 网关组合 `operator_bringup.launch.py` / 受管 `operator_mode.launch.py` | `rectangle_config:=...`、`l_config:=...`，分别传给对应控制器 |

网关入口的 `rectangle_config`、`l_config` 未指定或为空时，`operator_bringup` 会解析为对应包安装目录下的默认 `config/pool_cleaning.yaml`、`config/l_cleaning.yaml`；显式路径不存在或指向目录时直接报错。该修复防止外层空参数令控制器跳过整份 YAML；独立入口的参数仍是 `config`，不要混用。`use_map_tf`、`require_localization_ready`、地图及控制阈值等为只读参数，不能用 `ros2 param set` 修改运行中的配置；修改后需重新加载地图或重启对应控制器。

两个任务共用 `PoolController._start_blockers()`，但分别维护 TF 缓存、接收时间和任务状态。预检检查当前任务阶段、时钟、定位状态、位姿接收及源时间有效性；开启实际输出时还检查 bridge 与租约服务。默认 `pose_max_age_sec=0.25`，位姿源时间年龄必须在 `[0, 250 ms)`，距离最近一次接收时间戳前进的新位姿也必须小于 250 ms。重复读取旧 TF 不会刷新接收计时。因此，共享定位显示就绪或矩形 `can_start=true`，都不能代替对 `/l_cleaning/status` 的检查。

带地图自动任务的启动预检不要求有效左墙观测；到达起始范围后才获取墙线。`can_start=true` 表示当前预检通过，实际 `start` 仍会再次预检，并选择起始墙、校验有效起始区间，实际输出还需成功获取租约。矩形与开放路线的候选墙不同，所以预检通过不保证实际启动请求一定成功。

若标准地图任务显示 `pose_source=map_pose`，应先核对配置是否加载及对应 `map_pose` 话题是否有发布者；这与已经选择 TF、但位姿数据过期是不同问题。排查命令、此次配置漏加载故障原因与验证记录见[定位正常但开放任务不能启动](../../DEPLOY_NANO.md#定位正常但开放任务不能启动)。

## 构建与离线验证

```bash
cd /home/bboss/new-nano/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_pool_cleaning
source install/local_setup.bash
ros2 run f1_pool_cleaning pool_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --packages-select f1_localization f1_pool_cleaning --return-code-on-test-failure
colcon test-result --verbose
```

演示和测试使用合成反馈、localhost DDS 和模拟 bridge，不连接串口或实机。

阶段 4 本地验证重点：四条边的边界几何、目标墙关联、40 cm 端点停车、0.5 s 后退右推、±20° 粗略右转反馈、三次换边、`completed_edges` 和按边覆盖区间。实船仍必须记录无刷动作、四边轨迹、刷头接触和未覆盖区间；本地回放不能替代清洁效果证据。具体分项与复测说明见[开发与验收文档](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)。

## 启动

Nano 源码同步和完整用法见 [下水测试步骤](DEPLOY_TEST_NANO.md)。现在支持统一启动：

`map:=` 是本包唯一的地图输入：池尺寸、朝向、四边边界和定位门限都由它推导，
换池子只换这一个路径。地图不是单一封闭池、或推导失败时启动直接报错，
不会退回任何内置矩形。离线检查某张地图：

```bash
ros2 run f1_pool_cleaning map_profile ~/surface_ws/maps/lidar_20261021_145054.pbstream
```

默认自动选边无需增加地图以外的启动参数。保留指定起始边的用法：

```bash
ros2 launch f1_pool_cleaning pool_bringup.launch.py \
  map:="$PWD/maps/lidar_20261021_145054.pbstream" \
  start_policy:=specified default_edge_id:=edge_1
# 也可在任务空闲时为下一次 start 指定边；cancel 会清除此选择。
ros2 service call /pool_cleaning/set_edge f1_interfaces/srv/SetEdge '{edge_id: edge_2}'
```

启动状态为 `APPROACH_START → ALIGN_START / MOVE_START_ALONG / MOVE_START_SIDE → ACQUIRE_WALL → APPROACH_WALL → FOLLOW`。动作根据位姿闭环，在动作类型切换时发停止指令周期；靠近时滚刷关闭。获取租约后才允许运动。

矩形从实际首次有效开刷位置清到起始边末端，再按相邻边顺时针清洗其余三边；若起始边前段尚未清洗，再经原转弯动作返回起始边，清到首次开刷位置加默认 10 cm 重叠后结束。不搜索其他缺口的补洗路线。开放墙链仅沿选中位置到链尾的后续路线执行，未经过的起始前段不属于本次要求区间。

`COMPLETE` 表示本次路线执行结束，不保证覆盖记录无缺口。状态包含 `acquisition_mode`、`start_edge_id`、`start_region`、`first_brush_contact`、`returning_to_start`、`remaining_edges`、`executed_edge_ids`、`completion_scope`、`coverage_complete`、`required_contact_intervals` 和 `remaining_contact_intervals`。覆盖只累计上一条仍有效的开刷命令与新鲜有效墙观测之间的连续进度，默认缺口容差 5 cm。真实覆盖不足会保留并显示，不发起额外导航。

可在控制器 YAML 中设置 `approach_along_tolerance_m`、`approach_heading_tolerance_rad`、`approach_arrival_hold_sec`、`approach_corner_tolerance_m`、`approach_coverage_overlap_m`、`approach_coverage_tolerance_m`。原 `navigation_*` 参数不再用于任务执行。

```bash
# Nano 终端 A：自动加载环境，启动冻结地图、定位管理器、bridge、感知和清洗控制器。
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano
# Nano 终端 B：查看定位/墙线状态，然后手动开始任务。
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano status
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano start
```

统一入口保持任务 `IDLE`，不会在没有初始位姿时创建新的定位轨迹，也不会自动开始清洗。启动时可加 `start_rviz:=true` 自动打开保存的 RViz2 配置：

```bash
bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano bringup start_rviz:=true
```

在 RViz2 中确认 Fixed Frame 为 `map`，显示 Map、LaserScan、TF，然后点击 `2D Pose Estimate` 在地图中标定船体位置和船头方向。确认 `/pool_cleaning/localization_status` 的 `state` 变为 `LOCALIZATION_READY` 后，再运行 `status` 和 `start`。定位和 bridge 已运行时，在终端 A 命令末尾增加 `bringup start_localization:=false start_bridge:=false`，只启动缺少的部分；已有清洗控制器应先退出。快捷脚本默认开启硬件输出许可，默认 ROS 域为 33；已设置的域和 RMW 环境会保留。也可加载环境后运行 `ros2 launch f1_pool_cleaning pool_bringup.launch.py map:=/绝对路径/现场地图.pbstream output_enabled:=true`。原分开启动入口继续保留，默认预览不申请租约：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream
ros2 topic echo /pool_cleaning/status
ros2 topic echo /pool_cleaning/command_preview
```

没有 `LOCALIZATION_READY` 时 `/pool_cleaning/start` 会拒绝申请任务租约；定位跳变、TF/雷达过期或轨迹结束会关闭输出并使任务进入 `PAUSED`。定位恢复后不会自动恢复旧任务，必须检查原因、调用 `cancel`，再次在 RViz2 标定后再 `start`。

实机使用同一个入口，显式开启输出，再启动任务：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py \
  map:=/home/nvidia/surface_ws/maps/lidar_20261021_145054.pbstream output_enabled:=true
# 另一终端：
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
# 暂停、停止或取消：
ros2 service call /pool_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /pool_cleaning/stop std_srvs/srv/Trigger '{}'
ros2 service call /pool_cleaning/cancel std_srvs/srv/Trigger '{}'
```

实机需要已有 bridge 的串口/运动许可、主电机方向确认和 `allow_fifth_thruster=true`。`mode=BOTH` 由固件混控四个主电机；沿墙默认滚刷推进，也可按上述规则开启主电机提速，LEFT/RIGHT 纠偏优先。换边 `MOVE_INWARD` 阶段会发送主电机 REVERSE 并同时向右侧推 0.5 s。保持第五路既有“正号向左”的映射。

Qt App 经网关分别连接矩形与 L 的任务接口。顶部“停止手动输出”及断线只影响网关手动会话；自动任务使用任务面板的停止、取消或暂停，详见[网关控制说明](../f1_operator_gateway/README.md)。

状态前部新增 `left_wall_distance_cm`：船中心到左墙的垂直距离，单位厘米，保留 1 位小数，例如 `34.0`。原 `wall_distance_m` 米制字段继续提供相同距离；墙线无效时这两个字段均为 `null`，不把占位值 0 当作实际距离。

状态还包含 `wall_angle_deg`、`wall_valid`、`wall_reason`、`heading_band_deg`、`map_front_clearance_m`、刷盘/侧推请求及租约状态；`localization_ready`、`localization_state`、`localization_reason` 来自定位管理器。`wall_reason` 保留感知的拒绝原因，持续丢墙暂停时 `reason` 也会带出该原因。查看单条完整状态：`ros2 topic echo /pool_cleaning/status --once --full-length`。预览 token 始终为空。`STOPPED` 只表示输出已关闭，不代表船体已经停稳。

阶段 4 状态还包含 `current_edge_id`、`transfer_next_edge`、`completed_edges`、`edge_progress_m`、`edge_entry_m`、`edge_stop_m`、`cleaned_intervals` 和按边覆盖记录。本次路线执行结束后 `phase=COMPLETE`；`executed_edge_ids` 表示执行过的边，`completed_edges` 仅包含覆盖记录满足要求区间的边，`coverage_complete` 独立报告覆盖结果。覆盖区间的统计基准是 `left_brush_contact_centerline`，表示左侧滚刷接触中心线的连续轨迹；它用于核对一周任务的边段覆盖，不能单独等同于池底面积覆盖率。

请使用本包更新后的默认配置。历史诊断目录中的 `rectangle_current_side.yaml` 仍保存当时的 ±3°、8 秒限制，不是本版测试参数。

## App 轨迹、停止与墙距扩展（2026-10-31）

`/pool_cleaning/stop`（Trigger）进入 `STOPPED`，关闭全部任务输出并释放租约，保留覆盖记录与当前边快照；`cancel` 额外清除当前边关联。都不会自动续跑，自动任务再次 start 会重置进度并生成新 run_id。

ROS 状态 JSON 的 `expected_track` 提供 map 坐标系的 `cleaning_segments` 和 `start_region`（参考点及到达范围四边形）；`navigation_path` 保留为空数组。目标线起点由接触起点减去 `brush_x_m` 换算，终点直接取 `墙长 - front_stop_m`；两端均向水侧偏移 `wall_target_m`。默认目标墙距 33 cm、开刷范围 32–36 cm、船中心距墙端 40 cm 停车。各边独立，开放墙链只包含本次后续任务段。`expected_track` 保留 `basis=boat_center_stop` 及距离参数用于 ROS 诊断；2026-11-09 起网关不再向 App 转发，画布仅显示实际轨迹。启动前后均使用相同停车终点；覆盖统计仍使用刷头接触区间，不以目标线代替覆盖判定。

`center_wall_distance_m` 使用左墙观测的垂直距离，并同时检查源时间与接收时间是否小于 wall_max_age_sec；无效/过期时为 null。兼容字段 wall_distance_m、left_wall_distance_cm 使用同样有效性条件，wall_valid 不再对过期观测保持 true。历史轨迹由网关采集，详见[App 操作说明](../../operator_app/README.md)。
