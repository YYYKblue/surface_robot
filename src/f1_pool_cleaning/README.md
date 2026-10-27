# f1_pool_cleaning：泳池沿边清洗

2026-10-26 阶段 4：矩形水池自动清洁一周的软件功能已完成。任务沿用阶段 1 的 Cartographer 固定地图定位，默认从 `edge_0`（第一条长边）启动；完成直边刷墙后自动按 `edge_0 → edge_1 → edge_2 → edge_3` 换边，第四条边完成后进入 `COMPLETE`。船体按 50 cm × 45 cm 矩形建模，稳定刷墙区间为船中心到左墙 32–36 cm，目标中心值 34 cm。

阶段 1 的软件流程已经具备：定位启动默认不创建新轨迹；操作者在 RViz2 的 `map` 坐标系使用 `2D Pose Estimate` 后，管理器通过 Cartographer `/start_trajectory` 创建带初始位姿的纯定位轨迹。启动入口同时运行栅格地图节点发布 `/map`，管理器检查轨迹状态、`map → base_link` TF 新鲜度、地图边界、雷达有效率和扫描/地图重合，并在稳定窗口后发布 `/pool_cleaning/localization_status` 的 `LOCALIZATION_READY`。阶段 2 的边选择、靠墙和入口调整，阶段 3 的自动换边，以及阶段 4 的四边编排和按边覆盖记录均已接入当前包。代码和本地测试已覆盖完整一周流程；实船清洁效果、地图误差和规定次数的现场重复证据仍需按开发验收文档归档。

完整状态机和 ROS 数据流见[沿墙清洗流程图](../../docs/pool_cleaning/WALL_CLEANING_FLOW.md)。

当前执行器参数：滚刷 30%，第五电机双向推力 ±30%，滚刷角度门控 ±10°，四主电机从偏角超过 ±3° 就开始纠偏，开刷与纠偏可以同时进行。中心墙距 32–36 cm 时第五路关闭并允许滚刷工作；超出该区间时先侧推调整。换边时滚刷关闭，`REVERSE` 与向右侧推同时持续 0.5 s。

大偏角丢墙修复：本任务的墙线识别范围从约 ±34.4° 扩大到 ±70°，相邻有效观测的角度变化容差从约 11.5° 调整为 30°。这是感知识别范围，滚刷门控仍为 ±10°。运行中收到新鲜但未拟合到墙的扫描时进入 `WAIT_WALL`，全部输出关闭，保持任务与租约，最多等待 1.5 秒；墙线恢复后自动继续纠偏或刷洗。超过等待时间进入 `PAUSED`。初次 start 仍需有效墙线；取得租约的瞬间若短暂丢墙，同样进入等待恢复。

反向纠偏复现修复：盲船头下转向会缩短可见左墙段，本任务拟合所需最短墙段从 0.50 m 调整为 0.30 m，其他点数、残差与内点比例条件保留。墙线关联有效期从 1 秒延长至 2 秒，长于本任务 1.5 秒的丢墙恢复窗口，避免等待期间丢弃墙身份、把后方短墙当成左墙并发出相反转向。详见 [代码复现分析](../../docs/single_side_trial/HEADING_FEEDBACK_20261023.md)。

| 条件 | 输出 |
|---|---|
| 左墙夹角在 **[-10°, +10°]**，包括边界 | 滚刷 **30%** 开启推进；四主电机同时按 ±3° 死区纠偏 |
| 夹角 **>+3° / <-3°** | 四主电机 LEFT / RIGHT 25%、BOTH，直到回到 ±3° |
| 夹角 **>+10°** | 关滚刷，四个主电机 **LEFT 25%、BOTH** 持续纠偏 |
| 夹角 **<-10°** | 关滚刷，四个主电机 **RIGHT 25%、BOTH** 持续纠偏 |
| 夹角回到 ±10° 内 | 当帧恢复滚刷，继续纠偏至 ±3°；没有脉冲间隔或中位等待 |
| 中心到左墙 **>36 cm** | 第五电机向左推，默认 **+30%** |
| 中心到左墙 **<32 cm** | 第五电机向右推，默认 **-30%** |
| 中心墙距 **32–36 cm**，包括边界 | 第五电机关闭；不保留上一次推力 |
| 船中心到前墙 **≤30 cm** | 滚刷、主电机、第五电机全部关闭，进入换边停稳流程 |

`ψ = WallObservation.tangent_rad` 是左墙切线相对船头的夹角；正误差发 LEFT，负误差发 RIGHT。每个新墙观测都重新计算航向与侧推，纠偏时第五电机仍工作。超出 ±10° 也可以启动任务，先纠偏再开滚刷。

船头转向左墙时，正确左墙观测应为负角度，控制器应发 RIGHT（`direction=4`）。`CORRECT_HEADING` 现在可以同时开启滚刷：角度在 3° 到 10° 之间时两者同时工作。左右指令与实船转向的对应关系仍需用现场 CTRL 和实际动作确认，软件方向表本身不能证明电机接线与安装正确。

换边阶段保留转移数据新鲜度、通信超时、执行器命令期限、遥控接管和取消/暂停关闭输出；转角使用连续地图航向反馈，不用固定时间代替 90°完成判断。地图边身份与左墙目标关联由 `EdgeTarget` 恢复，防止换边后误认相邻墙。

默认无固定运行时长上限。启动门控在 `LOCALIZATION_CHECK` 中完成定位检查和地图选边，直边状态序列为 `APPROACH_WALL → APPROACH_EDGE → FOLLOW ↔ CORRECT_HEADING/WAIT_WALL → EDGE_STOP`，换边状态序列为 `EDGE_STOP → MOVE_INWARD → TURN_TO_NEXT_EDGE → APPROACH_NEXT_EDGE`；端点和后退右推之间各保留一个停止指令周期，不等待物理停稳。转向目标航向误差放宽为 ±20°，转向和整段换边的时间超时默认关闭。临时丢墙为 `WAIT_WALL`。`wall_recovery_sec` 默认 1.5 秒；持续丢墙、反馈过期、定位退化或通信异常进入 `PAUSED`。手动暂停、取消和过期数据不会因反馈恢复而自动续跑；暂停后必须 `cancel` 再 `start`。

任务启动时，操作者只需将船粗略放在 `edge_0` 附近并在 RViz2 发布大致初始位姿。当前配置将 `default_edge_id` 固定为 `edge_0`；控制器仍会检查位姿距离、左侧关系和边走廊，靠墙阶段关闭滚刷并使用第五路调整到 32–36 cm，同时纠正航向，航向稳定后再用滚刷沿边推进到入口。位姿不在 `edge_0` 可用范围、墙位于右侧或目标墙不匹配时拒绝启动。每条边在船中心距前方墙约 30 cm 处换边，第四条边完成后进入 `COMPLETE`。

## 前墙距离

现有雷达安装有船头盲区，因此仍用已有冻结地图的 `map → base_link` 位姿计算前墙距离；这版没有改雷达外参。默认地图矩形长约 2.883 m、宽约 1.520 m，参数来自 `20261021_145054` 地图。

前墙距离统一用于四条边的端点判断；当前边使用 `EdgeSegment` 的地图投影。令池坐标为 `x`、船头与长轴夹角为 `yaw`：

```text
朝长轴正向：船中心到前墙距离 = length - x
朝长轴反向：船中心到前墙距离 = x
```

船体位姿随矩形地图一致旋转 180° 时，计算出的前墙中心距离相同。有效前方雷达观测给出的是船体前缘净空，控制器会加回船长半尺寸后再与 30 cm 中心距离阈值比较；`front_observed=false` 不阻止启动，不会把未知距离当成零。

30 cm 指 **base_link 中心到前方墙**。刷墙目标是 **base_link 中心到左墙 32–36 cm**；左侧刷尖约 33 cm，34 cm 作为名义中心值。到达边端点后同时发出 `REVERSE` 和向右侧推 0.5 s，再使用地图航向反馈完成右转。地图距离仍依赖真实定位精度，不能发现雷达盲区新增障碍。

## 包内职责与接口

当前本包执行矩形四条地图边，提供边选择、靠墙/入口调整、沿边控制、自动换边、按边覆盖区间记录和一周结束状态报告。凸多边形与凹多边形仍按[开发与验收文档](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)中的后续阶段执行。

- `perception.py` / `perception_node.py`：墙线拟合、关联和空间观测；`perception_config.py` 只含感知与几何参数。
- `geometry.py`：矩形/有序边界、顺时针左墙边段、投影和边选择。
- `mission.py` / `config.py`：边段状态机、靠墙/入口调整、端点停止、后退右推、右转和四边任务参数。
- `controller_node.py` / `lease.py`：任务服务、状态、预览与 bridge 租约输出。
- `ros_support.py` / `runner.py`：公共消息转换、时间戳和节点退出处理。
- `localization.py` / `localization_node.py`：RViz 初始位姿、Cartographer 轨迹生命周期和定位就绪门控。
- `pool_localization.launch.py` / `pool_localization.lua`：冻结地图定位、静态雷达 TF 和定位管理器。

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
| `start`、`pause`、`cancel` | `std_srvs/Trigger`，任务控制 |
| `set_edge` | `f1_interfaces/SetEdge`，显式指定当前地图边 |
| `edge_target` | `f1_interfaces/EdgeTarget`，地图位姿生成的目标墙上下文 |

`f1_interfaces`、`f1_nano_bridge`、`f1_mapping_bringup` 继续独立。
旧包升级须先按 [迁移说明](MIGRATION.md) 清理旧源码/安装产物并更新自定义参数文件中的节点名。
实测尺寸与历史雷达视场记录见 [MEASUREMENTS.md](MEASUREMENTS.md)。

## 构建与离线验证

```bash
cd /home/bboss/new-nano/surface_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_pool_cleaning
source install/local_setup.bash
ros2 run f1_pool_cleaning pool_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --packages-select f1_pool_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_pool_cleaning --verbose
```

演示和测试使用合成反馈、localhost DDS 和模拟 bridge，不连接串口或实机。

阶段 4 本地验证重点：四条边的边界几何、目标墙关联、30 cm 端点停车、0.5 s 后退右推、±20° 粗略右转反馈、三次换边、`completed_edges` 和按边覆盖区间。实船仍必须记录无刷动作、四边轨迹、刷头接触和未覆盖区间；本地回放不能替代清洁效果证据。具体分项与复测说明见[开发与验收文档](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)。

## 启动

Nano 源码同步和完整用法见 [下水测试步骤](DEPLOY_TEST_NANO.md)。现在支持统一启动：

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

在 RViz2 中确认 Fixed Frame 为 `map`，显示 Map、LaserScan、TF，然后点击 `2D Pose Estimate` 在地图中标定船体位置和船头方向。确认 `/pool_cleaning/localization_status` 的 `state` 变为 `LOCALIZATION_READY` 后，再运行 `status` 和 `start`。定位和 bridge 已运行时，在终端 A 命令末尾增加 `bringup start_localization:=false start_bridge:=false`，只启动缺少的部分；已有清洗控制器应先退出。快捷脚本默认开启硬件输出许可，默认 ROS 域为 33；已设置的域和 RMW 环境会保留。也可加载环境后运行 `ros2 launch f1_pool_cleaning pool_bringup.launch.py output_enabled:=true`。原分开启动入口继续保留，默认预览不申请租约：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py
ros2 topic echo /pool_cleaning/status
ros2 topic echo /pool_cleaning/command_preview
```

没有 `LOCALIZATION_READY` 时 `/pool_cleaning/start` 会拒绝申请任务租约；定位跳变、TF/雷达过期或轨迹结束会关闭输出并使任务进入 `PAUSED`。定位恢复后不会自动恢复旧任务，必须检查原因、调用 `cancel`，再次在 RViz2 标定后再 `start`。

实机使用同一个入口，显式开启输出，再启动任务：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py output_enabled:=true
# 另一终端：
ros2 service call /pool_cleaning/start std_srvs/srv/Trigger '{}'
# 暂停或取消：
ros2 service call /pool_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /pool_cleaning/cancel std_srvs/srv/Trigger '{}'
```

实机需要已有 bridge 的串口/运动许可、主电机方向确认和 `allow_fifth_thruster=true`。`mode=BOTH` 由固件混控四个主电机；这版不发主电机 FORWARD/REVERSE，推进仍由滚刷提供。保持第五路既有“正号向左”的映射。

状态前部新增 `left_wall_distance_cm`：船中心到左墙的垂直距离，单位厘米，保留 1 位小数，例如 `34.0`。原 `wall_distance_m` 米制字段继续提供相同距离；墙线无效时这两个字段均为 `null`，不把占位值 0 当作实际距离。

状态还包含 `wall_angle_deg`、`wall_valid`、`wall_reason`、`heading_band_deg`、`map_front_clearance_m`、刷盘/侧推请求及租约状态；`localization_ready`、`localization_state`、`localization_reason` 来自定位管理器。`wall_reason` 保留感知的拒绝原因，持续丢墙暂停时 `reason` 也会带出该原因。查看单条完整状态：`ros2 topic echo /pool_cleaning/status --once --full-length`。预览 token 始终为空。`STOPPED` 只表示输出已关闭，不代表船体已经停稳。

阶段 4 状态还包含 `current_edge_id`、`transfer_next_edge`、`completed_edges`、`edge_progress_m`、`edge_entry_m`、`edge_stop_m`、`cleaned_intervals` 和按边覆盖记录。第四条边结束时 `phase=COMPLETE`，`completed_edges` 应为 `edge_0` 至 `edge_3`。覆盖区间的统计基准是 `left_brush_contact_centerline`，表示左侧滚刷接触中心线的连续轨迹；它用于核对一周任务的边段覆盖，不能单独等同于池底面积覆盖率。

请使用本包更新后的默认配置。历史诊断目录中的 `rectangle_current_side.yaml` 仍保存当时的 ±3°、8 秒限制，不是本版测试参数。
