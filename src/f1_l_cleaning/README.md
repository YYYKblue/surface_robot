# f1_l_cleaning：开放多段直墙清洗

更新：2026-11-08，源码基线 `0c0e4e2`。本包从原“大 L 两段墙”扩展为有方向的开放墙链，支持单条直墙及多次左转、右转。保留包名、`l_controller`、`l_bringup.launch.py`、`/l_cleaning/*` 和协议任务类型 `l`；App 显示“开放多边清洗”。定位与硬件输出继续复用共享组件，默认预览、等待显式 start。

## 路线与地图

`map:=` 仍是地图输入。同名 `.pbstream/.yaml/.pgm` 放在一起，定位使用 pbstream，路线使用栅格。地图文件不纳入 Git。

地图处理位于 `f1_pool_cleaning/map_profile.py` 与 `open_route.py`：

1. 按最大空闲连通域的外轮廓顺序分段拟合，合并栅格造成的共线碎片。
2. 检查左侧实体墙、右侧空闲水域支持；持续缺墙和未知边界断开，不只用整条线的平均分数。
3. 连续小角度同向弯折按曲线处理，不能当成一串直墙；相邻有限线段只有在端部附近才连接。
4. 输出全部候选开放墙链，按长度降序排列。默认 `route_chain_index: 0` 选择最长的一条。不同墙链不自动转场、不拼接、不闭合首尾。

当前提取最短段为 0.65 m，短栅格缺口容差 0.20 m，几何连接容差 0.50 m。自动拟合属于启发式识别；不代表所有候选都是已确认的实墙。曲线、分叉、缺乏支持的边界不作为任意转场任务自动执行。闭合矩形仍使用矩形任务。

本地 `lidar_20261026_170703` 默认得到五段连续路线：右下阶梯的三条短墙，接原大 L 下边和左边。当前拟合长度约为 **1.00 / 2.07 / 1.26 / 8.91 / 11.84 m**，转角约为 **右 64.15° / 右 93.13° / 左 94.54° / 右 92.70°**。这不是船体航点或实测尺寸；原来的 AB/BC 现在对应 `edge_3/edge_4`，旧的 `default_edge_id` 或 `set_edge` 选择需重新核对。

离线查看：

```bash
ros2 run f1_pool_cleaning map_profile "$PWD/maps/lidar_20261026_170703.yaml" --json
# 检查另一条独立候选墙链（0 为最长；并非自动转场顺序）
ros2 run f1_pool_cleaning map_profile "$PWD/maps/lidar_20261026_170703.yaml" --route-chain 1 --json
```

输出中的 `leg_lengths_m`、`turns_deg` 对应全部边及转角，正角为左转，负角为右转；`chain_lengths_m` 列出候选墙链总长度。旧 `turn_deg` 为首个转角，保留兼容。

配置文件中的 `route_chain_index` 选择墙链，修改后重启；它是控制器 YAML 参数，不是直接传入 launch 的参数。`route_xy: [x0, y0, x1, y1, ...]` 可覆盖已成功推导的路线，接受两个以上有限点，保持顺序，不自动反向或闭合。调用方必须保证墙在左、水在右；保留重复点、自交、掉头重叠和原有有效清洗区间校验。不能使用 `boundary_xy`，手填路线仍不能绕过地图推导失败。

使用自定义配置时，复制 `config/l_cleaning.yaml` 并修改 `l_controller.ros__parameters.route_chain_index`。独立 `l_bringup.launch.py` / `l_cleaning.launch.py` 通过 `config:=/绝对路径/配置.yaml` 加载；网关组合或受管入口使用 `l_config:=/绝对路径/配置.yaml`。

## 沿墙与换边动作

初始选墙、起始区域寻找和沿墙逻辑沿用原实现：左侧射线命中有限任务墙，锁定最近投影点，以旋转、前后移动、侧移进入起始范围，再确认新鲜墙观测。不会搜索绕障路径。墙距 32–36 cm 时按航向条件开刷，默认由 50% 滚刷沿墙推进，四主电机纠偏。任务开始前可通过 App 或 `/l_cleaning/set_forward_boost`（`std_srvs/srv/SetBool`）启用四电推 25 档前进辅助；仅稳定沿墙且船中心距当前墙端大于 80 cm 时允许，进入近端后本墙段锁定关闭，纠偏优先。运行中及暂停时不能修改，重载地图/控制器重启后默认关闭，详见[共享提速规则](../f1_pool_cleaning/README.md)。侧推幅度 50%。沿墙、初始侧移及左转换边侧移使用 `side_percent`，右转换边侧推使用独立参数 `transfer_side_percent`。

每边船中心距端点 ≤0.40 m 时关刷，进入换边；最后一边结束则停止。Windows App 蓝色虚线按 33 cm 名义墙距绘制，末端橙色方框标出距墙端 40 cm 的停车点，图例显示 32–36 cm 开刷范围。启动后只显示本次剩余墙链，各段不相连、不闭合；轨迹生成与矩形共用同一实现，详见[目标轨迹协议](../f1_pool_cleaning/README.md)。

**右转保持原动作：**停止指令周期 → 同时后退、50% 右推 0.50 s → 停止 → 按地图航向右转 → 在 ±20°内切换新墙 → 复用靠墙、纠偏和清洗。

**新增左转：**

```text
EDGE_STOP
  → LEFT_EXIT_WALL：保持旧墙航向，主电机前进走出墙端
  → LEFT_SHIFT_TO_ENTRY：保持旧墙航向，向左侧移至新墙入口沿程位置
  → TURN_LEFT_TO_NEXT_EDGE：停止平移，按地图航向左转
  → APPROACH_NEXT_EDGE：切换墙目标，等待新鲜观测后复用沿墙控制
```

左转全程关刷，平移时发生航向偏差先停止平移并纠偏；动作或方向变化先发一个停止周期。前行、侧移用地图位姿判定到位，左转使用原转向航向容差。约 90°左转时，第二步侧移与新墙平行，因此按新墙入口沿程位置结束，不能按距新墙距离结束。

| 新参数 | 默认值 | 含义 |
|---|---|---|
| `left_exit_forward_m` | 0.35 m | 相对当前墙端的最小前行目标；同时考虑下一边名义入口在旧墙方向上的投影，取较大值 |
| `left_transfer_position_tolerance_m` | 0.05 m | 前行和侧移到位容差 |

这些是动作目标，不是船体净空保证。按本轮要求，**未增加相邻转角位置、船体/刷头扫掠或环境碰撞安全检查**；原入口余量、端点停车区、沿墙走廊、定位、数据新鲜度、租约和停止保护保留。停指令周期不代表实测船体已经停稳。短边和急转角的实船效果尚未验证。

换边期间目标墙无效，不用旧墙反馈开刷；换边完成增加目标代次并拒绝切换前排队的旧观测。持续丢墙按原 1.5 s 规则暂停。原转移/转向超时默认仍为 0，可按现场配置启用。

## 任务范围

初始区域寻找保持原规则：可以从任意选中墙的附近开始，随后只执行该位置及后续边，到墙链末端停止，不返回、不补洗。

**要执行全部五条边，应从 `edge_0` 的入口附近开始。** `specified/default_edge_id` 只指定第一条墙，参考点仍取船中心最近投影，不会自动把船送回整个墙链起点。默认射线命中 `edge_3` 时只执行 `edge_3/edge_4`，不会把未经过的前三条边算成已清洗。

状态中的 `route_chain_index`、`route_chain_count` 分别报告所选候选链索引和候选总数。

`COMPLETE` 表示本次路线结束；`completion_scope=remaining_open_route`，`coverage_complete` 和 `remaining_contact_intervals` 单独报告刷头轨迹覆盖缺口。既有角部留白不变，不将整场路线执行完成等同于实际去污效果。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_l_cleaning
source install/local_setup.bash
ros2 launch f1_l_cleaning l_bringup.launch.py \
  map:="$PWD/maps/lidar_20261026_170703.pbstream" start_rviz:=true
```

雷达需独立运行。先在 App 或 RViz 的 map 坐标系发布初始位姿，等待定位就绪，再调用任务：

```bash
ros2 topic echo /l_cleaning/localization_status --once
ros2 service call /l_cleaning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /l_cleaning/status --once --full-length
ros2 service call /l_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /l_cleaning/cancel std_srvs/srv/Trigger '{}'
```

需要指定首边时，launch 加 `start_policy:=specified default_edge_id:=edge_0`。默认只预览；实际输出仍需 `output_enabled:=true` 及 bridge 的对应许可。本次开发未连接硬件或下发实船动作。

复用已有节点时用 `start_localization:=false` / `start_bridge:=false`；仅清洗入口为 `l_cleaning.launch.py`，也需同一地图。只有一套 bridge、定位和地图发布链。网关组合入口继续兼容 `task_type=l`，独立 bringup 的网关定位前缀为 `/l_cleaning`，组合入口共享 `/f1_localization`。

`pause` 后需 cancel 再启动；`stop` 保留进度快照，`cancel` 清除任务边关联；重新 start 创建新任务，不是断点续跑。App 的“停止手动输出”不停止自动任务。

## 验证

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/f1_operator_gateway/test/cyclonedds_localhost.xml"
colcon test --packages-select f1_pool_cleaning f1_l_cleaning --executor sequential --return-code-on-test-failure
colcon test-result --verbose
QT_QPA_PLATFORM=offscreen python3 -m pytest operator_app/test -q
```

测试覆盖旋转地图与不同分辨率的五段墙链、短墙、缺墙断开、独立链选择、原右转换边、45°/90°/135°左转、混合转向连续执行、数据过期、墙目标失效与预览启动。部署地图存在时额外检查三条短墙加大 L 的提取结果。动作级模拟只验证控制输出和状态流，不能证明实船动力学或空间可行性。
