# 左侧单墙清洁：back2 分步控制

修订：2026-10-21。适配 `firmware`（back2，协议 v2） 和v2 ROS `f1_nano_bridge`，固件保持只读。左刷、纵向推进器、贴墙池壁可见为用户已确认条件；船体/支架/刷头几何及25%最低档动力学仍需实测。

## 当前实现

用户指定跳过完整标定、先关闭刷盘看大致效果时，使用独立的 [关闭刷盘试跑说明](TRIAL_NO_BRUSH.md) 和 `config/supervised_no_brush_trial.yaml`。该模式强制关刷、以中心距墙 45 cm 为目标、最长 10 s；状态标记未标定，不修改正常模式的标定要求。

原始扫描校验、稳健左墙拟合、连续关联、前方检查、有限转向扫掠检查、互斥方向脉冲、刷洗窗口、开始/暂停/恢复/取消和bridge租约管理。人工将机器人放到近似平行的起始墙段后运行：`ALIGN_WALL → APPROACH_WALL → CLEAN_WALL`。

本次移除连续 `surge±yaw` 混控及6%～12%低推力参数。back2一次只能选择前进/后退/左转/右转，主轮最低档是125 μs偏移；上位机不能编码更低档位或同时前进与转向。

不包含任意起点导航、固定地图定位、自动过角、完整池边路线、实测速度/接触力闭环或覆盖统计。运行到期暂停，不宣告全池完成；刷头指令时长不是清洁效果证明。

## 新控制策略

- 对齐根据墙切线角选择LEFT/RIGHT；靠墙根据距离误差生成有限接近角，角误差进入死区才FORWARD。
- 固定BOTH和25%档位，默认前进脉冲0.20 s、转向脉冲0.10 s，结束或换方向后STOP等待至少0.30 s，再重新决策。第五路始终关闭，不自动倒车。
- 绝对direction_until由bridge独立执行，重发不会延长脉冲。valid_until仍限制新计算结果时效；预定脉冲结束与任务租约到期分开处理。
- 靠近前方停车距离时缩短前进脉冲；不能用减小方向通道幅度模拟减速。
- 转向须有对应left_turn_clear/right_turn_clear，并通过有界角度内刷头压缩检查，否则暂停。扫掠基于连续旋转矩形包络、雷达偏移、角域边界余量及有效回波检查。
- 当前扫掠检查保守，约280°扫描可能因盲区使左右转向均不可用；左墙和前方可见不等于转向可行。不会把无回波作为自由空间。这个限制需纳入实机可行性评估。
- 接触/角度窗口持续0.50 s后，只在FORWARD脉冲期间开刷；STOP/纠偏/脱离窗口都关刷。相同窗口不证明真实接触力。
- 扫描时间超过150 ms、墙失联/跳变、前方未知、障碍、墙端、过大角度或压缩超限均暂停。重复扫描不续命，命令期限为计算+100 ms和扫描+150 ms中的较早值。
- RC、epoch、连接、感知代次改变后保留暂停，需要明确resume和新租约。迟到的acquire响应只释放，不重启动作。

## 接口

| 接口 | 内容 |
| --- | --- |
| `/wall_cleaning/wall_observation` | `WallObservation`，原扫描时间、墙线、前方和两方向转向空间判断 |
| `/wall_cleaning/wall_marker` | base_link墙段可视化，200 ms过期 |
| `/wall_cleaning/effort_preview` | `EffortPreview`，互斥direction、speed_percent、valid_until、direction_until和辅助目标 |
| `/wall_cleaning/status` | `CleaningStatus`，阶段、原因、授权与能力边界 |
| `/wall_cleaning/start`、`pause`、`resume`、`cancel` | `std_srvs/Trigger`，开始/暂停/恢复/取消 |
| `/wall_cleaning/reset_observation` | 重选墙段；活动任务会因感知代次变化暂停 |

`EffortPreview`已删除连续surge/yaw和四电机百分比字段，所有依赖者需重建。硬件命令仅通过新bridge，本包不打开串口。

## 构建与预览

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
colcon build --symlink-install --packages-up-to f1_wall_cleaning \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 launch f1_wall_cleaning wall_cleaning.launch.py
```

默认output_enabled=false，不创建执行器发布者或bridge服务客户端；几何未确认时start拒绝，仍可显示墙线。launch不启动雷达、bridge或建图。

```bash
ros2 topic echo /wall_cleaning/status
ros2 topic echo /wall_cleaning/effort_preview
ros2 service call /wall_cleaning/start std_srvs/srv/Trigger '{}'
ros2 service call /wall_cleaning/pause std_srvs/srv/Trigger '{}'
ros2 service call /wall_cleaning/resume std_srvs/srv/Trigger '{}'
ros2 service call /wall_cleaning/cancel std_srvs/srv/Trigger '{}'
```

RViz Fixed Frame设为base_link，添加wall_marker。合成示例使用独立domain，两个终端相同ROS环境，分别运行：

```bash
export ROS_DOMAIN_ID=176
ros2 run f1_wall_cleaning wall_demo_scan
```

```bash
export ROS_DOMAIN_ID=176
ros2 launch f1_wall_cleaning wall_cleaning.launch.py \
  config:=$(ros2 pkg prefix f1_wall_cleaning)/share/f1_wall_cleaning/config/demo_only.yaml \
  scan_topic:=/wall_cleaning/demo_scan output_enabled:=false
```

第三个同环境终端调用start。演示几何是虚构值、扫描为完整环周、没有水上动力学；不得用于实机。录包可使用仿真时间但保持输出关闭，回放暂停后任务不会自动恢复。

## 实机测量与门控

2026-10-21 用户提供的静态尺寸、约束核算及待测项目见 [实机测量记录](MEASUREMENTS.md)。已录入 [实测配置副本](config/measured_wall_cleaning.yaml)，停车距离与脉冲响应尚未标定，两个确认开关仍为 false，不能直接启用实机输出。

运行配置复制自 `config/wall_cleaning.yaml`。未知尺寸保持零值，测量后才geometry_confirmed=true：

| 参数 | 含义 |
| --- | --- |
| hull_length_m / hull_width_m | 以base_link为中心、包住全部刚性船体/支架的矩形 |
| brush_x_m / brush_tip_y_m | 刷头未压缩接触点，x前、y左 |
| brush_compression_m / brush_max_compression_m | 目标/允许最大压缩量 |
| stopping_distance_m | 25%允许脉冲工况含延迟、惯性、水流漂移的实测停车余量 |
| front_slow_distance_m | 大于停车余量的缩短前进脉冲区 |
| forward_pulse_sec / turn_pulse_sec | 实测可控的动作持续时间；默认仅待调初值 |
| neutral_dwell_sec | 至少0.30 s，需覆盖实测停止和观察稳定时间 |
| turn_sweep_rad | 转向脉冲包括最坏通信/执行器/惯性余转的角度上界 |
| pulse_response_calibrated | 确认最低档脉冲、停止、转向角上界、旋转中心与位移漂移符合包络假设后才true |

旧low_thrust_calibrated不复用。需在低风险开阔环境先做标定，不能为了开始任务直接勾选。初始脉冲、增益和0.08 rad角度上界未验收。转向检查不构成完整三维扫掠规划；最低档或盲区不满足时保留暂停，不声称稳定贴墙。

实机启用需按 [bridge说明](../f1_nano_bridge/README.md) 确认板上back2、方向与串口，单独启动桥；随后：

```bash
ros2 launch f1_wall_cleaning wall_cleaning.launch.py \
  config:=/absolute/path/measured_wall_cleaning.yaml output_enabled:=true
```

真实输出拒绝仿真时间和未知几何；正常模式还拒绝未标定脉冲，独立的 supervised_trial 模式按上述约束允许现场试跑。启动不自动acquire，仍需显式start。默认单墙最长60 s、对齐/靠墙20 s。暂停释放自己的令牌；服务不可达时不再发指令，等待原始期限到期，不覆盖其他任务。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/bboss/new-nano/surface_ws
source install/local_setup.bash
colcon test --packages-select f1_nano_bridge f1_wall_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_wall_cleaning --verbose
```

使用合成扫描、实际ROS节点和bridge Session；无实机串口。覆盖外参/墙距、280°前向和转向盲区、扫掠障碍、脉冲/中位等待、绝对截止、接触窗口、RC/迟到租约/时钟故障和默认无硬件输出。结果见下方记录。

### Nano 上 ROS 测试超时

如果 `colcon build` 全部完成，但 `colcon test` 在 `ros` 用例报告
`PAUSED_SAFETY / bridge revoked lease`，应排查运行时期限和调度，不是编译依赖。
旧测试将感知、控制器和模拟 bridge 放在同一个单线程 executor 中，以50 Hz注入扫描；
感知回调耗时会同时阻塞控制命令和模拟串口心跳。本机给感知增加30 ms耗时后，
复现了RC恢复、前方未知两项测试失败，记录到命令期限到期。
这说明存在测试调度问题，但不能单凭原日志确认Nano首次撤销租约的具体原因。

Nano第二次日志确认：普通RC用例通过，仅额外增加30 ms感知耗时的用例失败；
首次撤销为 `command deadline expired`，拒收计数0，最大发送间隔25.5 ms。
因此该次触发的是命令到期，而非80 ms发送间隔保护。30 ms是实际计算之外的
额外延迟，并非总处理耗时；原日志仍不足以区分计算耗时、扫描排队和DDS投递延迟。

测试现将感知放到独立 executor 线程，合成扫描与 `wall_demo_scan` 一致使用20 Hz，
并增加持续30 ms感知延迟下的完整RC恢复测试。CycloneDDS复测还发现，
新观测到达后等待下一个50 ms控制定时回调可能错过命令期限；实机控制现于新观测
到达时立即计算，定时器继续检查断流和超时，重复扫描仍不续期。原100 ms命令期限、150 ms扫描年龄、
80 ms bridge发送间隔限制保持不变。失败诊断包含租约撤销历史和最大发送间隔，
避免后续中位握手将首次错误原因覆盖。

后续修正将固定船体转向包络移至初始化计算，扫描聚类改为NumPy批量计算；
每帧仍重新检查当前障碍、盲区和墙线。扫描、观测订阅均只保留最新一帧，
防止处理稍慢时积压旧数据。新增回归验证缓存不会沿用旧的安全判断，
以及感知处理额外延迟180 ms后必须暂停、恢复新扫描也不得自动恢复任务。
失败诊断现包含 `process_ms`、`queue_age_ms`、`result_age_ms`、
命令 `receive_gap_ms` / `remaining_ms`，可进一步定位Nano的实际瓶颈。

实际雷达约50 Hz输入的处理能力仍须在Nano上单独验证。
如果复测仍失败，请保留完整断言，特别是新增的撤销原因列表和发送间隔。
`colcon test-result` 会同时统计CTest包装项和pytest子项，一项pytest失败可能在汇总中出现两次。

尚未进行水上动力学、真实接触、清洁效果、跨机DDS或全池任务验收。当前实现与实机验收边界见本说明；原独立方案文档未包含在此仓库中。

### 2026-10-21 back2 验证记录

| 项目 | 结果 |
| --- | --- |
| 相关接口、bridge、IMU依赖、单墙包构建 | 完成 |
| 感知 | 15项通过，含方向相关盲区与连续扫掠中的障碍 |
| 控制与配置 | 26项通过，含方向反转先STOP、中位等待、脉冲上限和最低档限制 |
| ROS接入/启动 | 8项通过，含默认无输出、RC恢复、迟到授权、时钟和SIGINT退出 |
| 合计 | **49项实际pytest通过，0失败、0跳过** |
| 现有bridge回归 | 52项通过；其内另执行26项原固件控制回归通过 |
| 固件只读复核 | back2目录540个文件的内容哈希在验证前后一致 |

`colcon test-result`的52包含3个CTest包装用例，不是52个独立算法测试。ROS测试记录了1条服务应答超时警告，所有断言通过；保留该限制，不据此认定跨进程/跨机DDS或实机链路已验收。

### 2026-10-21 Nano 测试时序修正复测

相关6包重新构建通过。修正后使用CycloneDDS运行本包完整pytest：
**50项通过，0失败、0跳过**（15项感知、26项控制、9项ROS），
包含新增30 ms感知延迟下的RC恢复用例。此结果来自本机ROS Humble环境，
Nano硬件及实际雷达50 Hz负载尚待复测。

### 2026-10-21 Nano 第二次失败后的性能修正

- 同一台本机、相同1441点合成扫描、200帧统计：单帧处理耗时中位数
  从6.745 ms降至1.451 ms；初始化时另计算一次转向包络，耗时约6.3 ms。
  该数据不是Nano实测，不包含DDS和ROS消息转换。
- 200组变化扫描（墙距、角度、噪声、缺失回波）与修正前的完整观测结果逐项一致。
- Fast DDS与CycloneDDS分别运行完整pytest，均为**52项通过，0失败、0跳过**
  （16项感知、26项控制、10项ROS），额外30 ms延迟测试保留。
  Fast DDS记录两条服务应答超时警告；CycloneDDS无pytest警告。
- 命令100 ms、扫描150 ms和bridge发送间隔80 ms限制未放宽。
