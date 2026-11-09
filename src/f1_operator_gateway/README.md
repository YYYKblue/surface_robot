# f1_operator_gateway

更新：2026-11-09，源码基线 `9db6798`。Windows Qt App 的 Nano 侧 WebSocket 入口，提供手动控制、双任务操作、初始位姿、状态转发，以及受管建图、地图保存与选择。模块职责与入口对比见[架构导读](../../docs/ARCHITECTURE.md)。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
colcon build --symlink-install --packages-up-to f1_operator_gateway
source install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

包的运行依赖包含完整部署需要的 `f1_lidar_driver`、bridge、定位和两个清洗包，`--packages-up-to` 会构建这些依赖。网关运行需要 `python3-websockets`，通过 rosdep 或 `sudo apt install python3-websockets` 安装。Windows 依赖见 [App 说明](../../operator_app/README.md)。

## 推荐：App 管理建图和地图选择

```bash
ros2 launch f1_operator_gateway operator_system.launch.py \
  workspace_root:=/home/nvidia/surface_ws bind_address:=0.0.0.0
```

不要求 `map:=`，在 `maps/` 为空时也可启动。入口常驻一套 bridge 和网关；网关异步管理 `operator_mode.launch.py` 子进程，在“待机 / 建图 / 固定地图定位与清洗”之间切换，雷达独立运行；迁移到项目内驱动的步骤见 [`f1_lidar_driver`](../f1_lidar_driver/README.md)，本入口不启动第二套驱动。默认 `output_enabled:=false`；实际输出需显式开启并核对桥配置。不能与原完整 bringup 并行启动；复用 bridge 时设 `start_bridge:=false`。

参数：`workspace_root` 必填绝对路径；`map_timezone` 默认 `Asia/Shanghai`；`mapping_resolution` 默认 `0.05` m；可配置 `scan_topic`、`publish_lidar_tf`、`bridge_config`、`rectangle_config`、`l_config`、`localization_config`。`maps/` 不能是符号链接。`workspace_root/maps/.operator-runtime.lock` 防止同工作空间重复运行管理实例。

`rectangle_config`、`l_config` 未指定或为空时，分别加载 `f1_pool_cleaning/config/pool_cleaning.yaml`、`f1_l_cleaning/config/l_cleaning.yaml`（均从 ROS 选中的包安装目录解析）。对当前地图支持的任务，显式配置路径必须是现有文件，否则地图启动失败并报告参数名和路径。该规则同时适用于 App 管理模式和直接 `operator_bringup` 启动，避免外层空参数使控制器跳过 YAML。`localization_config` 仍保留原有空值语义。标准清洗 YAML 中 `use_map_tf` 和 `require_localization_ready` 均为 `true`；运行时核查及本次故障原因见[部署排查](../../DEPLOY_NANO.md#定位正常但开放任务不能启动)。

Nano 的 `deploy/f1-operator.service` 在启动前通过 `wait-operator-clock.py --offline-after 20` 优先等待 NTP 同步；20 秒后仍未同步则检查离线本地时间，要求日期合理且重新连续稳定 5 秒。离线时间不保证日历准确，脚本不会设置系统时间，目标机应通过 RTC 或已有持久化恢复机制建立时间。时间持续不稳定或日期无效时仍等待，可用 `journalctl -u f1-operator.service -f` 查看原因。运行中时间故障先撤销控制并尽力发送中性输出，再关闭桥接，以退出码 75 触发整个受管 launch 退出；systemd 重新检查时间后恢复为无地图待机，不恢复任务。600 秒内最多启动 5 次，超过限额后需排除故障再 reset-failed/start。服务模板开启串口及运动许可；直接 launch 没有启动检查和 systemd 自动恢复。详见[部署说明](../../DEPLOY_NANO.md#app-建图选图与开机服务)。

- 开始建图检查连续扫描的帧名、时间戳、角度元数据和有效回波。进入建图后使用现有手动操船；不以固定地图定位就绪作为建图条件。
- 清洗运行/操作结果未确认时拒绝切换；在模式切换、保存操作前停止手动输出并等待租约交接。
- 只停止本管理器启动的进程组。旧进程退出后等待 DDS 清除旧地图/节点，发现外部地图发布者或已知冲突节点则拒绝启动。
- 切换清除旧地图、TF 缓存、任务轨迹和定位状态，按新地图身份/会话时间过滤迟到状态。每次加载后重新设置初始位姿。
- 子模式任一进程退出会结束整组模式，网关报告错误；网关/bridge 退出时系统入口关闭。常驻网关不会因正常结束建图而退出。
- 目录扫描、预览、几何推导、进程等待和栅格导出在后台工作线程执行；Cartographer WriteState 使用 ROS 异步服务，不阻塞手动控制超时检查。

### 地图文件与恢复

名称只接受中文/字母/数字/下划线/短横线（最多 80 字符），拒绝路径和 Windows 保留名称。空名称使用 Nano 配置时区的时间戳；重名自动加序号。保存先写 `maps/.snapshot-<id>/`，调用 `/write_state`（包含未完成子图），再运行 `cartographer_pbstream_to_ros_map` 导出相同快照。

三份文件校验后以不覆盖已有文件的方式发布到 `maps/`，YAML 写入同名 PGM 的相对路径。目录列表拒绝不完整三件套；发布并非三个文件的单一文件系统事务，突然掉电可能留下不可用条目，原快照仍可供检查。快照文件在导出失败时保留，App 显示“待导出”，可重试；重新启动网关仍可发现保留快照。写状态服务长时间无响应时显示“结果待确认”，保持操作锁定而不重复发起写入。

“保存快照”不结束轨迹，也不承诺闭环已完成；“保存并结束”三件套成功后才退出。地图列表的完整性校验不等于 Cartographer 内容和现场地图质量验收；加载时继续由 Cartographer 和已有几何推导校验，失败不会自动开始任务。

运行日志：`workspace_root/log/operator_runtime/<session_id>.log`；失败导出日志在对应 `.snapshot-<id>/export.log`。未保存地图不跨进程重启恢复。

### 建图与地图协议

保持 `f1-operator-v1`，受管入口的 `hello.capabilities` 增加 `mapping` 和 `map_catalog`。

```json
{"op":"mapping","action":"start","request_id":"r1"}
{"op":"mapping","action":"save_end","name":"一号池","request_id":"r2"}
{"op":"mapping","action":"list","request_id":"r3"}
{"op":"mapping","action":"preview","name":"一号池","request_id":"r4"}
{"op":"mapping","action":"load","name":"一号池","request_id":"r5"}
```

其他动作：`save`（保存继续）、`discard`（必须 `confirm:true`）、`retry_export`（同名待导出快照）。每个请求必须有 `request_id`，沿用每客户端去重、单活动控制端、请求结果语义。`list/preview` 可由观察端读取。新连接不自动重发旧请求。

`reply` 的列表结果含 `maps`、`maps_directory`，预览含 `preview`，保存含 `saved: {name, paths}`。`telemetry.runtime` 包含 `mode/phase/session_id/current_map/target_map/message/busy/scan_fresh/map_available/pose_fresh/operation/last_operation/last_saved/maps_directory`。操作被接受不代表扫描匹配精度或船体已静止；建图状态与固定地图定位就绪独立。

原有单独网关和下面的静态组合入口保持兼容，不声明 `mapping/map_catalog`。

启动权限需按入口区分：上述 launch 默认预览，但仓库 `deploy/f1-operator.service` 模板设置 `F1_OUTPUT_ENABLED=true`。安装模板前应按部署目标核对该值；开机仍只待机，不自动恢复旧任务。完整步骤见[开机服务部署](../../DEPLOY_NANO.md#app-建图选图与开机服务)。

## 兼容：指定已有地图启动 App 清洗系统

先运行雷达驱动，并同步同名 `.pbstream`、`.yaml`、`.pgm`。以下默认只预览，不输出硬件动作：

```bash
ros2 launch f1_operator_gateway operator_bringup.launch.py \
  map:="$PWD/maps/现场地图.pbstream" bind_address:=0.0.0.0
```

该入口只启动一套 Cartographer/定位管理器、一套 bridge、一个网关，并启动地图几何校验通过的清洗控制器。定位统一使用 `/f1_localization/*`，任务分别使用 `/pool_cleaning/*`、`/l_cleaning/*`。不能与已有完整 bringup 重复启动。

- 地图不支持某种任务时，不启动该控制器，并把具体推导错误传给 App；另一种可用任务仍可运行。
- 开放任务保留类型 `l`，支持多段墙链；默认最长连续链，其他链在 `l_config` 中通过 `route_chain_index` 选择，不自动转场。完整闭合矩形不作为开放路线。
- `start_bridge:=false` 复用已有 bridge；`start_localization:=false` 要求已有定位使用 `cleaning_prefix:=f1_localization` 且地图相同。
- 可指定 `rectangle_config`、`l_config`、`localization_config`、`bridge_config`、`scan_topic`；已有雷达 TF 时设 `publish_lidar_tf:=false`。
- 需要现场实际输出时加入 `output_enabled:=true`。复用的 bridge 需自行配置对应串口、运动、方向和第五路许可。
- 支持 `start_rviz:=true`，App 初始位姿发布不依赖 RViz 运行。
- 类型选择不负责切换地图；不同地图需要重新启动对应配置、发布新位姿并重新等待定位稳定。

监听默认 `127.0.0.1:8765`。跨机设 `bind_address:=0.0.0.0`，Windows 连接 Nano 实际 IP。当前仍用于可信局域网，没有新增 TLS 或登录认证。

## 复用现有节点／仅手动控制

已有矩形任务或只做手动控制时，单独启动网关：

```bash
ros2 launch f1_operator_gateway operator_gateway.launch.py bind_address:=0.0.0.0
```

该入口不创建 bridge、雷达、定位或清洗节点。已有大 L bringup 时：

```bash
ros2 launch f1_operator_gateway operator_gateway.launch.py \
  bind_address:=0.0.0.0 localization_prefix:=/l_cleaning
```

**不再重映射 `/pool_cleaning/status/start/pause/stop/cancel` 到 L 接口。** 网关同时管理两种任务，旧重映射命令会混淆任务类型。`localization_prefix` 只选择共享定位管理器的接口；若手动组合两个控制器，应给它们设置相同的 `localization_status_topic`。

原有矩形与 L bringup 独立使用仍兼容各自的定位前缀。

## 状态与控制语义

| 接口 | 用途 |
|---|---|
| `/f1/actuator_command`、`acquire_control`、`release_control` | 手动控制与单租约 |
| `/f1/control_state` | 控制源、通信、执行器许可 |
| `/map`、TF、`/pavo2_scan` | 显示底图、船体位姿及扫描健康检查；不再生成 App 雷达点 |
| `/pool_cleaning/status`、`/l_cleaning/status` | 两种任务的独立状态 |
| 两组任务的 `start/pause/stop/cancel` | 任务操作 |
| 两组任务的 `set_forward_boost`（`std_srvs/srv/SetBool`） | 启动前设置主电机前进辅助 |
| `${localization_prefix}/set_initial_pose` | 提交 map 坐标系初始位姿 |
| `${localization_prefix}/localization_status` | 定位诊断、稳定窗口与就绪状态 |

控制器状态新增 `task_type`、`map_id`、`stamp_ns`、`run_id`、`completed_at_ns`、`can_start`、`start_blockers`。启动检查复用于自动任务的 `/start` 和状态发布；在获取租约后仍复查定位/反馈。射线选墙在启动时锁定，获取租约后以简单动作靠近，进入范围后确认墙面；不把贴墙条件强加到启动前。

网关检查定位/任务的接收时间和 ROS 源时间戳（0.6 秒），校验地图与任务类型。某任务状态过期不会把已知运行任务视为结束。启动请求串行处理；先停止手动会话，等待待返回的手动申请/释放和控制器状态更新，再发 start。等待超过 3 秒但尚未发送时放弃启动；已发出的操作超时保持结果待确认，不自动重发。开始服务成功后，还需收到新的 `run_id` 状态才确认启动。启动过程中收到暂停/停止/取消会按时序处理，防止迟到的启动响应覆盖停止意图。

初始位姿服务成功只表示请求已接受；新初始位姿确认前不复用旧定位就绪状态。任务运行期间禁止网关发布新初始位姿。RViz/其他 ROS 客户端仍可直接操作 ROS 服务，控制器原有定位失效停机检查保留。

网关明确拒绝自动任务运行/操作处理中新的手动输入；bridge 单租约继续承担硬件互斥。多客户端可观察遥测，首次控制操作绑定活动端，其他端控制请求被拒绝。

`manual_stop` 只停止网关手动会话，不调用自动任务暂停/取消。App 已将按钮改为“停止手动输出”。App 断线不会主动取消自动任务；待发送的启动交接会撤销，已发送的任务操作仍需根据机器人状态确认。

## WebSocket 协议

协议名仍为 `f1-operator-v1`；`hello` 新增 `client_id` 和 `capabilities`。手动消息字段与原版本一致。

`forward_boost` capability 表示网关支持启动前提速设置。请求示例：

```json
{"op":"task","action":"set_forward_boost","task_type":"l","request_id":"boost-1","enabled":true}
```

`enabled` 必须是布尔值，`request_id` 必填；沿用任务操作的串行、去重和权限控制。网关检查新鲜任务状态、地图/任务类型及服务，仅 `IDLE/CANCELLED/STOPPED/COMPLETE` 可修改，控制器再次校验，暂停不放行。服务成功后还等待请求发出之后、开关一致的新鲜状态；超时表示结果待确认，不自动重发。设置确认前不能开始任务。遥测透传控制器的 `forward_boost_*` 字段，并提供 `can_set_forward_boost` 与 `forward_boost_setting_error`。开关由控制器持有，App 重连不重置或重发，重载控制器默认关闭。

`diagnostic_history` 能力对应独立 `{"type":"diagnostics","diagnostics":{"session":"...","events":[]}}` 消息。网关保留最近 50 条桥状态变化和手动停止事件，hello 时补发历史，之后只发送新增事件。事件带单调递增 `id`、`time_unix`、`source`、`reason`、`details`；App 按 session/id 去重。手动停止记录清除前的输入年龄及超时阈值，桥事件记录 CTRL/ACK 年龄和最近撤销原因，不包含控制 token。进程重启不保留旧历史。新版 App 仍可读取旧遥测内嵌的 diagnostics。

```json
{"op":"task","task_type":"l","action":"start","request_id":"唯一请求编号"}
{"op":"initial_pose","x":1.2,"y":3.4,"yaw":1.57,"map_revision":1,"request_id":"另一请求编号"}
```

`task_type` 为 `rectangle/l`；`action` 为 `start/pause/stop/cancel`。位姿使用米、弧度，yaw 在 [-π, π]；`map_revision` 必须匹配当前地图。网关生成 ROS 时间戳、平面四元数和默认协方差。

`reply` 回传 `request_id`、`success`、`message`，可能带 `pending=true`（尚未确认）。网关缓存每客户端最近的请求结果，重复编号不重复执行；App 不自动重试操作。

`telemetry` 默认 10 Hz，包含 `state/pose/tasks/localization/active_tasks/task_busy/operation_pending/can_set_initial_pose/active_client/stream`。`tasks` 分别保存 `rectangle/l` 的状态、新鲜度和启动条件。2026-11-09 起不再上报 scan、expected_track、旧版重复 task、任务内重复 localization_snapshot、actual_track 和每帧诊断历史。ROS 雷达、控制器的目标计算与覆盖判据不受显示裁剪影响。

`stream` 提供网关 session、递增 sequence、generated_unix、generated_monotonic、server_queue_ms、tick_gap_ms、control_age_ms 及 transport 队列统计。单调时钟只在本机计算间隔，不能与 Windows 单调时钟直接相减。App 用真实收包时刻、GUI 等待时间及 server_queue_ms 判断遥测新鲜度；网络传输时延仍需单独测量。hello 的 session_id 和 tracks 的 session_id 均是网关会话，与 runtime.session_id 的建图/定位会话不同。

`map` 的几何字段保持 width/height/resolution/origin/frame_id/revision；显示数据采用 `encoding="indexed8_zlib"`、`data_zlib_base64`。解码为每格一个索引字节：0 未知、1 空闲、2 占用；行序沿用 ROS 栅格，App 显示时翻转 Y。只压缩显示颜色，不改变 ROS 原始地图或位姿坐标。后台编码仅保留一个待处理地图和一个最新结果，`map_update_period_sec` 默认 2 秒；受管 localization 模式首次有效地图后冻结，重新加载地图时重置，mapping 模式低频处理最新地图。地图 revision 仅主线程提交结果后更新；新连接 hello 补发缓存地图。地图及清空消息携带 `runtime_session_id`，App 据此拒绝上次建图/定位会话迟到的图片，避免地图和位姿错配；非受管入口该字段为空。保存地图 preview 仍使用限制尺寸的旧 data 数组格式。

每客户端仅有一个发送任务，telemetry/tracks/map 各一份最新待发消息，在途消息最多一份；操作回复、hello、diagnostics 使用上限 32 的可靠 FIFO。可靠消息优先，核心遥测与轨迹/地图公平交替；溢出或单次发送超过 2 秒明确关闭连接，防止无限累积。跨线程唤醒也合并。新协议需同步升级 Windows App；`compact_stream_v1/indexed8_map` 标识这组能力，旧 App 无法完整显示压缩地图和独立轨迹。

## 轨迹与距离遥测（2026-10-31）

`hello.capabilities` 新增 `task_stop`、`task_tracks`。任务 `stop` 调用对应控制器的 Trigger `/stop`，进入 `STOPPED`，保留任务进度；`cancel` 另外清除当前边关联。两者均关闭输出、释放租约，不支持断点续跑；显式 start 创建新 run_id。开始服务尚未返回时的停止也参与既有串行操作机制。

每个 `tasks[kind]` 新增：

- 任务状态顶层的 `start_edge_id/start_region/first_brush_contact/returning_to_start/remaining_edges/executed_edge_ids`：锁定起始墙、参考范围、首次有效开刷位置和执行进度，由网关透传。`completion_scope` 为 `rectangle_route` 或 `remaining_open_route`（旧版控制器使用 `remaining_l_route`，新版 App 仍兼容该值）；`coverage_complete/required_contact_intervals/remaining_contact_intervals` 独立报告覆盖情况，`COMPLETE` 不保证缺口为空。旧 `repair_pass/active_cleaning_interval` 已不由新控制器输出。
- `route_chain_index/route_chain_count`：开放任务所选候选墙链的零基索引与候选总数；矩形任务为 null。候选链之间不自动转场。
- `center_wall_distance_m`、`wall_fresh`、`wall_age_sec`：有效左墙线到 base_link 原点的垂直距离及源数据新鲜度；距离不可用时为 null。

实际轨迹由独立 `{"type":"tracks","session_id":"网关会话","tasks":{"rectangle":{},"l":{}}}` 消息发送，各任务值为 `{frame_id,map_id,run_id,segments,point_count,truncated}`。每 0.5 秒检查一次，内容变化才发送；hello 补发完整最新轨迹。历史按任务类型、地图、run_id 隔离；约 5 Hz 采样上限、3 cm 位移阈值，最多 2000 点。定位失效、重定位、时间回退、超过 0.6 秒采样缺口或超过 0.35 m 跳变时分段；任务停止/取消/完成后不继续记录手动运动。网关持续运行期间保留最近一次任务，支持 App 重连；网关重启丢失缓存。新任务或地图身份变化重置，点数上限达到后丢弃最早数据并设置 truncated。该轨迹用于观察，不作为覆盖完成判据。App 不再显示预期轨迹、起始范围和停车标记。

## 验证

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/f1_operator_gateway/test/cyclonedds_localhost.xml"
colcon test --executor sequential --packages-select f1_operator_gateway f1_localization f1_pool_cleaning f1_l_cleaning --return-code-on-test-failure
colcon test-result --verbose
```

网关测试包括协议、异步启动交接、过期状态、重复请求、初始位姿，以及真实 WebSocket 到模拟 ROS 服务的集成测试；不会连接硬件。Windows 现场仍需验收坐标方向、Wi-Fi 断连、硬件停止及真实清洗效果。

上述 DDS 文件**仅用于本机测试**，使用 localhost 单播与 1200 字节报文/1024 字节分片，不用于连接 Nano。当前开发环境曾出现“能发现节点，但接收不到清洗状态”：独立字符串测试中 10/1000 字节消息正常，3000/10000 字节消息丢失；使用此配置后四种大小均可接收。实机终端不要继承这份测试配置。

### 本次验证记录（2026-10-30）

以下分节为各次修改当时的记录，不是本轮文档核对重新执行的结果；七包记录早于雷达驱动纳入工作空间，当前完整构建为八包。测试数也受每次选择的包和用例范围影响，不应相加作为当前总量。

- `colcon build --packages-up-to f1_operator_gateway`：七个包构建成功。
- 四个受影响 ROS 包通过 CTest：定位 30、矩形清洗 193、L 清洗 22、网关 37 个 pytest 用例，共 282 个，零失败；使用上述仅限本机的 DDS 配置。
- Qt 无显示环境测试 6 个通过，覆盖旋转地图坐标转换、初始朝向拖拽、命令队列、定位门控、完成展示及遥测过期。
- 本地真实地图 `lidar_20261026_170703.pbstream` 的完整组合启动检查通过：一套 Cartographer/定位、一套 bridge、L 任务和网关；地图尺寸 1054×863，WebSocket 初始位姿经真实定位服务接受。没有扫描输入时定位始终未就绪，开始条件保持关闭；所有硬件输出禁用，退出正常。
- 未进行 Windows 原生打包、跨机 Wi-Fi 或实船验收；本地真实地图检查不证明定位精度和实际清洗效果。

### 轨迹与停止扩展验证（2026-10-31）

- 三个受影响包构建成功；矩形清洗 199、L 清洗 23、网关 45 个 pytest 用例经 CTest 全部通过，共 267 个，无失败/跳过。
- Qt 无显示环境 9 个用例通过，分别覆盖 L/矩形轨迹与完成流程、停止/取消、墙距失效和断线历史保留；人工检查模拟界面中的图层与按钮布局。
- 网关集成测试覆盖 WebSocket → ROS 服务的新任务启动、停止、取消与状态回传。本机缺少 WebSocket 依赖时曾导致集成测试启动失败，补充到 `/tmp/f1-operator-deps` 并设置 PYTHONPATH 后重跑通过；部署依赖仍由 package.xml/requirements.txt 声明。
- 使用前述 localhost DDS 配置，不连接实船。未进行 Windows 原生打包或现场运动/测距验收。


### App 建图与选图验证（2026-10-31）

- 七个 ROS 包构建成功。受影响四包的 pytest 用例：定位 30、矩形清洗 199、L 清洗 23、网关 66，共 318 个，通过且无跳过。此统计只包含本次受影响包，不包含工作空间其他包的历史测试结果。
- 网关新增路径/名称校验、重名保护、快照恢复、去重、模式互斥、保存失败保留会话、写入超时待确认测试。
- 真实 Cartographer 集成使用旋转雷达的合成扫描，验证 WebSocket 开始建图、断线保留会话、中文名称三件套保存、重新加载定位地图、重新进入建图再放弃退出。测试不连接硬件，使用 localhost DDS 与隔离域。
- Qt 共 11 个用例，覆盖建图/保存/列表/选图/独立预览、文字输入不触发驾驶、断线重连不重放旧命令及已有清洗流程。Linux offscreen 检查界面布局。
- 未进行 Windows 原生打包或实船建图精度、Wi-Fi 延迟与运动验收。首次实机部署需要同步 App 与 Nano 后端，并使用新的受管入口。
- 修改完成后尝试检查 Nano 雷达：`192.168.1.188:22` 重试后可连通，但 `nvidia` 的现有 SSH 认证失败；未读取目标机服务配置或扫描数据，也未修改/重启目标机服务。雷达开机自启与实时扫描状态仍待可用登录方式后核验。

### 控制器配置加载修复验证（2026-11-09）

本次修复对应源码 `9db6798`。修复前已在 Nano 确认 `/l_controller` 的 `use_map_tf=false`、`require_localization_ready=false`，启动日志提示参数文件 `.` 无效；本地用真实嵌套 launch 上下文复现了空参数覆盖默认配置路径的问题。

- 新增 `test/test_launch_config.py` 并注册到 CMake 测试：本地 8 项通过，覆盖受管/直接组合入口、默认 YAML、TF 与定位就绪参数、自定义 YAML 保留、缺失路径和目录拒绝。用修复前代码运行同组测试为 5 失败、3 通过。该组只解析 launch 和参数，不启动 ROS 节点或硬件。
- 本地既有 `test/test_bringup.py` 两项集成测试通过，使用隔离域和 localhost DDS，硬件输出关闭。开发机缺少的 `websockets` 从 Nano 复制到 `/tmp/nano-launch-config-repair/test-deps` 后通过 `PYTHONPATH` 使用，未修改系统 Python。Nano 同样运行新增配置测试，8 项通过。这些是修复时的结果，本轮文档更新未重复运行。
- 已备份并同步启动修复、测试注册和测试文件到 `/home/nvidia/surface_ws`。修复部署备份为 `backups/launch-config-fix-20261109-519932b1/`，包含变更清单及 `validation_before.json`、`validation_after.json`、`final_parameters.json`。
- 在任务空闲、桥无控制者且输出中性时，通过网关重新加载同一张 `20261108_190209` 地图，并恢复重载前的实时初始位姿；主服务和串口桥未重启。两个控制器实际加载了各自 YAML，`use_map_tf=true`、`require_localization_ready=true`、`pose_source=tf`；矩形 39 项、开放任务 41 项 YAML 参数核对一致，仅 `output_enabled` 按服务配置显式覆盖。
- 重载后连续 31 帧遥测（超过 3 秒）中，两任务均为 `IDLE`、`can_start=true`、`start_blockers=[]`，定位就绪。该次运行日志为 `log/operator_runtime/f16b7ba7c78149fc955fc446d04889c9.log`，未再出现参数文件路径警告。验证过程未发送清洗启动或运动指令。
- 修复后操作者反馈“目前测试正常”。该反馈确认现场使用恢复正常；未提供具体任务次数、全程轨迹或覆盖报告，不据此扩展为完整清洗、换边及避障验收。

复查启动配置解析可在已构建并加载 ROS 环境的工作空间执行：

```bash
python3 -m pytest -q src/f1_operator_gateway/test/test_launch_config.py
```

同日文档同步前再次只读核查，现场已切换到地图 `1109new`、会话 `d0d98cc294e546868600eccb7fd3e883`。该地图只支持矩形任务，矩形控制器两个定位参数均为 `true`，定位就绪，连续 10 帧遥测中矩形任务为 `CANCELLED` 且 `can_start=true`。开放任务未运行的原因是地图没有有效开放墙链，不是配置故障复发；此次同步保留该现场会话，不为重现旧验证而重新加载地图。
