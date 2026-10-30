# f1_operator_gateway

更新：2026-10-30。Windows Qt App 的 Nano 侧 WebSocket 入口，提供手动控制、双任务操作、初始位姿服务和状态转发。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/surface_ws
colcon build --symlink-install --packages-up-to f1_operator_gateway
source install/local_setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

包的运行依赖包含组合启动需要的 bridge、定位和两个清洗包，`--packages-up-to` 会构建这些依赖。网关运行需要 `python3-websockets`，通过 rosdep 或 `sudo apt install python3-websockets` 安装。Windows 依赖见 [App 说明](../../operator_app/README.md)。

## 推荐：一个入口启动 App 清洗系统

先运行雷达驱动，并同步同名 `.pbstream`、`.yaml`、`.pgm`。以下默认只预览，不输出硬件动作：

```bash
ros2 launch f1_operator_gateway operator_bringup.launch.py \
  map:="$PWD/maps/现场地图.pbstream" bind_address:=0.0.0.0
```

该入口只启动一套 Cartographer/定位管理器、一套 bridge、一个网关，并启动地图几何校验通过的清洗控制器。定位统一使用 `/f1_localization/*`，任务分别使用 `/pool_cleaning/*`、`/l_cleaning/*`。不能与已有完整 bringup 重复启动。

- 地图不支持某种任务时，不启动该控制器，并把具体推导错误传给 App；另一种可用任务仍可运行。
- 不会在矩形地图上任意猜选两条边作为 L 路线。当前 L 推导存在歧义时不可用。
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

**不再重映射 `/pool_cleaning/status/start/pause/cancel` 到 L 接口。** 网关同时管理两种任务，旧重映射命令会混淆任务类型。`localization_prefix` 只选择共享定位管理器的接口；若手动组合两个控制器，应给它们设置相同的 `localization_status_topic`。

原有矩形与 L bringup 独立使用仍兼容各自的定位前缀。

## 状态与控制语义

| 接口 | 用途 |
|---|---|
| `/f1/actuator_command`、`acquire_control`、`release_control` | 手动控制与单租约 |
| `/f1/control_state` | 控制源、通信、执行器许可 |
| `/map`、TF、`/pavo2_scan` | 地图、船体位姿、按扫描时间转换的地图雷达点 |
| `/pool_cleaning/status`、`/l_cleaning/status` | 两种任务的独立状态 |
| 两组任务的 `start/pause/cancel` | 任务操作 |
| `${localization_prefix}/set_initial_pose` | 提交 map 坐标系初始位姿 |
| `${localization_prefix}/localization_status` | 定位诊断、稳定窗口与就绪状态 |

控制器状态新增 `task_type`、`map_id`、`stamp_ns`、`run_id`、`completed_at_ns`、`can_start`、`start_blockers`。启动检查复用于自动任务的 `/start` 和状态发布；在获取租约后仍复查定位/反馈。规划和找墙发生在开始以后，不把贴墙条件强加到自动启动前。

网关检查定位/任务的接收时间和 ROS 源时间戳（0.6 秒），校验地图与任务类型。某任务状态过期不会把已知运行任务视为结束。启动请求串行处理；先停止手动会话，等待待返回的手动申请/释放和控制器状态更新，再发 start。等待超过 3 秒但尚未发送时放弃启动；已发出的操作超时保持结果待确认，不自动重发。开始服务成功后，还需收到新的 `run_id` 状态才确认启动。启动过程中收到暂停/取消会按时序处理，防止迟到的启动响应覆盖停止意图。

初始位姿服务成功只表示请求已接受；新初始位姿确认前不复用旧定位就绪状态。任务运行期间禁止网关发布新初始位姿。RViz/其他 ROS 客户端仍可直接操作 ROS 服务，控制器原有定位失效停机检查保留。

网关明确拒绝自动任务运行/操作处理中新的手动输入；bridge 单租约继续承担硬件互斥。多客户端可观察遥测，首次控制操作绑定活动端，其他端控制请求被拒绝。

`manual_stop` 只停止网关手动会话，不调用自动任务暂停/取消。App 已将按钮改为“停止手动输出”。App 断线不会主动取消自动任务；待发送的启动交接会撤销，已发送的任务操作仍需根据机器人状态确认。

## WebSocket 协议

协议名仍为 `f1-operator-v1`；`hello` 新增 `client_id` 和 `capabilities`。手动消息字段与原版本一致。

```json
{"op":"task","task_type":"l","action":"start","request_id":"唯一请求编号"}
{"op":"initial_pose","x":1.2,"y":3.4,"yaw":1.57,"map_revision":1,"request_id":"另一请求编号"}
```

`task_type` 为 `rectangle/l`；`action` 为 `start/pause/cancel`。位姿使用米、弧度，yaw 在 [-π, π]；`map_revision` 必须匹配当前地图。网关生成 ROS 时间戳、平面四元数和默认协方差。

`reply` 回传 `request_id`、`success`、`message`，可能带 `pending=true`（尚未确认）。网关缓存每客户端最近的请求结果，重复编号不重复执行；App 不自动重试操作。

`telemetry` 包含 `state/pose/scan/tasks/localization/active_tasks/task_busy/operation_pending/can_set_initial_pose/active_client`。`tasks` 分别保存 `rectangle/l` 及其新鲜度和启动条件；`scan` 是 `frame_id=map` 的点列表，缺少对应时间 TF 或扫描过期时为 null。保留 `task` 字段作为矩形旧版状态兼容入口。

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

- `colcon build --packages-up-to f1_operator_gateway`：七个包构建成功。
- 四个受影响 ROS 包通过 CTest：定位 30、矩形清洗 193、L 清洗 22、网关 37 个 pytest 用例，共 282 个，零失败；使用上述仅限本机的 DDS 配置。
- Qt 无显示环境测试 6 个通过，覆盖旋转地图坐标转换、初始朝向拖拽、命令队列、定位门控、完成展示及遥测过期。
- 本地真实地图 `lidar_20261026_170703.pbstream` 的完整组合启动检查通过：一套 Cartographer/定位、一套 bridge、L 任务和网关；地图尺寸 1054×863，WebSocket 初始位姿经真实定位服务接受。没有扫描输入时定位始终未就绪，开始条件保持关闭；所有硬件输出禁用，退出正常。
- 未进行 Windows 原生打包、跨机 Wi-Fi 或实船验收；本地真实地图检查不证明定位精度和实际清洗效果。
