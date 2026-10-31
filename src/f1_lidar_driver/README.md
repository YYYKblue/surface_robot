# f1_lidar_driver：项目内 PAVO2 驱动

从 Nano 当前使用的 `pavo2_ros` 2.1.2 适配源码迁入 `surface_ws`。本包直接编译厂商 SDK 和 ROS 2 节点，不需要 `/home/nvidia/ros2_ws` 的源码、库文件或环境。当前完成的是 PC 端封装；Nano 同步、构建和服务切换由操作者执行。

## 职责与兼容性

- 驱动负责连接 PAVO2 并发布 `sensor_msgs/msg/LaserScan`；不启动建图、定位、bridge 或清洗，不发布安装 TF。
- 默认话题 `/pavo2_scan`、坐标系 `pavo2s_frame`；现有 Cartographer、网关和清洗感知可继续订阅。
- 包名变为 `f1_lidar_driver`，可执行文件和节点名保留 `pavo2_scan_node`。所有驱动 C++、SDK 源文件保持与导入源一致，详见 [来源记录](vendor/PROVENANCE.md)。
- 距离/角度换算、无效回波置零、读取前取时间戳、Reliable/depth=10 QoS 和现有超时重连行为均保留。首次连接失败仍退出，自动服务继续采用原有启动等待与重试策略。
- RViz 和视场标记是可选查看工具。`lidar_view.launch.py` 仅启动查看器，不额外启动驱动。

## PC / Nano 分别构建

在各自主机的 `surface_ws` 根目录构建，不复制 PC 的 `build/install/log` 到 Nano：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-up-to f1_lidar_driver \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 pkg executables f1_lidar_driver
ros2 launch f1_lidar_driver lidar.launch.py --show-args
```

`--packages-up-to f1_mapping_bringup` 或 `--packages-up-to f1_operator_gateway` 也会构建本包；运行依赖仅负责安装/构建，不会自动启动另一个驱动。SDK 使用 C++14 和 pthread，由本机编译为内部静态库并链接进节点，无预编译 ARM64/x86 SDK 依赖。

## 参数与启动

唯一的常规参数入口为 `config/pavo2.yaml`。修改后重启驱动生效；没有新增动态参数更新功能。

| 参数 | 当前默认值 | 说明 |
|---|---|---|
| `lidar_ip` / `lidar_port` | `10.10.10.121` / `2368` | 雷达地址 |
| `local_ip` / `local_port` | `10.10.10.100` / `0` | 本机绑定地址；端口 0 由系统选择 |
| `conn_protocol` | `0` | 沿用现有 SDK 协议选择 |
| `frame_id` | `pavo2s_frame` | 与当前安装 TF 一致 |
| `topic` | `pavo2_scan` | 根命名空间下为 `/pavo2_scan` |
| `range_min` / `range_max` | `0.10` / `100.0` | 米 |
| `inverted` | `false` | 沿用现有点序方向 |
| `timeout` | `180` | SDK 读取超时，毫秒 |

`local_ip` 只控制驱动 socket 绑定，**不会配置网卡**。Nano 有线连接还需保留 `10.10.10.100/24` 静态地址。源码节点默认帧名仍是原来的 `pavo2_frame`，因此使用本包 launch/YAML，避免裸运行节点导致与项目 TF 不匹配。

没有 systemd 驱动实例时，可以前台运行：

```bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1_lidar_driver lidar.launch.py
# 使用自定义完整参数文件：
# ros2 launch f1_lidar_driver lidar.launch.py config:=/绝对路径/pavo2.yaml
```

可选查看已运行驱动（需要图形桌面）：

```bash
ros2 launch f1_lidar_driver lidar_view.launch.py
```

查看器默认针对 `/pavo2_scan` 和 `pavo2s_frame`；改话题/帧名时同步调整 RViz 配置。安装 TF 继续由建图/定位模式发布；不要在驱动内增加重复 TF。

## Nano 由操作者切换现有开机服务

2026-10-31 只读检查确认 Nano 现有 `/etc/systemd/system/pavo2-scan.service` 已启用，但仍指向旧 `ros2_ws`。其旧源码当前存在。以下步骤需要你同步本仓库到 Nano、在 Nano 构建成功后执行；本次 PC 修改没有替换或重启远端服务。

1. 在没有建图、定位或清洗作业运行时切换，先备份现有服务：

```bash
lidar_service_backup="/home/nvidia/pavo2-scan.service.before-surface-$(date +%Y%m%d_%H%M%S)"
cp /etc/systemd/system/pavo2-scan.service "$lidar_service_backup"
```

2. 核对 `deploy/pavo2-scan.service` 中的用户和 `/home/nvidia/surface_ws` 路径，以及 `config/pavo2.yaml`。模板保留原服务名称、日志位置、Restart=always 和启动前探测；探测脚本与驱动读取同一份 YAML 中的雷达 IP。
3. 替换同一个服务，避免旧、新驱动同时启动：

```bash
sudo systemctl stop pavo2-scan.service
sudo install -m 644 \
  /home/nvidia/surface_ws/install/f1_lidar_driver/share/f1_lidar_driver/deploy/pavo2-scan.service \
  /etc/systemd/system/pavo2-scan.service
sudo systemctl daemon-reload
sudo systemctl enable --now pavo2-scan.service
systemctl show pavo2-scan.service -p UnitFileState -p ActiveState -p ExecStart --no-pager
journalctl -u pavo2-scan.service -n 30 --no-pager
tail -n 30 /home/nvidia/pavo2_scan_driver.log
```

服务脚本仅加载系统 ROS 与新工作空间 `install/local_setup.bash`，不加载登录 shell 的旧 overlay。`F1_LIDAR_WORKSPACE` 指定新工作空间；可在服务中额外设置 `F1_LIDAR_CONFIG` 指向自定义 YAML。原 `pavo2s-scan.service` 保持禁用。`f1-operator.service` 与雷达服务独立运行，App 模式切换继续复用雷达。

4. 雷达供电并接线后，在加载新工作空间的终端检查：

```bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic info /pavo2_scan --verbose
ros2 run f1_mapping_bringup scan_check --samples 100
```

只构建本驱动时，扫描检查工具还需要 `colcon build --packages-up-to f1_mapping_bringup`。确认只有一个扫描发布者、帧名正确、检查工具无消息问题后，再验收 App 建图与定位。未接雷达时服务反复等待/重试是现有行为，不代表迁移已通过实机验收。

需要回退时，停止新服务、把第 1 步的备份恢复到 `/etc/systemd/system/pavo2-scan.service`，执行 `daemon-reload` 后启动。实机验收通过前保留旧 `ros2_ws`；本次不删除旧工作空间。

## PC 验证

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select f1_lidar_driver --return-code-on-test-failure
colcon test-result --test-result-base build/f1_lidar_driver --verbose
```

测试固定使用本机隔离 ROS 域 178 和 localhost DDS：检查原生产转换函数的角度、距离、强度、翻转、时间戳和 QoS，以及已安装 launch 和服务启动脚本读取自定义配置、首次连接失败退出。启动前探测使用模拟 ping，验证配置来源、成功后结束及不可达时继续交给驱动处理。连接失败测试只访问 loopback，不连接 Nano 或物理雷达。`test/cyclonedds_localhost.xml` 仅用于测试。

2026-10-31 PC 验证记录：`--packages-up-to f1_operator_gateway` 构建全部 8 个项目包成功；雷达包 9 个测试用例通过（2 个 C++、7 个 Python），现有建图包 22 个测试通过；21 个保留的驱动源码、SDK 及来源资料文件与导入记录的 SHA-256 一致。`colcon test-result` 还会计入 3 个 CTest 汇总项，显示 12 tests，不表示有 12 个独立用例。

PC x86_64 构建和模拟消息验证不能替代 Nano ARM64 构建、真实扫描精度、断线恢复或开机验收。
