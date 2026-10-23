# 简化单边沿墙控制验证

日期：2026-10-23。本记录对应本次 ±10°、40 cm 墙距、80 cm 前墙停车的简化版本，替代第一版策略说明；[第一版验证记录](VALIDATION_20261023.md) 保留作历史记录。

下述初版记录保留 15% / 80 cm 参数；首轮下水后的 **30% / 50 cm** 调整与回归结果见文末，当前参数以包内配置和 README 为准。

## 最终行为

- 夹角在 [-10°, +10°] 内，包括边界，开启 15% 滚刷推进，主电机中位。
- 超出角度范围，关滚刷，以 LEFT/RIGHT 25%、BOTH 持续控制四个主电机纠偏；每次新观测更新输出，没有脉冲/中位等待。恢复角度后立即恢复滚刷。
- 第五路独立双向控制：墙距 >42 cm 为逻辑向左 +15%，<38 cm 为向右 -15%，38–42 cm 关闭。
- 船体前缘到前墙 ≤80 cm 时关闭滚刷、侧推和主电机，结束本次单边任务。
- 前墙仍使用已有地图定位，按船头朝向选择前方短墙，支持沿长轴正反两个方向。前方观测未知不阻止启动；可用前方观测达到同一阈值也会停机。
- 不再检查转向空间、刚性最小墙距、刷毛接触/压缩、地图墙身份、定位跳变或协方差门槛。墙线有效性、数据新鲜度、命令期限、bridge 租约和遥控接管仍正常工作。
- 默认 `run_timeout_sec=0`，没有固定时长上限；可另设正值做限时试验。

## 已完成验证

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to f1_rectangle_cleaning
source install/local_setup.bash
/usr/bin/python3 -m pytest -q \
  src/f1_rectangle_cleaning/test/test_mission.py \
  src/f1_wall_cleaning/test/test_perception.py \
  src/f1_wall_cleaning/test/test_control.py
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  colcon test --packages-select f1_rectangle_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose
ros2 run f1_rectangle_cleaning rectangle_demo
```

- 5 包构建成功。
- 纯逻辑与感知/单墙控制回归：115 passed，其中矩形逻辑 53 项、感知/单墙控制 62 项。
- 矩形 ROS 集成：19 项通过，包含实际 DDS 服务、TF/topic 位姿、模拟 bridge 租约、持续转向、第五路反向与取消/失联/遥控接管撤销输出。
- `colcon test-result`：74 tests、0 errors、0 failures、0 skipped，含上述矩形包 72 项与 2 项 CTest 包装。
- 合成演示：`FOLLOW → CORRECT_HEADING → FOLLOW → STOPPED`。
- 前墙距离测试覆盖船体角度投影、仅位姿更新触发停车、长轴反向停车，以及矩形坐标与船体位姿一致旋转 180° 时的距离不变性。

首次 ROS 测试因沙箱禁止枚举网络接口而无法创建 DDS 节点；在允许本机接口访问、仅 localhost 的隔离域 178 内重跑后通过。全部反馈与 bridge 均为本机合成数据，没有打开串口或连接实机。

## 部署与实测范围

本次未同步 Nano、未启动实船输出。按 [Nano 同步和测试步骤](../../src/f1_rectangle_cleaning/DEPLOY_TEST_NANO.md) 更新源码并在 Nano 构建后运行。历史诊断文件 `rectangle_current_side.yaml` 仍保留旧 ±3° 和 8 秒限制，不用于本版。

本次没有实测滚刷推进效果、四主电机持续纠偏响应、第五路反向推力效果、地图定位精度或关闭输出后的滑行距离。80 cm 是船体前缘净空的关闭输出阈值，`STOPPED` 不表示实测停稳。

## 首轮下水后的参数调整：30% / 50 cm

用户反馈首轮测试大致达到任务效果，本次按要求调整：

- `brush_percent: 30.0`：滚刷输出提高到 30%。
- `side_percent: 30.0`：第五路向左/向右输出改为 +30% / -30%，参数校验上限相应从 20% 调整至 30%。
- `front_stop_m: 0.50`：地图和有效前方观测统一在船体前缘净空 ≤50 cm 时关闭全部输出；感知配置的停止距离同步为 0.50 m。

±10° 航向控制、38–42 cm 中心墙距范围及任务接口保持原流程。配置类默认值、安装 YAML、演示和当前操作文档均已同步。

本机验证：`colcon build --symlink-install --packages-select f1_rectangle_cleaning` 成功；56 项任务逻辑与 19 项 ROS 集成测试全部通过，含 50 cm 边界、50.1 cm/70 cm/80 cm 继续刷洗、第五路双向 30% 和组合执行器输出。已验证安装目录中的控制与感知参数可正常加载。ROS 测试使用仅 localhost 的隔离域 178 与模拟 bridge。

本次未向 Nano 同步或启动实船输出；同步后需在 Nano 构建并重启清洗 launch 才会采用新参数。
