# 简化单边沿墙控制验证

> 历史记录：两个清洗包现已合并为 `f1_pool_cleaning`；下文旧包名、命令与测试数量保留当时含义。当前操作见 [合并说明](../../src/f1_pool_cleaning/MIGRATION.md)。
>
> 更新（2026-10-26）：本文仅保留单边控制的历史验证证据，不是矩形四边自动清洁一周的验收记录。当前矩形流程见[开发与验收文档](../pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)。

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

本次未同步 Nano、未启动实船输出。按 [Nano 同步和测试步骤](../../src/f1_pool_cleaning/DEPLOY_TEST_NANO.md) 更新源码并在 Nano 构建后运行。历史诊断文件 `rectangle_current_side.yaml` 仍保留旧 ±3° 和 8 秒限制，不用于本版。

本次没有实测滚刷推进效果、四主电机持续纠偏响应、第五路反向推力效果、地图定位精度或关闭输出后的滑行距离。80 cm 是船体前缘净空的关闭输出阈值，`STOPPED` 不表示实测停稳。

## 首轮下水后的参数调整：30% / 50 cm

用户反馈首轮测试大致达到任务效果，本次按要求调整：

- `brush_percent: 30.0`：滚刷输出提高到 30%。
- `side_percent: 30.0`：第五路向左/向右输出改为 +30% / -30%，参数校验上限相应从 20% 调整至 30%。
- `front_stop_m: 0.50`：地图和有效前方观测统一在船体前缘净空 ≤50 cm 时关闭全部输出；感知配置的停止距离同步为 0.50 m。

±10° 航向控制、38–42 cm 中心墙距范围及任务接口保持原流程。配置类默认值、安装 YAML、演示和当前操作文档均已同步。

本机验证：`colcon build --symlink-install --packages-select f1_rectangle_cleaning` 成功；56 项任务逻辑与 19 项 ROS 集成测试全部通过，含 50 cm 边界、50.1 cm/70 cm/80 cm 继续刷洗、第五路双向 30% 和组合执行器输出。已验证安装目录中的控制与感知参数可正常加载。ROS 测试使用仅 localhost 的隔离域 178 与模拟 bridge。

本次未向 Nano 同步或启动实船输出；同步后需在 Nano 构建并重启清洗 launch 才会采用新参数。

## 大偏角丢墙暂停修复与左墙距离显示

用户现场状态确认 `phase=PAUSED`、`reason=left wall invalid`，这表明左墙观测失效触发了整船输出关闭。旧版代码的感知门限为墙角绝对值 ≤0.60 rad（约 34.4°）、相邻有效观测角度差 ≤0.20 rad（约 11.5°）。本机合成扫描复现了两种暂停路径：逐渐偏转至 36° 后失去墙线，以及从 0° 跳变至 20° 后关联失败。现场具体触发哪一个感知门限尚未由原始扫描确认。

最终修改：

- 矩形任务的感知配置使用 ±70° 墙角范围和 30° 相邻有效观测角度差；滚刷门控仍为 ±10°。共享配置校验支持小于 90° 的墙角上限，以便加载新配置。
- 新鲜扫描中短暂拟合失败进入 `WAIT_WALL`，用新鲜中位命令保持任务和租约，全部执行器关闭。默认最多等待 1.5 秒；有效墙线恢复后重新计算纠偏或滚刷输出。不会用旧角度在丢墙期间发转向命令。
- 显式 start 仍要求有效墙线；有效 start 后、异步取得租约时若刚好短暂丢墙，进入同一等待恢复流程。
- 持续丢墙超过窗口进入 `PAUSED`；手动暂停/取消、反馈过期或通信故障仍锁定停机。前墙阈值在等待丢墙期间仍有效。
- 状态前部新增 `left_wall_distance_cm`，显示船中心到左墙的垂直距离（厘米，1 位小数），原 `wall_distance_m` 继续保留。墙观测无效时两个距离均为 `null`。新增 `wall_valid`、`wall_reason` 和 `wall_recovery_sec`；感知原因区分无墙/拟合拒绝与关联距离/角度跳变。
- 滚刷/侧推仍为 30%，前墙阈值仍为 50 cm。

验证结果：

- `colcon build --symlink-install --packages-up-to f1_rectangle_cleaning`：5 包成功。
- `ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test --executor sequential --packages-select f1_wall_cleaning f1_rectangle_cleaning --return-code-on-test-failure`：两包成功。
- 矩形包 68 项任务逻辑、22 项 ROS 集成测试通过；共享单墙包 73 项测试通过，共 163 项独立用例。
- `colcon test-result`：矩形包 92 项、单墙包 76 项，均 0 errors、0 failures、0 skipped；总数含 5 项 CTest 包装。
- 大偏角端到端纯逻辑测试读取实际任务感知 YAML，使用雷达原外参和 280° 视场，覆盖正负 20°、40°、60°、68° 后回到 8°，验证墙线跟踪、转向指令及刷盘恢复。
- ROS 模拟租约测试覆盖短暂丢墙期间输出全关但租约保持、恢复后转向/开刷、持续丢墙超时释放租约、取得租约期间丢墙，以及有效距离显示为 40.0 cm、无效距离显示为 null。

全部为本机合成扫描、合成反馈和模拟 bridge，未连接或同步 Nano。现场纠偏响应仍需实船验证。按 [部署步骤](../../src/f1_pool_cleaning/DEPLOY_TEST_NANO.md) 同步两个清洗包、构建并重启清洗 launch 后生效。

## 后续反向纠偏代码分析与修复

针对“开刷后船头向左墙偏转，纠偏时仍继续向左墙转”的反馈，已在现有 280° 视场、近后墙矩形扫描中复现：0.50 m 最短可见墙段门槛使左墙丢失，1 秒关联超时后将后墙识别为左墙，导致角度由负变正并从 RIGHT 改发 LEFT。日志中的 36.5 cm、约 1 秒等待、恢复后 40.6 cm 可用此构造场景复现，但原日志缺失角度与实际 CTRL，不能据此认定现场唯一原因。

当前配置使用 0.30 m 最短墙段、2 秒墙关联有效期；纠偏死区独立设为 ±3°，滚刷门槛仍为 ±10°，允许开刷同时纠偏。状态前部显示墙角与方向。完整推导、复现表与测试结果见 [反向纠偏分析](HEADING_FEEDBACK_20261023.md)。两个清洗包共 169 项独立测试通过，另验证了实际 C 固件的 45 种方向/档位/轮组编码组合。仍未向 Nano 同步或进行实船验证。
