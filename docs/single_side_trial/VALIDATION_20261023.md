# 第一版单边策略：本机验证记录

日期：2026-10-23。修改范围：`src/f1_rectangle_cleaning`。修改前源码备份位于本机 `/tmp/f1_rectangle_cleaning_before_single_side_20261023`（临时目录，不作为长期备份）。

## 已通过

- `colcon build --symlink-install --packages-up-to f1_rectangle_cleaning`：5 包构建成功。
- `colcon test --packages-select f1_rectangle_cleaning --return-code-on-test-failure`：成功。
- `colcon test-result --test-result-base build/f1_rectangle_cleaning --verbose`：58 tests、0 errors、0 failures、0 skipped；含 42 项纯逻辑、14 项 ROS 测试及 2 项 CTest 包装。
- 合成演示：`FOLLOW → CORRECT_HEADING → NEUTRAL_DWELL → CORRECT_HEADING → NEUTRAL_DWELL → FOLLOW → STOPPED`，未进入换边流程。
- 冻结地图启动冒烟验证：隔离 localhost 域加载原始 `lidar_20261021_145054.pbstream`，`get_trajectory_states` 返回 trajectory 0 为 FROZEN；显式调用 `start_trajectory` 返回 Success，创建 trajectory 1。发现并修复 gflags 布尔参数需使用 `=false` 的传参问题。

## 测试覆盖

刷盘 15% 与主轮 STOP；第五路 +15% 滞回严格边界和历史保持；±3° 刷盘门控；正/负夹角 LEFT/RIGHT；固定 0.1 s 脉冲截止时间；至少 0.5 s 主电机中位；角度回归/变号提前截断；转动空间不足暂停；盲船头下地图 0.80 m 停车；船体转角包络；仅位姿更新也能停车；可用前方障碍证据；过期/重复/未来反馈；地图墙身份和航向一致性；任务超时；暂停不自动恢复；TF 原时间戳与跳变；模拟 bridge 组合输出、token 隔离、RC 接管、epoch 变化、wall/pose 与 bridge 反馈中断、取消、迟到租约释放。

DDS 测试仅使用合成输入与模拟 bridge；未打开串口或连接 Nano。沙箱内网络接口枚举受限，DDS 验证在允许本机接口访问的隔离 localhost 域运行。

## 尚需实机验证

本次没有实测固定地图定位精度、重定位是否正确区分 A 墙、15% 刷盘推进效果、刷毛接触情况、第五路引起的偏航、0.1 s 纠偏实际转角、关闭输出后的滑行距离。定位冒烟验证不等于有真实扫描的定位精度验证。

原地图矩形参数是估计值；TF 无协方差；前方盲区新增障碍不可检测。80 cm 是关闭输出触发阈值，STOPPED 不等于船体停稳。按用户选择保留雷达纠偏空间检查，空间未知时暂停，人工摆正后 cancel/start。
