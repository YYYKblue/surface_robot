# 两个清洗包合并为 f1_pool_cleaning

2026-10-23：`f1_rectangle_cleaning` 的当前单边控制和 `f1_wall_cleaning` 的墙线感知合并到本包。旧脉冲控制器、旧试跑配置及其专用测试删除；保留感知、连续纠偏、双向侧推、丢墙恢复、定位、租约和对应回归测试。合并当时尚未新增自动换边或四边清洗；当前版本已在此基础上实现矩形四边自动清洁一周，详见[开发与验收文档](../../docs/pool_cleaning/DEVELOPMENT_AND_ACCEPTANCE.md)。

## 入口变化

| 原入口 | 新入口 |
|---|---|
| 两个旧功能包 | `f1_pool_cleaning` |
| `rectangle_bringup.launch.py` | `pool_bringup.launch.py` |
| `rectangle_cleaning.launch.py` | `pool_cleaning.launch.py` |
| `rectangle_localization.launch.py` | `pool_localization.launch.py` |
| `rectangle_nano` / `rectangle_demo` | `pool_nano` / `pool_demo` |
| `rectangle_controller` / `rectangle_perception` | `pool_controller` / `pool_perception` |
| `rectangle_preview.yaml` | `pool_cleaning.yaml` |
| `rectangle_localization.lua` | `pool_localization.lua` |
| `/rectangle_cleaning/*` | `/pool_cleaning/*` |
| `/wall_cleaning/wall_observation`、`wall_marker`、`reset_observation` | `/pool_cleaning/` 下的同名接口 |

不保留旧包或旧控制入口。自定义 YAML 顶层节点名必须同步更新；原控制参数名和数值不变。感知参数中的 `brush_allowed` 及旧脉冲/试跑参数已移除。话题、服务、RViz 配置、录包命令和外部订阅者也需更新；历史录包中的名称不自动变化。

`f1_interfaces` 中的消息/服务定义不变。当前预览仍为 `ActuatorCommand`，状态仍为 JSON `String`；旧 `EffortPreview` / `CleaningStatus` 类型保留兼容，但新包不发布。硬件桥、固件与建图入口不变。

## 首次升级 PC 或 Nano

先退出旧清洗节点及相关构建/测试。在没有加载旧工作空间安装环境的新终端中进入实际工作空间根目录。下面适用于本项目默认的隔离安装布局，将遗留文件移到工作空间之外保留备份：

```bash
cd ~/surface_ws
cleaning_backup="$(mktemp -d /tmp/f1_cleaning_migration.XXXXXX)"
for package in f1_rectangle_cleaning f1_wall_cleaning; do
  for tree in src build install; do
    if [ -e "$tree/$package" ]; then
      mkdir -p "$cleaning_backup/$tree"
      mv "$tree/$package" "$cleaning_backup/$tree/"
    fi
  done
done
echo "旧包备份：$cleaning_backup"
```

PC 工作空间为 `/home/bboss/new-nano/surface_ws`，替换上面的 `cd`。如果通过版本更新已经删除旧源码，仍需检查 build/install。普通 rsync 新包不会删除 Nano 遗留的两个旧包；不要把备份放回 `src/`，否则 colcon 仍会发现旧包。

同步新源码后，重新构建整个工作空间以刷新安装环境：

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 pkg prefix f1_pool_cleaning
ros2 pkg executables f1_pool_cleaning
ros2 run f1_pool_cleaning pool_demo
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon test \
  --executor sequential --packages-select f1_pool_cleaning --return-code-on-test-failure
colcon test-result --test-result-base build/f1_pool_cleaning --verbose
```

仓库应发现四个包：`f1_interfaces`、`f1_mapping_bringup`、`f1_nano_bridge`、`f1_pool_cleaning`。额外安装的雷达驱动另计。

默认预览（需要外部雷达与定位输入，但不连接硬件桥输出）：

```bash
ros2 launch f1_pool_cleaning pool_cleaning.launch.py
ros2 topic echo /pool_cleaning/status --once --full-length
```

Nano 实机统一入口改为 `bash ~/surface_ws/src/f1_pool_cleaning/scripts/pool_nano`，完整步骤见 [部署说明](DEPLOY_TEST_NANO.md)。该快捷脚本沿用原有硬件许可行为，但任务仍等待显式 `start`。所有旧清洗进程退出后再运行新入口。
