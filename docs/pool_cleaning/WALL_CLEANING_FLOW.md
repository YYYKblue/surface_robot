# 沿墙清洗流程图

核对：2026-10-31。带地图任务使用 `AutonomousMission` 射线选墙及简单靠近，随后复用 `Mission` 沿墙与相邻换边。没有路径搜索、地图足迹或雷达环境碰撞门控，靠近阶段不绕障；定位、控制租约、通信期限、停止和取消仍生效。

## 总体流程

```mermaid
flowchart TD
    LOC[固定地图定位就绪] --> START[开始新任务]
    START --> RAY[沿船体 +y 射线选择第一面有限墙段]
    RAY --> HIT{命中任务墙?}
    HIT -- 否 --> FAIL[拒绝启动，调整朝向后重试]
    HIT -- 是 --> LOCK[锁定起始墙和船中心最近投影点]
    LOCK --> LEASE[取得控制权，保持目标锁定]
    LEASE --> RANGE{已在合理起始范围?}
    RANGE -- 否 --> TURN[关刷旋转对齐清洗方向]
    TURN --> ALONG[前进或后退调整沿墙位置]
    ALONG --> SIDE[左右平移进入起始范围]
    SIDE --> RANGE
    RANGE -- 是 --> WALL[等待指定墙的新鲜有效观测]
    WALL --> OLD[旧版墙距调整、纠偏、沿墙清洗]
    OLD --> TYPE{路线类型}
    TYPE -- 矩形 --> RECT[相邻右转，顺时针清洗后续三边]
    RECT --> PREFIX[若从边中部开始，返回起始边补前段]
    PREFIX --> DONE[COMPLETE，记录覆盖缺口]
    TYPE -- L形 --> L[沿当前段及后续段到 C]
    L --> DONE
```

角点使用 1 cm 容差，按规定清洗方向与船头夹角较小者选择，相同则按边 ID 排序。无交点不延长墙、不回退最近墙。`specified/default_edge_id` 和 `set_edge` 仍可覆盖射线选择。

起始参考位置采用船中心到有限墙段的最近点，并按入口/末端余量修正。默认到达范围为沿墙 ±10 cm、航向 ±20°、距墙段不超过 85 cm，墙在左侧、船在水侧，且未进入前墙 30 cm 停车区。连续满足 0.2 s 后确认墙面；已在范围内不先退到 60 cm。靠近期间只消费位姿反馈，确认墙面后才要求墙观测新鲜。

矩形记录实际首次有效开刷位置，绕行后补齐其前段并重叠 10 cm；不搜索其他缺口的补洗路线。L 形从 AB 中间开始只清后续 AB/BC，从 BC 开始只清后续 BC，到 C 停止，不返回也不补洗。覆盖统计只记录连续有效开刷进度；路线完成不保证全墙已覆盖。

Windows App 显示起始墙、参考点、紫色起始范围、当前动作和矩形补前段阶段。`expected_track.navigation_path` 为空。`completion_scope` 区分矩形路线与本次剩余 L 路线，`coverage_complete` 和 `remaining_contact_intervals` 分别报告覆盖结果。

## 沿墙清洗与自动换边状态机

下图保留旧版沿墙和相邻换边控制。矩形外层按四边顺时针编排，必要时返回起始边补前段；L 形只沿选中边之后的路线到终点。路线执行完成与覆盖记录无缺口是独立字段。

```mermaid
flowchart TD
    READY[进入起始范围并确认墙面\n或相邻换边后重新靠墙] --> CHECK{定位、pose、所需 wall\n数据流均新鲜?}
    CHECK -- 否 --> REJECT[暂停\n保持输出关闭]
    CHECK -- 是 --> SELECT{当前边可用?}
    SELECT -- 否 --> REJECT
    SELECT -- 是 --> AW[APPROACH_WALL\n调整到左墙 0.32–0.36 m]

    AW --> D{中心到左墙\n是否在 0.32–0.36 m?}
    D -- 否 --> SIDE[第五路侧推\n>0.36 m: +30%\n<0.32 m: -30%\n滚刷关闭]
    SIDE --> AW
    D -- 是 --> AE[APPROACH_EDGE\n保持墙距，驶向当前边入口]

    AE --> FRONT
    FRONT{当前边前方\n中心距离 <= 0.30 m?}
    FRONT -- 是 --> ESTOP[EDGE_STOP\n提交停止指令周期\n记录当前边清洗区间]
    FRONT -- 否 --> ENTRY{edge progress >= entry_s?}
    ENTRY -- 否 --> AE
    ENTRY -- 是 --> WALL{本帧墙线有效?}

    WALL -- 否 --> WAIT[WAIT_WALL\n中位输出，保持任务/租约]
    WALL -- 是 --> DIST{墙距是否在\n0.32–0.36 m?}
    DIST -- 否 --> AW
    DIST -- 是 --> HEADING{航向误差处于哪个区间?}
    HEADING -- 绝对角度 ≤ 3° --> FOLLOW[FOLLOW\n滚刷 30%\n第五路关闭]
    HEADING -- 3° < 绝对角度 ≤ 10° --> CORRECT1[CORRECT_HEADING\n纠偏 + 滚刷 30%]
    HEADING -- 绝对角度 > 10° --> CORRECT2[CORRECT_HEADING\n纠偏，滚刷关闭]
    FOLLOW --> FRONT
    CORRECT1 --> FRONT
    CORRECT2 --> FRONT

    WAIT --> RECOVER{1.5 s 内墙线恢复?}
    RECOVER -- 是 --> FRONT
    RECOVER -- 否 --> PAUSE[PAUSED\n左墙持续丢失，输出关闭]

    ESTOP --> NEXT{存在下一条边?}
    NEXT -- 否 --> COMPLETE([COMPLETE\n本次路线执行结束])
    NEXT -- 是 --> MI[MOVE_INWARD\n0.5 s: reverse + 向右侧推\n滚刷关闭]
    MI --> TURN[TURN_TO_NEXT_EDGE\n只按地图航向反馈右转]
    TURN --> YAW{目标航向误差\n<= ±20°?}
    YAW -- 否 --> TURN
    YAW -- 是 --> AN[APPROACH_NEXT_EDGE\n切换 edge_target\n清除旧墙关联]
    AN --> AW2[按新边重新靠墙]
    AW2 --> D

    WATCH[任意活动状态] --> FAULT{pose/wall/定位/租约/时钟/\n走廊/进度检查失败?}
    FAULT -- 是 --> PAUSE
    FAULT -- 否 --> WATCH
    PAUSE --> CANCEL[/cancel/]
    CANCEL --> IDLE2([CANCELLED\n清除边关联后可重新 start])
```

### 控制规则与代码对应

| 流程节点 | 当前实现 |
| --- | --- |
| 左墙距离控制 | `Mission._update_edge()`：墙距大于 `0.36 m` 时第五路 `+30%`，小于 `0.32 m` 时 `-30%`，在区间内关闭第五路。 |
| 航向纠偏 | `Mission._update_edge()`：墙角绝对值超过 `3°` 进入 `CORRECT_HEADING`；正角发 `direction=3`，负角发 `direction=4`。 |
| 滚刷门控 | 航向绝对值不超过 `10°` 且墙距在稳定区间时开刷 `30%`；纠偏和滚刷可以同时工作。 |
| 丢墙恢复 | `WAIT_WALL` 最多等待 `1.5 s`；期间输出中位命令，超时转 `PAUSED`。 |
| 端点判断 | 使用地图边投影的船中心到边端距离；小于等于 `0.30 m` 进入 `EDGE_STOP`。雷达前方观测只作为辅助安全信息。 |
| 自动换边 | `EDGE_STOP → MOVE_INWARD → TURN_TO_NEXT_EDGE → APPROACH_NEXT_EDGE`；换边期间旧墙目标无效，转弯完成后重新关联新墙。 |
| 整场完成 | `AutonomousMission` 编排矩形一圈加起始边前段，或 L 形后续路线。`COMPLETE` 表示路线执行结束，覆盖缺口单独报告，不追加路径搜索。 |
| 输出门控 | `controller_node.py` 在每个 tick 检查数据流、定位和 lease；异常进入 `PAUSED` 并调用 `_stop_outputs()`。 |

## 关键接口

```text
/pool_cleaning/localization_status  定位门控结果
/pool_cleaning/map_pose             固定地图位姿（use_map_tf=false 时）
/pool_cleaning/wall_observation     墙线感知结果
/pool_cleaning/edge_target          当前地图边生成的目标墙上下文
/pool_cleaning/start                启动任务
/pool_cleaning/pause                暂停并关闭输出
/pool_cleaning/stop                 停止并保留进度快照；再次 start 是新任务
/pool_cleaning/cancel               取消并清除边关联
/pool_cleaning/command_preview      执行器命令预览；live 模式另经 lease 发往 bridge
/pool_cleaning/status               状态、原因、边进度和覆盖区间
```

状态机的主要实现位于 [mission.py](../../src/f1_pool_cleaning/f1_pool_cleaning/mission.py)，默认参数位于 [pool_cleaning.yaml](../../src/f1_pool_cleaning/config/pool_cleaning.yaml)。

组合入口的定位状态为 `/f1_localization/localization_status`；L 任务的控制接口为 `/l_cleaning/*`。`stop/cancel` 均不支持断点续跑，App 的 `manual_stop` 仅停止手动会话。

> 说明：`COMPLETE` 表示本次路线执行结束；`coverage_complete` 单独表示记录的要求区间是否满足覆盖容差；`cleaned_intervals` 的统计基准是左侧滚刷接触中心线，不应单独解释为池底面积覆盖率。实船漂移、角部留白和实际清洁效果仍需现场验收记录。
