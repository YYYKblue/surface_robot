# 沿墙清洗流程图

本文档按当前仓库中的 `f1_pool_cleaning` 实现整理，描述从定位就绪、启动任务、左墙跟随，到四条边自动换边并结束的实际软件流程。图中的状态名与 `Mission.phase`、话题名和服务名保持一致。

## 总体流程

```mermaid
flowchart TD
    A[启动 pool_bringup / pool_cleaning] --> B[固定地图定位]
    B --> B1[RViz 2D Pose Estimate]
    B1 --> B2[启动 Cartographer 纯定位轨迹]
    B2 --> B3{定位门控通过?}
    B3 -- 否 --> B4[LOCALIZATION_CHECK\n不允许 start]
    B4 --> B3
    B3 -- 是 --> B5[LOCALIZATION_READY]

    L[LaserScan] --> P[Perception\n扫描有效性检查]
    P --> P1[聚类 / 角点拆分 / 鲁棒直线拟合]
    T[地图边 EdgeTarget\n期望距离与切线] --> P2{目标墙匹配?}
    P1 --> P2
    P2 -- 否 --> W0[WallObservation.valid=false\n记录拒绝原因]
    P2 -- 是 --> P3[墙线关联与 front 安全观测]
    P3 --> W1[WallObservation]

    M[map→base_link TF 或 map_pose] --> C[PoolController]
    B5 --> C
    W0 --> C
    W1 --> C
    C --> C1[构造 PoolModel\n选择 default_edge_id=edge_0]
    C1 --> C2[发布 EdgeTarget\n保持左墙身份]
    C2 --> T
    C --> S[/pool_cleaning/start/]
    S --> G{启动门控通过?}
    G -- 否 --> R[拒绝启动\n输出关闭]
    G -- 是 --> Q[获取 bridge lease\n或进入 preview]
    Q --> F[沿墙清洗状态机]
    F --> O[/command_preview\n及 live lease 输出/]
    O --> H[status JSON\nphase / edge / coverage / reason]

    X[/pause 或 /cancel/] --> E[输出关闭]
    E --> E1[PAUSED 或 CANCELLED\n不会自动续跑]
```

定位门控由 [localization.py](../../src/f1_localization/f1_localization/localization.py) 负责；墙线提取和关联由 [perception.py](../../src/f1_pool_cleaning/f1_pool_cleaning/perception.py) 负责；任务服务、数据新鲜度检查、租约和输出由 [controller_node.py](../../src/f1_pool_cleaning/f1_pool_cleaning/controller_node.py) 负责。

## 沿墙清洗与自动换边状态机

```mermaid
flowchart TD
    IDLE([IDLE]) --> START[/start/]
    START --> CHECK{定位、pose、wall\n数据流均新鲜?}
    CHECK -- 否 --> REJECT[拒绝 start\n保持输出关闭]
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
    NEXT -- 否 --> COMPLETE([COMPLETE\n完成 edge_0..edge_3\n输出关闭])
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
| 一周结束 | `PoolModel.next_edge()` 返回空时，当前边记录完成并进入 `COMPLETE`，不再转移。 |
| 输出门控 | `controller_node.py` 在每个 tick 检查数据流、定位和 lease；异常进入 `PAUSED` 并调用 `_stop_outputs()`。 |

## 关键接口

```text
/pool_cleaning/localization_status  定位门控结果
/pool_cleaning/map_pose             固定地图位姿（use_map_tf=false 时）
/pool_cleaning/wall_observation     墙线感知结果
/pool_cleaning/edge_target          当前地图边生成的目标墙上下文
/pool_cleaning/start                启动任务
/pool_cleaning/pause                暂停并关闭输出
/pool_cleaning/cancel               取消并清除边关联
/pool_cleaning/command_preview      执行器命令预览；live 模式另经 lease 发往 bridge
/pool_cleaning/status               状态、原因、边进度和覆盖区间
```

状态机的主要实现位于 [mission.py](../../src/f1_pool_cleaning/f1_pool_cleaning/mission.py)，默认参数位于 [pool_cleaning.yaml](../../src/f1_pool_cleaning/config/pool_cleaning.yaml)。

> 说明：`COMPLETE` 表示软件状态机完成 `edge_0` 到 `edge_3` 的边段任务；`cleaned_intervals` 的统计基准是左侧滚刷接触中心线，不应单独解释为池底面积覆盖率。实船漂移、角部留白和实际清洁效果仍需现场验收记录。
