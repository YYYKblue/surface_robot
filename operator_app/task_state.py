"""UI labels and derived task state; no robot control decisions live here."""
PHASES = {
    "IDLE": "待启动", "CANCELLED": "已取消", "STOPPED": "任务已停止（重新开始将创建新任务）", "PAUSED": "已暂停（取消后可重新开始）",
    "COMPLETE": "任务完成", "ACQUIRING": "正在获取控制权", "PLANNING": "正在规划路线",
    "NAVIGATING": "正在导航到清洗起点", "SETTLING": "正在调整姿态", "ACQUIRE_WALL": "正在寻找左墙",
    "DOCK_WALL": "正在靠墙对齐", "APPROACH_EDGE": "正在接近清洗边",
    "APPROACH_WALL": "正在靠墙", "APPROACH_NEXT_EDGE": "正在靠近下一面墙", "FOLLOW": "沿墙清洗中",
    "CORRECT_HEADING": "正在调整航向", "WAIT_WALL": "等待墙面重新识别",
    "EDGE_STOP": "当前边结束，正在停止输出", "MOVE_INWARD": "正在后退侧移",
    "TURN_TO_NEXT_EDGE": "正在转向下一条边",
}
TASK_NAMES = {"l": "L 清洗", "rectangle": "矩形清洗"}
BLOCKERS = {
    "task_stale": "任务不可用或状态过期", "task_service": "任务服务不可用",
    "localization": "定位尚未就绪", "feedback": "反馈数据未就绪", "pose": "位姿数据未就绪",
    "scan": "雷达数据未就绪", "bridge": "硬件控制条件未满足", "bridge_service": "控制权服务不可用",
    "map": "地图未匹配", "clock": "控制器时钟异常", "phase": "任务运行中或需要先取消暂停任务",
    "task_type": "任务类型未确认", "schema": "控制器需要升级", "busy": "已有任务运行或状态待确认",
    "pending": "操作处理中",
}


def blocker_text(items):
    return "\n".join(dict.fromkeys(BLOCKERS.get(item.get("code"), item.get("message", "条件未满足")) for item in items))
