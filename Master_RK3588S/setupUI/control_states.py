from enum import Enum, IntEnum


class Tc264State(IntEnum):
    STATE_IDLE = 0
    STATE_TRACK = 1
    STATE_AVOID_CAR = 2
    STATE_AVOID_HUMAN = 3
    STATE_COLLECT_GOLD = 4
    STATE_RECOVER_LINE = 5
    STATE_LINE_LOSS_SAFE_STOP = 6
    STATE_SAFE_STOP = 7


class TaskState(str, Enum):
    NORMAL_TRACK = "NORMAL_TRACK"
    AVOID_CAR = "AVOID_CAR"
    AVOID_HUMAN = "AVOID_HUMAN"
    COLLECT_GOLD = "COLLECT_GOLD"
    RECOVER_LINE = "RECOVER_LINE"
    LINE_LOSS_SAFE_STOP = "LINE_LOSS_SAFE_STOP"


class PlannerMode(str, Enum):
    TRACK_CENTER = "TRACK_CENTER"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    APPROACH_TARGET = "APPROACH_TARGET"
    HOLD_LAST = "HOLD_LAST"
    SAFE_STOP = "SAFE_STOP"


def enum_value(value):
    return value.value if isinstance(value, Enum) else value


TASK_TO_TC264_STATE = {
    TaskState.NORMAL_TRACK.value: Tc264State.STATE_TRACK,
    TaskState.AVOID_CAR.value: Tc264State.STATE_AVOID_CAR,
    TaskState.AVOID_HUMAN.value: Tc264State.STATE_AVOID_HUMAN,
    TaskState.COLLECT_GOLD.value: Tc264State.STATE_COLLECT_GOLD,
    TaskState.RECOVER_LINE.value: Tc264State.STATE_RECOVER_LINE,
    TaskState.LINE_LOSS_SAFE_STOP.value: Tc264State.STATE_LINE_LOSS_SAFE_STOP,
}


def task_state_to_tc264_state(task_state):
    key = enum_value(task_state)
    return TASK_TO_TC264_STATE.get(key, Tc264State.STATE_TRACK)
