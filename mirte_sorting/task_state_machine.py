"""
task_state_machine.py
─────────────────────
All mission states and allowed transitions.
"""

from enum import Enum, auto


class MissionState(Enum):
    # Happy path
    INIT                    = auto()
    SAVE_START_POSE         = auto()
    WAIT_FOR_NAV2           = auto()
    NAVIGATE_TO_SOURCE      = auto()
    DETECT_OBJECT           = auto()
    PICK_OBJECT             = auto()
    NAVIGATE_TO_DESTINATION = auto()
    DETECT_BIN              = auto()
    DROP_OBJECT             = auto()
    CHECK_REMAINING         = auto()
    RETURN_TO_START         = auto()
    TASK_COMPLETE           = auto()
    # Failure states
    NAVIGATION_FAILED       = auto()
    OBJECT_NOT_FOUND        = auto()
    PICK_FAILED             = auto()
    BIN_NOT_FOUND           = auto()
    DROP_FAILED             = auto()
    RECOVERY                = auto()
    ABORT                   = auto()


TRANSITIONS: dict[MissionState, list[MissionState]] = {
    MissionState.INIT:                    [MissionState.SAVE_START_POSE],
    MissionState.SAVE_START_POSE:         [MissionState.WAIT_FOR_NAV2],
    MissionState.WAIT_FOR_NAV2:           [MissionState.NAVIGATE_TO_SOURCE],
    MissionState.NAVIGATE_TO_SOURCE:      [MissionState.DETECT_OBJECT,
                                           MissionState.NAVIGATION_FAILED],
    MissionState.DETECT_OBJECT:           [MissionState.PICK_OBJECT,
                                           MissionState.OBJECT_NOT_FOUND],
    MissionState.PICK_OBJECT:             [MissionState.NAVIGATE_TO_DESTINATION,
                                           MissionState.PICK_FAILED],
    MissionState.NAVIGATE_TO_DESTINATION: [MissionState.DETECT_BIN,
                                           MissionState.NAVIGATION_FAILED],
    MissionState.DETECT_BIN:              [MissionState.DROP_OBJECT,
                                           MissionState.BIN_NOT_FOUND],
    MissionState.DROP_OBJECT:             [MissionState.CHECK_REMAINING,
                                           MissionState.DROP_FAILED],
    MissionState.CHECK_REMAINING:         [MissionState.NAVIGATE_TO_SOURCE,
                                           MissionState.RETURN_TO_START],
    MissionState.RETURN_TO_START:         [MissionState.TASK_COMPLETE,
                                           MissionState.NAVIGATION_FAILED],
    MissionState.TASK_COMPLETE:           [],
    MissionState.NAVIGATION_FAILED:       [MissionState.RECOVERY,
                                           MissionState.ABORT],
    MissionState.OBJECT_NOT_FOUND:        [MissionState.NAVIGATE_TO_SOURCE,
                                           MissionState.ABORT],
    MissionState.PICK_FAILED:             [MissionState.DETECT_OBJECT,
                                           MissionState.ABORT],
    MissionState.BIN_NOT_FOUND:           [MissionState.NAVIGATE_TO_DESTINATION,
                                           MissionState.ABORT],
    MissionState.DROP_FAILED:             [MissionState.DETECT_BIN,
                                           MissionState.ABORT],
    MissionState.RECOVERY:                [MissionState.NAVIGATE_TO_SOURCE,
                                           MissionState.NAVIGATE_TO_DESTINATION,
                                           MissionState.RETURN_TO_START,
                                           MissionState.ABORT],
    MissionState.ABORT:                   [],
}


class StateMachine:
    def __init__(self):
        self.state = MissionState.INIT
        self._retries: dict[MissionState, int] = {}

    def transition(self, new_state: MissionState) -> bool:
        if new_state not in TRANSITIONS.get(self.state, []):
            return False
        self.state = new_state
        return True

    def force(self, new_state: MissionState):
        self.state = new_state

    def increment_retries(self) -> int:
        n = self._retries.get(self.state, 0) + 1
        self._retries[self.state] = n
        return n

    def reset_retries(self):
        self._retries[self.state] = 0

    def retries(self) -> int:
        return self._retries.get(self.state, 0)

    def is_done(self) -> bool:
        return self.state in (MissionState.TASK_COMPLETE, MissionState.ABORT)

    def __repr__(self):
        return f"StateMachine(state={self.state.name})"
