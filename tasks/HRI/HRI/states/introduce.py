"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

Ported from SMACH to YASMIN.
"""

import yasmin
import yasmin_ros
from yasmin import Blackboard
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from lasr_skills.wait import Wait
from lasr_skills.say import Say

from HRI.states import ClearSeatingDetections, GetGuestData, GetIntroductionStr, Recognise
from lasr_skills.look_to_point import LookToPoint


class GetLookPoint(yasmin.State):
    """
    Builds a PointStamped from seated_guest_locs[person_index] and stores
    it in the blackboard.
    Replaces the smach.CBState that did the same in the SMACH version.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("seated_guest_locs")
        self.add_input_key("person_index")
        self.add_output_key("pointstamped")
        self.node = yasmin_ros.logger_node

    def execute(self, blackboard: Blackboard) -> str:
        index = blackboard["person_index"]
        if index < len(blackboard["seated_guest_locs"]):
            header = Header()
            header.frame_id = "map"
            look_point = PointStamped(
                header=header,
                point=blackboard["seated_guest_locs"][index],
            )
            blackboard["pointstamped"] = look_point
            yasmin.YASMIN_LOG_INFO(
                f"Look point set to: {look_point.point.x}, "
                f"{look_point.point.y}, {look_point.point.z}"
            )
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_ERROR("Index out of bounds for seated_guest_locs.")
            return "failed"


class CheckDone(yasmin.State):
    """
    Replaces the smach.Iterator exhausted_outcome logic.
    Increments person_index and loops back or exits when all guests introduced.
    """

    def __init__(self):
        super().__init__(outcomes=["continue", "done"])
        self.add_input_key("person_index")
        self.add_input_key("seated_guest_locs")
        self.add_output_key("person_index")

    def execute(self, blackboard: Blackboard) -> str:
        next_index = blackboard["person_index"] + 1
        if next_index < len(blackboard["seated_guest_locs"]):
            blackboard["person_index"] = next_index
            return "continue"
        return "done"


class ResetIndex(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded"])
        self.add_output_key("person_index")

    def execute(self, blackboard: Blackboard) -> str:
        blackboard["guest_id"] = 'guest1'
        return "succeeded"


class Introduce(yasmin.StateMachine):
    """
    State machine that introduces a guest to all other guests/host present in
    the seating area.

    Replaces smach.Iterator with a CheckDone loop pattern.

    Blackboard keys required before calling sm():
        - guest_data: Dict of all guests keyed by id
        - guest_seat_point: PointStamped of the incoming guest's seat
        - seated_guest_locs: List of Point locations of all seated guests
        - person_index: Set to 0 before calling sm()
    """

    def __init__(self, can_detect_second_guest: bool = True):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_input_key("guest_seat_point")
        self.add_input_key("seated_guest_locs")

        self.add_state(
            "RESET_INDEX",
            ResetIndex(),
            transitions={"succeeded": "GET_LOOK_POINT_1"},
        )
        # Builds look point from seated_guest_locs[person_index]
        self.add_state(
            "GET_LOOK_POINT_1",
            GetLookPoint(),
            transitions={
                "succeeded": "LOOK_TO_GUEST_1",
                "failed": "failed",
            },
        )

        self.add_state(
            "LOOK_TO_GUEST_1",
            LookToPoint(),
            transitions={
                "succeeded": "WAIT",
                "aborted": "failed",
            },
            # remappings={"look_point": "pointstamped"},
        )

        self.add_state(
            "WAIT",
            Wait(0.25),
            transitions={
                "succeeded": "RECOGNISE",
                "failed": "failed",
            },
        )

        self.add_state(
            "RECOGNISE",
            Recognise(can_detect_second_guest=can_detect_second_guest),
            transitions={
                "succeeded": "GET_GUEST_DATA_1",
                "failed": "failed",
            },
        )

        self.add_state(
            "GET_GUEST_DATA_1",
            GetGuestData(guest_to_introduce='guest2', guest_to_introduce_to='guest1'),
            transitions={
                "succeeded": "GET_INTRODUCTION_STR_1",
                "failed": "failed",
            },
        )

        self.add_state(
            "GET_INTRODUCTION_STR_1",
            GetIntroductionStr(),
            transitions={
                "succeeded": "SAY_INTRODUCTION",
                "failed": "failed",
            },
        )

        self.add_state(
            "SAY_INTRODUCTION",
            Say(),
            transitions={
                "succeeded": "LOOK_TO_GUEST_2",
                "aborted": "LOOK_TO_GUEST_2",
                "canceled": "LOOK_TO_GUEST_2",
            },
        )

        self.add_state(
            "LOOK_TO_GUEST_2",
            LookToPoint(),
            transitions={
                "succeeded": "GET_GUEST_DATA_2",
                "aborted": "failed",
                "timeout": "GET_GUEST_DATA_2",
            },
            # remappings={"pointstamped": "guest_seat_point"},
        )

        self.add_state(
            "GET_GUEST_DATA_2",
            GetGuestData(guest_to_introduce='guest1', guest_to_introduce_to='guest2'),
            transitions={
                "succeeded": "GET_INTRODUCTION_STR_2",
                "failed": "failed",
            },
        )

        self.add_state(
            "GET_INTRODUCTION_STR_2",
            GetIntroductionStr(),
            transitions={
                "succeeded": "SAY_INTRODUCTION_2",
                "failed": "failed",
            },
        )

        self.add_state(
            "SAY_INTRODUCTION_2",
            Say(),
            transitions={
                "succeeded": "CHECK_DONE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        # Replaces smach.Iterator exhausted_outcome
        self.add_state(
            "CHECK_DONE",
            CheckDone(),
            transitions={
                "continue": "GET_LOOK_POINT_1",
                "done": "CLEAR_SEATING_DETECTIONS",
            },
        )

        self.add_state(
            "CLEAR_SEATING_DETECTIONS",
            ClearSeatingDetections(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )
