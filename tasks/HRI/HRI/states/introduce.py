"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

Ported from SMACH to YASMIN.
"""

import yasmin
import yasmin_ros
from shapely.geometry import Polygon as ShapelyPolygon

import numpy as np

from lasr_skills import (
    Say,
    Detect3DInArea,
    PlayMotion,
    Wait,
    LookToPoint,
)

from geometry_msgs.msg import Point, PointStamped, Pose
from std_msgs.msg import Header

from HRI.states import (
    ClearSeatingDetections,
    GetGuestData,
    GetIntroductionStr,
    Recognise,
)


class Introduce(yasmin.StateMachine):
    """
    State machine that introduces a guest to all other guests/host present in
    the seating area.

    Replaces smach.Iterator with a CheckDone loop pattern.

    Blackboard keys required before calling sm():
        - guest_data: Dict of all guests keyed by id
        - seated_guest_locs: List of Point locations of all seated guests
        - person_index: Set to 0 before calling sm()
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")

        self._node = yasmin_ros.logger_node
        self.flag = True

        self.sofa_area = ShapelyPolygon(
            [
                np.array(self._node.get_parameter("sofa_area.top_left").value),
                np.array(self._node.get_parameter("sofa_area.top_right").value),
                np.array(self._node.get_parameter("sofa_area.bottom_right").value),
                np.array(self._node.get_parameter("sofa_area.bottom_left").value),
            ]
        )

        loop_state = yasmin.CbState(
            outcomes=["succeeded", "continue", "failed"],
            callback=self._loop_person_index,
        )
        loop_state.add_input_key("person_index")
        loop_state.add_input_key("guest_data")
        loop_state.add_input_key("introduce_detections")
        loop_state.add_output_key("person_index")
        loop_state.add_output_key("person_point_stamped")

        guest_loop = yasmin.CbState(
            outcomes=["succeeded", "continue"], callback=self._loop_guest
        )
        guest_loop.add_input_key("guest_data")
        guest_loop.add_output_key("guest_data")
        guest_loop.add_output_key("guest_point_stamped")
        guest_loop.add_output_key("introduce_to")
        guest_loop.add_output_key("relevant_guest_data")

        fallback_loop = yasmin.CbState(
            outcomes=["succeeded", "continue"], callback=self._loop_str
        )

        fallback_loop.add_input_key("guest_data")
        fallback_loop.add_output_key("introduce_to")
        fallback_loop.add_output_key("relevant_guest_data")

        self.add_state(
            "RESET_SEATING_DETECTIONS",
            ClearSeatingDetections(),
            transitions={"succeeded": "DETECT_PEOPLE", "failed": "failed"},
        )

        self.add_state(
            "DETECT_PEOPLE",
            Detect3DInArea(
                area_polygon=self.sofa_area, filter=["person"], z_min=-10, z_max=10
            ),
            transitions={"succeeded": "SAY_LOOK_AT_ME", "failed": "failed"},
            remappings={"detections_3d": "introduce_detections"},
        )

        self.add_state(
            "SAY_LOOK_AT_ME",
            Say(text="Please look at me, for the introduction."),
            transitions={
                "succeeded": "LOOP_PERSON_STATE",
                "aborted": "LOOP_PERSON_STATE",
                "canceled": "LOOP_PERSON_STATE",
            },
        )

        self.add_state(
            "LOOP_PERSON_STATE",
            loop_state,
            transitions={
                "succeeded": "GRAB_GUEST_POINT",
                "continue": "LOOK_AT_PERSON",
                "failed": "FALLBACK_SPEECH",
            },
        )

        self.add_state(
            "LOOK_AT_PERSON",
            LookToPoint(),
            transitions={
                "succeeded": "WAIT",
                "aborted": "WAIT",
                "canceled": "failed",
                "timeout": "WAIT",
            },
            remappings={"pointstamped": "person_point_stamped"},
        )

        self.add_state(
            "WAIT", Wait(2), transitions={"succeeded": "RECOGNISE", "failed": "failed"}
        )

        self.add_state(
            "RECOGNISE",
            Recognise(),
            transitions={
                "succeeded": "RESET_HEAD_1",
                "aborted": "RESET_HEAD_1",
                "no_detections": "RESET_HEAD_1",
            },
        )

        self.add_state(
            "RESET_HEAD_1",
            PlayMotion("look_centre"),
            transitions={
                "succeeded": "LOOP_PERSON_STATE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "FALLBACK_SPEECH",
            fallback_loop,
            transitions={"succeeded": "succeeded", "continue": "GET_FALLBACK_STR"},
        )

        self.add_state(
            "GET_FALLBACK_STR",
            GetIntroductionStr(),
            transitions={"succeeded": "SAY_FALLBACK", "failed": "failed"},
        )

        self.add_state(
            "SAY_FALLBACK",
            Say(),
            transitions={
                "succeeded": "FALLBACK_SPEECH",
                "aborted": "FALLBACK_SPEECH",
                "canceled": "FALLBACK_SPEECH",
            },
        )

        self.add_state(
            "GRAB_GUEST_POINT",
            guest_loop,
            transitions={"succeeded": "succeeded", "continue": "GET_INTRODUCTION_STR"},
        )

        self.add_state(
            "GET_INTRODUCTION_STR",
            GetIntroductionStr(),
            transitions={"succeeded": "LOOK_AT_GUEST", "failed": "failed"},
        )

        self.add_state(
            "LOOK_AT_GUEST",
            LookToPoint(),
            transitions={
                "succeeded": "SAY_INTRODUCTION",
                "aborted": "SAY_INTRODUCTION",
                "canceled": "failed",
                "timeout": "SAY_INTRODUCTION",
            },
            remappings={"pointstamped": "guest_point_stamped"},
        )

        self.add_state(
            "SAY_INTRODUCTION",
            Say(),
            transitions={
                "succeeded": "RESET_HEAD_2",
                "aborted": "RESET_HEAD_2",
                "canceled": "RESET_HEAD_2",
            },
        )

        self.add_state(
            "RESET_HEAD_2",
            PlayMotion("look_centre"),
            transitions={
                "succeeded": "GRAB_GUEST_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def _loop_person_index(self, blackboard):
        try:
            guest1point = blackboard["guest_data"]["guest1"]["seated_point"]
            guest2point = blackboard["guest_data"]["guest2"]["seated_point"]
            people_det = len(blackboard["introduce_detections"])
            index = blackboard["person_index"]

            yasmin.YASMIN_LOG_INFO(str(index))
            yasmin.YASMIN_LOG_INFO("Guest1 point: " + str(guest1point))
            yasmin.YASMIN_LOG_INFO("Guest2 point: " + str(guest2point))
            yasmin.YASMIN_LOG_INFO("Total people: " + str(people_det))

            if guest1point is not None and guest2point is not None:
                return "succeeded"
            elif index < people_det:
                point = blackboard["introduce_detections"][index].point
                point_stamped = PointStamped(header=Header(frame_id="map"), point=point)
                blackboard["person_point_stamped"] = point_stamped
                index += 1
                blackboard["person_index"] = index
                return "continue"

            return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_INFO(f"An error has occured with loop_person_index: {e}")
            return "failed"

    def _loop_str(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Flag is: " + str(self.flag))
        if self.flag and isinstance(self.flag, bool):
            blackboard["introduce_to"] = blackboard["guest_data"]["guest1"]["name"]
            blackboard["relevant_guest_data"] = blackboard["guest_data"]["guest2"]
            self.flag = False
            return "continue"
        elif not self.flag and isinstance(self.flag, bool):
            blackboard["introduce_to"] = blackboard["guest_data"]["guest2"]["name"]
            blackboard["relevant_guest_data"] = blackboard["guest_data"]["guest1"]
            self.flag = None
            return "continue"
        else:
            yasmin.YASMIN_LOG_INFO("Introduction finished")
            return "succeeded"

    def _loop_guest(self, blackboard):
        if (
            blackboard["guest_data"]["guest1"]["seating_detection"]
            and blackboard["guest_data"]["guest2"]["seating_detection"]
        ):
            return "succeeded"

        id = (
            "guest1"
            if not blackboard["guest_data"]["guest1"]["seating_detection"]
            else "guest2"
        )

        point = blackboard["guest_data"][id]["seated_point"]
        blackboard["guest_point_stamped"] = PointStamped(
            header=Header(frame_id="map"), point=point
        )
        yasmin.YASMIN_LOG_INFO(id)
        yasmin.YASMIN_LOG_INFO(str(point))
        blackboard["guest_data"][id]["seating_detection"] = True
        blackboard["introduce_to"] = blackboard["guest_data"][id]["name"]
        blackboard["relevant_guest_data"] = (
            blackboard["guest_data"]["guest2"]
            if id == "guest1"
            else blackboard["guest_data"]["guest1"]
        )
        return "continue"
