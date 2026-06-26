"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

Ported from SMACH to YASMIN.
"""

import yasmin
import yasmin_ros
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    Say,
    DetectAllInPolygon,
    StartEyeTracker,
    StopEyeTracker,
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
        - guest_seat_point: PointStamped of the incoming guest's seat
        - seated_guest_locs: List of Point locations of all seated guests
        - person_index: Set to 0 before calling sm()
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_input_key("guest_seat_point")
        self.add_input_key("seated_guest_locs")

        self._node = yasmin_ros.logger_node

        self.seating_area = ShapelyPolygon(
            [
                self._node.get_parameter("seat_area.top_left").value,
                self._node.get_parameter("seat_area.top_right").value,
                self._node.get_parameter("seat_area.bottom_right").value,
                self._node.get_parameter("seat_area.bottom_left").value,
            ]
        )

        loop_state = yasmin.CbState(
            outcomes=["succeeded", "continue", "failed"],
            callback=self._loop_person_index,
        )
        loop_state.add_input_key("person_index")
        loop_state.add_input_key("people_det")
        loop_state.add_input_key("guest_data")
        loop_state.add_output_key("person_index")
        loop_state.add_output_key("person_point")

        guest_loop = yasmin.CbState(
            outcomes=["succeeded", "continue"], callback=self._loop_guest
        )
        guest_loop.add_input_key("guest_data")
        guest_loop.add_output_key("guest_data")

        host_point = yasmin.CbState(
            outcomes=["succeeded", "failed"],
            callback=self._get_host,
        )

        host_point.add_input_key("guest_data")
        host_point.add_output_key("host_point")

        self.add_state(
            "RESET_SEATING_DETECTIONS",
            ClearSeatingDetections(),
            transitions={"succeeded": "LOOP_PERSON_STATE", "failed": "failed"},
        )

        # self.add_state(
        #     "FIND_PEOPLE",
        #     DetectAllInPolygon(
        #         polygon=self.seating_area,
        #         object_filter=["person"],
        #         min_coverage=0.7,
        #         min_new_object_dist=0.50,
        #         min_confidence=0.5,
        #     ),
        #     transitions={"succeeded": "LOOP_PERSON_STATE", "failed": "failed"},
        #     remappings={"detected_objects": "people_detected"},
        # )

        self.add_state(
            "LOOP_PERSON_STATE",
            loop_state,
            transitions={
                "succeeded": "GRAB_GUEST_POINT",
                "continue": "LOOK_AT_PERSON",
                "failed": "failed",
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
                "aborted": "failed",
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
            "GRAB_GUEST_POINT",
            guest_loop,
            transitions={"succeeded": "GET_HOST", "continue": "GET_INTRODUCTION_STR"},
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

        self.add_state(
            "GET_HOST",
            host_point,
            transitions={
                "succeeded": "LOOK_AT_HOST",
                "failed": "failed",
            },
        )

        self.add_state(
            "LOOK_AT_HOST",
            LookToPoint(),
            transitions={
                "succeeded": "SAY_INTRODUCTION",
                "aborted": "SAY_INTRODUCTION",
                "canceled": "failed",
                "timeout": "SAY_INTRODUCTION",
            },
            remappings={"pointstamped": "host_pointstamped"},
        )

        self.add_state(
            "SAY_HOST",
            Say(
                text="Hello host! I have a bag for you. Can you stand in front of me to lead the way."
            ),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )

    def _loop_person_index(self, blackboard):
        guest1point = blackboard["guest_data"]["guest1"]["seated_point"]
        guest2point = blackboard["guest_data"]["guest2"]["seated_point"]
        host = blackboard["guest_data"]["host"]["seated_point"]
        people_det = len(blackboard["people_det"])
        index = blackboard["person_index"]

        indexes = [i for i in range(people_det)]

        yasmin.YASMIN_LOG_INFO(str(index))
        yasmin.YASMIN_LOG_INFO("Guest1 point: " + str(guest1point))
        yasmin.YASMIN_LOG_INFO("Guest2 point: " + str(guest2point))
        yasmin.YASMIN_LOG_INFO("Host point: " + str(host))
        yasmin.YASMIN_LOG_INFO("Total detections (seats + people): " + str(people_det))

        if guest1point is not None and guest2point is not None and host is not None:
            return "succeeded"
        elif index < people_det:
            point = blackboard["people_det"][index].point
            point_stamped = PointStamped(header=Header(frame_id="map"), point=point)
            blackboard["person_point_stamped"] = point_stamped
            index += 1
            blackboard["person_index"] = index
            return "continue"
        elif guest2point is not None and host is not None:
            index2 = blackboard["seat_indexes"]["guest2"]
            indexh = blackboard["seat_indexes"]["host"]
            for i in indexes:
                if i != index2 and i != indexh:
                    index = i
            blackboard["guest_data"]["guest1"]["seated_point"] = blackboard[
                "people_det"
            ][index].point
            guest2point = blackboard["guest_data"]["guest1"]["seated_point"]
            yasmin.YASMIN_LOG_INFO("Fallback Guest1 point: " + str(guest2point))
            return "succeeded"
        elif guest1point is not None and host is not None:
            index2 = blackboard["seat_indexes"]["guest1"]
            indexh = blackboard["seat_indexes"]["host"]
            for i in indexes:
                if i != index2 and i != indexh:
                    index = i
            blackboard["guest_data"]["guest2"]["seated_point"] = blackboard[
                "people_det"
            ][index].point
            guest2point = blackboard["guest_data"]["guest2"]["seated_point"]
            yasmin.YASMIN_LOG_INFO("Fallback Guest2 point: " + str(guest2point))
            return "succeeded"

        return "failed"

    def _get_host(self, blackboard):
        if blackboard["guest_data"]["host"]["seated_point"]:
            poinstamped = PointStamped(
                header=Header(frame_id="map"),
                point=blackboard["guest_data"]["host"]["seated_point"],
            )
            blackboard["host_pointstamped"] = poinstamped
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_INFO(f"No host")
            return "failed"

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
