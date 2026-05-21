"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

"""

import smach
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from lasr_skills.look_to_point import LookToPoint
from lasr_skills.say import Say
from lasr_skills.wait import Wait

from .clearSeatingDetections import ClearSeatingDetections
from .getGuestData import GetGuestData
from .getIntroductionStr import GetIntroductionStr
from .recognise import Recognise


class Introduce(smach.StateMachine):

    _guest_to_introduce: str

    def _get_look_point(self, userdata: smach.UserData) -> str:
        """
        Callback to get the look point based on the current person detection index.

        Args:
            userdata (smach.UserData): User data containing the people detections and index.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        index = userdata.person_index
        if index < len(userdata.seated_guest_locs):
            header = Header()
            header.frame_id = "map"
            look_point = PointStamped(
                header=header, point=userdata.seated_guest_locs[index]
            )
            userdata.look_point = look_point
            self._node.get_logger().info(
                f"Look point set to: {look_point.point.x}, "
                f"{look_point.point.y}, {look_point.point.z}"
            )
            return "succeeded"
        else:
            self._node.get_logger().error("Index out of bounds for people detection points.")
            return "failed"

    def __init__(self, node: Node, guest_to_introduce: str, can_detect_second_guest: bool = False):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "guest_seat_point", "seated_guest_locs"],
        )
        self._node = node
        self._guest_to_introduce = guest_to_introduce
        with self:
            introduction_iterator = smach.Iterator(
                it=lambda: range(len(self.userdata.seated_guest_locs)),
                it_label="person_index",
                input_keys=[
                    "seated_guest_locs",
                    "guest_data",
                    "guest_seat_point",
                    "look_point",
                ],
                output_keys=["look_point", "relevant_guest_data", "introduce_to"],
                exhausted_outcome="succeeded",
                outcomes=["succeeded", "failed"],
            )
            with introduction_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "succeeded", "failed"],
                    input_keys=[
                        "guest_data",
                        "guest_seat_point",
                        "seated_guest_locs",
                        "person_index",
                        "look_point",
                        "introduce_to",
                        "relevant_guest_data",
                        "named_guest_detection",
                    ],
                    output_keys=[
                        "guest_data",
                        "guest_seat_point",
                        "seated_guest_locs",
                        "person_index",
                        "look_point",
                        "introduce_to",
                        "relevant_guest_data",
                        "named_guest_detection",
                    ],
                )
                with container_sm:
                    smach.StateMachine.add(
                        "GET_LOOK_POINT_1",
                        smach.CBState(
                            self._get_look_point,
                            input_keys=[
                                "seated_guest_locs",
                                "person_index",
                                "look_point",
                            ],
                            output_keys=["look_point"],
                            outcomes=["succeeded", "failed"],
                        ),
                        transitions={
                            "succeeded": "LOOK_TO_GUEST_1",
                            "failed": "failed",
                        },
                        remapping={"look_point": "pointstamped"},
                    )
                    """
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_1",
                        LookToPoint(node),
                        transitions={
                            "succeeded": "WAIT",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                    )
                    """
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_1",
                        smach.CBState(
                        lambda ud: "succeeded",
                        outcomes=["succeeded"],
                        ),
                        transitions={"succeeded": "WAIT"},
                    )
                    smach.StateMachine.add(
                        "WAIT",
                        Wait(node, 0.25),
                        transitions={"succeeded": "RECOGNISE", "failed": "failed"},
                    )
                    smach.StateMachine.add(
                        "RECOGNISE",
                        Recognise(
                            node=node,
                            can_detect_second_guest=can_detect_second_guest,
                        ),
                        transitions={
                            "succeeded": "GET_GUEST_DATA_1",
                            "failed": "failed",
                        },
                        remapping={
                            "guest_data": "guest_data",
                            "guest_seat_point": "guest_seat_point",
                            "named_guest_detection": "named_guest_detection",
                        },
                    )
                    smach.StateMachine.add(
                        "GET_GUEST_DATA_1",
                        GetGuestData(guest_to_introduce=self._guest_to_introduce),
                        transitions={
                            "succeeded": "GET_INTRODUCTION_STR_1",
                            "failed": "failed",
                        },
                        remapping={"relevant_guest_data": "relevant_guest_data"},
                    )

                    smach.StateMachine.add(
                        "GET_INTRODUCTION_STR_1",
                        GetIntroductionStr(),
                        transitions={
                            "succeeded": "SAY_INTRODUCTION",
                            "failed": "failed",
                        },
                        remapping={"introduction_str": "text"},
                    )
                    smach.StateMachine.add(
                        "SAY_INTRODUCTION",
                        Say(node),
                        transitions={
                            "succeeded": "LOOK_TO_GUEST_2",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"text": "text"},
                    )
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_2",
                        smach.CBState(
                        lambda ud: "succeeded",
                        outcomes=["succeeded"],
                        ),
                        transitions={"succeeded": "GET_GUEST_DATA_2"},
                    )
                    """
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_2",
                        LookToPoint(node),
                        transitions={
                            "succeeded": "GET_GUEST_DATA_2",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                        remapping={"pointstamped": "guest_seat_point"},
                    )
                    """
                    smach.StateMachine.add(
                        "GET_GUEST_DATA_2",
                        GetGuestData(guest_to_introduce_to=self._guest_to_introduce),
                        transitions={
                            "succeeded": "GET_INTRODUCTION_STR_2",
                            "failed": "failed",
                        },
                        remapping={
                            "relevant_guest_data": "relevant_guest_data",
                            "introduce_to": "introduce_to",
                        },
                    )
                    smach.StateMachine.add(
                        "GET_INTRODUCTION_STR_2",
                        GetIntroductionStr(),
                        transitions={
                            "succeeded": "SAY_INTRODUCTION_2",
                            "failed": "failed",
                        },
                        remapping={"introduction_str": "text"},
                    )
                    smach.StateMachine.add(
                        "SAY_INTRODUCTION_2",
                        Say(node),
                        transitions={
                            "succeeded": "continue",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"text": "text"},
                    )
                smach.Iterator.set_contained_state(
                    "CONTAINER_SM", container_sm, loop_outcomes=["continue"]
                )
            self.add(
                "INTRODUCTION_ITERATOR",
                introduction_iterator,
                transitions={
                    "succeeded": "CLEAR_SEATING_DETECTIONS",
                    "failed": "CLEAR_SEATING_DETECTIONS",
                },
            )

            self.add(
                "CLEAR_SEATING_DETECTIONS",
                ClearSeatingDetections(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"guest_data": "guest_data"},
            )
