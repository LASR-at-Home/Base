import smach
import rospy

from typing import List
from shapely.geometry import Polygon

import numpy as np

from geometry_msgs.msg import Point, PointStamped
from lasr_skills import (
    PlayMotion,
    Detect3DInArea,
    LookToPoint,
    Say,
    Wait,
)

from std_msgs.msg import Header


class SeatGuest(smach.StateMachine):

    def __init__(
        self,
    ):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["empty_seat_detections"]
        )
        with self:
            smach.StateMachine.add(
                "SAY_SIT",
                Say("Please take a seat."),
                transitions={
                    "succeeded": "WAIT_FOR_GUEST_SEAT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_GUEST_SEAT",
                Wait(5),
                transitions={
                    "succeeded": "RESET_HEAD",
                    "failed": "RESET_HEAD",
                },
            )

            smach.StateMachine.add(
                "RESET_HEAD",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
