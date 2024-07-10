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
    WaitForPerson,
    Wait,
)

from receptionist.states import PointCloudSweep, RunAndProcessDetections

from std_msgs.msg import Header


class SeatGuest(smach.StateMachine):

    class SelectSeat(smach.State):

        def __init__(self):

            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["empty_seat_detections"],
                output_keys=["seat_position"],
            )

        def execute(self, userdata) -> str:
            if len(userdata.empty_seat_detections) == 0:
                return "failed"

            seat = userdata.empty_seat_detections[0][0]
            userdata.seat_position = PointStamped(
                point=seat.point, header=Header(frame_id="map")
            )
            return "succeeded"

    def __init__(
        self,
    ):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["empty_seat_detections"]
        )
        with self:

            smach.StateMachine.add(
                "SELECT_SEAT",
                self.SelectSeat(),
                transitions={
                    "succeeded": "LOOK_TO_POINT",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "LOOK_TO_POINT",
                LookToPoint(),
                transitions={
                    "succeeded": "SAY_SIT",
                    "aborted": "failed",
                    "timed_out": "SAY_SIT",
                },
                remapping={"pointstamped": "seat_position"},
            )
            smach.StateMachine.add(
                "SAY_SIT",
                Say("Please sit in the seat that I am looking at."),
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
