import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import SetupPlanningScene

from lasr_skills import Say, DetectDoorOpening, GoToLocation, DetectAllInPolygon

from std_msgs.msg import Empty


class StoringGroceries(smach.StateMachine):

    def __init__(self) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/restaurant/start",
                    Empty,
                    lambda _: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Storing Groceries task."),
                transitions={
                    "succeeded": "",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SETUP_PLANNING_SCENE",
                SetupPlanningScene(),
                transitions={
                    "succeeded": "SAY_WAITING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="Waiting for the door to open."),
                transitions={
                    "succeeded": "WAIT_FOR_DOOR_TO_OPEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(),
                transitions={
                    "door_opened": "GO_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TABLE",
                Say(text="I am going to the table"),
                transitions={
                    "door_opened": "GO_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TO_TABLE",
                GoToLocation(location_param="/storing_groceries/table"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectAllInPolygon(
                    ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
                    object_filter=[
                        k for k in rospy.get_param("/storing_groceries/objects")
                    ],
                    min_coverage=1.0,
                    min_confidence=0.7,
                ),
            )

            smach.StateMachine.add("")
