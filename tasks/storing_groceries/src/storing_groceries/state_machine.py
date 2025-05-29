from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import (
    AskAndListen,
    GoToLocation,
    LookToPoint,
    PlayMotion,
    Say,
    Wait,
    WaitForPersonInArea,
)
from lasr_vision_msgs.srv import Recognise
from storing_groceries.states import (
    HandleNameInterest,
    HandleDrink,
    CompareInterest,
    IntroduceAndSeatGuest,
)
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header


class StoringGroceries(smach.StateMachine):
    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        table_pose: Pose,
        table_area: Polygon,
        cabinet_pose: Pose,
        # search_motions: List[str],
        cabinet_area: Polygon,
        shelf_area: Polygon,
        shelf_point: Point,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.table_pose = table_pose
        self.table_area = table_area
        self.cabinet_pose = cabinet_pose
        self.cabinet_area = cabinet_area
        self.shelf_point = shelf_point #is it given?
        self.shelf_area = shelf_area

        with self:
            self.userdata.confidence = 0 #modify needed
            self.userdata.dataset = "storing_groceries"
            self.userdata.shelf_position = PointStamped()
            self.userdata.object_position = PointStamped()
            self.userdata.sereal_position = PointStamped()
            self.userdata.sereal_container_position = PointStamped()

            def wait_cb(ud, msg):
                rospy.loginfo("Received start signal")
                return False

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/storing_groceries/start",
                    Empty,
                    wait_cb,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "invalid": "SAY_START",
                    "preempted": "WAIT_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Waiting for door open."),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            """
            WaitDoorOpen
            """

            smach.StateMachine.add(
                "WAIT_DOOR_OPEN",
                Say(text="wait door open is ongoing"),
                transitions={
                    "succeeded": "SAY_TASK_START",
                    "aborted": "GO_TO_TABLE",
                    "preempted": "GO_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_TASK_START",
                Say(text="Start of storing_groceries task."),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            """
            ObjectSortingLoop
            """

            self.go_to_table()

            smach.StateMachine.add(
                "OBJECT_SORTING_LOOP",
                Say(text="object sorting loop is ongoing"),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            """
            PourCereal
            """

            smach.StateMachine.add(
                "POUR_CEREAL",
                Say(text="Pour cereal is ongoing"),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "aborted": "SAY_FINISHED",
                    "preempted": "SAY_FINISHED",
                },
            )

            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },
            )

    def go_to_table(self, cereal=False) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_TABLE",
            GoToLocation(self.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_TABLE",
                "failed": f"SAY_ARRIVE_TABLE",
            },
        )
        
        if(cereal):
            smach.StateMachine.add(
                f"SAY_ARRIVE_CEREAL",
                Say(text="Arrive table"),
                transitions={
                    "succeeded": f"DETECT_CEREAL",
                    "aborted": f"DETECT_CEREAL",
                    "preempted": f"DETECT_CEREAL",
                },
            )    
        else:   
            smach.StateMachine.add(
                f"SAY_ARRIVE_TABLE",
                Say(text="Arrive table"),
                transitions={
                    "succeeded": f"OBJECT_SORTING_LOOP",
                    "aborted": f"OBJECT_SORTING_LOOP",
                    "preempted": f"OBJECT_SORTING_LOOP",
                },
            )

    def go_to_cabinet(self) -> None:
        """Adds the states to go to cabinet area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_CABINET",
            GoToLocation(self.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_CABINET",
                "failed": f"SAY_ARRIVE_CABINET",
            },
        )

        smach.StateMachine.add(
            f"SAY_ARRIVE_CABINET",
            Say(text="Arrive cabinet"),
            transitions={
                "succeeded": f"DETECT_CABINET",
                "aborted": f"DETECT_CABINET",
                "preempted": f"DETECT_CABINET",
            },
        )
