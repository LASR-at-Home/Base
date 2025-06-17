from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import (
    Say, GoToLocation
)
from lasr_vision_msgs.srv import Recognise
from storing_groceries.states import WaitDoorOpen, ObjectSortingLoop, PourCereal
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
        self.shelf_point = shelf_point
        self.shelf_area = shelf_area

        with self:
            self.userdata.dataset = "storing_groceries"
            # self.userdata.wait_pose = wait_pose
            # self.userdata.table_pose = table_pose
            # self.userdata.cabinet_pose = cabinet_pose

            self.userdata.shelf_position = PointStamped()
            self.userdata.object_position = PointStamped()
            self.userdata.sereal_position = PointStamped()
            self.userdata.sereal_container_position = PointStamped()
            # self.userdata.table_objects=[]
            # self.userdata.not_graspable=[]

            # def wait_cb(ud, msg):
            #     rospy.loginfo("Received start signal")
            #     return False

            # smach.StateMachine.add(
            #     "WAIT_START",
            #     smach_ros.MonitorState(
            #         "/storing_groceries/start",
            #         Empty,
            #         wait_cb,
            #     ),
            #     transitions={
            #         "valid": "WAIT_START",
            #         "invalid": "SAY_START",
            #         "preempted": "WAIT_START",
            #     },
            # )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Ready to start"),
                transitions={
                    "succeeded": "GO_TO_WAITING_AREA",
                    "aborted": "GO_TO_WAITING_AREA",
                    "preempted": "GO_TO_WAITING_AREA",
                },
            )

            self.go_to_waiting_area()

            """
            WaitDoorOpen
            """

            smach.StateMachine.add(
                "SAY_WAIT",
                Say(text="Waiting for door to open"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "WAIT_DOOR_OPEN",
                    "preempted": "WAIT_DOOR_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_DOOR_OPEN",
                WaitDoorOpen(),
                transitions={
                    "succeeded": "OBJECT_SORTING_LOOP",
                    "aborted": "OBJECT_SORTING_LOOP",
                    "preempted": "OBJECT_SORTING_LOOP",
                },
            )

            """
            ObjectSortingLoop
            """

            smach.StateMachine.add(
                "OBJECT_SORTING_LOOP",
                ObjectSortingLoop(self.table_pose, self.cabinet_pose),
                transitions={
                    "succeeded": "SAY_DONE_OBJECT_SORTING_LOOP",
                    "failed": "SAY_DONE_OBJECT_SORTING_LOOP",
                    "escape": "SAY_DONE_OBJECT_SORTING_LOOP",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_OBJECT_SORTING_LOOP",
                Say("No more object to put in cabinet"),                
                transitions={
                    "succeeded": "POUR_CEREAL",
                    "aborted": "POUR_CEREAL",
                    "preempted": "POUR_CEREAL",
                },
            )

            """
            + ShoppingBagSortingLoop
            """

            """
            PourCereal
            """

            smach.StateMachine.add(
                "POUR_CEREAL",
                PourCereal(self.table_pose),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "failed": "SAY_FINISHED",
                },
            )

            """
            Finish
            """

            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },
            )
    
    def go_to_waiting_area(self) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_WAITING_AREA",
            GoToLocation(self.wait_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_WAITING_AREA",
                "failed": f"SAY_ARRIVE_WAITING_AREA",
            },
        )
    
        smach.StateMachine.add(
            f"SAY_ARRIVE_WAITING_AREA",
            Say(text="Arrived waiting area"),
            transitions={
                "succeeded": f"SAY_WAIT",
                "aborted": f"SAY_WAIT",
                "preempted": f"SAY_WAIT",
            },
        )

   