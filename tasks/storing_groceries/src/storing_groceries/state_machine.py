from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import (
    Say,
)
from lasr_vision_msgs.srv import Recognise
from storing_groceries.states import WaitDoorOpen
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
                Say(text="Start task storing groceries."),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "failed":"GO_TO_WAIT_LOCATION",
                },
            )

            """
            WaitDoorOpen
            """

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
                Say(text="object sorting loop is ongoing"),
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
                Say(text="Pour cereal is ongoing"),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "aborted": "SAY_FINISHED",
                    "preempted": "SAY_FINISHED",
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

   