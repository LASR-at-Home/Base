from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import Say, GoToLocation
from lasr_vision_msgs.srv import Recognise
from give_me_a_hand.states import FIND_OPERATORS, ObjectSortingLoop, PourCereal
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header


class GiveMeAHand(smach.StateMachine):
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

        # Strategy1. Give every possible pose and pick close, wider to narrow (exclude impossible)
        # Strategy2. If location changes. Yolo detection?

        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.table_pose = table_pose
        self.table_area = table_area
        self.cabinet_pose = cabinet_pose
        self.cabinet_area = cabinet_area
        self.shelf_point = shelf_point
        self.shelf_area = shelf_area

        with self:
            self.userdata.dataset = "give_me_a_hand"
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
            #         "/give_me_a_hand/start",
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

            # self.go_to_waiting_area() #may be we don't need this

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
                    "succeeded": "FIND_OPERATORS",
                    "aborted": "FIND_OPERATORS",
                    "preempted": "FIND_OPERATORS",
                },
            )

            """
            FIND_OPERATORS_LOOP
            """

            smach.StateMachine.add(
                "HANDLE_OPERATORS",
                FindOperators(self.table_pose, self.cabinet_pose),
                transitions={
                    "succeeded": "COMMUNICATE_OPERATORS",
                    "failed": "COMMUNICATE_OPERATORS",
                    "escape": "SAY_FINISHED",
                },
            )

            smach.StateMachine.add(
                "COMMUNICATE_OPERATORS",
                CommunicateOperators("No more object to put in cabinet"),
                transitions={
                    "succeeded": "GRASP_AND_PUT",
                    "aborted": "GRASP_AND_PUT",
                    "preempted": "GRASP_AND_PUT",
                },
            )

            smach.StateMachine.add(
                "GRASP_AND_PUT",
                GraspAndPut(self.table_pose),
                transitions={
                    "succeeded": "FIND_OPERATORS",
                    "failed": "FIND_OPERATORS",
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
        """Adds the states to go to table area."""

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
