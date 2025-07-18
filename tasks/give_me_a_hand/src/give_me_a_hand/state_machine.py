from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import Say, GoToLocation
from lasr_vision_msgs.srv import Recognise
from give_me_a_hand.states import FIND_OPERATORS, ObjectSortingLoop, PourCereal, HandleRequest, CommunicateOperators
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header



class GiveMeAHand(smach.StateMachine):
    def __init__(
        self,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Strategy1. Give every possible pose and pick close, wider to narrow (exclude impossible)
        # Strategy2. If location changes. Yolo detection?
        with self:
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

            waiting_area = Pose()
            waiting_area=rospy.get_param("/give_me_a_hand/living_room/shelves"),

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
                "FIND_OPERATORS",
                FindOperators(),
                transitions={
                    "succeeded": "GO_TO_OPERATORS",
                    "failed": "GO_TO_OPERATORS",
                    "escape": "SAY_FINISHED",
                },
            )

            smach.StateMachine.add(
                f"GO_TO_OPERATORS",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={
                    "succeeded": f"SAY_COMMUNICATE_OPERATORS",
                    "failed": f"ENABLE_MOVEBASE_MANAGER",
                },
            )

            smach.StateMachine.add(
                "ENABLE_MOVEBASE_MANAGER",
                smach_ros.ServiceState(
                "/pal_startup_control/start",
                StartupStart,
                request=StartupStartRequest("move_base", ""),
                ),
                transitions={
                    "succeeded": f"GO_TO_OPERATORS",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            ),

            smach.StateMachine.add(
                "SAY_COMMUNICATE_OPERATORS",
                Say("ongoing"),
                transitions={
                    "succeeded": "GRASP_AND_PUT",
                    "aborted": "GRASP_AND_PUT",
                    "preempted": "GRASP_AND_PUT",
                },
            )

            smach.StateMachine.add(
                "SAY_GRASP_AND_PLACE",
                Say("ongoing"),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "aborted": "SAY_FINISHED",
                    "preempted": "SAY_FINISHED",
                },
            )

            # smach.StateMachine.add(
            #     "COMMUNICATE_OPERATORS",
            #     CommunicateOperators("No more object to put in cabinet"),
            #     transitions={
            #         "succeeded": "GRASP_AND_PUT",
            #         "aborted": "GRASP_AND_PUT",
            #         "preempted": "GRASP_AND_PUT",
            #     },
            # )

            # smach.StateMachine.add(
            #     "GRASP_AND_PUT",
            #     GraspAndPut(),
            #     transitions={
            #         "succeeded": "FIND_OPERATORS",
            #         "failed": "FIND_OPERATORS",
            #     },
            # )

#Detect human collect object only near human image

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

 go_to_(self, customer_approach_pose, "succeeded") # Operator

            # smach.StateMachine.add(
            #     "GO_TO_OPERATORS",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={"succeeded": "GET_ORDER", "failed": "TRUN_MOVE_BASE"},
            # )

    def go_to_(self, pose, next_state) -> None:
        """Adds the states to go to table area."""

        smach.StateMachine.add(
            f"GO_TO_{next_state}",
            GoToLocation(pose),
            transitions={
                "succeeded": f"{next_state}",
                "failed": f"GO_TO_RECOVERY",
            },
        )

        smach.StateMachine.add(
            "ENABLE_MOVEBASE_MANAGER",
            smach_ros.ServiceState(
            "/pal_startup_control/start",
            StartupStart,
            request=StartupStartRequest("move_base", ""),
            ),
            transitions={
                "succeeded": f"GO_TO_{next_state}",
                "preempted": "failed",
                "aborted": "failed",
            },
        ),