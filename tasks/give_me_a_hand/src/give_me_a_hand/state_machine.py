#!/usr/bin/env python
from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import Say, GoToLocation, AskAndListen,DetectDoorOpening
from lasr_vision_msgs.srv import Recognise
from give_me_a_hand.states import WaitDoorOpen, HandoverAndDeliver
# from give_me_a_hand.states import  RecoverCommand #, HandoverAndDeliver, GraspAndPut, CommunicateOperators,
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header
from give_me_a_hand.states import RecoverCommand
from smach import CBState





class GiveMeAHand(smach.StateMachine):
    def __init__(
        self,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            def wait_cb(ud, msg):
                rospy.loginfo("Received start signal")
                return False

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/give_me_a_hand/start",
                    Empty,
                    wait_cb,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "invalid": "SAY_WAIT",
                    "preempted": "WAIT_START",
                },
            )


            # # self.go_to_waiting_area() #may be we don't need this

            # """
            # WaitDoorOpen
            # """

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
                DetectDoorOpening(timeout=15.0),
                transitions={"door_opened" : "SAY_GOING_TO_KITCHEN"}
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_KITCHEN",
                Say(text="Door is open, going to kitchen"),
                transitions={
                    "succeeded": "GO_TO_OPERATORS",
                    "aborted": "GO_TO_OPERATORS",
                    "preempted": "GO_TO_OPERATORS",
                },
            )

            # smach.StateMachine.add(
            #     "WAIT_DOOR_OPEN",
            #     WaitDoorOpen(),
            #     transitions={
            #         "succeeded": "GO_TO_OPERATORS",
            #         "aborted": "GO_TO_OPERATORS",
            #         "preempted": "GO_TO_OPERATORS",
            #     },
            # )

            # """
            # FIND_OPERATORS_LOOP
            # """

            kitchen_area = Pose()

            kitchen_area.position.x = 5.180957294827783
            kitchen_area.position.y = 5.355385375101781
            kitchen_area.position.z = 0.0
            kitchen_area.orientation.x = 0.0
            kitchen_area.orientation.y = 0.0
            kitchen_area.orientation.z = 0.7062973515469307
            kitchen_area.orientation.w = 0.707915285325717

            smach.StateMachine.add(
                f"GO_TO_OPERATORS",
                GoToLocation(kitchen_area),
                transitions={
                    "succeeded": f"ASK_AND_LISTEN",
                    "failed": f"GO_TO_OPERATORS",
                },
            )

            # smach.StateMachine.add(
            #     "FIND_OPERATORS",
            #     FindOperators(),
            #     transitions={
            #         "succeeded": "GO_TO_OPERATORS",
            #         "failed": "GO_TO_OPERATORS",
            #         "escape": "SAY_FINISHED",
            #     },
            # )

            # smach.StateMachine.add(
            #     f"GO_TO_OPERATORS",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={
            #         "succeeded": f"COMMUNICATE_OPERATOR",
            #         "failed": f"COMMUNICATE_OPERATOR",
            #     },
            # )

            # smach.StateMachine.add(
            #     "ENABLE_MOVEBASE_MANAGER",
            #     smach_ros.ServiceState(
            #     "/pal_startup_control/start",
            #     StartupStart,
            #     request=StartupStartRequest("move_base", ""),
            #     ),
            #     transitions={
            #         "succeeded": f"GO_TO_OPERATORS",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="Hello. I am unable to detect you.I will give you 15 seconds to prepare to approach me."),
                transitions={
                    "succeeded": "ASK_AND_LISTEN",
                    "aborted": "ASK_AND_LISTEN",
                    "preempted": "ASK_AND_LISTEN",
                },
            )

            smach.StateMachine.add(
                "ASK_AND_LISTEN",
                AskAndListen(f"Hi, please tell me your command."),
                transitions={
                    "succeeded": "RECOVER_COMMAND",
                    "failed": "RECOVER_COMMAND",
                },
            )

            # smach.StateMachine.add(
            #     "COMMUNICATE_OPERATOR",
            #     CommunicateOperators(),
            #     transitions={
            #         "succeeded": "succeeded",
            #         "failed": "succeeded",
            #     },
            # )

            smach.StateMachine.add(
                "RECOVER_COMMAND",
                RecoverCommand(),
                transitions={
                    "succeeded": "CHECK_LOCATION_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "CHECK_LOCATION_1",
                CBState(
                    lambda ud: "got_location" if ud.location else "no_location",
                    input_keys=["location"],  
                    outcomes=["got_location", "no_location"]
                ),
                transitions={
                    "got_location": "GRASP_AND_PLACE",      
                    "no_location": "ASK_AND_LISTEN_2"  
                }
            )
            
            smach.StateMachine.add(
                "ASK_AND_LISTEN_2",
                AskAndListen(f"we can place on the shelf, fridge, table, dishwasher or sink. Where would you like to place it?"),
                transitions={
                    "succeeded": "RECOVER_COMMAND_2",
                    "failed": "failed",
                },
            )


            smach.StateMachine.add(
                "RECOVER_COMMAND_2",
                RecoverCommand(),
                transitions={
                    "succeeded": "CHECK_LOCATION_2",
                    "failed": "ASK_AND_LISTEN_3",
                },
            )   

            smach.StateMachine.add(
                "CHECK_LOCATION_2",
                CBState(
                    lambda ud: "got_location" if ud.location else "no_location",
                    input_keys=["location"],  
                    outcomes=["got_location", "no_location"]
                ),
                transitions={
                    "got_location": "GRASP_AND_PLACE",      
                    "no_location": "ASK_AND_LISTEN_3"  
                }
            )

            smach.StateMachine.add(
                "ASK_AND_LISTEN_3",
                AskAndListen(f"Sorry, I couldn't hear you. we can place on the shelf, fridge, table, dishwasher or sink. Where would you like to place it?"),
                transitions={
                    "succeeded": "RECOVER_COMMAND_3",
                    "failed": "GRASP_AND_PLACE", # TODO: get random location
                },
            )
            smach.StateMachine.add(
                "RECOVER_COMMAND_3",
                RecoverCommand(),
                transitions={
                    "succeeded": "CHECK_LOCATION_3",
                    "failed": "GRASP_AND_PLACE",
                },
            )

            smach.StateMachine.add(
                "CHECK_LOCATION_3",
                CBState(
                    lambda ud: "got_location" if ud.location else "no_location",
                    input_keys=["location"],  
                    outcomes=["got_location", "no_location"]
                ),
                transitions={
                    "got_location": "GRASP_AND_PLACE",      
                    "no_location": "GRASP_AND_PLACE"  
                }
            )

            smach.StateMachine.add(
                "GRASP_AND_PLACE",
                HandoverAndDeliver(),
                transitions={
                    "succeeded": "GO_TO_OPERATORS",
                    "failed": "GO_TO_OPERATORS",
                    "escape": "GO_TO_OPERATORS",
                },
                remapping={"place_location_name" : "location"}
            )


            # smach.StateMachine.add(
            #     "GRASP_AND_PLACE",
            #     HandoverAndDeliver(),
            #     transitions={
            #         "succeeded": "GO_TO_OPERATORS",
            #         "aborted": "GO_TO_OPERATORS",
            #         "preempted": "GO_TO_OPERATORS",
            #     },
            # )

            # """
            # Finish
            # """

            # smach.StateMachine.add(
            #     "SAY_FINISHED",
            #     Say(text="I am done."),
            #     transitions={
            #         "succeeded": "succeeded",
            #         "aborted": "failed",
            #         "preempted": "succeeded",
            #     },
            # )