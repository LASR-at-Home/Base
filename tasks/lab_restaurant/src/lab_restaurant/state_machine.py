#!/usr/bin/env python

import rospy
import smach
from geometry_msgs.msg import Pose


from lasr_skills import (
    GoToLocation,
    Say
)


class LabRestaurant(smach.StateMachine):
    def __init__(
            self,
            loc_A: Pose,
            loc_B: Pose
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        self.loc_A = loc_A
        self.loc_B = loc_B

        with self:
            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of lab restaurant task."),
                transitions={
                    "succeeded": "GO_TO_LOC_A",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_LOC_A",
                GoToLocation(loc_A),
                transitions={
                    "succeeded": "GO_TO_LOC_B",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_LOC_B",
                GoToLocation(loc_B),
                transitions={
                    "succeeded": "GO_TO_LOC_A",
                    "failed": "failed",
                },
            )


'''

if __name__ == '__main__':
    # Initialise the node and create the state machine.
    rospy.init_node('main_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    # Add the states.
    with sm:
        smach.StateMachine.add(

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of lab restaurant task."),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_1",
                }
            )
        )







        
            "GO_TO_TABLE",
            GoToLocation(),
            remapping={"location": "table_location"},
            transitions={"succeeded": "GET_ORDER", "failed": "GO_TO_START"},
        )

        smach.StateMachine.add(
            "GET_ORDER",
            AskAndListen(),
            remapping={"transcribed_speech": "order_string"},
            transitions={"succeeded": "GO_TO_KITCHEN", "failed": "GET_ORDER"},
        )

        smach.StateMachine.add(
            "GO_TO_KITCHEN",
            GoToLocation(),
            remapping={"location": "kitchen_location"},
            transitions={"succeeded": "succeeded", "failed": "succeeded"},
        )

        smach.StateMachine.add(
            "LOOK_FOR_FOOD",
            Detect(),
            remapping={"detections": "food_detections"},
            transitions={"succeeded": "succeeded", "failed": "succeeded"},
        )
        '''





