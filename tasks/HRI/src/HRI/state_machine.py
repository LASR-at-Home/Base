from typing import List, Tuple, Dict

import rclpy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose

# from lasr_skills import (

# )

from HRI.states import *

from shapely.geometry import Polygon
from std_msgs.msg import Empty


class HRI(smach.StateMachine):
    def __init__(self, node):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        smach.StateMachine.add(
            "DETECT DOORBELL",
            DetectDoorbell(node),
            transitions={
                "valid": "APPROACH_GUEST",
                "invalid": "DETECT_DOORBELL",
                "preempted": "DETECT_DOORBELL",
            },
        )

        smach.StateMachine.add(
            "APPROACH_GUEST",
            ApproachGuest(node),
            transitions={
                "valid": "FACE_GUEST",
                "invalid": "GREET_GUEST",
                "preempted": "GREET_GUEST",
            },
        )

        smach.StateMachine.add(
            "FACE GUEST",
            FaceGuest(node),
            transitions={
                "valid": "GREET GUEST",
                "invalid": "FACE_GUEST",
                "preempted": "FACE_GUEST",
            },
        )

        smach.StateMachine.add(
            "GREET_GUEST",
            GreetGuest(node),
            transitions={
                "valid": "RECOGNISE_NAME_AND_DRINK",
                "invalid": "GREET_GUEST",
                "preempted": "GREET_GUEST",
            },
        )

        smach.StateMachine.add(
            "RECOGNISE_NAME_AND_DRINK",
            RecogniseNameAndDrink(node),
            transitions={
                "valid": "GUIDE_GUEST_TO_LIVING_ROOM",
                "invalid": "RECOGNISE_NAME_AND_DRINK",
                "preempted": "RECOGNISE_NAME_AND_DRINK",
            },
        )

        smach.StateMachine.add(
            "GUIDE_GUEST_TO_LIVING_ROOM",
            GuideGuestsToLivingroom(node),
            transitions={
                "valid": "OFFER_A_FREE_SEAT",
                "invalid": "GUIDE_GUEST_TO_LIVING_ROOM",
                "preempted": "GUIDE_GUEST_TO_LIVING_ROOM",
            },
        )

        smach.StateMachine.add(
            "OFFER_A_FREE_SEAT",
            OfferFreeSeat(node),
            transitions={
                "valid": "INTRODUCE_GUESTS_TO_EACHOTHER",
                "invalid": "OFFER_A_FREE_SEAT",
                "preempted": "OFFER_A_FREE_SEAT",
            },
        )

        smach.StateMachine.add(
            "INTRODUCE_GUESTS_TO_EACHOTHER",
            IntroduceGuestsToEachother(node),
            transitions={
                "valid": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
                "invalid": "INTRODUCE_GUESTS_TO_EACHOTHER",
                "preempted": "INTRODUCE_GUESTS_TO_EACHOTHER",
            },
        )

        smach.StateMachine.add(
            "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
            AskSecondGuestForBagToHost(node),
            transitions={
                "valid": "PICK_UP_BAG",
                "invalid": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
                "preempted": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
            },
        )

        smach.StateMachine.add(
            "PICK_UP_BAG",
            PickUpBag(node),
            transitions={
                "valid": "FOLLOW_HOST",
                "invalid": "LISTEN_TO_HOST_DROP_INSTRUCTION",
                "preempted": "LISTEN_TO_HOST_DROP_INSTRUCTION",
            },
        )

        smach.StateMachine.add(
            "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
            ListenToHostDropBagInstruction(node),
            transitions={
                "valid": "DROP_BAG",
                "invalid": "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
                "preempted": "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
            },
        )

        smach.StateMachine.add(
            "DROP_BAG",
            DropBag(node),
            transitions={
                "valid": "succeeded",
                "invalid": "DROP_BAG",
                "preempted": "DROP_BAG",
            },
        )
