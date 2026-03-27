from typing import List, Tuple, Dict

import rclpy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
#skills to be imported
# from lasr_skills import (
# )

from HRI.states import *

from shapely.geometry import Polygon
from std_msgs.msg import Empty

from Base.tasks.HRI.HRI.states.check_sofa import CheckSofa


class HRI(smach.StateMachine):
    def __init__(
            self, node
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        #commented incase Detect Doorbell was not implemented
        # smach.StateMachine.add(
        #     "DETECT DOORBELL",
        #     DetectDoorbell(node),
        #     transitions={
        #         "valid": "APPROACH_GUEST",
        #         "invalid": "DETECT_DOORBELL",
        #         "preempted": "DETECT_DOORBELL",
        #     },
        # )

        #start door state machine goes here (by Fadi and Aldrich)


        smach.StateMachine.add(
            "APPROACH_GUEST",
            ApproachGuest(node),
            transitions={
                "valid": "FACE_GUEST",
                "invalid": "GREET_GUEST",
                "preempted": "GREET_GUEST",
            },
        )

        #face guest and greet them concurrently state machine (by Fadi)

        smach.StateMachine.add(
            "GET_NAME_AND_DRINK",
            GetNameAndDrink(node),
            transitions={
                "valid": "GUIDE_GUEST_TO_LIVING_ROOM",
                "invalid": "GET_NAME_AND_DRINK",
                "preempted": "GET_NAME_AND_DRINK",
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
            "CHECK_SOFA",
            CheckSofa(node),
            transitions={
                "valid": "OFFER_A_FREE_SEAT",
                "invalid": "CHECK_SOFA",
                "preempted": "CHECK_SOFA",
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