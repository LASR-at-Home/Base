"""
State for following the host with the guest's bag. The robot turns to face the
host, tells them it's carrying a bag from their guest, asks them to show where
to put it, announces it's ready to walk, and then runs the actual follow
state.
"""

import smach
from rclpy.node import Node
from lasr_skills import FacePerson, Say, Wait


class FollowHost(smach.StateMachine):
    def __init__(self, node: Node):
        smach.StateMachine.__init__(
            self,
            outcomes=["valid", "invalid", "preempted"],
        )
        self._node = node

        with self:
            smach.StateMachine.add(
                "FACE_HOST",
                FacePerson(node),
                transitions={
                    "finished": "INFORM_HAS_BAG",
                    "truncated": "INFORM_HAS_BAG",
                    "failed": "invalid",
                },
            )

            smach.StateMachine.add(
                "INFORM_HAS_BAG",
                Say(node, text="I have a bag from your guest."),
                transitions={
                    "succeeded": "ASK_FOR_GUIDANCE",
                    "aborted": "invalid",
                    "preempted": "preempted",
                },
            )

            smach.StateMachine.add(
                "ASK_FOR_GUIDANCE",
                Say(node, text="Where should I put it? Please guide me to the destination."),
                transitions={
                    "succeeded": "ANNOUNCE_READY",
                    "aborted": "invalid",
                    "preempted": "preempted",
                },
            )

            smach.StateMachine.add(
                "ANNOUNCE_READY",
                Say(node, text="I am ready to follow you."),
                transitions={
                    "succeeded": "FOLLOW_PLACEHOLDER",
                    "aborted": "invalid",
                    "preempted": "preempted",
                },
            )
            smach.StateMachine.add(
                "FOLLOW_PLACEHOLDER",
                Wait(node, 0),
                transitions={
                    "succeeded": "valid",
                    "failed": "invalid",
                },
            )
