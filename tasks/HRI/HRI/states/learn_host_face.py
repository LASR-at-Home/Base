import smach
from smach_ros import RosState

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from .hri_learn_faces import HRILearnFaces
from lasr_skills import LookToPoint

"""
    This is used by seat_guest.py
"""


class GetLookPoint(RosState):
    """State to get the look point for the guest to be seated."""

    def __init__(self, node: Node):
        RosState.__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["seated_guest_locs"],
            output_keys=["pointstamped"],
        )

    def execute(self, userdata):
        """Set the pointstamped to the guest seat point."""
        if not userdata.seated_guest_locs:
            self.node.get_logger().warn("No seated guest locations provided.")
            return "failed"
        point = userdata.seated_guest_locs[0]
        userdata.pointstamped = PointStamped(header=Header(frame_id="map"), point=point)
        return "succeeded"


class LearnHostFace(smach.StateMachine):
    """State machine to learn the host's face. Assumes seated guest is the host"""

    def __init__(self, node: Node):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "seated_guest_locs"],
            output_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "GET_HOST_LOOK_POINT",
                GetLookPoint(node=node),
                transitions={"succeeded": "LOOK_TO_HOST", "failed": "failed"},
            )
            smach.StateMachine.add(
                "LOOK_TO_HOST",
                LookToPoint(node=node),
                transitions={
                    "succeeded": "LEARN_HOST_FACE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"pointstamped": "pointstamped"},
            )
            smach.StateMachine.add(
                "LEARN_HOST_FACE",
                HRILearnFaces(node=node, guest_id="host", dataset_size=5),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"guest_data": "guest_data"},
            )
