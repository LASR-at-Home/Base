import yasmin
from yasmin import State, StateMachine

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from .hri_learn_faces import HRILearnFaces
from lasr_skills import LookToPoint

"""
    This is used by seat_guest.py
"""


class GetLookPoint(State):
    """State to get the look point for the guest to be seated."""

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("seated_guest_locs")

        self.add_output_key("pointstamped")

    def execute(self, blackboard):
        """Set the pointstamped to the guest seat point."""
        if not blackboard["seated_guest_locs"]:
            yasmin.YASMIN_LOG_WARN("No seated guest locations provided.")
            return "failed"
        point = blackboard["seated_guest_locs"][0]
        blackboard["pointstamped"] = PointStamped(header=Header(frame_id="base_footprint"), point=point) #TODO: Change to 'map' when 2dnav is fixed
        return "succeeded"


class LearnHostFace(StateMachine):
    """State machine to learn the host's face. Assumes seated guest is the host"""

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            handle_sigint=True
            )
        
        self.add_input_key("guest_data")
        self.add_input_key("seated_guest_locs")

        self.add_output_key("guest_data")

        self.add_state(
            "GET_HOST_LOOK_POINT",
            GetLookPoint(),
            transitions={"succeeded": "LOOK_TO_HOST", "failed": "failed"},
        )
        self.add_state(
            "LOOK_TO_HOST",
            LookToPoint(),
            transitions={
                "succeeded": "LEARN_HOST_FACE",
                "aborted": "failed",
                "canceled": "failed",
            }
        )
        self.add_state(
            "LEARN_HOST_FACE",
            HRILearnFaces(guest_id="host", dataset_size=5),
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
