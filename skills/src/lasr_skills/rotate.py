from typing import List, Union, Optional

import rclpy
from rclpy.wait_for_message import wait_for_message

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


import yasmin
import yasmin_ros
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers, ServiceState
from yasmin_viewer import YasminViewerPub

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from lasr_skills import GoToLocation
from scipy.spatial.transform import Rotation as R
import numpy as np


class Rotate(StateMachine):
    class GetRotatedPose(State):
        def __init__(self, angle: Optional[float] = None):
            super().__init__(outcomes=["succeeded"])
            if angle in None:
                self.add_input_key("angle")
            self.add_output_key("target_pose")
            self.angle = angle


            self.current_robot_point = None
            self.robot_pose_sub = self.node.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self.robot_point_cb,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                ),
            )
        def robot_point_cb(self, msg: PoseWithCovarianceStamped):
            self.robot_pose = msg

        def execute(self, blackboard):

            current_orientation = np.array(
                [
                    self.robot_pose.pose.pose.orientation.x,
                    self.robot_pose.pose.pose.orientation.y,
                    self.robot_pose.pose.pose.orientation.z,
                    self.robot_pose.pose.pose.orientation.w,
                ]
            )

            rot_matrix = R.from_quat(current_orientation)
            new_rot_matrix = rot_matrix * R.from_euler(
                "z", blackboard["angle"] if self.angle is None else self.angle, degrees=True
            )
            new_pose = Pose(
                position=self.robot_pose.pose.pose.position,
                orientation=Quaternion(*new_rot_matrix.as_quat()),
            )

            blackboard["target_pose"] = new_pose

            return "succeeded"

    def __init__(self, angle: Optional[float] = None):
        super().__init__(outcomes=["succeeded", "failed"])
        if angle in None:
            self.add_input_key("angle")
        self.angle = angle

    
        self.add_state(
            "GET_ROTATED_POSE",
            self.GetRotatedPose(angle),
            transitions={"succeeded": "ROTATE"},
        )
        self.add_state(
            "ROTATE",
            GoToLocation(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
            remapping={"location": "target_pose"},
        )
