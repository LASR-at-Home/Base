from typing import List, Union, Optional

import rclpy
from rclpy.wait_for_message import wait_for_message

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


import yasmin
import yasmin_ros
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers, ServiceState
from yasmin_viewer import YasminViewerPub

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Point, PointStamped
from lasr_skills import GoToLocation
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class Rotate(StateMachine):
    class GetRotatedPose(State):
        def __init__(
                self,
                angle: Optional[float] = None,
                target_point: Optional[Point] = None,
                mode: Optional[str] = None,  # "angle" or "point"
            ):

            super().__init__(outcomes=["succeeded", "failed"])

            if angle is None:
                self.add_input_key("angle")
            self.angle = angle

            if target_point is None:
                self.add_input_key("target_point")
            self.target_point = target_point

            self.mode = mode   

            self.add_output_key("target_pose")

            self.robot_pose = None
            self.robot_pose_sub = yasmin_ros.logger_node.create_subscription(
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

        def calcuate_pose_from_angle(self):

            current_orientation = np.array(
                [
                    self.robot_pose.pose.pose.orientation.x,
                    self.robot_pose.pose.pose.orientation.y,
                    self.robot_pose.pose.pose.orientation.z,
                    self.robot_pose.pose.pose.orientation.w,
                ]
            )

            rot_matrix = R.from_quat(current_orientation)
            new_rot_matrix = rot_matrix * R.from_euler("z", self.angle, degrees=True
            )

            matrix = new_rot_matrix.as_quat()
            return Pose(
                position=self.robot_pose.pose.pose.position,
                orientation=Quaternion(
                    x=matrix[0],
                    y=matrix[1],
                    z=matrix[2],
                    w=matrix[3],
                ),
            )
        
        def calculate_pose_from_point(self):
            rx = self.robot_pose.pose.pose.position.x
            ry = self.robot_pose.pose.pose.position.y

            dx = self.target_point.x - rx
            dy = self.target_point.y - ry
            target_yaw = math.atan2(dy, dx)

            new_rot_matrix = R.from_euler('z', target_yaw, degrees=False)
            matrix = new_rot_matrix.as_quat()

            return Pose(
                position=self.robot_pose.pose.pose.position,
                orientation=Quaternion(
                    x=matrix[0],
                    y=matrix[1],
                    z=matrix[2],
                    w=matrix[3],
                ),
            )
        
        def execute(self, blackboard):

            if ("angle" in blackboard.keys() 
                and blackboard["angle"] is not None 
                and self.angle is None):
                self.angle = blackboard["angle"]

            if ("target_point" in blackboard.keys() 
                and blackboard["target_point"] is not None 
                and self.target_point is None):
                if isinstance(blackboard["target_point"], PointStamped):
                    self.target_point = blackboard["target_point"].point
                else:
                    self.target_point = blackboard["target_point"]

            goal = None

            if self.mode == "angle" and self.angle is not None:      # Rotate using angle
                goal = self.calcuate_pose_from_angle()
            elif self.mode == "point" and self.target_point is not None:    # Rotate to face point
                goal = self.calculate_pose_from_point()
            else:                           # Not Specified
                if self.angle is not None:
                    goal = self.calcuate_pose_from_angle()
                elif self.target_point is not None:
                    goal = self.calculate_pose_from_point()
                else:
                    yasmin.YASMIN_LOG_INFO("Rotation angle or target point not Specified")
                    return "failed"

            if goal is not None:
                blackboard["target_pose"] = goal
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Rotation Failed")
                return "failed"

    def __init__(
            self, 
            angle: Optional[float] = None, 
            target_point: Optional[Point] = None, 
            mode: Optional[str] = None):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("target_point")
        self.add_input_key("angle")

        self.add_state(
            "GET_ROTATED_POSE",
            self.GetRotatedPose(angle=angle, target_point=target_point, mode=mode),
            transitions={"succeeded": "ROTATE", "failed": "failed"},
        )
        self.add_state(
            "ROTATE",
            GoToLocation(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
            remappings={"location": "target_pose"},
        )

def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = Rotate(angle=180)
    sm.set_sigint_handler(True)
    bb = Blackboard()

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(outcome)

    rclpy.shutdown()

if __name__ == "__main__":
    main()