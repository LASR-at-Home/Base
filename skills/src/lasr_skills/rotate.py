from typing import Optional

import numpy as np
import rospy
import smach
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from lasr_skills import GoToLocation
from scipy.spatial.transform import Rotation as R


class Rotate(smach.StateMachine):

    class GetRotatedPose(smach.State):
        def __init__(self, angle: Optional[float] = None):
            super().__init__(
                outcomes=["succeeded"],
                input_keys=["angle"] if angle is None else [],
                output_keys=["target_pose"],
            )

        def execute(self, userdata):
            robot_pose_with_covariance = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            )
            robot_pose = PoseStamped(
                pose=robot_pose_with_covariance.pose.pose,
                header=robot_pose_with_covariance.header,
            )
            current_orientation = np.array(
                [
                    robot_pose.pose.orientation.x,
                    robot_pose.pose.orientation.y,
                    robot_pose.pose.orientation.z,
                    robot_pose.pose.orientation.w,
                ]
            )

            rot_matrix = R.from_quat(current_orientation)
            new_rot_matrix = rot_matrix * R.from_euler(
                "z", userdata.angle, degrees=True
            )
            new_pose = Pose(
                position=robot_pose.pose.position,
                orientation=Quaternion(*new_rot_matrix.as_quat()),
            )

            userdata.target_pose = new_pose

            return "succeeded"

    def __init__(self, angle: Optional[float] = None):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["angle"] if angle is None else [],
        )

        with self:
            smach.StateMachine.add(
                "GET_ROTATED_POSE",
                self.GetRotatedPose(angle),
                transitions={"succeeded": "ROTATE"},
            )
            smach.StateMachine.add(
                "ROTATE",
                GoToLocation(safe_navigation=False),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"location": "target_pose"},
            )
