from typing import Literal, Optional

import rospy
import smach
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from scipy.spatial.transform import Rotation as R


class Rotate(smach.State):

    _angular_vel: float = 0.5
    _tol: float = 1.0
    angle: Optional[float]

    def __init__(self, angle: Optional[float] = None):
        super().__init__(
            outcomes=["succeeded"], input_keys=["angle"] if angle is None else []
        )
        self.angle = angle
        self._cmd_vel_pub = rospy.Publisher(
            "/mobile_base_controller/cmd_vel", Twist, queue_size=10, latch=True
        )

    def execute(self, userdata) -> Literal["succeeded"]:
        initial_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        initial_orientation = [
            initial_pose.pose.pose.orientation.x,
            initial_pose.pose.pose.orientation.y,
            initial_pose.pose.pose.orientation.z,
            initial_pose.pose.pose.orientation.w,
        ]
        initial_rotation = R.from_quat(initial_orientation)

        accumulated_rotation = 0.0

        twist = Twist()
        angle = self.angle or userdata.angle
        twist.angular.z = self._angular_vel if angle > 0 else -self._angular_vel

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            current_pose = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            )
            current_orientation = [
                current_pose.pose.pose.orientation.x,
                current_pose.pose.pose.orientation.y,
                current_pose.pose.pose.orientation.z,
                current_pose.pose.pose.orientation.w,
            ]
            current_rotation = R.from_quat(current_orientation)

            delta_rotation = initial_rotation.inv() * current_rotation
            delta_angle = delta_rotation.as_euler("xyz", degrees=True)[2]
            accumulated_rotation += delta_angle

            initial_rotation = current_rotation

            if abs(accumulated_rotation) >= abs(angle) - self._tol:
                break
            self._cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.angular.z = 0
        self._cmd_vel_pub.publish(twist)

        return "succeeded"
