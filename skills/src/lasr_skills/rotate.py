from typing import Optional

import numpy as np
import rospy
import smach
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class Rotate(smach.StateMachine):

    class RotateInPlace(smach.State):
        def __init__(self, angle: Optional[float] = None, angular_speed: float = 0.5):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["angle"] if angle is None else [],
            )
            self.angle = angle  # Degrees
            self.angular_speed = angular_speed  # Radians per second
            self.cmd_vel_pub = rospy.Publisher(
                "/mobile_base_controller/cmd_vel", Twist, queue_size=10
            )

        def execute(self, userdata):
            angle_to_rotate = self.angle if self.angle is not None else userdata.angle
            angle_to_rotate_rad = np.deg2rad(angle_to_rotate)

            try:
                # Get current yaw
                start_pose = rospy.wait_for_message(
                    "/robot_pose", PoseWithCovarianceStamped
                )
                q = start_pose.pose.pose.orientation
                _, _, start_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                # Calculate target yaw
                target_yaw = start_yaw + angle_to_rotate_rad
                target_yaw = np.arctan2(
                    np.sin(target_yaw), np.cos(target_yaw)
                )  # normalize

                rate = rospy.Rate(10)
                twist = Twist()

                while not rospy.is_shutdown():
                    current_pose = rospy.wait_for_message(
                        "/robot_pose", PoseWithCovarianceStamped
                    )
                    q = current_pose.pose.pose.orientation
                    _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                    error = target_yaw - current_yaw
                    error = np.arctan2(np.sin(error), np.cos(error))  # normalize again

                    if abs(error) < 0.01:  # tolerance in radians
                        break

                    twist.angular.z = (
                        self.angular_speed if error > 0 else -self.angular_speed
                    )
                    self.cmd_vel_pub.publish(twist)
                    rate.sleep()

                # Stop rotation
                self.cmd_vel_pub.publish(Twist())
                return "succeeded"
            except Exception as e:
                rospy.logerr(f"Rotation failed: {e}")
                return "failed"

    def __init__(self, angle: Optional[float] = None):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["angle"] if angle is None else [],
        )

        with self:
            smach.StateMachine.add(
                "ROTATE_IN_PLACE",
                self.RotateInPlace(angle),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
