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
            target_angle_deg = self.angle if self.angle is not None else userdata.angle
            target_angle_rad = np.deg2rad(target_angle_deg)

            try:
                # Get starting yaw
                initial_pose = rospy.wait_for_message(
                    "/robot_pose", PoseWithCovarianceStamped
                )
                q = initial_pose.pose.pose.orientation
                _, _, last_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                total_rotated = 0.0
                direction = np.sign(target_angle_rad)
                twist = Twist()
                twist.angular.z = direction * abs(self.angular_speed)

                rate = rospy.Rate(20)
                while not rospy.is_shutdown() and abs(total_rotated) < abs(
                    target_angle_rad
                ):
                    self.cmd_vel_pub.publish(twist)

                    # Wait for next yaw
                    pose = rospy.wait_for_message(
                        "/robot_pose", PoseWithCovarianceStamped
                    )
                    q = pose.pose.pose.orientation
                    _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                    delta_yaw = current_yaw - last_yaw
                    # Normalize the delta to [-pi, pi]
                    delta_yaw = np.arctan2(np.sin(delta_yaw), np.cos(delta_yaw))
                    total_rotated += delta_yaw
                    last_yaw = current_yaw

                    rate.sleep()

                # Stop the robot
                self.cmd_vel_pub.publish(Twist())
                return "succeeded"

            except Exception as e:
                rospy.logerr(f"Rotation failed: {e}")
                self.cmd_vel_pub.publish(Twist())
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
