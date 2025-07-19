import rospy
import smach
import math
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class GetPoses(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            output_keys=["survey_pose", "bar_pose", "bar_forward_pose"],
        )

    def execute(self, userdata):
        # Wait for the current robot pose
        msg = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        pose = msg.pose.pose

        # Store the current pose as survey_pose
        userdata.survey_pose = pose

        # Extract quaternion
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        # Convert to Euler angles
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)

        # ----------------------------------
        # bar_pose: rotated 180 degrees
        rotated_yaw = yaw + math.pi
        rotated_yaw = (rotated_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize
        new_quat = tf.transformations.quaternion_from_euler(0, 0, rotated_yaw)

        rotated_pose = Pose()
        rotated_pose.position = pose.position
        rotated_pose.orientation.x = new_quat[0]
        rotated_pose.orientation.y = new_quat[1]
        rotated_pose.orientation.z = new_quat[2]
        rotated_pose.orientation.w = new_quat[3]

        userdata.bar_pose = rotated_pose

        # ----------------------------------
        # bar_forward_pose: 1 meter forward from original pose
        forward_pose = Pose()
        forward_pose.orientation = pose.orientation

        # Calculate 1 meter ahead in the direction of yaw
        forward_pose.position.x = pose.position.x + 2 * math.cos(yaw)
        forward_pose.position.y = pose.position.y + 2 * math.sin(yaw)
        forward_pose.position.z = pose.position.z  # unchanged

        userdata.bar_forward_pose = forward_pose

        return "succeeded"
