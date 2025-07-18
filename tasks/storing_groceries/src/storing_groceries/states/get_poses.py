
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
            output_keys=["survey_pose", "bar_pose"],
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

        # Rotate yaw by 180 degrees
        rotated_yaw = yaw + math.pi
        # Normalise
        rotated_yaw = (rotated_yaw + math.pi) % (2 * math.pi) - math.pi
        # Convert back to quaternion
        new_quat = tf.transformations.quaternion_from_euler(0, 0, rotated_yaw)
        # Construct pose
        rotated_pose = Pose()
        rotated_pose.position = pose.position
        rotated_pose.orientation.x = new_quat[0]
        rotated_pose.orientation.y = new_quat[1]
        rotated_pose.orientation.z = new_quat[2]
        rotated_pose.orientation.w = new_quat[3]

        # Rotate the pose by 180 degrees and store as bar_pose
        userdata.bar_pose = rotated_pose

        return "succeeded"
