import rospy
import smach
import actionlib

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Spin(smach.State):
    """State that spins the robot in place by a specified angle."""

    _angle_radians: float
    _robot_pose: PoseWithCovarianceStamped
    _move_base_client: actionlib.SimpleActionClient

    def __init__(self, angle_radians: float):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
        )
        self._angle_radians = angle_radians
        self._robot_pose = None
        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self._move_base_client.wait_for_server()

    def _get_current_pose(self) -> None:
        """Sets the current pose of the robot in the map frame."""
        msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        self._robot_pose = msg.pose.pose

    def execute(self, userdata):
        self._get_current_pose()
        if self._robot_pose is None:
            rospy.logerr("Failed to get the robot's current pose.")
            return "failed"

        rospy.loginfo(f"Spinning robot by {self._angle_radians} radians.")

        current_orientation = R.from_quat(
            [
                self._robot_pose.orientation.x,
                self._robot_pose.orientation.y,
                self._robot_pose.orientation.z,
                self._robot_pose.orientation.w,
            ]
        )
        new_orientation = current_orientation * R.from_euler(
            "z", self._angle_radians, degrees=False
        )

        desired_pose = Pose(
            position=self._robot_pose.position,
            orientation=Quaternion(*new_orientation.as_quat()),
        )

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = desired_pose
        self._move_base_client.send_goal_and_wait(move_base_goal)

        return "succeeded"
