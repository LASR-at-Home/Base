import rospy
import smach_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


class GoToSemanticLocation(smach_ros.SimpleActionState):

    def __init__(self):
        super(GoToSemanticLocation, self).__init__(
            "move_base",
            MoveBaseAction,
            goal_cb=lambda ud, _: MoveBaseGoal(
                target_pose=PoseStamped(
                    pose=self._to_pose(rospy.get_param(f"{ud.location}/location")),
                    header=Header(frame_id="map"),
                )
            ),
            input_keys=["location"],
        )

    def _to_pose(self, location):
        return Pose(
            position=Point(**location["position"]),
            orientation=Quaternion(**location["orientation"]),
        )
