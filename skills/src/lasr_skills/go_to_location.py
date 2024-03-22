import smach_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class GoToLocation(smach_ros.SimpleActionState):

    def __init__(self):
        super(GoToLocation, self).__init__(
            "move_base",
            MoveBaseAction,
            goal_cb=lambda ud, _: MoveBaseGoal(
                target_pose=PoseStamped(pose=ud.location, header=Header(frame_id="map"))
            ),
            input_keys=["location"],
        )
