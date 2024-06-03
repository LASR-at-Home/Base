import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal


class GoToCounter(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.async_tts(
            "I am going to the counter to retrieve the order"
        )
        position = rospy.get_param("counter/location/position")
        orientation = rospy.get_param("counter/location/orientation")
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        return "done"
