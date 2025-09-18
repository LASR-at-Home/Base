import smach
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
from move_base_msgs.msg import MoveBaseGoal


class GoCloserToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        self.context.say(
            "I think there is a customer waiting. I will go and investigate."
        )
        location = rospy.get_param("/coffee_shop/wait/approach1")
        position, orientation = location["position"], location["orientation"]
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        return "done"