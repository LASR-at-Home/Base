import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal


class GoToWaitLocation(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done", "not done"])
        self.context = context
        self.done = False

    def execute(self, userdata):
        wait_location = rospy.get_param("/wait")
        position, orientation = (
            wait_location["location"]["position"],
            wait_location["location"]["orientation"],
        )
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        return "not done"
