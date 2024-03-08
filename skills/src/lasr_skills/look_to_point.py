import smach
import actionlib
from control_msgs.msg import PointHeadGoal
from control_msgs.msg import PointHeadAction
from geometry_msgs.msg import Point

class LookToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['point'])
        self.point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)

    def execute(self, userdata):
        ph_goal = PointHeadGoal()
        ph_goal.max_velocity = 1.0
        ph_goal.pointing_frame = 'head_2_link'
        ph_goal.pointing_axis = Point(1.0, 0.0, 0.0)
        ph_goal.target.header.frame_id = 'map'
        ph_goal.target.point = userdata.point
        self.point_head_client.send_goal_and_wait(ph_goal)
        return 'succeeded'