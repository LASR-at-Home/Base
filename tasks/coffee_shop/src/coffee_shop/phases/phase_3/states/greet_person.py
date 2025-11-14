import smach
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point


class GreetPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        self.context.say(
            "Hi there! My name is TIAGO. Please follow me, I'll guide you to a table."
        )
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.start_head_manager("head_manager", "")
        return "done"
