import smach
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
from move_base_msgs.msg import MoveBaseGoal
from lasr_skills import Say, LookToPoint

class GoCloserToPerson(smach.StateMachine):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:
            smach.StateMachine.add(
                "SAY_CUSTOMER_WAITING",
                Say(text="I think there is a customer waiting. I will go and investigate."),
                transitions={
                    "succeeded": "APPROACH_PERSON",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "APPROACH_PERSON",
                ApproachPerson(context),
                transitions={"done": "done"},
            )

class ApproachPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        # self.context.voice_controller.async_tts(
        #     "I think there is a customer waiting. I will go and investigate."
        # )
        location = rospy.get_param("/wait/approach1")
        position, orientation = location["position"], location["orientation"]
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        return "done"
