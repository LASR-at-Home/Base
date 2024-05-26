import smach_ros
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import smach
import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point, PointStamped


# class LookToPoint(smach_ros.SimpleActionState):
#     def __init__(self):
#         super(LookToPoint, self).__init__(
#             "/head_controller/point_head_action",
#             PointHeadAction,
#             goal_cb=lambda ud, _: PointHeadGoal(
#                 pointing_frame="head_2_link",
#                 pointing_axis=Point(1.0, 0.0, 0.0),
#                 max_velocity=1.0,
#                 target=ud.pointstamped,
#             ),
#             input_keys=["pointstamped"],
#         )


class LookToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "preempted", "aborted"],
            input_keys=["pointstamped"],
        )
        self.look_at_pub = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )

    def execute(self, userdata):
        goal = PointHeadGoal()
        goal.pointing_frame = "head_2_link"
        goal.pointing_axis = Point(1.0, 0.0, 0.0)
        goal.max_velocity = 1.0
        goal.target = userdata.pointstamped
        self.look_at_pub.send_goal(goal)
        self.look_at_pub.wait_for_result()
        rospy.sleep(3.0)
        return "succeeded"
