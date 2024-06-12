import smach_ros
import smach
import actionlib
import rospy
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
<<<<<<< HEAD
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
=======
from actionlib_msgs.msg import GoalStatus
>>>>>>> 16ff9e135b15e7fb0aeeeb5fe82ec14f7e0b2d2f


class LookToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
<<<<<<< HEAD
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
=======
            outcomes=["succeeded", "aborted", "timed_out"],
            input_keys=["point"],
        )
        self.client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        self.client.wait_for_server()

    def execute(self, userdata):
        # Define the goal
        goal = PointHeadGoal(
            pointing_frame="head_2_link",
            pointing_axis=Point(1.0, 0.0, 0.0),
            max_velocity=1.0,
            target=PointStamped(
                header=Header(frame_id="map"),
                point=userdata.point,
            ),
        )

        # Send the goal
        self.client.send_goal(goal)

        # Wait for the result with a timeout of 7 seconds
        finished_within_time = self.client.wait_for_result(rospy.Duration(7.0))

        if finished_within_time:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return "succeeded"
            else:
                return "aborted"
        else:
            self.client.cancel_goal()
            return "timed_out"
>>>>>>> 16ff9e135b15e7fb0aeeeb5fe82ec14f7e0b2d2f
