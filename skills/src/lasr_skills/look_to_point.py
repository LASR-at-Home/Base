import smach_ros
import smach
import actionlib
import rospy
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus


class LookToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "preempted", "aborted"],
            input_keys=["pointstamped"],
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
                point=userdata.pointstamped.point,
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
            return "succeeded"
            # return "timed_out"
