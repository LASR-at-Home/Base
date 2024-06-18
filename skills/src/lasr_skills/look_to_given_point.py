"""Similar to look_to_point but the input key is replaced with a passed argument for the point to look at"""

import smach_ros
import smach
import actionlib
import rospy
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus
from typing import List


class LookToGivenPoint(smach.State):
    def __init__(self, look_position: List[float]):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted", "timed_out"],
        )
        self.goal_pointstamped = PointStamped(
                point=Point(x=look_position[0], y=look_position[1], z=1.0)
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
                point=self.goal_pointstamped.point,
            ),
        )

        # Send the goal
        self.client.send_goal(goal)

        # Wait for the result with a timeout of 7 seconds
        finished_within_time = self.client.wait_for_result(rospy.Duration(7.0))

        if finished_within_time:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.sleep(1)
                return "succeeded"
            else:
                return "aborted"
        else:
            self.client.cancel_goal()
            return "timed_out"
