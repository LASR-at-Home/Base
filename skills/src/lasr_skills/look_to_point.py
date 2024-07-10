import smach
import actionlib
import rospy
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from actionlib_msgs.msg import GoalStatus

from typing import Union


class LookToPoint(smach.State):

    _pointstamped: Union[None, PointStamped]

    def __init__(
        self,
        pointstamped: Union[None, PointStamped] = None,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted", "timed_out"],
            input_keys=["pointstamped"] if pointstamped is None else [],
        )

        self._pointstamped = pointstamped

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
            target=(
                self._pointstamped
                if self._pointstamped is not None
                else userdata.pointstamped
            ),
        )

        # Send the goal
        self.client.send_goal(goal)

        # Wait for the result with a timeout of 7 seconds
        finished_within_time = self.client.wait_for_result(rospy.Duration(2.0))

        if finished_within_time:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return "succeeded"
            else:
                return "aborted"
        else:
            self.client.cancel_goal()
            return "timed_out"
