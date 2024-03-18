import smach
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from navigation_helpers import plan_to_radius

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Header
import rospy

from lasr_skills import GoToLocation


class GoToPerson(smach.StateMachine):

    class PlanToPerson(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["point"],
                output_keys=["target_location"],
            )

        def execute(self, userdata):
            robot_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
            points = plan_to_radius(robot_pose, userdata.point, 2.0)
            if not points:
                return "failed"
            userdata.target_location = points[0]
            return "succeeded"

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["point"],
        )

        with self:

            smach.StateMachine.add(
                "PLAN_TO_PERSON",
                self.PlanToPerson(),
                transitions={"succeeded": "GO_TO_LOCATION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_LOCATION",
                GoToLocation(),
                remapping={"location": "target_location"},
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
