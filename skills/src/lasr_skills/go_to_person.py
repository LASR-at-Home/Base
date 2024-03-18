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
import math
from scipy.spatial.transform import Rotation as R

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

            dist_x = points[0].position.x - robot_pose.pose.pose.position.x
            dist_y = points[0].position.y - robot_pose.pose.pose.position.y
            theta_deg = math.degrees(math.atan2(dist_y, dist_x))
            x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
            points[0].orientation.x = x
            points[0].orientation.y = y
            points[0].orientation.z = z
            points[0].orientation.w = w
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
