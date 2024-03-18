import smach
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from navigation_helpers import plan_to_radius, points_on_radius

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseWithCovarianceStamped,
    Quaternion,
    Point,
    PointStamped,
)
from std_msgs.msg import Header
import rospy
from tf.transformations import quaternion_about_axis
import math
import numpy as np
from scipy.spatial.transform import Rotation
import markers

from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi


class GoToPerson(smach.StateMachine):

    class PlanToPerson(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["point"],
                output_keys=["target_location"],
            )
            self.marker_pub = rospy.Publisher("/points", Marker, queue_size=100)

        def execute(self, userdata):
            robot_pose = rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped
            )  # Pose
            point = userdata.point  # Point
            points = plan_to_radius(robot_pose, point, 2.0)
            for candidate in points:
                markers.create_and_publish_marker(
                    self.marker_pub,
                    PointStamped(
                        header=Header(frame_id="map"), point=candidate.position
                    ),
                )

            userdata.target_location = points[0]
            # candidates = points_on_radius(point, 2.0)
            # for candidate in candidates:
            #     markers.create_and_publish_marker(
            #         self.marker_pub,
            #         PointStamped(header=Header(frame_id="map"), point=candidate),
            #     )

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
                SimpleActionState(
                    "move_base",
                    MoveBaseAction,
                    goal_cb=lambda ud, _: MoveBaseGoal(
                        target_pose=PoseStamped(
                            pose=ud.target_location, header=Header(frame_id="map")
                        )
                    ),
                    input_keys=["target_location"],
                ),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
