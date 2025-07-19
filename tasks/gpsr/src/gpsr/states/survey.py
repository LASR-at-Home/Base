from typing import Tuple, Optional
import rospy
import smach
import smach_ros
import numpy as np
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from lasr_skills import DetectHandUp3D, GetImageAndDepthImage, GoToLocation

from shapely import Polygon as ShapelyPolygon


class AnalyseDetections(smach.State):
    """
    State to check the number of hands_up detections"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["customer_found", "customer_not_found"],
            input_keys=["hands_up_detections"],
        )

    def execute(self, userdata):
        if len(userdata.hands_up_detections) > 0:
            return "customer_found"
        else:
            return "customer_not_found"


class Survey(smach.StateMachine):
    """
    Looks left and right, detecting waving people.
    Outputs a list of 3D-Detections of people with their hands up.
    """

    _same_detection_threshold: float
    _expected_no_customers: int

    def __init__(
        self,
        look_range_deg: Tuple[float, float],
        n_look_points: int,
        same_detection_threshold: float = 0.4,
        expected_no_customers: int = 2,
        polygon: Optional[ShapelyPolygon] = None,
    ) -> None:
        super().__init__(
            outcomes=["customer_found", "customer_not_found"],
            input_keys=["hands_up_detections"],
            output_keys=["hands_up_detections"],
        )

        look_range = (np.deg2rad(look_range_deg[0]), np.deg2rad(look_range_deg[1]))
        look_points = np.linspace(look_range[0], look_range[1], n_look_points)
        self._same_detection_threshold = same_detection_threshold
        self._expected_no_customers = expected_no_customers
        self.userdata.hands_up_detections = []

        with self:

            joint_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                input_keys=["hands_up_detections"],
                output_keys=["hands_up_detections"],
                it=look_points,
                it_label="head_1_joint",
                exhausted_outcome="succeeded",
            )

            with joint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "failed", "succeeded"],
                    input_keys=["head_1_joint", "hands_up_detections"],
                    output_keys=["hands_up_detections"],
                )
                with container_sm:
                    smach.StateMachine.add(
                        "LOOK",
                        smach_ros.SimpleActionState(
                            "/head_controller/follow_joint_trajectory",
                            FollowJointTrajectoryAction,
                            goal_cb=lambda ud, _: FollowJointTrajectoryGoal(
                                trajectory=JointTrajectory(
                                    joint_names=["head_1_joint", "head_2_joint"],
                                    points=[
                                        JointTrajectoryPoint(
                                            positions=[ud.head_1_joint, 0.0],
                                            time_from_start=rospy.Duration.from_sec(
                                                1.0
                                            ),
                                        )
                                    ],
                                )
                            ),
                            input_keys=["head_1_joint"],
                        ),
                        transitions={
                            "succeeded": "GET_IMAGE_AND_DEPTH",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "GET_IMAGE_AND_DEPTH",
                        GetImageAndDepthImage(),
                        transitions={
                            "succeeded": "DETECT_HAND_UP",
                            "failed": "GET_IMAGE_AND_DEPTH",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_HAND_UP",
                        DetectHandUp3D(target_frame="map", polygon=polygon),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "failed": "continue",
                        },
                    )

                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        smach.CBState(
                            self._handle_detections,
                            input_keys=["detected_people", "hands_up_detections"],
                            output_keys=["hands_up_detections"],
                            outcomes=["succeeded"],
                        ),
                        transitions={"succeeded": "continue"},
                    )
                smach.Iterator.set_contained_state(
                    "CONTAINER_SM",
                    container_sm,
                    loop_outcomes=["continue"],
                )
            smach.StateMachine.add(
                "LOOK_AND_DETECT",
                joint_iterator,
                transitions={
                    "succeeded": "ANALYSE_DETECTIONS",
                    "failed": "ANALYSE_DETECTIONS",
                },
            )

            smach.StateMachine.add(
                "ANALYSE_DETECTIONS",
                AnalyseDetections(),
                transitions={
                    "customer_found": "customer_found",
                    "customer_not_found": "customer_not_found",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )

    def _handle_detections(self, userdata) -> str:
        """
        userdata.detected_people: List of Detection3D objects containing
        detections of people with hands up. Sorted from far to near.

        Name of the detection is the string of the distance to the person
        e.g., "1.25m".

        """
        new_detections = userdata.detected_people

        for detection in new_detections:
            x, y = detection.point.x, detection.point.y
            is_new = all(
                math.dist([x, y], [existing.point.x, existing.point.y])
                >= self._same_detection_threshold
                for existing in userdata.hands_up_detections
            )
            if is_new:
                userdata.hands_up_detections.append(detection)

        if len(userdata.hands_up_detections) > self._expected_no_customers:
            rospy.logwarn(
                f"Detected {len(userdata.hands_up_detections)} waving people, "
                f"but expected {self._expected_no_customers}."
            )

        return "succeeded"
