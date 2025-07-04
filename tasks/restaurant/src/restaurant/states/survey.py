from typing import Tuple
import rospy
import smach
import smach_ros
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from lasr_skills import Detect3D
import actionlib
import math


class Survey(smach.StateMachine):
    """
    Looks left and right, detecting waving people.
    At the end, outputs the closest waving person.
    """

    def __init__(
        self,
        look_range_deg: Tuple[float, float],
        n_look_points: int,
        same_detection_threshold: float = 0.4,
    ) -> None:
        super().__init__(
            outcomes=["customer_found", "customer_not_found"],
            output_keys=["customer_approach_pose"],
        )

        look_range = (np.deg2rad(look_range_deg[0]), np.deg2rad(look_range_deg[1]))
        look_points = np.linspace(look_range[0], look_range[1], n_look_points)
        self._same_detection_threshold = same_detection_threshold
        self.userdata.all_detections = []
        self.userdata.all_images = []

        with self:

            joint_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                input_keys=["all_detections", "all_images"],
                output_keys=["all_detections", "all_images"],
                it=look_points,
                it_label="head_1_joint",
                exhausted_outcome="succeeded",
            )

            with joint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "failed", "succeeded"],
                    input_keys=["head_1_joint", "all_detections", "all_images"],
                    output_keys=["all_detections", "all_images"],
                )
                with container_sm:
                    smach.StateMachine.add(
                        "LOOK_AND_DETECT",
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
                            "succeeded": "DETECT_PEOPLE",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "DETECT_PEOPLE",
                        Detect3D(filter=["person"]),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "failed": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        smach.CBState(
                            self._handle_detections,
                            input_keys=[
                                "all_detections",
                                "all_images",
                                "detections_3d",
                                "image_raw",
                            ],
                            output_keys=["all_detections", "all_images"],
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
                    "succeeded": "PRINT_RESULTS",
                    "failed": "customer_not_found",
                },
            )
            smach.StateMachine.add(
                "PRINT_RESULTS",
                smach.CBState(
                    self._print_detections,
                    input_keys=["all_detections", "all_images"],
                    outcomes=["succeeded"],
                ),
                transitions={"succeeded": "DETECTION_ITERATOR"},
            )

            detection_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                input_keys=["all_detections", "all_images"],
                output_keys=["all_waving"],
                it=self.userdata.all_detections,
                it_label="detection",
                exhausted_outcome="succeeded",
            )

            with detection_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "failed", "succeeded"],
                    input_keys=["detection", "all_waving"],
                    output_keys=["all_waving"],
                )
                with container_sm:
                    smach.StateMachine.add(
                        "PRINT_DETECTION",
                        smach.CBState(
                            self._print_individual_detection,
                            input_keys=["detection"],
                            outcomes=["succeeded"],
                        ),
                        transitions={"succeeded": "continue"},
                    )
                smach.Iterator.set_contained_state(
                    "CONTAINER_SM", container_sm, loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "DETECTION_ITERATOR",
                detection_iterator,
                transitions={
                    "succeeded": "customer_found",
                    "failed": "customer_not_found",
                },
            )

    def _handle_detections(self, userdata) -> str:
        new_detections = userdata.detections_3d.detected_objects

        for detection in new_detections:
            x, y = detection.point.x, detection.point.y
            is_new = all(
                math.dist([x, y], [existing.point.x, existing.point.y])
                >= self._same_detection_threshold
                for existing, _ in userdata.all_detections
            )
            if is_new:
                userdata.all_detections.append((detection, userdata.image_raw))

        userdata.all_images.append(userdata.image_raw)
        return "succeeded"

    def _print_detections(self, userdata) -> str:
        rospy.loginfo(f"There are {len(userdata.all_detections)} detections.")
        rospy.loginfo(userdata.all_detections)
        return "succeeded"

    def _print_individual_detection(self, userdata) -> str:
        rospy.loginfo(userdata.detection[0])
        return "succeeded"
