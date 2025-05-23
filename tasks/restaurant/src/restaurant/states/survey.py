from typing import Optional

import navigation_helpers
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from lasr_skills import DetectGesture, PlayMotion, Say
from lasr_vision_msgs.msg import CDRequest
from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest


class Survey(smach.StateMachine):

    class HandleDetections(smach.StateMachine):
        class GetResponse(smach.State):
            def __init__(self):
                super().__init__(
                    outcomes=["succeeded", "failed"],
                    input_keys=["responses"],
                    output_keys=[
                        "response",
                        "responses",
                        "person_point",
                        "cropped_image",
                    ],
                )

            def execute(self, userdata):
                if len(userdata.responses[0].detections_3d) == 0:
                    rospy.logwarn("No response available, returning failed.")
                    return "failed"
                response = userdata.responses[0].detections_3d.pop(0)
                userdata.response = response
                userdata.cropped_image = userdata.responses[0].cropped_imgs.pop(0)
                userdata.person_point = response.point
                return "succeeded"

        class ComputeApproachPose(smach.State):

            def __init__(self):

                super().__init__(
                    outcomes=["succeeded", "failed"],
                    input_keys=["person_point"],
                    output_keys=["customer_approach_pose"],
                )

            def execute(self, userdata):
                robot_pose_with_covariance = rospy.wait_for_message(
                    "/robot_pose", PoseWithCovarianceStamped
                )
                robot_pose = PoseStamped(
                    pose=robot_pose_with_covariance.pose.pose,
                    header=robot_pose_with_covariance.header,
                )

                person_pose = PoseStamped(
                    pose=Pose(
                        position=userdata.person_point,
                        orientation=robot_pose.pose.orientation,
                    ),
                    header=robot_pose.header,
                )
                approach_pose = navigation_helpers.get_pose_on_path(
                    robot_pose,
                    person_pose,
                )
                rospy.loginfo(approach_pose)

                if approach_pose is None:
                    return "failed"

                approach_pose.pose.orientation = navigation_helpers.compute_face_quat(
                    approach_pose.pose,
                    person_pose.pose,
                )
                userdata.customer_approach_pose = approach_pose.pose

                return "succeeded"

        def __init__(self, table_pose: Optional[Pose] = None):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["responses"],
                output_keys=["responses", "customer_approach_pose"],
            )

            with self:

                smach.StateMachine.add(
                    "GET_RESPONSE",
                    self.GetResponse(),
                    transitions={"succeeded": "DETECT_GESTURE", "failed": "failed"},
                )

                smach.StateMachine.add(
                    "DETECT_GESTURE",
                    DetectGesture("waving"),
                    transitions={
                        "succeeded": (
                            "COMPUTE_APPROACH_POSE" if not table_pose else "SET_POSE"
                        ),
                        "failed": "GET_RESPONSE",
                    },
                    remapping={"img_msg": "cropped_image"},
                )

                @smach.cb_interface(
                    output_keys=["customer_approach_pose"], outcomes=["succeeded"]
                )
                def set_approach_pose(ud):
                    ud.customer_approach_pose = table_pose
                    return "succeeded"

                if table_pose is not None:
                    smach.StateMachine.add(
                        "SET_POSE",
                        smach.CBState(set_approach_pose),
                        transitions={"succeeded": "succeeded"},
                    )
                else:
                    smach.StateMachine.add(
                        "COMPUTE_APPROACH_POSE",
                        self.ComputeApproachPose(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                    )

    def __init__(self, table_pose: Optional[Pose] = None) -> None:
        super().__init__(
            outcomes=["customer_found", "customer_not_found"],
            output_keys=["customer_approach_pose"],
        )

        with self:

            motion_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=rospy.get_param("/restaurant/survey_motions"),
                it_label="motion_name",
                input_keys=[],
                output_keys=["customer_approach_pose"],
                exhausted_outcome="failed",
            )

            with motion_iterator:

                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["motion_name"],
                    output_keys=["customer_approach_pose"],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "SURVEY_MOTION",
                        PlayMotion(),
                        transitions={
                            "succeeded": "DETECT",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT",
                        smach_ros.ServiceState(
                            "/vision/cropped_detection",
                            CroppedDetection,
                            request=CroppedDetectionRequest(
                                requests=[
                                    CDRequest(
                                        method="closest",
                                        use_mask=True,
                                        yolo_model="yolov8x-seg.pt",
                                        yolo_model_confidence=0.5,
                                        yolo_nms_threshold=0.3,
                                        return_sensor_reading=False,
                                        object_names=["person"],
                                        polygons=[],
                                    )
                                ]
                            ),
                            output_keys=["responses"],
                            response_slots=["responses"],
                        ),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        self.HandleDetections(table_pose),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "continue",
                        },
                    )

                motion_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                )

            smach.StateMachine.add(
                "MOTION_ITERATOR",
                motion_iterator,
                transitions={
                    "succeeded": "customer_found",
                    "failed": "customer_not_found",
                },
            )
