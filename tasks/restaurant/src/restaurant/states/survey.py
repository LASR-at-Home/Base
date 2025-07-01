import navigation_helpers
import rospy
import smach
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from lasr_skills import PlayMotion, Rotate, Detect3D, DetectGesture
from cv_bridge import CvBridge
import cv2
import numpy as np


class Survey(smach.StateMachine):

    class HandleDetections(smach.StateMachine):
        class GetResponse(smach.State):
            def __init__(self):
                super().__init__(
                    outcomes=["succeeded", "failed"],
                    input_keys=["responses", "image_raw"],
                    output_keys=[
                        "response",
                        "responses",
                        "person_point",
                        "cropped_image",
                    ],
                )
                self.bridge = CvBridge()

            def execute(self, userdata):
                if len(userdata.responses.detected_objects) == 0:
                    rospy.logwarn("No response available, returning failed.")
                    return "failed"

                response = userdata.responses.detected_objects.pop(0)
                userdata.response = response

                cv_im = self.bridge.imgmsg_to_cv2(
                    userdata.image_raw, desired_encoding="rgb8"
                )
                mask = np.array(response.xyseg).reshape(-1, 2)
                stencil = np.zeros(cv_im.shape).astype(cv_im.dtype)
                colour = (255, 255, 255)
                cv2.fillPoly(stencil, [mask], colour)
                # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
                # with black elsewhere.
                masked_image = cv2.bitwise_and(cv_im, stencil)
                userdata.cropped_image = masked_image
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

        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["responses", "image_raw"],
                output_keys=["responses", "customer_approach_pose"],
            )

            with self:

                smach.StateMachine.add(
                    "GET_RESPONSE",
                    self.GetResponse(),
                    transitions={
                        "succeeded": "COMPUTE_APPROACH_POSE",
                        "failed": "failed",
                    },
                )

                smach.StateMachine.add(
                    "DETECT_GESTURE",
                    DetectGesture("waving"),
                    transitions={
                        "succeeded": "COMPUTE_APPROACH_POSE",
                        "failed": "GET_RESPONSE",
                    },
                    remapping={"img_msg": "cropped_image"},
                )

                smach.StateMachine.add(
                    "COMPUTE_APPROACH_POSE",
                    self.ComputeApproachPose(),
                    transitions={"succeeded": "succeeded", "failed": "GET_RESPONSE"},
                )

    def __init__(self) -> None:
        super().__init__(
            outcomes=["customer_found", "customer_not_found"],
            output_keys=["customer_approach_pose"],
        )

        with self:

            angle_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=rospy.get_param("/restaurant/survey_angle_increments"),
                it_label="angle_increment",
                input_keys=[],
                output_keys=["customer_approach_pose"],
                exhausted_outcome="failed",
            )

            with angle_iterator:

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
                            Detect3D(filter=["person"]),
                            transitions={
                                "succeeded": "HANDLE_DETECTIONS",
                                "failed": "failed",
                            },
                        )

                        smach.StateMachine.add(
                            "HANDLE_DETECTIONS",
                            self.HandleDetections(),
                            transitions={
                                "succeeded": "succeeded",
                                "failed": "continue",
                            },
                            remapping={
                                "responses": "detections_3d",
                                "image_raw": "image_raw",
                            },
                        )

                    motion_iterator.set_contained_state(
                        "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                    )

                angle_container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["angle_increment"],
                    output_keys=["customer_approach_pose"],
                )
                with angle_container_sm:

                    smach.StateMachine.add(
                        "ROTATE",
                        Rotate(),
                        transitions={"succeeded": "MOTION_ITERATOR"},
                        remapping={"angle": "angle_increment"},
                    )

                    smach.StateMachine.add(
                        "MOTION_ITERATOR",
                        motion_iterator,
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "failed",
                        },
                    )
                angle_iterator.set_contained_state(
                    "CONTAINER_STATE", angle_container_sm, loop_outcomes=["continue"]
                )

            smach.StateMachine.add(
                "ANGLE_ITERATOR",
                angle_iterator,
                transitions={
                    "succeeded": "customer_found",
                    "failed": "customer_not_found",
                },
            )
