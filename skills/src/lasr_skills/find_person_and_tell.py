#!/usr/bin/env python3
import smach
import smach_ros
import rospy

from lasr_skills import (
    GoToLocation,
    AskAndListen,
    DetectGesture,
    DetectClothing,
    DetectPose,
)
import navigation_helpers

from geometry_msgs.msg import (
    Pose,
    PoseWithCovarianceStamped,
    Polygon,
    PoseStamped,
    Point,
    Quaternion,
)
from lasr_vision_msgs.msg import CDRequest, CDResponse
from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetection,
)


from typing import List, Literal


class FindPersonAndTell(smach.StateMachine):
    class GetLocation(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location_index", "waypoints"],
                output_keys=["location"],
            )

        def execute(self, userdata) -> str:
            userdata.location = userdata.waypoints[userdata.location_index]
            return "succeeded"

    class ComputePath(smach.State):
        def __init__(self, waypoints: List[Pose]):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["current_pose"],
                output_keys=["waypoints"],
            )
            self._waypoints = waypoints

        def execute(self, userdata) -> str:
            current_pose: Pose = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            ).pose.pose
            userdata.waypoints = navigation_helpers.min_hamiltonian_path(
                current_pose, self._waypoints
            )
            return "succeeded"

    class HandleDetections(smach.StateMachine):
        class GetResponse(smach.State):
            def __init__(self):
                smach.State.__init__(
                    self,
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

        class ApproachPerson(smach.StateMachine):
            class ComputeApproachPose(smach.State):
                def __init__(self):
                    smach.State.__init__(
                        self,
                        outcomes=["succeeded", "failed"],
                        input_keys=["person_point"],
                        output_keys=["approach_pose"],
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

                    approach_pose.pose.orientation = (
                        navigation_helpers.compute_face_quat(
                            approach_pose.pose,
                            person_pose.pose,
                        )
                    )
                    userdata.approach_pose = approach_pose.pose

                    return "succeeded"

            def __init__(self):
                smach.StateMachine.__init__(
                    self,
                    outcomes=["succeeded", "failed"],
                    input_keys=["person_point"],
                )

                with self:

                    smach.StateMachine.add(
                        "COMPUTE_APPROACH_POSE",
                        self.ComputeApproachPose(),
                        transitions={"succeeded": "GO_TO_PERSON", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        GoToLocation(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "failed",
                        },
                        remapping={"location": "approach_pose"},
                    )

        def __init__(
            self,
            query: Literal["name", "pose", "gesture", "clothes"],
        ):
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["responses"],
                output_keys=["responses", "query_result"],
            )

            with self:

                if query == "name":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "GO_TO_PERSON",
                            "failed": "failed",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        self.ApproachPerson(),
                        transitions={
                            "succeeded": "ASK_NAME",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"location": "approach_pose"},
                    )

                    smach.StateMachine.add(
                        "ASK_NAME",
                        AskAndListen(
                            "What is your name?",
                        ),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "failed",
                        },
                        remapping={"transcribed_speceh": "query_result"},
                    )

                elif query == "gesture":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={"succeeded": "DETECT_GESTURE", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "DETECT_GESTURE",
                        DetectGesture(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={
                            "img_msg": "cropped_image",
                            "detected_gesture": "query_result",
                        },
                    )

                elif query == "pose":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_POSE",
                            "failed": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_POSE",
                        DetectPose(),
                        transitions={
                            "succeeded": "GO_TO_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={
                            "img_msg": "cropped_image",
                            "detected_pose": "query_result",
                        },
                    )

    def __init__(
        self,
        waypoints: List[Pose],
        polygon: Polygon,
        query: Literal["name", "pose", "gesture"],
    ):

        assert query in ["name", "pose", "gesture"], "Invalid query"

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["query_result"]
        )

        with self:

            smach.StateMachine.add(
                "COMPUTE_PATH",
                self.ComputePath(waypoints),
                transitions={"succeeded": "WAYPOINT_ITERATOR", "failed": "failed"},
            )

            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=lambda: range(len(waypoints)),
                it_label="location_index",
                input_keys=["waypoints"],
                output_keys=["person_point"],
                exhausted_outcome="failed",
            )

            with waypoint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["location_index", "waypoints"],
                    output_keys=["person_point"],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "GET_LOCATION",
                        self.GetLocation(),
                        transitions={"succeeded": "GO_TO_LOCATION", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_LOCATION",
                        GoToLocation(),
                        transitions={
                            "succeeded": "DETECT",
                            "failed": "failed",
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
                                        yolo_model="yolo11n-seg.pt",
                                        yolo_model_confidence=0.5,
                                        yolo_nms_threshold=0.3,
                                        return_sensor_reading=False,
                                        object_names=["person"],
                                        polygons=[polygon],
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
                        self.HandleDetections(query),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "continue",
                        },
                    )
                waypoint_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "WAYPOINT_ITERATOR",
                waypoint_iterator,
                {"succeeded": "succeeded", "failed": "failed"},
            )
