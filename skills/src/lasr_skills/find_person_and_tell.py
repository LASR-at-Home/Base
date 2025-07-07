#!/usr/bin/env python3
import smach
import smach_ros
import rospy

from lasr_skills import (
    GoToLocation,
    AskAndListen,
    DetectGesture,
    DetectPose,
    Detect3DInArea,
)
from lasr_skills.vision import CropImage3D
import navigation_helpers

from geometry_msgs.msg import (
    Pose,
    PoseWithCovarianceStamped,
    Polygon,
    PoseStamped,
    Point,
    Quaternion,
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
                    input_keys=["cropped_detections"],
                    output_keys=[
                        "detection",
                        "cropped_detections",
                        "person_point",
                        "cropped_image",
                    ],
                )

            def execute(self, userdata):
                if len(userdata.cropped_detections["person"]) == 0:
                    rospy.logwarn("No response available, returning failed.")
                    return "failed"
                closest_person = userdata.cropped_detections["person"].pop(0)
                userdata.cropped_detection = closest_person["detection_3d"]
                userdata.cropped_image = closest_person["cropped_image"]
                userdata.person_point = closest_person["detection_3d"].point
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
                input_keys=["cropped_detections"],
                output_keys=["cropped_detections", "query_result"],
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
                            "Please say Hi Tiago for me to begin listening. What is your name?",
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
                            "succeeded": "DETECT_IN_ROOM",
                            "failed": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_IN_ROOM",
                        Detect3DInArea(
                            area_polygon=polygon,
                            image_topic="/xtion/rgb/image_raw",
                            depth_image_topic="/xtion/depth_registered/image_raw",
                            depth_camera_info_topic="/xtion/depth_registered/camera_info",
                            model="yolo11n-seg.pt",
                            filter=["person"],
                            confidence=0.5,
                            target_frame="map",
                        ),
                        transitions={
                            "succeeded": "CROP_TO_CLOSEST_PERSON",
                            "failed": "failed",
                        },
                        remapping={
                            "detections_3d": "detections_3d",
                            "image_raw": "image_raw",
                        },
                    )

                    smach.StateMachine.add(
                        "CROP_TO_CLOSEST_PERSON",
                        CropImage3D(
                            robot_pose_topic="/robot_pose",
                            filters=["person"],
                            crop_logic="nearest",
                            crop_type="masked",
                        ),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "failed": "continue",
                        },
                        remapping={
                            "cropped_detections": "cropped_detections"  # Dict[List[Dict[str, Union[Image, Detection3D]]]]
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
