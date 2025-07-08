#!/usr/bin/env python3
import smach
import smach_ros
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from lasr_skills import (
    GoToLocation,
    DetectGesture,
    DetectClothing,
    DetectPose,
    Detect3DInArea,
)
from lasr_skills.vision import CropImage3D
import navigation_helpers
import difflib
from geometry_msgs.msg import (
    Pose,
    PoseWithCovarianceStamped,
    Polygon,
    PoseStamped,
    Point,
    Quaternion,
)

from typing import List, Literal
import itertools


class CountPeople(smach.StateMachine):

    class FilterPeople(smach.State):
        def __init__(self, distance_threshold: float = 0.25):
            super().__init__(
                outcomes=["succeeded"],
                input_keys=["all_people", "filtered_people"],
                output_keys=["filtered_people"],
            )
            # Distance in m for duplicate detection
            self.distance_threshold = distance_threshold

        def execute(self, userdata) -> str:
            if len(userdata.filtered_people) == 0:
                userdata.filtered_people.extend(userdata.all_people)
                return "succeeded"

            for potential_new_person in userdata.all_people:
                is_duplicate = False
                for existing_person in userdata.filtered_people:
                    distance = (
                        (potential_new_person.x - existing_person.x) ** 2
                        + (potential_new_person.y - existing_person.y) ** 2
                    ) ** 0.5
                    if distance < self.distance_threshold:
                        is_duplicate = True
                        break

                if not is_duplicate:
                    userdata.filtered_people.append(potential_new_person)

            return "succeeded"

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
                    input_keys=["cropped_images", "image_raw"],
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

        class AddPerson(smach.State):
            def __init__(self):
                smach.State.__init__(
                    self,
                    outcomes=["succeeded"],
                    input_keys=["person_point", "all_people"],
                    output_keys=["all_people"],
                )

            def execute(self, userdata):
                userdata.all_people.append(userdata.person_point)
                return "succeeded"

        def __init__(
            self,
            criteria: Literal["pose", "gesture", "clothes"],
            criteria_value: str,
        ):
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["cropped_detections", "all_people"],
                output_keys=["cropped_detections", "all_people"],
            )

            with self:

                if criteria == "gesture":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_GESTURE",
                            "failed": "succeeded",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_GESTURE",
                        DetectGesture(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            # "missing_keypoints": "GET_RESPONSE",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

                elif criteria == "pose":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_POSE",
                            "failed": "succeeded",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_POSE",
                        DetectPose(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

                elif criteria == "clothes":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_CLOTHING",
                            "failed": "succeeded",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_CLOTHING",
                        DetectClothing(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={
                            "img_msg": "cropped_image",
                            "detected_clothing": "detected_clothing",
                        },
                    )
                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

    class CountPeople(smach.State):

        def __init__(self):
            super().__init__(
                outcomes=["succeeded"],
                input_keys=["filtered_people"],
                output_keys=["people_count"],
            )

        def execute(self, userdata):
            count = len(userdata.filtered_people)
            rospy.loginfo(f"Number of people detected: {count}")
            userdata.people_count = count
            return "succeeded"

    def __init__(
        self,
        waypoints: List[Pose],
        polygon: Polygon,
        criteria: Literal["pose", "gesture", "clothes"],
        criteria_value: str,
    ):

        assert criteria in ["pose", "gesture", "clothes"], "Invalid criteria"

        if criteria == "gesture":
            assert criteria_value in [
                "raising_left_arm",
                "raising_right_arm",
                "pointing_to_the_right",
                "pointing_to_the_left",
                "waving",
            ], "Invalid gesture"
        elif criteria == "pose":
            assert criteria_value in [
                "sitting",
                "standing",
                "lying_down",
            ], "Invalid pose"
        elif criteria == "clothes":
            color_list = [
                "green",
                "blue",
                "yellow",
                "black",
                "white",
                "red",
                "orange",
                "gray",
                "brown",
            ]
            clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
            clothes_list = [
                "t shirts",
                "shirts",
                "blouses",
                "sweaters",
                "coats",
                "jackets",
            ]
            color_clothe_list: List[str] = []
            for a, b in list(itertools.product(color_list, clothe_list)):
                color_clothe_list = color_clothe_list + [a + " " + b]
            color_clothes_list: List[str] = []
            for a, b in list(itertools.product(color_list, clothes_list)):
                color_clothes_list = color_clothes_list + [a + " " + b]
            assert (
                criteria_value in color_clothe_list + color_clothes_list
            ), "Invalid clothing"

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["people_count"]
        )

        with self:

            self.userdata.all_people = []
            self.userdata.filtered_people = []

            smach.StateMachine.add(
                "COMPUTE_PATH",
                self.ComputePath(waypoints),
                transitions={"succeeded": "WAYPOINT_ITERATOR", "failed": "failed"},
            )

            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded"],
                it=lambda: range(len(waypoints)),
                it_label="location_index",
                input_keys=["waypoints", "all_people", "filtered_people"],
                output_keys=["all_people", "filtered_people"],
                exhausted_outcome="succeeded",
            )

            with waypoint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["waypoints", "location_index", "all_people"],
                    output_keys=[
                        "all_people",
                        "image_raw",
                        "cropped_detections",
                        "filtered_people",
                    ],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "GET_LOCATION",
                        self.GetLocation(),
                        transitions={
                            "succeeded": "GO_TO_LOCATION",
                            "failed": "continue",
                        },
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
                        self.HandleDetections(criteria, criteria_value),
                        transitions={
                            "succeeded": "FILTER_PEOPLE",
                            "failed": "continue",
                        },
                    )
                    # We only filter across different images, based on distance delta in the
                    # map frame.
                    smach.StateMachine.add(
                        "FILTER_PEOPLE",
                        self.FilterPeople(),
                        transitions={"succeeded": "continue"},
                        remapping={
                            "all_people": "all_people",
                            "filtered_people": "filtered_people",
                        },
                    )

                waypoint_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "WAYPOINT_ITERATOR", waypoint_iterator, {"succeeded": "COUNT_PEOPLE"}
            )

            smach.StateMachine.add(
                "COUNT_PEOPLE",
                self.CountPeople(),
                transitions={"succeeded": "succeeded"},
            )
