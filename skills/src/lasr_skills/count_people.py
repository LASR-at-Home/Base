#!/usr/bin/env python3
import smach
import smach_ros
import rospy
#from lasr_skills.fake_nav import GoToLocation
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
    YoloDetection,
    YoloDetectionRequest
)


from typing import List, Literal
import itertools


class CountPeople(smach.StateMachine):

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
                    input_keys=[
                        "responses",
                    ],
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
                rospy.loginfo(f"Got {len(userdata.responses[0].detections_3d)} detections")
                rospy.loginfo(f"Got {len(userdata.responses[0].cropped_imgs)} cropped images")
                response = userdata.responses[0].detections_3d.pop(0)
                userdata.response = response
                userdata.cropped_image = userdata.responses[0].cropped_imgs.pop(0)
                userdata.person_point = response.point
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
                input_keys=["responses", "all_people"],
                output_keys=["responses", "all_people"],
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
                            #"missing_keypoints": "GET_RESPONSE",
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
                        remapping={"img_msg": "cropped_image"},
                    )
                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

    class CountPeople(smach.State):

        def __init__(self, distance_threshold: float = 1.0):
            # Call the SMACH State constructor
            super().__init__(
                outcomes=["succeeded"],
                input_keys=["all_people"],
                output_keys=["people_count"],
            )
            # Now store your threshold
            self.distance_threshold = distance_threshold

        def execute(self, userdata):
            # your existing code that uses self.distance_threshold
            people = []
            for person in userdata.all_people:
                if not any(
                    navigation_helpers.euclidian_distance(person, p)
                    < self.distance_threshold
                    for p in people
                ):
                    people.append(person)
            userdata.people_count = len(people)
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
            color_list = ["green", "blue", "yellow", "black", "white", "red", "orange", "gray", "brown"]
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

            smach.StateMachine.add(
                "COMPUTE_PATH",
                self.ComputePath(waypoints),
                transitions={"succeeded": "WAYPOINT_ITERATOR", "failed": "failed"},
            )

            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded"],
                it=lambda: range(len(waypoints)),
                it_label="location_index",
                input_keys=["waypoints", "all_people"],
                output_keys=["all_people"],
                exhausted_outcome="succeeded",
            )

            with waypoint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["waypoints", "location_index", "all_people"],
                    output_keys=["all_people","image_raw", "responses"],
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
                            "succeeded": "GET_IMAGE",
                            "failed": "failed",
                        },
                    )
                    @smach.cb_interface(output_keys=["image_raw"], outcomes=["succeeded"])
                    def get_image(userdata):
                        userdata.image_raw = rospy.wait_for_message("/camera/rgb/image_raw", Image)
                        return "succeeded"
                    smach.StateMachine.add(
                        "GET_IMAGE",
                        smach.CBState(get_image),
                        transitions={"succeeded": "DETECT"},
                        remapping={"image_raw": "image_raw"},
                    )

                    smach.StateMachine.add(
                        "DETECT",
                        smach_ros.ServiceState(
                            "/yolo/detect",
                            YoloDetection,
                            request=YoloDetectionRequest(
                                model="yolo11n-seg.pt",
                                confidence=0.5,
                                filter=["person"]
                            ),
                            request_slots=["image_raw"],
                            response_slots=["detected_objects"],
                        ),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "aborted": "continue",
                            "preempted": "continue",
                        },
                        remapping={
                            "image_raw": "image_raw",
                            "detected_objects": "responses",
                        },
                    )


                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        self.HandleDetections(criteria, criteria_value),
                        transitions={
                            "succeeded": "continue",
                            "failed": "continue",
                        },
                    )
                waypoint_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, 
                    loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "WAYPOINT_ITERATOR", waypoint_iterator, {"succeeded": "COUNT_PEOPLE"}
            )

            smach.StateMachine.add(
                "COUNT_PEOPLE",
                self.CountPeople(),
                transitions={"succeeded": "succeeded"},
            )
