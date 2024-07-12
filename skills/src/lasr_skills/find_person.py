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
import itertools


class FindPerson(smach.StateMachine):

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
            criteria: Literal["name", "pose", "gesture", "clothes"],
            criteria_value: str,
        ):
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["responses"],
                output_keys=["responses", "person_point"],
            )

            with self:

                if criteria == "name":

                    class HandleSpeechResponse(smach.State):

                        def __init__(self):
                            smach.State.__init__(
                                self,
                                outcomes=["succeeded", "failed"],
                                input_keys=["transcribed_speech"],
                            )

                        def execute(self, userdata):
                            # TODO: make this smarter,e.g. levenshtein distance
                            if "yes" in userdata.transcribed_speech.lower():
                                return "succeeded"
                            return "failed"

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
                            "succeeded": "CHECK_NAME",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"location": "approach_pose"},
                    )

                    smach.StateMachine.add(
                        "CHECK_NAME",
                        AskAndListen(
                            f"I'm looking for {criteria_value}. Are you {criteria_value}?"
                        ),
                        transitions={
                            "succeeded": "HANDLE_SPEECH_RESPONSE",
                            "failed": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "HANDLE_SPEECH_RESPONSE",
                        HandleSpeechResponse(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                    )

                elif criteria == "gesture":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={"succeeded": "DETECT_GESTURE", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "DETECT_GESTURE",
                        DetectGesture(criteria_value),
                        transitions={
                            "succeeded": "GO_TO_PERSON",
                            # "missing_keypoints": "GET_RESPONSE",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        self.ApproachPerson(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"location": "approach_pose"},
                    )

                elif criteria == "pose":

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
                        DetectPose(criteria_value),
                        transitions={
                            "succeeded": "GO_TO_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        self.ApproachPerson(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"location": "approach_pose"},
                    )

                elif criteria == "clothes":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_CLOTHING",
                            "failed": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_CLOTHING",
                        DetectClothing(criteria_value),
                        transitions={
                            "succeeded": "GO_TO_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        self.ApproachPerson(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"location": "approach_pose"},
                    )

    def __init__(
        self,
        waypoints: List[Pose],
        polygon: Polygon,
        criteria: Literal["name", "pose", "gesture", "clothes"],
        criteria_value: str,
    ):

        assert criteria in ["name", "pose", "gesture", "clothes"], "Invalid criteria"

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
            color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
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

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

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
                            "failed": "GO_TO_LOCATION",
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
                        self.HandleDetections(criteria, criteria_value),
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


if __name__ == "__main__":
    import rospy
    from geometry_msgs.msg import Point, Quaternion, Pose

    rospy.init_node("find_person")

    waypoints = [
        Pose(
            position=Point(x=2.46, y=-2.15, z=0),
            orientation=Quaternion(x=0, y=0, z=-0.993, w=0.116),
        ),
        Pose(
            position=Point(x=0.88, y=1.04, z=0),
            orientation=Quaternion(x=0, y=0, z=0.125, w=0.992),
        ),
        Pose(
            position=Point(x=4.05, y=-2.16, z=0),
            orientation=Quaternion(x=0, y=0, z=-0.995, w=0.090),
        ),
    ]

    polygon = Polygon(
        [
            Point(x=-0.4806538224220276, y=-5.140193462371826, z=0.002532958984375),
            Point(x=6.777748107910156, y=-5.0068678855896, z=0.002532958984375),
            Point(x=7.11236047744751, y=2.4408864974975586, z=-0.001434326171875),
            Point(x=-4.766469955444336, y=2.666473627090454, z=-0.005340576171875),
        ]
    )

    find_person = FindPerson(
        waypoints=waypoints,
        polygon=polygon,
        criteria="name",
        criteria_value="jared",
    )
    print(find_person.execute())
