import smach
import smach_ros
import rospy

from lasr_skills import GoToLocation, AskAndListen, DetectGesture
import navigation_helpers

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Polygon
from lasr_vision_msgs.msg import CDRequest, CDResponse
from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    CroppedDetection,
    Recognise,
)
from typing import List, Literal


class FindPerson(smach.StateMachine):

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
                userdata.response = userdata.responses[0].detections_3d.pop(0)
                userdata.cropped_image = userdata.responses[0].cropped_imgs.pop(0)
                return "succeeded"

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
                        transitions={"succeeded": "GO_TO_PERSON", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        GoToLocation(),
                        transitions={
                            "succeeded": "ASK_NAME",
                            "failed": "failed",
                        },
                        remapping={"location": "person_point"},
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
                            "succeeded": "succeeded",
                            "missing_keypoints": "GET_RESPONSE",
                            "failed": "GET_RESPONSE",
                        },
                    )

                elif criteria == "pose":
                    raise NotImplementedError("Pose criteria not implemented")
                elif criteria == "clothes":
                    raise NotImplementedError("Clothes criteria not implemented")

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
            ], "Invalid gesture"
        elif criteria == "pose":
            assert criteria_value in [
                "sitting",
                "standing",
                "lying_down",
            ], "Invalid pose"

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
                            "succeeded": "DETECT3D",
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
