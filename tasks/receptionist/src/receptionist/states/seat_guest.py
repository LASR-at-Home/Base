import smach
import rospy

import numpy as np
import tf2_ros as tf

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point as ShapelyPoint

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from tf2_geometry_msgs import do_transform_point

from receptionist.states import ReceptionistLearnFaces
from lasr_vision_msgs.msg import Detection3D
from lasr_skills import (
    PlayMotion,
    Detect3DInArea,
    LookToPoint,
    Say,
    Wait,
    DetectAllInPolygon,
)


class LearnHostFace(smach.StateMachine):
    """State machine to learn the host's face."""

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "seated_guest_locs"],
            output_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "GET_HOST_LOOK_POINT",
                GetLookPoint(),
                transitions={"succeeded": "LOOK_TO_HOST", "failed": "failed"},
            )
            smach.StateMachine.add(
                "LOOK_TO_HOST",
                LookToPoint(),
                transitions={
                    "succeeded": "LEARN_HOST_FACE",
                    "aborted": "failed",
                    "timed_out": "failed",
                },
                remapping={"pointstamped": "pointstamped"},
            )
            smach.StateMachine.add(
                "LEARN_HOST_FACE",
                ReceptionistLearnFaces(guest_id="host", dataset_size=5),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"guest_data": "guest_data"},
            )


class GetLookPoint(smach.State):
    """State to get the look point for the guest to be seated."""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["seated_guest_locs"],
            output_keys=["pointstamped"],
        )

    def execute(self, userdata):
        """Set the pointstamped to the guest seat point."""
        if not userdata.seated_guest_locs:
            rospy.logwarn("No seated guest locations provided.")
            return "failed"
        point = userdata.seated_guest_locs[0]
        userdata.pointstamped = PointStamped(header=Header(frame_id="map"), point=point)
        return "succeeded"


class ProcessDetections(smach.State):

    _max_people_on_sofa: int
    _sofa_point: Point
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(
        self,
        sofa_point: Point,
        left_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
        max_people_on_sofa: int = 2,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["non_sofa_detections", "sofa_detections"],
            output_keys=["guest_seat_point", "seated_guest_locs", "seating_string"],
        )
        self._max_people_on_sofa = max_people_on_sofa
        self._sofa_point = sofa_point
        self._left_sofa_area = left_sofa_area
        self._right_sofa_area = right_sofa_area
        self._tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
        self._tf_listener = tf.TransformListener(self._tf_buffer)

    def _determine_side_of_sofa(self, sofa_detection: Detection3D) -> str:
        """Determines which side of the sofa is empty, in order to seat
        the guest there.

        Args:
            sofa_detection (Detection3D): Detection of the other
            guest who is already sat on the sofa.

        Returns:
            str: "left" or "right" - which side of the sofa is empty.
        """
        sofa_guest_point = sofa_detection.point

        if self._left_sofa_area.contains(
            ShapelyPoint(sofa_guest_point.x, sofa_guest_point.y)
        ):
            result = "right"
        elif self._right_sofa_area.contains(
            ShapelyPoint(sofa_guest_point.x, sofa_guest_point.y)
        ):
            result = "left"
        else:
            rospy.logwarn(
                "Sofa guest point is not within the left or right sofa area. "
                "Defaulting to 'right'."
            )
            result = "right"

        return result

    def execute(self, userdata):
        """
        Input:
            userdata.non_sofa_detections (List[Detection3D]): List of detected objects that are not on the sofa
            userdata.sofa_detections (List[Detection3D]): List of detected objects on the sofa
        """

        seat_sofa = True
        seated_guests_loc = [
            detection.point
            for detection in userdata.non_sofa_detections
            if detection.name == "person"
        ]
        seated_guests_sofa_loc = [
            detection.point
            for detection in userdata.sofa_detections
            if detection.name == "person"
        ]
        seated_guest_locs = seated_guests_loc + seated_guests_sofa_loc
        rospy.loginfo(
            f"Detected {len(seated_guest_locs)} seated guests in the seating area."
        )
        rospy.loginfo(f"Detections are: {seated_guest_locs}")
        if len(seated_guest_locs) > 2:
            rospy.logwarn(
                f"Too many people detected: {len(seated_guest_locs)} detected, max allowed is 2."
            )
            userdata.seated_guest_locs = seated_guest_locs[:2]
        else:
            userdata.seated_guest_locs = seated_guest_locs
        if len(userdata.sofa_detections) > self._max_people_on_sofa:
            rospy.logwarn(
                f"Too many people on the sofa: {len(userdata.sofa_detections)} detected, max allowed is {self._max_people_on_sofa}."
            )
            seat_sofa = False
        elif len(userdata.sofa_detections) == self._max_people_on_sofa:
            rospy.loginfo(f"Sofa max capacity has been reached.")
            seat_sofa = False

        if seat_sofa:
            userdata.guest_seat_point = PointStamped(
                header=Header(frame_id="map"), point=self._sofa_point
            )
            if len(userdata.sofa_detections) == 0:
                userdata.seating_string = "The sofa that I'm looking at is empty. Please take a seat anywhere on the sofa."
            elif len(userdata.sofa_detections) == 1:
                seating_side = self._determine_side_of_sofa(userdata.sofa_detections[0])
                userdata.seating_string = (
                    "The sofa that I'm looking at is occupied by one person. "
                    f"Please take a seat next to them on the {seating_side} side of the sofa."
                )
        else:
            seated_guests_xywh = [
                detection.xywh
                for detection in userdata.non_sofa_detections
                if detection.name == "person"
            ]
            done = False
            for detection in userdata.non_sofa_detections:
                if done:
                    break
                if detection.name == "chair":
                    # Check if a person is sitting on the chair
                    chair_bbox = detection.xywh
                    overlap_pct = 0.0
                    for guest_xywh in seated_guests_xywh:
                        overlap_pct_current = (
                            np.maximum(
                                0,
                                np.minimum(
                                    chair_bbox[0] + chair_bbox[2],
                                    guest_xywh[0] + guest_xywh[2],
                                )
                                - np.maximum(chair_bbox[0], guest_xywh[0]),
                            )
                            * np.maximum(
                                0,
                                np.minimum(
                                    chair_bbox[1] + chair_bbox[3],
                                    guest_xywh[1] + guest_xywh[3],
                                )
                                - np.maximum(chair_bbox[1], guest_xywh[1]),
                            )
                        ) / (chair_bbox[2] * chair_bbox[3])
                        overlap_pct = max(overlap_pct, overlap_pct_current)
                    if overlap_pct > 0.5:
                        rospy.loginfo(
                            f"Detected a person sitting on a chair with bbox {chair_bbox}, with overlap percentage {overlap_pct:.2f}."
                        )
                        continue
                    else:
                        rospy.loginfo(
                            f"No person detected sitting on chair with bbox {chair_bbox}."
                        )
                        userdata.guest_seat_point = PointStamped(
                            header=Header(frame_id="map"),
                            point=Point(
                                x=detection.point.x,
                                y=detection.point.y,
                                z=detection.point.z,
                            ),
                        )
                        userdata.seating_string = "The sofa is full, but I have found a chair for you. Please take a seat on the chair that I'm looking at."
                        done = True

            if not done:
                userdata.seating_string = "Uh oh, I couldn't find a seat for you. Please take a seat anywhere in the seating area."
                userdata.guest_seat_point = PointStamped(
                    header=Header(frame_id="map"),
                    point=Point(
                        x=self._sofa_point.x, y=self._sofa_point.y, z=self._sofa_point.z
                    ),
                )

        return "succeeded"


class SeatGuest(smach.StateMachine):
    def __init__(
        self,
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        sofa_point: Point,
        left_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
        max_people_on_sofa: int = 2,
        learn_host: bool = False,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_seat_point", "seated_guest_locs"],
        )
        seating_area_minus_sofa = seating_area.difference(sofa_area)

        with self:
            self.userdata.seated_guest_locs = []
            smach.StateMachine.add(
                "SAY_FINDING_SEAT",
                Say("I will now find a seat for you."),
                transitions={
                    "succeeded": "LOOK_TO_SOFA",
                    "aborted": "LOOK_TO_SOFA",
                    "preempted": "LOOK_TO_SOFA",
                },
            )
            smach.StateMachine.add(
                "LOOK_TO_SOFA",
                LookToPoint(
                    pointstamped=PointStamped(
                        header=Header(frame_id="map"), point=sofa_point
                    )
                ),
                transitions={
                    "succeeded": "DETECT_SOFA",
                    "aborted": "failed",
                    "timed_out": "failed",
                },
            )
            smach.StateMachine.add(
                "DETECT_SOFA",
                Detect3DInArea(sofa_area, filter=["person"], confidence=0.7),
                transitions={"succeeded": "RESET_HEAD_1", "failed": "failed"},
                remapping={"detections_3d": "sofa_detections"},
            )
            smach.StateMachine.add(
                "RESET_HEAD_1",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "DETECT_NON_SOFA",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_NON_SOFA",
                DetectAllInPolygon(
                    seating_area_minus_sofa,
                    object_filter=["person", "chair"],
                    min_coverage=0.7,
                    min_new_object_dist=0.50,
                    min_confidence=0.5,
                ),
                transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
                remapping={"detected_objects": "non_sofa_detections"},
            )
            # Process detections
            if learn_host:
                detection_transition = "SAY_AND_LEARN_HOST_FACE"
            else:
                detection_transition = "LOOK_TO_SEAT"
            smach.StateMachine.add(
                "PROCESS_DETECTIONS",
                ProcessDetections(
                    max_people_on_sofa=max_people_on_sofa,
                    sofa_point=sofa_point,
                    left_sofa_area=left_sofa_area,
                    right_sofa_area=right_sofa_area,
                ),
                transitions={"succeeded": detection_transition, "failed": "failed"},
                remapping={
                    "guest_seat_point": "guest_seat_point",
                    "seating_detections": "seating_detections",
                },
            )
            if learn_host:
                # Look to the only person detection and learn the host's face.
                sm_con = smach.Concurrence(
                    outcomes=["succeeded", "failed"],
                    input_keys=["guest_data", "seated_guest_locs"],
                    output_keys=["guest_data", "seated_guest_locs"],
                    default_outcome="failed",
                    outcome_map={
                        "succeeded": {
                            "SAY_LEARN_HOST_FACE": "succeeded",
                            "LEARN_HOST_FACE": "succeeded",
                        },
                        "failed": {
                            "SAY_LEARN_HOST_FACE": "aborted",
                            "LEARN_HOST_FACE": "failed",
                        },
                    },
                )
                with sm_con:
                    smach.Concurrence.add(
                        "SAY_LEARN_HOST_FACE",
                        Say("I'm quickly remembering the host's face."),
                    )
                    smach.Concurrence.add("LEARN_HOST_FACE", LearnHostFace())
                smach.StateMachine.add(
                    "SAY_AND_LEARN_HOST_FACE",
                    sm_con,
                    transitions={"succeeded": "LOOK_TO_SEAT", "failed": "LOOK_TO_SEAT"},
                )

            smach.StateMachine.add(
                "LOOK_TO_SEAT",
                LookToPoint(),
                transitions={
                    "succeeded": "SAY_SEAT_GUEST",
                    "aborted": "SAY_SEAT_GUEST",
                    "timed_out": "SAY_SEAT_GUEST",
                },
                remapping={"pointstamped": "guest_seat_point"},
            )
            smach.StateMachine.add(
                "SAY_SEAT_GUEST",
                Say(),
                transitions={
                    "succeeded": "WAIT_FOR_GUEST_TO_SEAT",
                    "aborted": "WAIT_FOR_GUEST_TO_SEAT",
                    "preempted": "WAIT_FOR_GUEST_TO_SEAT",
                },
                remapping={"text": "seating_string"},
            )
            smach.StateMachine.add(
                "WAIT_FOR_GUEST_TO_SEAT",
                Wait(5.0),
                transitions={"succeeded": "RESET_HEAD_2", "failed": "RESET_HEAD_2"},
            )

            smach.StateMachine.add(
                "RESET_HEAD_2",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )
