import smach
import rospy

from typing import List
from shapely.geometry import Polygon as ShapelyPolygon

import numpy as np

from geometry_msgs.msg import Point, PointStamped
from lasr_skills import (
    PlayMotion,
    Detect3DInArea,
    LookToPoint,
    Say,
    Wait,
    DetectAllInPolygon,
)

from std_msgs.msg import Header


class ProcessDetections(smach.State):

    _max_people_on_sofa: int
    _sofa_point: Point

    def __init__(self, sofa_point: Point, max_people_on_sofa: int = 2):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["non_sofa_detections", "sofa_detections"],
            output_keys=["guest_seat_point", "seated_guest_locs"],
        )
        self._max_people_on_sofa = max_people_on_sofa
        self._sofa_point = sofa_point

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
        userdata.seated_guest_locs = seated_guests_loc + seated_guests_sofa_loc
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
                header=Header(frame_id="map"),
                point=self._sofa_point,
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
                        done = True

        return "succeeded"


class SeatGuest(smach.StateMachine):

    def __init__(
        self,
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        sofa_point: Point,
        max_people_on_sofa: int = 2,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[""],
            output_keys=["guest_seat_point", "seated_guest_locs"],
        )
        seating_area_minus_sofa = seating_area.difference(sofa_area)

        with self:
            smach.StateMachine.add(
                "LOOK_TO_SOFA",
                LookToPoint(
                    pointstamped=PointStamped(
                        header=Header(frame_id="map"),
                        point=sofa_point,
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
                Detect3DInArea(
                    sofa_area,
                    filter=["person"],
                    confidence=0.7,
                ),
                transitions={
                    "succeeded": "RESET_HEAD_1",
                    "failed": "failed",
                },
                remapping={
                    "detected_objects": "sofa_detections",
                },
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
                    min_converage=1.0,
                    min_new_object_dist=0.50,
                    min_confidence=0.7,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS",
                    "failed": "failed",
                },
                remapping={
                    "detected_objects": "non_sofa_detections",
                },
            )
            # Process detections
            smach.StateMachine.add(
                "PROCESS_DETECTIONS",
                ProcessDetections(
                    max_people_on_sofa=max_people_on_sofa, sofa_point=sofa_point
                ),
                transitions={
                    "succeeded": "LOOK_TO_SEAT",
                    "failed": "failed",
                },
                remapping={
                    "guest_seat_point": "guest_seat_point",
                    "seating_detections": "seating_detections",
                },
            )

            smach.StateMachine.add(
                "LOOK_TO_SEAT",
                LookToPoint(),
                transitions={
                    "succeeded": "SAY_SEAT_GUEST",
                    "aborted": "failed",
                    "timed_out": "failed",
                },
                remapping={
                    "pointstamped": "guest_seat_point",
                },
            )
            smach.StateMachine.add(
                "SAY_SEAT_GUEST",
                Say("Please sit on the seat that I am looking at."),
                transitions={
                    "succeeded": "WAIT_FOR_GUEST_TO_SEAT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "WAIT_FOR_GUEST_TO_SEAT",
                Wait(
                    timeout=rospy.Duration(5.0),
                ),
                transitions={
                    "succeeded": "RESET_HEAD_2",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "RESET_HEAD_2",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
