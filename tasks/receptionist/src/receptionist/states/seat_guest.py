#!/usr/bin/env python3
import smach

from typing import List
from shapely.geometry import Polygon

import numpy as np

from lasr_skills import PlayMotion, Detect3DInArea, LookToPoint, Say


class SeatGuest(smach.StateMachine):

    _motions: List[str] = ["look_down_left", "look_down_right", "look_down_centre"]

    class ProcessDetections(smach.State):

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded"],
                input_keys=[
                    "detections_3d",
                    "people_detections",
                    "seat_detections",
                ],
                output_keys=["people_detections", "seat_detections"],
            )

        def execute(self, userdata):
            for det in userdata.detections_3d:
                if det.name == "person":
                    userdata.people_detections.append(det)
                else:
                    userdata.seat_detections.append(det)
            return "succeeded"

    class DetermineSeat(smach.State):

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["people_detections", "seat_detections"],
                output_keys=["seat_position"],
            )

        def execute(self, userdata) -> str:
            for seat in userdata.seat_detections:

                seat_is_free: bool = True
                seat_contours: np.ndarray = np.array(seat.xyseg).reshape(-1, 2)
                seat_polygon: Polygon = Polygon(seat_contours)

                for person in userdata.people_detections:

                    person_contours: np.ndarray = np.array(person.xyseg).reshape(-1, 2)
                    person_polygon: Polygon = Polygon(person_contours)

                    if person_polygon.intersects(seat_polygon):
                        seat_is_free = False
                        break

                if seat_is_free:
                    userdata.seat_position = seat.point
                    print(seat.point)
                    return "succeeded"

            return "failed"

    def __init__(
        self,
        seat_area: Polygon,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:

            motion_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=self._motions,
                it_label="motion",
                input_keys=["people_detections", "seat_detections"],
                output_keys=[],
                exhausted_outcome="succeeded",
            )

            with motion_iterator:

                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["motion", "people_detections", "seat_detections"],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "LOOK",
                        PlayMotion(),
                        transitions={
                            "succeeded": "DETECT",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"motion_name": "motion"},
                    )
                    smach.StateMachine.add(
                        "DETECT",
                        Detect3DInArea(seat_area, filter=["chair", "person"]),
                        transitions={"succeeded": "PROCESS", "failed": "failed"},
                    )
                    smach.StateMachine.add(
                        "PROCESS",
                        self.ProcessDetections(),
                        transitions={"succeeded": "continue"},
                    )

                smach.Iterator.set_contained_state(
                    "CONTAINER_SM", container_sm, loop_outcomes=["continue"]
                )

            smach.StateMachine.add(
                "MOTION_ITERATOR",
                motion_iterator,
                transitions={"succeeded": "DETERMINE_SEAT", "failed": "failed"},
            )
            smach.StateMachine.add(
                "DETERMINE_SEAT",
                self.DetermineSeat(),
                transitions={"succeeded": "LOOK_TO_POINT", "failed": "failed"},
            )
            smach.StateMachine.add(
                "LOOK_TO_POINT",
                LookToPoint(),
                transitions={
                    "succeeded": "SAY_SIT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"point": "seat_position"},
            )
            smach.StateMachine.add(
                "SAY_SIT",
                Say("Please sit in the seat that I am looking at."),
                transitions={
                    "succeeded": "RESET_HEAD",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )  # TODO: sleep after this.

            smach.StateMachine.add(
                "RESET_HEAD",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )


if __name__ == "__main__":
    import rospy

    rospy.init_node("test_find_empty_seat")
    sm = SeatGuest(
        seat_area=Polygon([[-0.39, 0.87], [-0.74, 2.18], [1.26, 2.64], [1.54, 1.26]]),
    )
    # TODO: stop doing this
    sm.userdata.people_detections = []
    sm.userdata.seat_detections = []
    sm.execute()
