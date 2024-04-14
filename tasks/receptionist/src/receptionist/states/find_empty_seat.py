#!/usr/bin/env python3
import smach

from typing import List
from shapely.geometry import Polygon

from lasr_skills import PlayMotion, Detect3DInArea


class FindEmptySeat(smach.StateMachine):

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
                if det[0].name == "person":
                    userdata.bulk_people_detections_3d.append(det)
                else:
                    userdata.bulk_seat_detections_3d.append(det)
            return "succeeded"

    class DetermineSeat(smach.State):

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["people_detections", "seat_detections"],
                output_keys=[],  # ["seat"],
            )

        def execute(self, userdata):
            print(userdata.people_detections)
            print(userdata.seat_detections)
            return "succeeded"

    def __init__(
        self,
        motions: List[str],
        seat_area: Polygon,
    ):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["seat"]
        )
        # TODO: stop doing this
        self.userdata.people_detections = []
        self.userdata.seat_detections = []
        with self:
            motion_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=motions,
                it_label="motion",
                input_keys=[],
                output_keys=["seat"],
                exhausted_outcome="failed",
            )

            with motion_iterator:

                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"], input_keys=["motion"]
                )

                with container_sm:
                    smach.StateMachine.add(
                        "LOOK",
                        PlayMotion(),
                        transitions={
                            "succeeded": "continue",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"motion_name": "motion"},
                    )
                    smach.StateMachine.add(
                        "DETECT",
                        Detect3DInArea(seat_area, filter=["chair", "person"]),
                        transitions={"succeeded": "DETECT", "failed": "failed"},
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
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


if __name__ == "__main__":
    import rospy

    rospy.init_node("test_find_empty_seat")
    sm = FindEmptySeat(
        motions=["look_left", "look_right", "look_centre"], seat_area=Polygon()
    )
    sm.execute()
