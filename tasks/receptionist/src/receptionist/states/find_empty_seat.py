#!/usr/bin/env python3
import smach

from typing import List
from shapely.geometry import Polygon

from lasr_skills import PlayMotion, Detect3DInArea


class FindEmptySeat(smach.StateMachine):

    def __init__(
        self,
        motions: List[str],
        seat_area: Polygon,
    ):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["seat"]
        )
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

                smach.Iterator.set_contained_state(
                    "CONTAINER_SM", container_sm, loop_outcomes=["continue"]
                )

            smach.StateMachine.add(
                "MOTION_ITERATOR",
                motion_iterator,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


if __name__ == "__main__":
    import rospy

    rospy.init_node("test_find_empty_seat")
    sm = FindEmptySeat(
        motions=["look_left", "look_right", "look_centre"], seat_area=Polygon()
    )
    sm.execute()
