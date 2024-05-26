#!/usr/bin/env python3
# # looks at one direction left
# ~recognise
# returns
# bbox
# for eveyrone known in the fram e
# # greps the host(name) if it can see then otherwise fail
# look
# at
# them
# got
# to
# next
# motion
# of
# failed
#
# #
# # add speeing look once recognise given
#
# in look
# to
# point
# 192


""" 
State machine for the nicole task. It finds a person by given name and then looks at them.
"""
import rospy
import smach
from lasr_skills.look_at_person import LookAtPerson
from typing import List, Union
from geometry_msgs.msg import Point


class FindAndLookAt(smach.StateMachine):
    def __init__(
        self,
        guest_name: str,
        look_positions: Union[List[Point], None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_name"],
            output_keys=[],
        )

        if look_positions is None:
            all_look_positions: List[Point] = []
            # look straight, left, right
            look_positions = [
                Point(0.0, 0.0, 0.0),
                Point(1.0, -1.0, 0.0),
                Point(-1.0, -1.0, 0.0),
            ]
        else:
            all_look_positions = look_positions

        with self:
            # look straight
            # check if name is in the frame
            # if not look left
            # if not look right
            # if not fail
            look_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=lambda: range(len(all_look_positions)),
                it_label="point",
                input_keys=["points"],
                output_keys=["point"],
                exhausted_outcome="failed",
            )
            with look_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["point"],
                    output_keys=[],
                )

                with container_sm:
                    smach.StateMachine.add(
                        smach_ros.ServiceState(
                            "/recognise",
                            Recognise,
                            request_slots=["image_raw", "dataset", "confidence"],
                            response_slots=["detections"],
                            request_cb=lambda ud, _: Recognise.Request(
                                image=ud.image_raw, dataset=ud.dataset, confidence=ud.confidence
                            ),
                            response_cb=lambda ud, response: ud.detections,
                        ),
                        transitions={
                            "succeeded": "LOOK_AT_PERSON",
                            "failed": "failed",
                            "preempted": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "LOOK_AT_PERSON",
                        LookAtPerson(),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "continue",
                        },
                    )

                smach.StateMachine.add(
                    "LOOK_ITERATOR",
                    look_iterator,
                    transitions={"succeeded": "succeeded", "failed": "failed"},
                )


if __name__ == "__main__":
    rospy.init_node("test_find_and_look_at")

    sm = FindAndLookAt(
        "Nicole",
        [
            Point(0.0, 0.0, 0.0),
            Point(1.0, -1.0, 0.0),
            Point(-1.0, -1.0, 0.0),
        ],
    )
    sm.userdata.guest_name = "Nicole"

    outcome = sm.execute()

    rospy.loginfo(f"Outcome: {outcome}")
    rospy.spin()
