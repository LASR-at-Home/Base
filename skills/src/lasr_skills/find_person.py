#!/usr/bin/env python3

import smach
from smach_ros import SimpleActionState

from geometry_msgs.msg import Pose

from typing import List

from lasr_skills import Detect3D, LookToPoint

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class FindPerson(smach.StateMachine):

    class HandleDetections(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections"],
                output_keys=["point"],
            )

        def execute(self, userdata):
            if len(userdata.detections.detected_objects) == 0:
                return "failed"

            userdata.point = userdata.detections.detected_objects[0].point
            return "succeeded"

    def __init__(self, waypoints: List[Pose]):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        self.waypoints = waypoints

        with self:
            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=self.waypoints,
                it_label="location",
                input_keys=[],
                output_keys=[],
                exhausted_outcome="failed",
            )

            with waypoint_iterator:

                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["location"],
                )

                with container_sm:

                    smach.StateMachine.add(
                        "GO_TO_LOCATION",
                        SimpleActionState(
                            "move_base",
                            MoveBaseAction,
                            goal_cb=lambda ud, _: MoveBaseGoal(target_pose=ud.location),
                        ),
                        transitions={
                            "succeeded": "DETECT3D",
                            "preempted": "continue",
                            "aborted": "continue",
                        },
                    )
                    smach.StateMachine.add(
                        "DETECT3D",
                        Detect3D(filter=["person"]),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "failed": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        self.HandleDetections(),
                        transitions={
                            "succeeded": "LOOK_AT_PERSON",
                            "failed": "continue",
                        },
                    )
                    smach.StateMachine.add(
                        "LOOK_AT_PERSON",
                        LookToPoint(),
                        transitions={"succeeded": "succeeded"},
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
    from smach_ros import IntrospectionServer

    rospy.init_node("find_person")
    sm = FindPerson([])
    sis = IntrospectionServer("find_person", sm, "/FIND_PERSON")
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()
