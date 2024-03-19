#!/usr/bin/env python3

import smach
from smach_ros import SimpleActionState

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header

from typing import List

from lasr_skills import Detect3D, LookToPoint, GoToPerson, GoToLocation, AskAndListen


import navigation_helpers

import rospy


class FindNamedPerson(smach.StateMachine):

    class HandleDetections(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["person_point", "person_max_point"],
            )

        def execute(self, userdata):
            if len(userdata.detections_3d.detected_objects) == 0:
                return "failed"

            userdata.person_point = userdata.detections_3d.detected_objects[0].point
            userdata.person_max_point = userdata.detections_3d.detected_objects[
                0
            ].max_point
            return "succeeded"

    class HandleResponse(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["transcribed_speech"],
            )

        def execute(self, userdata):
            if "yes" in userdata.transcribed_speech.lower():
                return "succeeded"
            return "failed"

    def __init__(self, waypoints: List[Pose], name: str):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        current_point = rospy.wait_for_message(
            "/amcl_pose", PoseWithCovarianceStamped
        ).pose.pose
        self.waypoints = navigation_helpers.min_hamiltonian_path(
            current_point, waypoints
        )

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
                        GoToLocation(),
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
                            "succeeded": "GO_TO_PERSON",
                            "failed": "continue",
                        },
                    )
                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        GoToPerson(),
                        transitions={"succeeded": "LOOK_AT_PERSON", "failed": "failed"},
                        remapping={"point": "person_point"},
                    )
                    smach.StateMachine.add(
                        "LOOK_AT_PERSON",
                        LookToPoint(),
                        transitions={
                            "succeeded": "succeeded",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={
                            "point": "person_max_point",
                        },
                    )
                    smach.StateMachine.add(
                        "CHECK_NAME",
                        AskAndListen(f"I'm looking for {name}. Are you {name}?"),
                        transitions={
                            "succeeded": "HANDLE_RESPONSE",
                            "failed": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "HANDLE_RESPONSE",
                        self.HandleResponse(),
                        transitions={"succeeded": "succeeded", "failed": "continue"},
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
    from geometry_msgs.msg import Pose, Point, Quaternion

    rospy.init_node("find_person")

    sm = FindPerson(
        [
            Pose(
                position=Point(1.446509335239015, -2.2460076897430974, 0.0),
                orientation=Quaternion(
                    0.0, 0.0, -0.6459923698644408, 0.763343866207703
                ),
            ),
            Pose(
                position=Point(-2.196456229125565, -0.27387058028873024, 0.0),
                orientation=Quaternion(
                    0.0, 0.0, 0.9778384065708362, 0.20936105329073992
                ),
            ),
            Pose(
                position=Point(-0.8129574905602319, -5.8536586556997445, 0.0),
                orientation=Quaternion(
                    0.0, 0.0, -0.9982013547068731, 0.05995044171116081
                ),
            ),
        ]
    )
    sis = IntrospectionServer("find_person", sm, "/FIND_PERSON")
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()
