#!/usr/bin/env python3

import smach
from smach_ros import SimpleActionState

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header

from typing import List

from lasr_skills import Detect3D, LookToPoint, GoToPerson

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import navigation_helpers


class FindPerson(smach.StateMachine):

    class HandleDetections(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["point"],
            )

        def execute(self, userdata):
            if len(userdata.detections_3d.detected_objects) == 0:
                return "failed"

            userdata.point = userdata.detections_3d.detected_objects[0].point
            return "succeeded"

    def __init__(self, waypoints: List[Pose]):
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
                        SimpleActionState(
                            "move_base",
                            MoveBaseAction,
                            goal_cb=lambda ud, _: MoveBaseGoal(
                                target_pose=PoseStamped(
                                    pose=ud.location, header=Header(frame_id="map")
                                )
                            ),
                            input_keys=["location"],
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
                            "succeeded": "GO_TO_PERSON",
                            "failed": "continue",
                        },
                    )
                    # TODO: better approach person.
                    ## Figure out direction vector of person, assume the person is facing us
                    ## move to the person's location + 1m in the direction of the person
                    ## turn to face the person
                    ## Detect their eyes, and look into them
                    smach.StateMachine.add(
                        "GO_TO_PERSON",
                        GoToPerson(),
                        transitions={"succeeded": "LOOK_AT_PERSON", "failed": "failed"},
                    )
                    smach.StateMachine.add(
                        "LOOK_AT_PERSON",
                        LookToPoint(),
                        transitions={
                            "succeeded": "succeeded",
                            "aborted": "failed",
                            "preempted": "failed",
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
        ]
    )
    sis = IntrospectionServer("find_person", sm, "/FIND_PERSON")
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()
