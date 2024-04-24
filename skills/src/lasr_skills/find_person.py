import smach
import rospy

from lasr_skills import Detect3D, GoToLocation
import navigation_helpers

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

from typing import List


class FindPerson(smach.StateMachine):

    class GetLocation(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location_index", "waypoints"],
                output_keys=["location"],
            )

        def execute(self, userdata):
            userdata.location = userdata.waypoints[userdata.location_index]
            return "succeeded"

    class GetPose(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["current_pose"],
            )

        def execute(self, userdata):
            userdata.current_pose = rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped
            ).pose.pose
            return "succeeded"

    class ComputePath(smach.State):
        def __init__(self, waypoints: List[Pose]):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["current_pose"],
                output_keys=["waypoints"],
            )
            self._waypoints = waypoints

        def execute(self, userdata):
            userdata.waypoints = navigation_helpers.min_hamiltonian_path(
                userdata.current_pose, self._waypoints
            )
            return "succeeded"

    class HandleDetections(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["person_point"],
            )

        def execute(self, userdata):
            if len(userdata.detections_3d.detected_objects) == 0:
                rospy.logwarn("No person detected, returning failed.")
                return "failed"
            userdata.person_point = userdata.detections_3d.detected_objects[0].point
            return "succeeded"

    def __init__(self, waypoints: List[Pose]):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["person_point"]
        )

        with self:

            smach.StateMachine.add(
                "GET_POSE",
                self.GetPose(),
                transitions={"succeeded": "COMPUTE_PATH", "failed": "failed"},
            )

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
                            "succeeded": "succeeded",
                            "failed": "continue",
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
