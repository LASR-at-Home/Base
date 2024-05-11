import smach
import rospy

from lasr_skills import Detect3D, GoToLocation, DetectGesture, PlayMotion
from lasr_skills.vision import GetImage
import navigation_helpers

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion


from typing import List, Union


class FindGesturePerson(smach.StateMachine):
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

    class HandleResponse(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["gesture_detected"],
            )

        def execute(self, userdata):
            if userdata.gesture_detected:
                return "succeeded"
            return "failed"

    def __init__(
        self,
        gesture: str,
        waypoints: Union[List[Pose], None] = None,
        location_param: Union[str, None] = None,
    ):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["person_point"]
        )
        self.gesture = gesture
        if waypoints is None and location_param is None:
            raise ValueError("Either waypoints or location_param must be provided")

        if waypoints is None:
            waypoints_to_iterate: List[Pose] = []

            room = rospy.get_param(location_param)
            beacons = room["beacons"]
            for beacon in beacons:
                waypoint: Pose = Pose(
                    position=Point(**beacons[beacon]["near_pose"]["position"]),
                    orientation=Quaternion(
                        **beacons[beacon]["near_pose"]["orientation"]
                    ),
                )
                waypoints_to_iterate.append(waypoint)
        else:
            waypoints_to_iterate: List[Pose] = waypoints

        with self:
            smach.StateMachine.add(
                "GET_POSE",
                self.GetPose(),
                transitions={"succeeded": "COMPUTE_PATH", "failed": "failed"},
            )

            smach.StateMachine.add(
                "COMPUTE_PATH",
                self.ComputePath(waypoints_to_iterate),
                transitions={"succeeded": "WAYPOINT_ITERATOR", "failed": "failed"},
            )

            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=lambda: range(len(waypoints_to_iterate)),
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
                            "succeeded": "PLAY_MOTION",
                            "failed": "continue",
                        },
                    ),
                    smach.StateMachine.add(
                        "PLAY_MOTION",
                        PlayMotion(motion_name="raise_torso"),
                        transitions={
                            "succeeded": "GET_IMAGE",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    ),
                    smach.StateMachine.add(
                        "GET_IMAGE",
                        GetImage(),
                        transitions={"succeeded": "CHECK_GESTURE", "failed": "failed"},
                    )
                    smach.StateMachine.add(
                        "CHECK_GESTURE",
                        DetectGesture(self.gesture),
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
