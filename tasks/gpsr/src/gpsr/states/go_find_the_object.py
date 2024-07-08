#!/usr/bin/env python3
from go_to_location import GoToLocation
import smach
from lasr_skills import Detect3D
from shapely.geometry.polygon import Polygon
from typing import List, Union
from geometry_msgs.msg import Pose, Point, Quaternion
from lasr_skills import Say, PlayMotion
import rospy

"""
location = rospy.get_param("/start") -> python dict
Pose(position : Point, orientation : Quaternion)
pose = Pose(position=Point(**location['pose']['position'], orientation=Quaternion(**location['pose']['orientation']))
"""


class GoFindTheObject(smach.StateMachine):
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

    class check_objects(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["detection_result"],
            )

        def move_around(self, detections):
            print("we're checking on the object")
            if len(detections) == 0:
                result = False
            else:
                result = True
            return result

        def execute(self, userdata):
            filtered_detections = (
                userdata.detections_3d
            )  # the outcome of the 3d detection
            print(filtered_detections)
            object_list = list()
            for i in filtered_detections.detected_objects:
                object_list.append(i.name)
            result = self.move_around(object_list)
            userdata.detection_result = result
            rospy.loginfo(filtered_detections)

            return "succeeded"

    class cumulate_result(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detection_result", "cumulated_result"],
                output_keys=["cumulated_result"],
            )

        def execute(self, userdata):
            if "cumulated_result" not in userdata:
                userdata.cumulated_result = list()
                userdata.cumulated_result.append(userdata.detection_result)
            else:
                userdata.cumulated_result.append(
                    userdata.detection_result
                )  # the outcome of the 3d detection
            return "succeeded"

    class detection_result(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["object_found", "object_not_found", "failed"],
                input_keys=["cumulated_result"],
                output_keys=["result", "cumulated_result"],
            )

        def execute(self, userdata):
            if any(userdata.cumulated_result):
                userdata.result = True
                userdata.cumulated_result = list()
                return "object_found"
            else:
                userdata.cumulated_result = list()
                userdata.result = False
                return "object_not_found"

    def __init__(
        self,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        waypoints: Union[List[Pose], None] = None,
        locations: Union[str, None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        motions: Union[List[str], None] = None,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        if waypoints is None and locations is None:
            raise ValueError("Either waypoints or location_param must be provided")

        if waypoints is None:
            waypoints_to_iterate: List[Pose] = []
            room = rospy.get_param(locations)
            beacons = room["beacons"]
            for beacon in beacons:
                waypoint = Pose(
                    position=Point(**beacons[beacon]["near_pose"]["position"]),
                    orientation=Quaternion(**beacons[beacon]["near_pose"]["orientation"])
                )
                waypoints_to_iterate.append(waypoint)
        else:
            waypoints_to_iterate: List[Pose] = waypoints

        if motions is None:
            motions = ["look_down", "look_down_left", "look_down_right"]

        say_object_found = Say("I found the object here!")
        say_object_not_found = Say("The object is not here")

        with self:
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
                            "succeeded": "INNER_ITERATOR",
                            "failed": "failed",
                        },
                    )

                    inner_iterator = smach.Iterator(
                        outcomes=["succeeded", "failed", "continue"],
                        it=lambda: range(len(motions)),
                        it_label="motion_index",
                        input_keys=["waypoints", "location_index"],
                        output_keys=["person_point"],
                        exhausted_outcome="succeeded",
                    )

                    with inner_iterator:
                        inner_container_sm = smach.StateMachine(
                            outcomes=["succeeded", "failed", "continue"],
                            input_keys=["motion_index", "location_index", "waypoints"],
                            output_keys=["person_point"],
                        )

                        with inner_container_sm:
                            smach.StateMachine.add(
                                "LOOK_AROUND",
                                PlayMotion(),
                                transitions={
                                    "succeeded": "DETECT_OBJECTS_3D",
                                    "aborted": "DETECT_OBJECTS_3D",
                                    "preempted": "failed",
                                },
                            )

                            smach.StateMachine.add(
                                "DETECT_OBJECTS_3D",
                                Detect3D(
                                    depth_topic=depth_topic,
                                    model=model,
                                    filter=filter,
                                    confidence=confidence,
                                    nms=nms,
                                ),
                                transitions={
                                    "succeeded": "RESULT",
                                    "failed": "failed",
                                },
                            )

                            smach.StateMachine.add(
                                "RESULT",
                                self.check_objects(),
                                transitions={
                                    "succeeded": "SAVE_RESULT",
                                    "failed": "failed",
                                },
                            )

                            smach.StateMachine.add(
                                "SAVE_RESULT",
                                self.cumulate_result(),
                                transitions={
                                    "succeeded": "continue",
                                    "failed": "failed",
                                },
                            )

                        inner_iterator.set_contained_state(
                            "INNER_CONTAINER_STATE", inner_container_sm, loop_outcomes=["continue"]
                        )

                    smach.StateMachine.add(
                        "INNER_ITERATOR",
                        inner_iterator,
                        {"succeeded": "CHECK_RESULT", "failed": "failed"},
                    )

                    smach.StateMachine.add(
                        "CHECK_RESULT",
                        self.detection_result(),
                        transitions={
                            "object_found": "SAY_OBJECT_FOUND",
                            "object_not_found": "SAY_OBJECT_NOT_FOUND",
                            "failed": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "SAY_OBJECT_FOUND",
                        say_object_found,
                        transitions={
                            "succeeded": "succeeded",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "SAY_OBJECT_NOT_FOUND",
                        say_object_not_found,
                        transitions={
                            "succeeded": "continue",
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

# if __name__ == "__main__":
#     import rospy
#     from sensor_msgs.msg import PointCloud2

#     location_list = list()
#     for i in range(3):
#         location = rospy.get_param(f"/LOCATION_{i}")
#         location_list.append(location)
#     rospy.init_node("test_find_object")
#     sm = GoFindTheObject(
#         Polygon(), filter=["cup"], locations=location_list, motions=motion
#     )
#     sm.userdata.pcl_msg = rospy.wait_for_message(
#         "/xtion/depth_registered/points", PointCloud2
#     )
#     sm.execute()
