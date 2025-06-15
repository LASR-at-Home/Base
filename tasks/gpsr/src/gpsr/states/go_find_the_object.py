#!/usr/bin/env python3
import smach
import smach_ros
from lasr_skills import GoToLocation, LookToPoint, Say
from shapely.geometry.polygon import Polygon
from typing import List, Union
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest
from lasr_vision_msgs.msg import CDRequest
import rospy

"""
location = rospy.get_param("/start") -> python dict
Pose(position : Point, orientation : Quaternion)
pose = Pose(position=Point(**location['pose']['position'], orientation=Quaternion(**location['pose']['orientation']))
"""


class GoFindTheObject(smach.StateMachine):
    class GetLocation(smach.State):
        def __init__(
            self,
            waypoints: List[Pose],
            polygons: List[Polygon],
            look_points: List[Point],
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location_index", "waypoints"],
                output_keys=["location", "polygon", "pointstamped"],
            )
            self.waypoints = waypoints
            self.polygons = polygons
            self.look_points = look_points

        def execute(self, userdata):
            userdata.location = self.waypoints[userdata.location_index]
            userdata.polygon = self.polygons[userdata.location_index]
            userdata.pointstamped = PointStamped(
                point=self.look_points[userdata.location_index],
                header=Header(frame_id="map"),
            )
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

    class detection_result(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["object_found", "object_not_found", "failed"],
                input_keys=["detection_result"],
                output_keys=["result"],
            )

        def execute(self, userdata):
            if any(userdata.detection_result):
                userdata.result = True
                return "object_found"
            else:
                userdata.result = False
                return "object_not_found"

    def __init__(
        self,
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,  # <- input
        waypoints: Union[List[Pose], None] = None,  # <- input
        locations: Union[str, None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        poly_points: List[
            Polygon
        ] = None,  # polygen list of polygen for every location   <- input
        look_points: List[
            Point
        ] = None,  # point to look at for each location      <- input
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        if waypoints is None and locations is None:
            raise ValueError("Either waypoints or location_param must be provided")
        if poly_points is None or len(poly_points) != len(waypoints_to_iterate):
            raise ValueError("Poly points must be provided for each waypoint")

        if look_points is None or len(look_points) != len(waypoints_to_iterate):
            raise ValueError("Look points must be provided for each waypoint")

        if waypoints is None:
            waypoints_to_iterate: List[Pose] = []
            room = rospy.get_param(locations)
            beacons = room["beacons"]
            for beacon in beacons:
                waypoint = Pose(
                    position=Point(**beacons[beacon]["near_pose"]["position"]),
                    orientation=Quaternion(
                        **beacons[beacon]["near_pose"]["orientation"]
                    ),
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
                output_keys=["cumulated_result"],
                exhausted_outcome="failed",
            )

            with waypoint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["location_index", "waypoints"],
                    output_keys=["cumulated_result"],
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
                            "succeeded": "LOOK_AT_POINT",
                            "failed": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "LOOK_AT_POINT",
                        LookToPoint(),
                        transitions={
                            "succeeded": "DETECT_OBJECTS_3D",
                            "failed": "failed",
                        },
                        remapping={"look_point": "look_point"},
                    )

                    smach.StateMachine.add(
                        "DETECT_OBJECTS_3D",
                        smach_ros.ServiceState(
                            "/vision/cropped_detection",
                            CroppedDetection,
                            request=CroppedDetectionRequest(
                                requests=[
                                    CDRequest(
                                        method="closest",
                                        use_mask=True,
                                        yolo_model=model,
                                        yolo_model_confidence=confidence,
                                        yolo_nms_threshold=nms,
                                        return_sensor_reading=False,
                                        object_names=filter,
                                        polygons=[],
                                    )
                                ]
                            ),
                            output_keys=["responses"],
                            response_slots=["responses"],
                        ),
                        transitions={
                            "succeeded": "RESULT",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"polygon": "polygon"},
                    )

                    smach.StateMachine.add(
                        "RESULT",
                        self.check_objects(),
                        transitions={
                            "succeeded": "CHECK_RESULT",
                            "failed": "failed",
                        },
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
