#!/usr/bin/env python3
from go_to_location import GoToLocation
import smach
from lasr_skills import Detect3D
from shapely.geometry.polygon import Polygon
from typing import List, Union
from geometry_msgs.msg import Pose, Point, Quaternion
from lasr_skills import Say, PlayMotion

"""
location = rospy.get_param("/start") -> python dict
Pose(position : Point, orientation : Quaternion)
pose = Pose(position=Point(**location['pose']['position'], orientation=Quaternion(**location['pose']['orientation']))
"""


class Go_find_the_object(smach.StateMachine):
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
        area_polygon: Polygon,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        locations: Union[List[dict], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        motions: Union[List[str], None] = None,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        say_object_found = Say("I found the object here!")
        say_object_not_found = Say("the object is not here")

        with self:
            state = 0

            # Outer loop iterates over locations
            for i, location in enumerate(locations):
                pose = Pose(
                    position=Point(**location["pose"]["position"]),
                    orientation=Quaternion(**location["pose"]["orientation"]),
                )

                # Add the initial state for going to this location
                smach.StateMachine.add(
                    f"LOCATION_{i}",
                    GoToLocation(pose),
                    transitions={
                        "succeeded": f"LOOK_AROUND_{state}",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                # Inner loop iterates over motions
                for index, direction in enumerate(motions):
                    print(direction)
                    # Add a state to perform the current motion
                    smach.StateMachine.add(
                        f"LOOK_AROUND_{state}",
                        PlayMotion(motion_name=direction),
                        transitions={
                            "succeeded": f"DETECT_OBJECTS_3D_{state}",
                            "aborted": f"DETECT_OBJECTS_3D_{state}",
                            "preempted": "failed",
                        },
                    )

                    # Add a state to detect 3D objects after performing the motion
                    smach.StateMachine.add(
                        f"DETECT_OBJECTS_3D_{state}",
                        Detect3D(
                            depth_topic=depth_topic,
                            model=model,
                            filter=filter,
                            confidence=confidence,
                            nms=nms,
                        ),
                        transitions={
                            "succeeded": f"RESULT_{state}",
                            "failed": "failed",
                        },
                    )

                    # Add a state to check objects
                    smach.StateMachine.add(
                        f"RESULT_{state}",
                        self.check_objects(),
                        transitions={
                            "succeeded": f"SAVE_RESULT_{state}",
                            "failed": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        f"SAVE_RESULT_{state}",
                        self.cumulate_result(),
                        transitions={
                            "succeeded": (
                                f"LOOK_AROUND_{state+1}"
                                if index != (len(motions) - 1)
                                else f"CHECK_RESULT_{i}"
                            ),
                            "failed": "failed",
                        },
                    )
                    state += 1

                smach.StateMachine.add(
                    f"CHECK_RESULT_{i}",
                    self.detection_result(),
                    transitions={
                        "object_found": f"SAY_OBJECT_FOUND_{i}",
                        "object_not_found": f"SAY_OBJECT_NOT_FOUND_{i}",
                        "failed": "failed",
                    },
                )
                # Add states to handle object found or not found results
                smach.StateMachine.add(
                    f"SAY_OBJECT_FOUND_{i}",
                    say_object_found,
                    transitions={
                        "succeeded": (
                            f"LOCATION_{i+1}"
                            if i != len(locations) - 1
                            else "succeeded"
                        ),
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                smach.StateMachine.add(
                    f"SAY_OBJECT_NOT_FOUND_{i}",
                    say_object_not_found,
                    transitions={
                        "succeeded": (
                            f"LOCATION_{i+1}"
                            if i != len(locations) - 1
                            else "succeeded"
                        ),
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )


if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import PointCloud2

    location_list = list()
    for i in range(3):
        location = rospy.get_param(f"/LOCATION_{i}")
        location_list.append(location)
    motion = ["look_down", "look_down_left", "look_down_right"]
    rospy.init_node("test_find_object")
    sm = Go_find_the_object(
        Polygon(), filter=["cup"], locations=location_list, motions=motion
    )
    sm.userdata.pcl_msg = rospy.wait_for_message(
        "/xtion/depth_registered/points", PointCloud2
    )
    sm.execute()
