import yasmin
from yasmin import StateMachine, State, Concurrence, Blackboard
import yasmin_ros
from yasmin_viewer import YasminViewerPub

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import tf2_ros as tf
from typing import Optional

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point as ShapelyPoint

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from lasr_vision_interfaces.msg import Detection3D
from lasr_skills import (
    PlayMotion,
    Detect3DInArea,
    LookToPoint,
    Say,
    Wait,
    ReceiveObject,
    DetectAllInPolygon,
    StopEyeTracker,
)

from HRI.states import HRILearnFaces

from yasmin_viewer import YasminViewerPub


class ProcessDetections(State):

    _max_people_on_sofa: int
    _sofa_point: Point
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(
        self,
        left_sofa_point: ShapelyPoint,
        right_sofa_point: ShapelyPoint,
        middle_sofa_point: ShapelyPoint,
        left_sofa_area: ShapelyPolygon,
        middle_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self._node = yasmin_ros.logger_node

        self.add_input_key("non_sofa_detections")
        self.add_input_key("sofa_detections")

        self.add_output_key("guest_seat_point")
        self.add_output_key("guest2_seat")
        self.add_output_key("seating_string")


        self.left_sofa_point = left_sofa_point
        self.right_sofa_point = right_sofa_point
        self.middle_sofa_point = middle_sofa_point
        self._left_sofa_area = left_sofa_area
        self._middle_sofa_area = middle_sofa_area
        self._right_sofa_area = right_sofa_area
        self._tf_buffer = tf.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self._node)

    def execute(self, blackboard):
        """
        Input:
            blackboard["non_sofa_detections"] (List[Detection3D]): List of detected objects that are not on the sofa
            blackboard["sofa_detections"] (List[Detection3D]): List of detected objects on the sofa
        """
        
        left_sofa_occupied = False
        middle_sofa_occupied = False
        right_sofa_occupied = False

        for detection in blackboard["people_detections"]:
            detection_point = ShapelyPoint(
                detection.point.x, detection.point.y, detection.point.z
            )
            if self._left_sofa_area.contains(detection_point):
                left_sofa_occupied = True
            elif self._middle_sofa_area.contains(detection_point):
                middle_sofa_occupied = True
            elif self._right_sofa_area.contains(detection_point):
                right_sofa_occupied = True
                
        blackboard['people_det'] = blackboard['people_detections']
                
        if not left_sofa_occupied and not right_sofa_occupied and not middle_sofa_occupied:
            blackboard["seating_string"] = (
                "The sofa that I'm looking at is empty. Please take a seat anywhere on the sofa."
            )
        elif not left_sofa_occupied and not right_sofa_occupied and middle_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by one person. Please take a seat in the left side or right side of the sofa.'
            )
        elif not left_sofa_occupied and not middle_sofa_occupied and right_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by one person. Please take a seat in the middle or left side of the sofa.'
            )
        elif not right_sofa_occupied and not middle_sofa_occupied and left_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by one person. Please take a seat in the middle or right side of the sofa.'
            )
        elif not left_sofa_occupied and right_sofa_occupied and middle_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by two people. Please take a seat on the left side of the sofa.'
            )
            blackboard['guest2_seat'] = self.left_sofa_point
        elif not right_sofa_occupied and left_sofa_occupied and middle_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by two people. Please take a seat on the right side of the sofa.'
            )
            blackboard['guest2_seat'] = self.right_sofa_point
        elif not middle_sofa_occupied and left_sofa_occupied and right_sofa_occupied:
            blackboard["seating_string"] = (
                'The sofa is currently occupied by two people. Please take a seat in the middle of the sofa.'
            )
            blackboard['guest2_seat'] = self.middle_sofa_point

        return "succeeded"


class SeatGuest(StateMachine):
    """
    args:
        node (Node): a node.
        seating_area (ShapelyPolygon): The general area for guest detection.
        sofa_area (ShapelyPolygon): The seatable sofa area.
        sofa_point (Point): The 3D coordinate where the robot initaily looks at sofa.
        left_sofa_area (ShapelyPolygon): An additional area.
        right_sofa_area (ShapelyPolygon): Geometric sub-region for the right side of the sofa.
        max_people_on_sofa (int): Maximum occupancy limit (default: 2).
        learn_host (bool): Whether to perform the host-learning routine (default: False).
    """

    def __init__(
        self,
        id
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_output_key("people_detections")
        self.guest_id = id

        self._node = yasmin_ros.logger_node
        self.__load_ros_parameters()

        self.add_state(
            "SAY_FINDING_SEAT",
            Say(text="I will now find a seat for you."),
            transitions={
                "succeeded": "RESET_HEAD_1",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "RESET_HEAD_1",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "DETECT_ALL_PEOPLE_SOFA",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "DETECT_ALL_PEOPLE_SOFA",
            Detect3DInArea(
                area_polygon=self.sofa_area,
                filter=["person"],
                model='yolo11n-seg.pt',
                z_min=-10,
                z_max=50,
                confidence=0.8,
                target_frame='map'
            ),
            transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
            remappings={"detections_3d": "people_detections"},
        )

        self.add_state(
            "PROCESS_DETECTIONS",
            ProcessDetections(
                left_sofa_point=self.left_sofa_point,
                right_sofa_point=self.right_sofa_point,
                middle_sofa_point=self.middle_sofa_point,
                left_sofa_area=self.left_sofa_area,
                middle_sofa_area=self.middle_sofa_area,
                right_sofa_area=self.right_sofa_area,
            ),
            transitions={"succeeded": "SAY_SEAT_GUEST", "failed": "failed"},
        )

        self.add_state(
            "SAY_SEAT_GUEST",
            Say(),
            transitions={
                "succeeded": "WAIT_FOR_GUEST_TO_SEAT",
                "aborted": "WAIT_FOR_GUEST_TO_SEAT",
                "canceled": "WAIT_FOR_GUEST_TO_SEAT",
            },
            remappings={"text": "seating_string"},
        )

        self.add_state(
            "WAIT_FOR_GUEST_TO_SEAT",
            Wait(wait_time=5.0),
            transitions={"succeeded": 'succeeded', "failed": "failed"},
        )
        

    def __load_ros_parameters(self):

        # Load parameters from file
        
        self.left_sofa_point = Point(
            x=self._node.get_parameter("left_sofa_point.x").value,
            y=self._node.get_parameter("left_sofa_point.y").value,
            z=self._node.get_parameter("left_sofa_point.z").value,
        )
        
        self.middle_sofa_point = Point(
            x=self._node.get_parameter("middle_sofa_point.x").value,
            y=self._node.get_parameter("middle_sofa_point.y").value,
            z=self._node.get_parameter("middle_sofa_point.z").value,
        )
        
        self.right_sofa_point = Point(
            x=self._node.get_parameter("right_sofa_point.x").value,
            y=self._node.get_parameter("right_sofa_point.y").value,
            z=self._node.get_parameter("right_sofa_point.z").value,
        )

        sofa_area = {
            "top_left": np.array(self._node.get_parameter("sofa_area.top_left").value),
            "top_right": np.array(
                self._node.get_parameter("sofa_area.top_right").value
            ),
            "bottom_right": np.array(
                self._node.get_parameter("sofa_area.bottom_right").value
            ),
            "bottom_left": np.array(
                self._node.get_parameter("sofa_area.bottom_left").value
            ),
        }
        
        dist_top = sofa_area['top_right'] + sofa_area['top_left']
        dist_bot = sofa_area['bottom_right'] + sofa_area['bottom_left']
        
        sofa_middle_top_left = dist_top / 3
        sofa_middle_top_right = (dist_top / 3) * 2
        sofa_middle_bottom_left = dist_bot / 3
        sofa_middle_bottom_right = (dist_bot / 3) * 2

        self.sofa_area = ShapelyPolygon(
            [
                sofa_area["top_left"],
                sofa_area["top_right"],
                sofa_area["bottom_right"],
                sofa_area["bottom_left"],
            ]
        )

        self.left_sofa_area = ShapelyPolygon(
            [
                sofa_area["top_left"],
                sofa_middle_top_left,
                sofa_middle_bottom_left,
                sofa_area["bottom_left"],
            ]
        )
        
        self.middle_sofa_area = ShapelyPolygon(
            [
                sofa_middle_top_left,
                sofa_middle_top_right,
                sofa_middle_bottom_left,
                sofa_middle_bottom_right
            ]
        )

        self.right_sofa_area = ShapelyPolygon(
            [
                sofa_middle_top_right,
                sofa_area["top_right"],
                sofa_area["bottom_right"],
                sofa_middle_bottom_right,
            ]
        )


try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor
from threading import Thread


class HRI_node(Node):
    def __init__(self):
        super().__init__(
            node_name="hri",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():
    rclpy.init()
    node = HRI_node()

    yasmin_ros.set_ros_loggers(node)

    try:
        sm = SeatGuest(learn_host=False)
        bb = Blackboard()

        bb["guest_data"] = {
            "host": {
                "name": "Fadi",
                "drink": "Fanta",
                "detection": False,
                "seating_detection": False,
            },
            "guest1": {
                "name": "Aldrich",
                "drink": "Coke",
                "detection": False,
                "seating_detection": False,
            },
            "guest2": {
                "name": "",
                "drink": "",
                "detection": False,
                "seating_detection": False,
            },
        }

        YasminViewerPub(sm, "HRI_SM3")

        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
