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

from .learn_host_face import LearnHostFace
from lasr_vision_interfaces.msg import Detection3D
from lasr_skills import (
    PlayMotion,
    Detect3DInArea,
    LookToPoint,
    Say,
    Wait,
    DetectAllInPolygon,
    StopEyeTracker,
)

from yasmin_viewer import YasminViewerPub


class ProcessDetections(State):

    _max_people_on_sofa: int
    _sofa_point: Point
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(
        self,
        sofa_point: Point,
        left_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
        max_people_on_sofa: int = 2,
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self._node = yasmin_ros.logger_node

        self.add_input_key("non_sofa_detections")
        self.add_input_key("sofa_detections")

        self.add_output_key("guest_seat_point")
        self.add_output_key("seated_guest_locs")
        self.add_output_key("seating_string")

        self._max_people_on_sofa = max_people_on_sofa
        self._sofa_point = sofa_point
        self._left_sofa_area = left_sofa_area
        self._right_sofa_area = right_sofa_area
        self._tf_buffer = tf.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self._node)

    def _determine_side_of_sofa(self, sofa_detection: Detection3D) -> str:
        """Determines which side of the sofa is empty, in order to seat
        the guest there.

        Args:
            sofa_detection (Detection3D): Detection of the other
            guest who is already sat on the sofa.

        Returns:
            str: "left" or "right" - which side of the sofa is empty.
        """
        sofa_guest_point = sofa_detection.point

        if self._left_sofa_area.contains(
            ShapelyPoint(sofa_guest_point.x, sofa_guest_point.y)
        ):
            result = "right"
        elif self._right_sofa_area.contains(
            ShapelyPoint(sofa_guest_point.x, sofa_guest_point.y)
        ):
            result = "left"
        else:
            yasmin.YASMIN_LOG_WARN(
                "Sofa guest point is not within the left or right sofa area. "
                "Defaulting to 'right'."
            )
            result = "right"

        return result

    def execute(self, blackboard):
        """
        Input:
            blackboard["non_sofa_detections"] (List[Detection3D]): List of detected objects that are not on the sofa
            blackboard["sofa_detections"] (List[Detection3D]): List of detected objects on the sofa
        """

        yasmin.YASMIN_LOG_WARN("Finding seat in seat guest")
        left_sofa_occupied = False
        right_sofa_occupied = False
        unseated_sofa_persons = []
        non_sofa_chairs = {}
        
        for detection in blackboard['seat_detections']:
            detection_point = ShapelyPoint(detection.point.x, detection.point.y, detection.point.z)
            if detection.name == "person":
                if self._left_sofa_area.contains(detection_point):
                    left_sofa_occupied = True
                elif self._right_sofa_area.contains(detection_point):
                    right_sofa_occupied = True
                else:
                    unseated_sofa_persons.append(detection_point)
            elif detection.name == 'chair' and not self._right_sofa_area.contains(detection_point) and not self._left_sofa_area.contains(detection_point):
                non_sofa_chairs.update({detection_point: False})
                
        for chair_detection in non_sofa_chairs.keys():
            for person_detection in unseated_sofa_persons:
                if chair_detection.distance(person_detection) < 0.2:
                    non_sofa_chairs[chair_detection] = True # Chair is occupied
                    break
        
        if left_sofa_occupied != right_sofa_occupied:
            seating_side = 'left' if right_sofa_occupied else 'right'
            blackboard["seating_string"] = (
                "The sofa that I'm looking at is occupied by one person. "
                f"Please take a seat next to them on the {seating_side} side of the sofa."
            )
            blackboard['guest_seat_point'] = PointStamped(
                header=Header(frame_id="map"), point=self._sofa_point
            )
        elif left_sofa_occupied and right_sofa_occupied:
            for chair in non_sofa_chairs.keys():
                if not non_sofa_chairs[chair]:
                    blackboard['seating_string'] = "The sofa that I'm looking at is at full capacity. I have found an extra seat for you. Please take sit down in the seat I am looking at."
                    blackboard['guest_seat_point'] = PointStamped(
                        header=Header(frame_id="map"), point=Point(x=chair.x, y=chair.y, z=chair.z)
                    )
                    break
        else:
            blackboard['seating_string'] = "The sofa that I'm looking at is empty. Please take a seat anywhere on the sofa."
            blackboard['guest_seat_point'] = PointStamped(
                header=Header(frame_id="map"), point=self._sofa_point
            )
                    
        return 'succeeded'


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
        learn_host: bool = False,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_output_key("guest_seat_point")

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
                "succeeded": "DETECT_NON_SOFA",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "DETECT_ALL_PEOPLE_SEATS",
            DetectAllInPolygon(
                polygon=self.seating_area,
                object_filter=["person", "chair"],
                min_coverage=1.0,
                min_new_object_dist=0.50,
                min_confidence=0.5,
            ),
            transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
            remappings={"detected_objects": "seat_detections"},
        )
        
        self.add_state(
            "PROCESS_DETECTIONS",
            ProcessDetections(
                max_people_on_sofa=self.max_people_on_sofa,
                sofa_point=self.sofa_point,
                left_sofa_area=self.left_sofa_area,
                right_sofa_area=self.right_sofa_area,
            ),
            transitions={"succeeded": "LOOK_TO_SEAT", "failed": "failed"},
        )

        self.add_state(
            "LOOK_TO_SEAT",
            LookToPoint(),
            transitions={
                "succeeded": "SAY_SEAT_GUEST",
                "aborted": "SAY_SEAT_GUEST",
                "canceled": "SAY_SEAT_GUEST",
                "timeout": "SAY_SEAT_GUEST",
            },
            remappings={"pointstamped": "guest_seat_point"},
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
            transitions={"succeeded": "RESET_HEAD_2", "failed": "RESET_HEAD_2"},
        )

        self.add_state(
            "RESET_HEAD_2",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )
        

    def __load_ros_parameters(self):

        # Load parameters from file
        self.seating_area = ShapelyPolygon(
            [
                self._node.get_parameter("seat_area.top_left").value,
                self._node.get_parameter("seat_area.top_right").value,
                self._node.get_parameter("seat_area.bottom_right").value,
                self._node.get_parameter("seat_area.bottom_left").value,
            ]
        )

        self.sofa_point = Point(
            x=self._node.get_parameter("sofa_point.x").value,
            y=self._node.get_parameter("sofa_point.y").value,
            z=self._node.get_parameter("sofa_point.z").value,
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
        sofa_middle_top = (sofa_area["top_right"] + sofa_area["top_left"]) / 2
        sofa_middle_bottom = (sofa_area["bottom_left"] + sofa_area["bottom_right"]) / 2

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
                sofa_middle_top,
                sofa_middle_bottom,
                sofa_area["bottom_left"],
            ]
        )

        self.right_sofa_area = ShapelyPolygon(
            [
                sofa_middle_top,
                sofa_area["top_right"],
                sofa_area["bottom_right"],
                sofa_middle_bottom,
            ]
        )

        self.max_people_on_sofa = int(
            self._node.get_parameter("max_people_on_sofa").value
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
        # TODO: Try with learn_host=True
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
