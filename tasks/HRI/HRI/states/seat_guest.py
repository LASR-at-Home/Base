import yasmin
from yasmin import StateMachine, State, Concurrence
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode


import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import tf2_ros as tf
import threading
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
)


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

        self._node = YasminNode.get_instance()

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

        seat_sofa = True
        seated_guests_loc = [
            detection.point
            for detection in blackboard["non_sofa_detections"]
            if detection.name == "person"
        ]
        seated_guests_sofa_loc = [
            detection.point
            for detection in blackboard["sofa_detections"]
            if detection.name == "person"
        ]
        seated_guest_locs = seated_guests_loc + seated_guests_sofa_loc
        yasmin.YASMIN_LOG_INFO(
            f"Detected {len(seated_guest_locs)} seated guests in the seating area."
        )
        yasmin.YASMIN_LOG_INFO(f"Detections are: {seated_guest_locs}")
        if len(seated_guest_locs) > 2:
            yasmin.YASMIN_LOG_WARN(
                f"Too many people detected: {len(seated_guest_locs)} detected, max allowed is 2."
            )
            blackboard["seated_guest_locs"] = seated_guest_locs[:2]
        else:
            blackboard["seated_guest_locs"] = seated_guest_locs
        yasmin.YASMIN_LOG_WARN(
            f"people on the sofa: {len(blackboard["sofa_detections"])} detected, max allowed is {self._max_people_on_sofa}."
        )
        if len(blackboard["sofa_detections"]) > self._max_people_on_sofa:
            yasmin.YASMIN_LOG_WARN(
                f"Too many people on the sofa: {len(blackboard["sofa_detections"])} detected, max allowed is {self._max_people_on_sofa}."
            )
            seat_sofa = False
        elif len(blackboard["sofa_detections"]) == self._max_people_on_sofa:
            yasmin.YASMIN_LOG_WARN(f"Sofa max capacity has been reached.")
            seat_sofa = False

        if seat_sofa:
            blackboard["guest_seat_point"] = PointStamped(
                header=Header(frame_id="map"), point=self._sofa_point
            )
            if len(blackboard["sofa_detections"]) == 0:
                blackboard["seating_string"] = "The sofa that I'm looking at is empty. Please take a seat anywhere on the sofa."
            elif len(blackboard["sofa_detections"]) == 1:
                seating_side = self._determine_side_of_sofa(blackboard["sofa_detections"][0])
                blackboard["seating_string"] = (
                    "The sofa that I'm looking at is occupied by one person. "
                    f"Please take a seat next to them on the {seating_side} side of the sofa."
                )
        else:
            seated_guests_xywh = [
                detection.xywh
                for detection in blackboard["non_sofa_detections"]
                if detection.name == "person"
            ]
            done = False
            for detection in blackboard["non_sofa_detections"]:
                if done:
                    break
                if detection.name == "chair":
                    # Check if a person is sitting on the chair
                    chair_bbox = detection.xywh
                    overlap_pct = 0.0
                    for guest_xywh in seated_guests_xywh:
                        overlap_pct_current = (
                            np.maximum(
                                0,
                                np.minimum(
                                    chair_bbox[0] + chair_bbox[2],
                                    guest_xywh[0] + guest_xywh[2],
                                )
                                - np.maximum(chair_bbox[0], guest_xywh[0]),
                            )
                            * np.maximum(
                                0,
                                np.minimum(
                                    chair_bbox[1] + chair_bbox[3],
                                    guest_xywh[1] + guest_xywh[3],
                                )
                                - np.maximum(chair_bbox[1], guest_xywh[1]),
                            )
                        ) / (chair_bbox[2] * chair_bbox[3])
                        overlap_pct = max(overlap_pct, overlap_pct_current)
                    if overlap_pct > 0.5:
                        yasmin.YASMIN_LOG_.info(
                            f"Detected a person sitting on a chair with bbox {chair_bbox}, with overlap percentage {overlap_pct:.2f}."
                        )
                        continue
                    else:
                        yasmin.YASMIN_LOG_.info(
                            f"No person detected sitting on chair with bbox {chair_bbox}."
                        )
                        blackboard["guest_seat_point"] = PointStamped(
                            header=Header(frame_id="map"),
                            point=Point(
                                x=detection.point.x,
                                y=detection.point.y,
                                z=detection.point.z,
                            ),
                        )
                        blackboard["seating_string"] = "The sofa is full, but I have found a chair for you. Please take a seat on the chair that I'm looking at."
                        done = True

            if not done:
                blackboard["seating_string"] = "Uh oh, I couldn't find a seat for you. Please take a seat anywhere in the seating area."
                blackboard["guest_seat_point"] = PointStamped(
                    header=Header(frame_id="map"),
                    point=Point(
                        x=self._sofa_point.x, y=self._sofa_point.y, z=self._sofa_point.z
                    ),
                )

        return "succeeded"


# TODO: update so that it the params are optional and directly loaded from the params (overriden by param if provided)
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
        node: Node,
        seating_area: Optional[ShapelyPolygon] = None,
        sofa_area: Optional[ShapelyPolygon] = None,
        sofa_point: Optional[Point] = None,
        left_sofa_area: Optional[ShapelyPolygon] = None,
        right_sofa_area: Optional[ShapelyPolygon] = None,
        max_people_on_sofa: Optional[int] = None,
        learn_host: bool = False,
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
            handle_sigint=True
        )
        self.add_input_key("guest_data")
        self.add_output_key("guest_seat_point")
        self.add_output_key("seated_guest_locs")

        self._node = YasminNode.get_instance()
        self.__load_ros_parameters()
        # TODO: Update to allow local paramters overriding ros param

        seating_area_minus_sofa = self.seating_area.difference(self.sofa_area)

        # self.userdata.z_sweep_min = (
        #     -0.5
        # )  # TODO: Remove when testing on robot move as paramter to detect3d...
        # self.userdata.z_sweep_max = 100  # TODO: Remove when testing on robot
        # self.blackboard["seated_guest_locs"] = []

        self.add_state(
            "SAY_FINDING_SEAT",
            Say(text="I will now find a seat for you."),
            transitions={
                "succeeded": "LOOK_TO_SOFA",
                "aborted": "LOOK_TO_SOFA",
                "preempted": "LOOK_TO_SOFA",
            },
        )
        self.add_state(
            "LOOK_TO_SOFA",
            LookToPoint(
                pointstamped=PointStamped(
                    header=Header(frame_id="map"), point=self.sofa_point
                )
            ),
            transitions={
                "succeeded": "DETECT_SOFA",
                "aborted": "failed",
                "preempted": "DETECT_SOFA",
            },
        )
        self.add_state(
            "DETECT_SOFA",
            Detect3DInArea(
                area_polygon=self.sofa_area,
                filter=["person"],
                confidence=0.7,
            ),
            transitions={"succeeded": "RESET_HEAD_1", "failed": "failed"},
            remapping={"detections_3d": "sofa_detections"},
        )
        self.add_state(
            "RESET_HEAD_1",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "DETECT_NON_SOFA",
                "aborted": "failed",
                "preempted": "failed",
            },
        )

        self.add_state(
            "DETECT_NON_SOFA",
            DetectAllInPolygon(
                polygon=seating_area_minus_sofa,  # TODO: Verify Potential type mismatch (BaseGeometry vs accepted ShapelyPolygon)
                object_filter=["person", "chair"],
                min_coverage=1.0,
                min_new_object_dist=0.50,
                min_confidence=0.5,
            ),
            transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
            remapping={"detected_objects": "non_sofa_detections"},
        )
        # Process detections
        if learn_host:
            detection_transition = "SAY_AND_LEARN_HOST_FACE"
        else:
            detection_transition = "LOOK_TO_SEAT"
        self.add_state(
            "PROCESS_DETECTIONS",
            ProcessDetections(
                max_people_on_sofa=self.max_people_on_sofa,
                sofa_point=self.sofa_point,
                left_sofa_area=self.left_sofa_area,
                right_sofa_area=self.right_sofa_area,
            ),
            transitions={"succeeded": detection_transition, "failed": "failed"}
        )
        if learn_host:
            # Look to the only person detection and learn the host's face.
            sm_con = Concurrence(
                states={
                    "SAY_LEARN_HOST_FACE": Say(text="I'm quickly remembering the host's face."),
                    "LEARN_HOST_FACE": LearnHostFace(),
                },
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "SAY_LEARN_HOST_FACE": "succeeded",
                        "LEARN_HOST_FACE": "succeeded",
                    },
                    "failed": {
                        "SAY_LEARN_HOST_FACE": "aborted",
                        "LEARN_HOST_FACE": "failed",
                    },
                },
            )

            sm_con.add_input_key("guest_data")
            sm_con.add_input_key("seated_guest_locs")
            sm_con.add_output_key("guest_data")
            sm_con.add_output_key("seated_guest_locs")


            self.add_state(
                "SAY_AND_LEARN_HOST_FACE",
                sm_con,
                transitions={"succeeded": "LOOK_TO_SEAT", "failed": "LOOK_TO_SEAT"},
            )

        self.add_state(
            "LOOK_TO_SEAT",
            LookToPoint(node=node),
            transitions={
                "succeeded": "SAY_SEAT_GUEST",
                "aborted": "SAY_SEAT_GUEST",
                "preempted": "SAY_SEAT_GUEST",
            },
            remapping={"pointstamped": "guest_seat_point"},
        )
        self.add_state(
            "SAY_SEAT_GUEST",
            Say(),  # TODO: verify no text needed
            transitions={
                "succeeded": "WAIT_FOR_GUEST_TO_SEAT",
                "aborted": "WAIT_FOR_GUEST_TO_SEAT",
                "preempted": "WAIT_FOR_GUEST_TO_SEAT",
            },
            remapping={"textsm_con": "seating_string"},
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
                "preempted": "succeeded",
            },
        )

    def __load_ros_parameters(self):
        # Declare parameters
        self._node.declare_parameter("sofa_point.x", 0.0)
        self._node.declare_parameter("sofa_point.y", 0.0)
        self._node.declare_parameter("sofa_point.z", 0.0)

        self._node.declare_parameter("seat_area.top_left", [0.0, 0.0])
        self._node.declare_parameter("seat_area.top_right", [0.0, 0.0])
        self._node.declare_parameter("seat_area.bottom_right", [0.0, 0.0])
        self._node.declare_parameter("seat_area.bottom_left", [0.0, 0.0])

        self._node.declare_parameter("sofa_area.top_left", [0.0, 0.0])
        self._node.declare_parameter("sofa_area.top_right", [0.0, 0.0])
        self._node.declare_parameter("sofa_area.bottom_right", [0.0, 0.0])
        self._node.declare_parameter("sofa_area.bottom_left", [0.0, 0.0])

        self._node.declare_parameter("max_people_on_sofa", 2)

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
            "top_right": np.array(self._node.get_parameter("sofa_area.top_right").value),
            "bottom_right": np.array(
                self._node.get_parameter("sofa_area.bottom_right").value
            ),
            "bottom_left": np.array(
                self._node.get_parameter("sofa_area.bottom_left").value
            ),
        }

        # TODO: Check if number of section on sofa depends on number of
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


def main():

    rclpy.init()

    try:
        outcome = SeatGuest(learn_host=True)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
