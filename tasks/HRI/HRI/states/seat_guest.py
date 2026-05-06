import smach
from smach_ros import RosState

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import tf2_ros as tf
import threading

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

class ProcessDetections(RosState):

    _max_people_on_sofa: int
    _sofa_point: Point
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(
        self,
        node: Node,
        sofa_point: Point,
        left_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
        max_people_on_sofa: int = 2,
    ):
        RosState.__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["non_sofa_detections", "sofa_detections"],
            output_keys=["guest_seat_point", "seated_guest_locs", "seating_string"],
        )
        self._max_people_on_sofa = max_people_on_sofa
        self._sofa_point = sofa_point
        self._left_sofa_area = left_sofa_area
        self._right_sofa_area = right_sofa_area
        self._tf_buffer = tf.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self.node)

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
            self.node.get_logger().warn(
                "Sofa guest point is not within the left or right sofa area. "
                "Defaulting to 'right'."
            )
            result = "right"

        return result

    def execute(self, userdata):
        """
        Input:
            userdata.non_sofa_detections (List[Detection3D]): List of detected objects that are not on the sofa
            userdata.sofa_detections (List[Detection3D]): List of detected objects on the sofa
        """

        seat_sofa = True
        seated_guests_loc = [
            detection.point
            for detection in userdata.non_sofa_detections
            if detection.name == "person"
        ]
        seated_guests_sofa_loc = [
            detection.point
            for detection in userdata.sofa_detections
            if detection.name == "person"
        ]
        seated_guest_locs = seated_guests_loc + seated_guests_sofa_loc
        self.node.get_logger().info(
            f"Detected {len(seated_guest_locs)} seated guests in the seating area."
        )
        self.node.get_logger().info(f"Detections are: {seated_guest_locs}")
        if len(seated_guest_locs) > 2:
            self.node.get_logger().warn(
                f"Too many people detected: {len(seated_guest_locs)} detected, max allowed is 2."
            )
            userdata.seated_guest_locs = seated_guest_locs[:2]
        else:
            userdata.seated_guest_locs = seated_guest_locs
        self.node.get_logger().warn(
                f"people on the sofa: {len(userdata.sofa_detections)} detected, max allowed is {self._max_people_on_sofa}."
            )
        if len(userdata.sofa_detections) > self._max_people_on_sofa:
            self.node.get_logger().warn(
                f"Too many people on the sofa: {len(userdata.sofa_detections)} detected, max allowed is {self._max_people_on_sofa}."
            )
            seat_sofa = False
        elif len(userdata.sofa_detections) == self._max_people_on_sofa:
            self.node.get_logger().info(f"Sofa max capacity has been reached.")
            seat_sofa = False

        if seat_sofa:
            userdata.guest_seat_point = PointStamped(
                header=Header(frame_id="map"), point=self._sofa_point
            )
            if len(userdata.sofa_detections) == 0:
                userdata.seating_string = "The sofa that I'm looking at is empty. Please take a seat anywhere on the sofa."
            elif len(userdata.sofa_detections) == 1:
                seating_side = self._determine_side_of_sofa(userdata.sofa_detections[0])
                userdata.seating_string = (
                    "The sofa that I'm looking at is occupied by one person. "
                    f"Please take a seat next to them on the {seating_side} side of the sofa."
                )
        else:
            seated_guests_xywh = [
                detection.xywh
                for detection in userdata.non_sofa_detections
                if detection.name == "person"
            ]
            done = False
            for detection in userdata.non_sofa_detections:
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
                        self.node.get_logger().info(
                            f"Detected a person sitting on a chair with bbox {chair_bbox}, with overlap percentage {overlap_pct:.2f}."
                        )
                        continue
                    else:
                        self.node.get_logger().info(
                            f"No person detected sitting on chair with bbox {chair_bbox}."
                        )
                        userdata.guest_seat_point = PointStamped(
                            header=Header(frame_id="map"),
                            point=Point(
                                x=detection.point.x,
                                y=detection.point.y,
                                z=detection.point.z,
                            ),
                        )
                        userdata.seating_string = "The sofa is full, but I have found a chair for you. Please take a seat on the chair that I'm looking at."
                        done = True

            if not done:
                userdata.seating_string = "Uh oh, I couldn't find a seat for you. Please take a seat anywhere in the seating area."
                userdata.guest_seat_point = PointStamped(
                    header=Header(frame_id="map"),
                    point=Point(
                        x=self._sofa_point.x, y=self._sofa_point.y, z=self._sofa_point.z
                    ),
                )

        return "succeeded"

class SeatGuest(smach.StateMachine): #TODO: update so that it the params are optional and directly loaded from the params (overriden by param if provided)
    '''
    args:
        node (Node): a node.
        seating_area (ShapelyPolygon): The general area for guest detection.
        sofa_area (ShapelyPolygon): The seatable sofa area.
        sofa_point (Point): The 3D coordinate where the robot initaily looks at sofa.
        left_sofa_area (ShapelyPolygon): An additional area.
        right_sofa_area (ShapelyPolygon): Geometric sub-region for the right side of the sofa[cite: 1].
        max_people_on_sofa (int): Maximum occupancy limit (default: 2)[cite: 1].
        learn_host (bool): Whether to perform the host-learning routine (default: False)[cite: 1].
    '''
    def __init__(
        self,
        node: Node,
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        sofa_point: Point,
        left_sofa_area: ShapelyPolygon,
        right_sofa_area: ShapelyPolygon,
        max_people_on_sofa: int = 2,
        learn_host: bool = False,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_seat_point", "seated_guest_locs"],
        )

        self.__node = node
        seating_area_minus_sofa = seating_area.difference(sofa_area)

        with self:
            self.userdata.z_sweep_min = 0.4 #TODO: Remove when testing on robot
            self.userdata.z_sweep_max = 1.2 #TODO: Remove when testing on robot
            self.userdata.seated_guest_locs = []
            smach.StateMachine.add(
                "SAY_FINDING_SEAT",
                Say(node=self.__node, text="I will now find a seat for you."),
                transitions={
                    "succeeded": "LOOK_TO_SOFA",
                    "aborted": "LOOK_TO_SOFA",
                    "preempted": "LOOK_TO_SOFA",
                },
            )
            smach.StateMachine.add(
                "LOOK_TO_SOFA",
                LookToPoint(
                    node=self.__node,
                    pointstamped=PointStamped(
                        header=Header(frame_id="map"), point=sofa_point
                    )
                ),
                transitions={
                    "succeeded": "DETECT_SOFA",
                    "aborted": "failed",
                    "timed_out": "failed",
                },
            )
            smach.StateMachine.add(
                "DETECT_SOFA",
                Detect3DInArea(node=self.__node, area_polygon=sofa_area, filter=["person"], confidence=0.7),
                transitions={"succeeded": "RESET_HEAD_1", "failed": "failed"},
                remapping={"detections_3d": "sofa_detections"},
            )
            smach.StateMachine.add(
                "RESET_HEAD_1",
                PlayMotion(node=self.__node, motion_name="look_centre"),
                transitions={
                    "succeeded": "DETECT_NON_SOFA",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_NON_SOFA",
                DetectAllInPolygon(
                    node=self.__node,
                    polygon=seating_area_minus_sofa,    #TODO: Verify Potential type mismatch (BaseGeometry vs accepted ShapelyPolygon) 
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
            smach.StateMachine.add(
                "PROCESS_DETECTIONS",
                ProcessDetections(
                    node=self.__node,
                    max_people_on_sofa=max_people_on_sofa,
                    sofa_point=sofa_point,
                    left_sofa_area=left_sofa_area,
                    right_sofa_area=right_sofa_area,
                ),
                transitions={"succeeded": detection_transition, "failed": "failed"},
                remapping={
                    "guest_seat_point": "guest_seat_point",
                    "seating_detections": "seating_detections",
                },
            )
            if learn_host:
                # Look to the only person detection and learn the host's face.
                sm_con = smach.Concurrence(
                    outcomes=["succeeded", "failed"],
                    input_keys=["guest_data", "seated_guest_locs"],
                    output_keys=["guest_data", "seated_guest_locs"],
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
                with sm_con:
                    smach.Concurrence.add(
                        "SAY_LEARN_HOST_FACE",
                        Say(node=self.__node, text="I'm quickly remembering the host's face."),
                    )
                    smach.Concurrence.add("LEARN_HOST_FACE", LearnHostFace(node=self.__node))
                smach.StateMachine.add(
                    "SAY_AND_LEARN_HOST_FACE",
                    sm_con,
                    transitions={"succeeded": "LOOK_TO_SEAT", "failed": "LOOK_TO_SEAT"},
                )

            smach.StateMachine.add(
                "LOOK_TO_SEAT",
                LookToPoint(node=self.__node),
                transitions={
                    "succeeded": "SAY_SEAT_GUEST",
                    "aborted": "SAY_SEAT_GUEST",
                    "timed_out": "SAY_SEAT_GUEST",
                },
                remapping={"pointstamped": "guest_seat_point"},
            )
            smach.StateMachine.add(
                "SAY_SEAT_GUEST",
                Say(node=self.__node),  #TODO: verify no text needed
                transitions={
                    "succeeded": "WAIT_FOR_GUEST_TO_SEAT",
                    "aborted": "WAIT_FOR_GUEST_TO_SEAT",
                    "preempted": "WAIT_FOR_GUEST_TO_SEAT",
                },
                remapping={"textsm_con": "seating_string"},
            )
            smach.StateMachine.add(
                "WAIT_FOR_GUEST_TO_SEAT",
                Wait(node=self.__node, wait_time=5.0),
                transitions={"succeeded": "RESET_HEAD_2", "failed": "RESET_HEAD_2"},
            )

            smach.StateMachine.add(
                "RESET_HEAD_2",
                PlayMotion(node=self.__node, motion_name="look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )


def main():

    rclpy.init()
    node = rclpy.create_node("hri")
        
    node.declare_parameter('sofa_point.x', 0.0)
    node.declare_parameter('sofa_point.y', 0.0)
    node.declare_parameter('sofa_point.z', 0.0)

    node.declare_parameter('seat_area.top_left', [3.559335708618164, 1.3495814800262451])
    node.declare_parameter('seat_area.top_right', [3.5363903045654297, -1.7019412517547607])
    node.declare_parameter('seat_area.bottom_right', [0.666893720626831, -1.5459266901016235])
    node.declare_parameter('seat_area.bottom_left', [0.9254391193389893, 1.7120180130004883])

    node.declare_parameter('sofa_area.top_left', [0.0, 0.0])
    node.declare_parameter('sofa_area.top_right', [0.0, 0.0])
    node.declare_parameter('sofa_area.bottom_right', [0.0, 0.0])
    node.declare_parameter('sofa_area.bottom_left', [0.0, 0.0])

    node.declare_parameter('max_people_on_sofa', 2)

    seat_polygon = ShapelyPolygon([
        node.get_parameter('seat_area.top_left').value,
        node.get_parameter('seat_area.top_right').value,
        node.get_parameter('seat_area.bottom_right').value,
        node.get_parameter('seat_area.bottom_left').value
    ])

    sofa_point = Point(
        x=node.get_parameter('sofa_point.x').value, 
        y=node.get_parameter('sofa_point.y').value, 
        z=node.get_parameter('sofa_point.z').value)

    sofa_area = {
      "top_left":       np.array(node.get_parameter('sofa_area.top_left').value),
      "top_right":      np.array(node.get_parameter('sofa_area.top_right').value),
      "bottom_right":   np.array(node.get_parameter('sofa_area.bottom_right').value),
      "bottom_left":    np.array(node.get_parameter('sofa_area.bottom_left').value)
    }
    
    #TODO: Check if number of section on sofa depends on number of  
    sofa_middle_top = (sofa_area["top_right"] + sofa_area["top_left"]) / 2
    sofa_middle_bottom = (sofa_area["bottom_left"] + sofa_area["bottom_right"]) / 2
    sofa_polygon = ShapelyPolygon([
        sofa_area["top_left"],
        sofa_area["top_right"],
        sofa_area["bottom_right"],
        sofa_area["bottom_left"],
    ])

    left_sofa_polygon = ShapelyPolygon([
        sofa_area["top_left"],
        sofa_middle_top,
        sofa_middle_bottom,
        sofa_area["bottom_left"],
    ])

    right_sofa_polygon = ShapelyPolygon([
        sofa_middle_top,
        sofa_area["top_right"],
        sofa_area["bottom_right"],
        sofa_middle_bottom,
    ])

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    sm = SeatGuest(
        node=node,

        seating_area = seat_polygon,

        sofa_point = sofa_point,
        sofa_area = sofa_polygon,
        left_sofa_area = left_sofa_polygon,  
        right_sofa_area = right_sofa_polygon,

        max_people_on_sofa = int(node.get_parameter('max_people_on_sofa').value),
        learn_host = True,
    )

    outcome = sm.execute()
    node.get_logger().info(f"State machine finished with outcome: {outcome}")


if __name__ == "__main__":
    main()