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
    AskAndListen,
    Say,
    Wait,
    WaitForPersonInArea
)

from HRI.states import (
    GetPersonPoint,
)

from yasmin_viewer import YasminViewerPub

class RequestHostForGuiding(StateMachine):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self._node = yasmin_ros.logger_node

        self.add_state(
            "ACKNOWLEDGE_BAG",
            Say(text="I have a bag for the host. Can the host stand in front of me? I will wait for you."),
            transitions={
                "succeeded": "WAIT_FOR_HOST",
                "aborted": "WAIT_FOR_HOST",
                "canceled": "WAIT_FOR_HOST",
            },
        )

        # This should define a baselink area (transformed to map) in front of the robot.
        self.add_state(
            "WAIT_FOR_HOST",
            #TODO: Update to take ros param or class param instead of hardcoded 
            # Baselink area is roughly 3m ahead of robot and 1.5m wide? Change as needed
            WaitForPersonInArea(), 
            transitions={
                "succeeded": "SAY_FOLLOW",  # Host is infront of the robot
                "failed": "WAIT_FOR_HOST",  # Still waiting on host
            },
            remappings={"detections_3d": "person_detections"},
        )
        self.add_state(
            "SAY_FOLLOW",
            Say(text="I will now follow you. "),
            transitions={
                "succeeded": "GET_PERSON_POINT",
                "aborted": "GET_PERSON_POINT",
                "canceled": "GET_PERSON_POINT",
            },
        )

        # Get the person's point in the area
        self.add_state(
            "GET_PERSON_POINT",
            GetPersonPoint(),
            transitions={
                "succeeded": "WAIT_FOR_MOVE",
                "failed": "WAIT_FOR_HOST",
            },
        )
        self.add_state(
            "WAIT_FOR_MOVE",
            Wait(wait_time=1.0),
            transitions={"succeeded": "RESET_HEAD_2", "failed": "RESET_HEAD_2"},
        )

        # Check if person is still in the area/ close to the point 
        self.add_state(
            "CHECK_IF_PERSON_MOVED",
            Detect3DInArea(
                area_polygon="",    #Update to use updated TF map of area infront of robot
                filter=["person"],
                z_min=-10,
                z_max=50.0,
                confidence=0.7,
            ),
            transitions={"succeeded": "GET_PERSON_POINT", "failed": "failed"},
            remappings={"detections_3d": "sofa_detections"},
        )
        # get new location of person
        self.add_state(
            "GET_PERSON_POINT",
            GetPersonPoint(),
            transitions={
                "succeeded": "CHECK_HOST_MOVED",
                "failed": "SAY_WAITING_FOR_GUEST",
            },
        )

        # STATE CHECK_HOST_MOVED: CALLBACK to check if person has moved away and we can move to thier old posisiton. 
        # Outcomes: {
        # "moved_away" (out of polygon of further away): "MOVE_TO_POINT", 
        # "to_close": WAIT (then CHECK_IF_PERSON_MOVED)},
        # "same_spot" (roughly same position): ASK_ARRIVED
        
        # COUNT NUMBER OF WAITS if > 3 then ASK_ARRIVED

        # STATE: MOVE_TO_POINT (updateable action state? on /navigate_to_pose or keep gotolocation?)

        self.add_state(
            "LOCATE_HOST",
            WaitForPersonInArea(), 
            transitions={
                "succeeded": "SAY_FOLLOW",  # Host is infront of the robot
                "failed": "WAIT_FOR_HOST",  # Still waiting on host
            },
            remappings={"detections_3d": "person_detections"},
        )

        self.add_state(
            "GREET_AND_ASK_GUEST",
            AskAndListen(
                tts_phrase="Say YES if we have arrived. NO if we have not.",
            ),
            transitions={
                "succeeded": "GET_NAME_DRINK_FACE",
                "failed": "GREET_AND_ASK_GUEST",
            },
            remappings={"transcribed_speech": "guest_transcription"},
        )

        # STATE: CHECK_RESPONSE: check if they have said yes or no.
        # Outcomes: {
        # "yes" : PLACE_BAG_SM, 
        # "no": LOCATE_HOST,
        