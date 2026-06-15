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
    FollowPerson,
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

        self.add_state(
            "FOLLLOW_HOST",
            FollowPerson(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",         # If failed we should try to drop bag anyway? or have person raise hand as secondary recovery
            },
        )
    
        