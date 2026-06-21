from typing import List, Dict, Optional


import rclpy
import yasmin
import yasmin_ros
import numpy as np
import cv2
import time
from yasmin import Blackboard
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters

from lasr_vision_interfaces.msg import Detection3D
from lasr_vision_interfaces.srv import Recognise3D, YoloDetection3D


class Recognise(yasmin_ros.ServiceState):
    def __init__(self):
        super().__init__(
            srv_type=Recognise3D,
            srv_name="/lasr_vision_reid/recognise/threed",
            create_request_handler=self._create_request,
            response_handler=self._handle_resp,
            outcomes=["no_detections"],
        )

        self.add_output_key("guest_data")

        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.data = None

        depth_info = message_filters.Subscriber(
            self._node,
            CameraInfo,
            "head_front_camera/depth/camera_info",
            qos_profile=camera_qos,
        )

        self.cache = message_filters.Cache(depth_info)

        image_sub = message_filters.Subscriber(
            self._node, Image, "head_front_camera/rgb/image_raw", qos_profile=camera_qos
        )

        depth_sub = message_filters.Subscriber(
            self._node,
            Image,
            "head_front_camera/depth/image_raw",
            qos_profile=camera_qos,
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.1
        )

        self.ts.registerCallback(self.callback)

    def callback(self, image_msg, depth_msg):
        if self.data is None:
            self.data = (image_msg, depth_msg)

    def _create_request(self, blackboard):
        self.data = None

        request = Recognise3D.Request()

        while self.data is None:
            yasmin.YASMIN_LOG_INFO("Waiting for synced rgb and depth frames")
            time.sleep(1)

        image, depth = self.data

        request.image_raw = image
        request.depth_image = depth
        request.depth_camera_info = self.cache.getLast()
        request.threshold = 0.5
        request.target_frame = "map"

        return request

    def _handle_resp(self, blackboard, response):
        if len(response.detections) == 0:
            return "no_detections"
        else:
            for detection in response.detections:
                if detection.name == "unknown":
                    continue
                yasmin.YASMIN_LOG_INFO(detection.name)
                yasmin.YASMIN_LOG_INFO(str(detection.point))
                blackboard["guest_data"][detection.name]["seated_point"] = detection.point
                blackboard['seat_indexes'][detection.name] = blackboard['person_index']
                return "succeeded"

        return "aborted"


def check(blackboard):
    dict = blackboard["guest_data"]
    yasmin.YASMIN_LOG_INFO(str(dict))
    return "succeeded"


def main():
    global check
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    check = yasmin.CbState(outcomes=["succeeded"], callback=check)

    sm.add_state(
        "RECOGNISE",
        Recognise(),
        transitions={
            "succeeded": "CHECK",
            "aborted": "failed",
            "no_detections": "failed",
        },
    )

    sm.add_state("CHECK", check, transitions={"succeeded": "succeeded"})

    bb = Blackboard()
    bb["guest_data"] = {
        "guest1": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
            "seated_point": None,
        }
    }

    outcome = sm(bb)

    rclpy.shutdown()
