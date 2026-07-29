import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


import yasmin
import yasmin_ros
from yasmin_ros import ServiceState

from lasr_vision_interfaces.srv import DetectFaces as DetectFacesSrv
from sensor_msgs.msg import Image
from cv2_pcl import pcl_to_img_msg

import time


# Redundent state and Service due to REiD
class DetectFaces(ServiceState):
    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
    ):
        super().__init__(
            srv_type=DetectFacesSrv,
            srv_name="/deepface/detect_faces",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "no_faces", "failed"],
            response_handler=self._response_handler,
        )

        self.add_input_key("pcl")
        self.add_output_key("detections")

        self.node = yasmin_ros.logger_node
        self._image_topic = image_topic

        self.image = None
        self.image_sub = self.node.create_subscriber(
            Image,
            self._image_topic,
            self.getImage,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

    def getImage(self, msg: Image):
        self.image = msg

    def _create_req(self, blackboard):
        img_msg = pcl_to_img_msg(blackboard["pcl_msg"])
        if img_msg is None:
            self.node.get_logger().info(
                f"No image from point cloud, waiting on topic: {self._image_topic}"
            )
            try:
                while rclpy.ok() and self.image == None:
                    time.sleep(1)

            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to get image from topic: {str(e)}"
                )
                return "failed"

        request = DetectFacesSrv.Request()
        request.image_raw = img_msg

        return request

    def _response_handler(self, blackboard, response):
        try:
            detections = response.detections
            if len(detections) > 0:
                blackboard["detections"] = detections
                return "succeeded"
            else:
                return "no_faces"
        except:
            self.node.get_logger().error("Detect faces service call failed.")
            return "failed"
