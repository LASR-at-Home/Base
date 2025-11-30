import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
import smach

from lasr_vision_interfaces.srv import DetectFaces as DetectFacesSrv
from sensor_msgs.msg import Image
from cv2_pcl import pcl_to_img_msg


class DetectFaces(smach.State):
    def __init__(
        self,
        node: Node,
        image_topic: str = "/xtion/rgb/image_raw",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["pcl_msg"],
            output_keys=["detections"],
        )
        self.node = node
        self._image_topic = image_topic
        self._detect_faces = self.node.create_client(DetectFacesSrv, "/detect_faces")

        while not self._detect_faces.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /detect_faces service...")

    def execute(self, userdata):
        img_msg = pcl_to_img_msg(userdata.pcl_msg)
        if img_msg is None:
            self.node.get_logger().info(
                f"No image from point cloud, waiting on topic: {self._image_topic}"
            )
            try:
                img_msg = wait_for_message(
                    node=self.node,
                    topic=self._image_topic,
                    msg_type=Image,
                    timeout_sec=5.0,
                )

            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to get image from topic: {str(e)}"
                )
                return "failed"

        request = DetectFacesSrv.Request()
        request.image_raw = img_msg

        future = self._detect_faces.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result():
            userdata.detections = future.result().detections
            return "succeeded"
        else:
            self.node.get_logger().error("Detect faces service call failed.")
            return "failed"
