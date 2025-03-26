import rclpy
from rclpy.node import Node
import re
from lasr_vision_deepface.nodes import deepface as face_recognition
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import Recognise, LearnFace, DetectFaces
from cv_bridge import CvBridge


class DeepFaceServiceNode(Node):
    def __init__(self):
        super().__init__("DeepFace_service_node")
        self.bridge = CvBridge()

        # Create service servers
        self.recognise_service = self.create_service(
            Recognise, "/deepface/recognise", self.recognise
        )
        self.detect_faces_service = self.create_service(
            DetectFaces, "/deepface/detect_faces", self.detect_faces
        )
        self.learn_face_service = self.create_service(
            LearnFace, "/deepface/learn_face", self.learn_face
        )

        # Publishers for debugging
        self.debug_publisher = self.create_publisher(Image, "debug_image", 1)
        self.cropped_detect_pub = self.create_publisher(
            Image, "cropped_detect_topic", 1
        )
        self.debug_inference_pub = self.create_publisher(Image, "debug_inference", 1)

        self.learn_face_debug_publishers = {}

        self.get_logger().info("DeepFace service node started.")

    def recognise(self, request: Recognise.Request, response: Recognise.Response):
        """Handles face recognition requests"""
        if not request.image_raw.data:
            self.get_logger().error("Received empty image! Dropping request.")
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                request.image_raw, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return response

        self.get_logger().info("Running DeepFace recognition...")

        return face_recognition.recognise(
            request, self.debug_inference_pub, self.get_logger(), self.cropped_detect_pub
        )

    def learn_face(self, request: LearnFace.Request, response: LearnFace.Response):
        """Handles learning new faces"""
        dataset_topic = re.sub(r"[\W_]+", "", request.dataset)

        if dataset_topic not in self.learn_face_debug_publishers:
            self.learn_face_debug_publishers[dataset_topic] = self.create_publisher(
                Image, f"/learn_face/debug/{dataset_topic}", 1
            )

        debug_publisher = self.learn_face_debug_publishers[dataset_topic]

        face_recognition.create_dataset(
            request.dataset, request.name, request.images, debug_publisher
        )

        return response

    def detect_faces(
        self, request: DetectFaces.Request, response: DetectFaces.Response
    ):
        """Handles face detection requests"""
        return face_recognition.detect_faces(
            request, self.debug_publisher, self.get_logger()
        )


def main(args=None):
    rclpy.init(args=args)
    node = DeepFaceServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
