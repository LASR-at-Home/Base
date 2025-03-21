import rclpy
from rclpy.node import Node
import re
from lasr_vision_deepface.nodes import deepface as face_recognition
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import Recognise, LearnFace, DetectFaces
from cv_bridge import CvBridge
import cv2


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

        # Subscribe to live camera feed
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # Publishers for debugging
        self.debug_publisher = self.create_publisher(Image, "debug_image", 1)
        self.cropped_detect_pub = self.create_publisher(
            Image, "cropped_detect_topic", 1
        )
        self.debug_inference_pub = self.create_publisher(Image, "debug_inference", 1)

        self.recognise_debug_publishers = {}
        self.learn_face_debug_publishers = {}

        self.get_logger().info(
            "DeepFace service node started, listening to /camera/image_raw"
        )

    def image_callback(self, msg):
        """Processes live images from the camera feed."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_image is None or cv_image.size == 0:
                self.get_logger().error(
                    "Received an empty or corrupted image from the camera."
                )
                return

            self.get_logger().info(
                f"Processing image from camera: shape={cv_image.shape}"
            )

            # Convert Image message to a DetectFaces_Request
            detect_faces_request = DetectFaces.Request()
            detect_faces_request.image_raw = msg  # Assign the ROS2 Image message

            # Call detect_faces with the correct request format
            response = face_recognition.detect_faces(
                detect_faces_request, self.debug_publisher, self.get_logger()
            )

            if response.detections:
                self.get_logger().info(
                    f"Detected {len(response.detections)} faces in live camera feed."
                )
            else:
                self.get_logger().info("No faces detected in the live feed.")

        except Exception as e:
            self.get_logger().error(f"Failed to process camera image: {e}")

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
            cv_image, self.debug_publisher, self.get_logger(), self.cropped_detect_pub
        )

    def learn_face(self, request: LearnFace.Request, response: LearnFace.Response):
        """Handles learning new faces"""
        if request.dataset in self.learn_face_debug_publishers:
            debug_publisher = self.learn_face_debug_publishers[request.dataset]
        else:
            topic_name = re.sub(r"[\W_]+", "", request.dataset)
            debug_publisher = self.create_publisher(
                Image, f"/learn_face/debug/{topic_name}", 1
            )

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
