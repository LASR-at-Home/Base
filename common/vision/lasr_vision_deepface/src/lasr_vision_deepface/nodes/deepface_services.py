import rclpy
from rclpy.node import Node
import re
from lasr_vision_deepface.nodes import deepface as face_recognition
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import Recognise, LearnFace, DetectFaces


class DeepFaceServiceNode(Node):
    def __init__(self):
        super().__init__("DeepFace_service_node")
        
 
        # Create service servers
        self.recognise_service = self.create_service(Recognise, "/deepface/recognise", self.recognise)
        self.detect_faces_service = self.create_service(DetectFaces, "/deepface/detect_faces", self.detect_faces)
        self.learn_face_service = self.create_service(LearnFace, "/deepface/learn_face", self.learn_face)

        # Publishers
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 1)
        self.cropped_detect_pub = self.create_publisher(Image, 'cropped_detect_topic', 1)
        self.debug_inference_pub = self.create_publisher(Image, 'debug_inference', 1)


        self.recognise_debug_publishers = {}
        self.learn_face_debug_publishers = {}

        self.get_logger().info("deepface service node started")
 
    def recognise(self, request: Recognise.Request, response: Recognise.Response):
        if request.dataset in recognise_debug_publishers:
            debug_publisher, similar_face_debug_publisher, cropped_face_publisher = (
                recognise_debug_publishers[request.dataset]
            )
        else:
            topic_name = re.sub(r"[\W_]+", "", request.dataset)
            debug_publisher = self.create_publisher(Image, f"/recognise/debug/{topic_name}", 1)
            similar_face_debug_publisher = self.create_publisher(Image, f"/recognise/debug/{topic_name}/similar_face", 1)
            cropped_face_publisher = self.create_publisher(Image, "/recognise/debug/cropped_query_face", 1)
            
            
            recognise_debug_publishers[request.dataset] = (
                debug_publisher,
                similar_face_debug_publisher,
                cropped_face_publisher,
            )
        return face_recognition.recognise(
            request, debug_publisher, similar_face_debug_publisher, cropped_face_publisher
        )


    def learn_face(self, request: LearnFace.Request, response: LearnFace.Response):
        if request.dataset in learn_face_debug_publishers:
            debug_publisher = learn_face_debug_publishers[request.dataset]
        else:
            topic_name = re.sub(r"[\W_]+", "", request.dataset)
            debug_publisher = self.create_publisher(Image,  f"/learn_face/debug/{topic_name}", 1)
        face_recognition.create_dataset(
            request.dataset,
            request.name,
            request.images,
            debug_publisher,
        )
        return LearnFaceResponse()


    def detect_faces(self, request: DetectFaces.Request, response: DetectFaces.Response):
        return face_recognition.detect_faces(request, detect_faces_debug_publisher)



def main(args=None):
    rclpy.init(args=args)
    node = DeepFaceServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()