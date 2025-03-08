#!/usr/bin/env python3
import cv2
import cv2_img


import rclpy
from rclpy.node import Node
from lasr_vision_interfaces.srv import Vqa
import cv_bridge
from lasr_vision_clip.clip_utils import query_image, load_model

def test_query_image(image_path, captions):
    """Function for testing query image.
    """

    img_cv2 = cv2.imread(image_path)
    if img_cv2 is None:
        print("Error: Could not load image!")
        return

    img_msg = cv2_img.cv2_img_to_msg(img_cv2)

    model = load_model(device="cpu")


    best_caption, score, annotated_img_msg = query_image(
        img_msg, model, captions, annotate=True
    )

    # Print the results
    print("Best Caption:", best_caption)
    print("Similarity Score:", score)

    annotated_cv2 = cv2_img.msg_to_cv2_img(annotated_img_msg)
    cv2.imshow("Annotated Image", annotated_cv2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


class VqaClient(Node):
    def __init__(self):
        super().__init__('vqa_client')
        self.cli = self.create_client(Vqa, 'clip_vqa/query_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the Vqa service...')
        self.get_logger().info('Vqa service is available.')

    def send_request(self, image_path: str, possible_answers: list):
        img_cv2 = cv2.imread(image_path)
        if img_cv2 is None:
            self.get_logger().error("Error: Could not load image from " + image_path)
            return None

        # Convert the OpenCV image to a ROS Image message using cv_bridge
        bridge = cv_bridge.CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img_cv2, encoding="bgr8")

        # Create the service request
        req = Vqa.Request()
        req.possible_answers = possible_answers
        req.image_raw = img_msg

        self.get_logger().info("Sending service request with the image and possible answers...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    # Enter the directory and name of the test file - should be in the same directory as the workspace
    image_path = "glasses2.jpeg"
    choices = ["Wearing glasses", "Not wearing glasses"] 

    
    rclpy.init(args=args)
    client = VqaClient()

    response = client.send_request(image_path, choices)
    if response is not None:
        client.get_logger().info("Service Response Received:")
        client.get_logger().info("Answer: " + response.answer)
        client.get_logger().info("Similarity: " + str(response.similarity))
    else:
        client.get_logger().error("No response received from the Vqa service.")

    client.destroy_node()
    rclpy.shutdown()
    

    test_query_image(image_path, choices)