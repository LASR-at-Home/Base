#!/usr/bin/env python3

import sys
import rclpy
from copy import deepcopy
from rclpy.node import Node
from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import Recognise, RecogniseRequest


class FaceRecognitionClient(Node):
    def __init__(self, listen_topic, dataset):
        super().__init__('face_recognition_client')

    # ROS2 Subscription
    self.subscription = self.create_subscription(
        Image, listen_topic, self.detect, 10
    )
    self.client = self.create_client(Recognise, '/recognise')

    if len(sys.argv) < 3:
        print("Usage: rosrun lase_recognition greet <source_topic> <dataset>")
        exit()

    listen_topic = sys.argv[1]
    dataset = sys.argv[2]
    people_in_frame = []


    people_in_frame = {}


    def detect(image):
        self.get_logger().info("Received image message")
        
        global people_in_frame
        
        req = Recognise.Request()
        req.image_raw = image
        req.dataset = self.dataset
        req.confidence = 0.4
        rfuture = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            for detection in response.detections:
                self.people_in_frame[detection.name] = self.get_clock().now()
            self.get_logger().info("Service succeeded!")
    

        else:
            self.get_logger().error("Service call failed!")
    


    def image_callback(image):
        global people_in_frame
        prev_people_in_frame = list(people_in_frame.keys())
        # remove detections from people_in_frame that are older than 5 seconds long
        detect(image)
        for person in list(people_in_frame.keys()):
            if self.get_clock().now() - people_in_frame[person] > rospy.Duration(10):
                del people_in_frame[person]
        if (
            list(people_in_frame.keys()) != prev_people_in_frame
            and len(people_in_frame) > 0
        ) or (len(prev_people_in_frame) == 0 and len(people_in_frame) > 0):
            greet()



    if __name__ == "__main__":
        detect()