#!/usr/bin/env python3.10
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os

def image_callback(msg):
    # convert ROS Image to OpenCV BGR
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # run inference (returns a list, we take the first result)
    result = model(cv_img)[0]
    # draw boxes & labels onto the image
    annotated = result.plot()  # returns a NumPy array (BGR)
    # convert back to ROS Image and publish
    out_msg = bridge.cv2_to_imgmsg(annotated, "bgr8")
    out_msg.header = msg.header
    pub_annotated.publish(out_msg)

if __name__ == "__main__":
    rospy.init_node("yolov11_node")
    # load model
    root = os.path.dirname(__file__)
    pkg  = os.path.dirname(root)
    model_path = os.path.join(pkg, "models", "best.pt")
    rospy.loginfo(f"Loading model from {model_path}")
    model = YOLO(model_path)

    # bridge & publisher
    bridge = CvBridge()
    pub_annotated = rospy.Publisher("yolov11/detections", Image, queue_size=1)
    pub_annotated.publish()

    # subscribe to your camera topic (e.g. /camera/image_raw)
    sub = rospy.Subscriber("xtion/rgb/image_raw", Image, image_callback, queue_size=1, buff_size=2**24)

    rospy.spin()
