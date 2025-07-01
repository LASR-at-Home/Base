#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('usb_cam_bridge')
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)   # /dev/video0

    if not cap.isOpened():
        rospy.logerr("Could not open /dev/video0")
        return

    rate = rospy.Rate(15)  # match your pipeline rate
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        # convert BGR OpenCV image to ROS Image message
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(img_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
