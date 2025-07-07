#!/usr/bin/env python3
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__=="__main__":
    rospy.init_node("webcam_pub")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("webcam frame dropped")
            continue
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()
