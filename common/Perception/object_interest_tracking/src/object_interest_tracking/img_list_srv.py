#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from object_interest_tracking.srv import ImgLstConverter, ImgLstConverterResponse

def imgConvert(img):
    res = ImgLstConverterResponse()
    a = CvBridge().imgmsg_to_cv2(img.img, desired_encoding='bgr8')
    res.data = a.flatten()
    res.dimensions = a.shape
    return res

rospy.init_node("temp")
rospy.Service('ImgLstConverter', ImgLstConverter, imgConvert)
rospy.spin()