#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty


def clear_octomap():
    rospy.wait_for_service('/clear_octomap')
    try:
        clear = rospy.ServiceProxy('/clear_octomap', Empty)
        clear()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
