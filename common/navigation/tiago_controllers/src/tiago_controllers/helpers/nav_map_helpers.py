#!/usr/bin/env python3
import rospy
from std_srvs.srv import  Empty

def clear_costmap():
    """
        Clears costmap using clear_octomap server
    """

    rospy.loginfo('waiting for clear_costmap')
    rospy.wait_for_service('/move_base/clear_costmaps', timeout=10)
    try:
        clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        response = clear_costmap()
        rospy.loginfo('clearing costmap done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
