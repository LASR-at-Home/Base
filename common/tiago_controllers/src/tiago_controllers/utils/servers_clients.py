#!/usr/bin/env python3
import rospy
import rosservice
from std_srvs.srv import SetBool, SetBoolRequest, Empty


def activate_robot_navigation(flag=True):
    if '/robot_start_navigation' not in rosservice.get_service_list():
        # Assume simulation
        return
    rospy.wait_for_service('/robot_start_navigation', timeout=10)
    try:
        nav = rospy.ServiceProxy('/robot_start_navigation', SetBool)
        nav_resp = nav(SetBoolRequest(flag))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)