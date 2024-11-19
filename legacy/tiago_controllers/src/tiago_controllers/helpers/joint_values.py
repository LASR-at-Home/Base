#!/usr/bin/env python3
import rospy
from control_msgs.srv import QueryTrajectoryState


def get_joint_values(srv_name):
    try:
        rospy.wait_for_service(srv_name)
        joint_srv = rospy.ServiceProxy(srv_name, QueryTrajectoryState)
        resp = joint_srv(rospy.Time.now())
        return resp.position
    except rospy.ServiceException:
        rospy.loginfo("service call failed")
        return None
