#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from common_math.math_ import euclidian_distance



def get_dist_to_door(is_robot, x=None, y=None):
    if is_robot:
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        print(f"robot pose: {robot_pose}")
        r = (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
    else:
        r = (x, y)

    door_position = get_pose_from_param("/door/pose")
    print(f"door pose: {door_position}")
    d = (door_position.position.x, door_position.position.y)

    dist = euclidian_distance(r, d)
    print(f"distance to door: {dist}")
    return dist

# def get_how_close_to_door(is_robot, min_dist=0.5):
#         dist = self.get_dist_to_door(is_robot)
#         return round(dist, 1) < min_dist
