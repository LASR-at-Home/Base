#!/usr/bin/env python3

import rospy
import rosservice
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, PoseStamped, Vector3
from std_msgs.msg import Header
from nav_msgs.srv import GetPlan
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def make_plan(current_pose, x, y, tol=0.01, max_dist=None):

    if '/move_base/NavfnROS/make_plan' in rosservice.get_service_list():
        make_plan_service = '/move_base/NavfnROS/make_plan'
    elif '/move_base/GlobalPlanner/make_plan' in rosservice.get_service_list():
        make_plan_service = '/move_base/GlobalPlanner/make_plan'
    else:
        rospy.loginfo('Failed to find the make_plan server')
        return False, None, None
    
    start = PoseStamped(header=Header(frame_id="map", stamp=rospy.Time.now()), pose=current_pose)
    goal = PoseStamped(header=Header(frame_id="map", stamp=rospy.Time.now()), pose=Pose(position=Point(x, y, 0), orientation=current_pose.orientation))
    rospy.wait_for_service(make_plan_service, timeout=5)
    make_plan_serv = rospy.ServiceProxy(make_plan_service, GetPlan)

    mp = make_plan_serv(start, goal, tol).plan
    if len(mp.poses) > 0:
        if max_dist is not None:
            dist = 0
            # zips the poses like 1 and 2, 2 and 3, etc
            for current, next in zip(mp.poses, mp.poses[1:]):
                dist+= euclidian_distance(
                    (current.pose.position.x, current.pose.position.y),
                    (next.pose.position.x, next.pose.position.y)
                )
            if dist > max_dist:
                return False, None, None
        return True, mp.poses[-1].pose, mp.poses
    return False, None, None


def draw_points(points):
    for x, y in points:
        marker_pub = rospy.Publisher.publish("/visualization_marker", Marker, queue_size=2)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.type = 8
        marker.colors.r = 0.0
        marker.colors.g = 1.0
        marker.colors.b = 0.0
        marker.colors.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1
        marker_pub.publish(marker)


def points_on_radius(x1, y1, radius=0.5):
    """

    Parameters
    ----------
    x1
    y1
    radius

    Returns
    -------
    following points from the given location 0:
    *    0    *
    *    *    *

    """
    # print( "here is it: ",  (x1 + 0.0, y1 - radius), " " ,
    #     (x1 - radius, y1 + 0.0), " ",
    #     (x1 + radius, y1 + 0.0), " ",
        # (x1 - radius, y1 - radius), " ",
        # (x1 + radius, y1 - radius))
    return [
        (x1 + 0.0, y1 - radius),
        (x1 + 0.0, y1 + radius),
        (x1 - radius, y1 + 0.0),
        (x1 + radius, y1 + 0.0),
        (x1 + radius, y1 + radius),
        (x1 - radius, y1 - radius),
        (x1 - radius, y1 + radius),
        (x1 + radius, y1 - radius),
    ]

def euclidian_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    a = np.array((x1, y1))
    b = np.array((x2, y2))
    return np.linalg.norm(a-b)

def plan_to_radius(current_pose, point, radius, tol=0.1):
    picked_points = []
    points = None
    for x,y in points_on_radius(*point, radius=radius):
        # print(f"the points are {x} , and {y}")
        success, point, points = make_plan(current_pose, x, y, tol)
        if success:
            dist = euclidian_distance((current_pose.position.x, current_pose.position.y), (x,y))
            picked_points.append([point, dist])

    # draw_points(picked_points[0:])
    print("-"*30)
    # if points:
        # print(f"points of journey {points}")
        # print(f"middle point of journey {points[round((len(points) - 1) / 2)].pose}")
    picked_points = sorted(picked_points, key = lambda x: x[1])
    return [point[0] for point in picked_points] if picked_points else []

NUMBER = 3

def get_journey_points(current_pose, x, y, tol=0.01):
    success, point, points = make_plan(current_pose, x, y, tol)
    if len(points) > NUMBER + 2:
        mid_points = [points[i] for i in range(points/NUMBER)]
    else:
        mid_points = points
    return mid_points
