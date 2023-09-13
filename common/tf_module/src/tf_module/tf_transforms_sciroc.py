#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs


def get_transform(from_frame, to_frame):
    try:
        t = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.5))
        return t
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def apply_transform(ps_, transform, is_input_ps=True, target="xtion_rgb_optical_frame"):
    if is_input_ps:
        tr_point = tf2_geometry_msgs.do_transform_point(ps_, transform)
    else:
        ps = PointStamped()
        ps.point.x = ps_[0]
        ps.point.y = ps_[1]
        ps.point.z = ps_[2]
        ps.header.frame_id = target
        ps.header.stamp = rospy.Time.now()
        tr_point = tf2_geometry_msgs.do_transform_point(ps, transform)

    return tr_point


def transform_point(point_source, from_frame, to_frame="map"):
    try:
        point_map = tf_buffer.transform(point_source, to_frame)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform point, warning: %s", str(e))
        tf_buffer.can_transform("map", from_frame, rospy.Time(0), rospy.Duration(4.0))
        point_map = tf_buffer.transform(point_source, to_frame)

    return point_map

def transform_point_2(point_source, from_frame="base_laser_link", to_frame="map"):
    points = None
    try:
        tr = get_transform(from_frame, to_frame)
        points = apply_transform(point_source, tr)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform point.")
        rospy.logwarn(e)

    return points

def transform_point_from_given(x_, y_, from_frame="base_laser_link", to_frame="map"):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "map"
    point.point.x = x_
    point.point.y = y_
    point.point.z = 0.0

    tr = get_transform(from_frame, to_frame)
    x, y, z = apply_transform(point, tr)
    return x, y, z


if __name__ == "__main__":
    rospy.init_node('tf_transforms_sciroc_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo('Ready to transform!')

    rospy.spin()
