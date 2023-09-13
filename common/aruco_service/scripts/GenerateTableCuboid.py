#!/usr/bin/env python

import yaml
import rospy
import datetime
from aruco_service.srv import TableNumber, TableNumberResponse
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
import rosparam
import tf2_ros
import tf2_geometry_msgs
import rospkg

# Units are in meters
# The marker should be placed on the bottom left corner of the table, with the x axis pointing to the right and the y axis pointing up
# Use 0 to inf for tables, use -1 for counter, use -2 for waiting area
TABLE_LONG_SIDE = 1.2
TABLE_SHORT_SIDE = 0.6
PADDING = 0.5

WAITING_AREA_LONG_SIDE = 1.2
WAITING_AREA_SHORT_SIDE = 0.6

r = rospkg.RosPack()
FILENAME = r.get_path('aruco_service') + "/test_check_table_sim.yaml"

def get_transform_to_marker(from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        t = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(5))
        return t
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def get_map_frame_pose(input_xy, transform):
    ps = PointStamped()
    ps.point.x = input_xy[0]
    ps.point.y = input_xy[1]
    ps.header.frame_id = "aruco_marker_frame"
    ps.header.stamp = rospy.Time.now()

    tr_point = tf2_geometry_msgs.do_transform_point(ps, transform)
    return (tr_point.point.x, tr_point.point.y)

def create_marker_msg(point, idx):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = point[0]
    marker_msg.pose.position.y = point[1]
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    return marker_msg

def generate_cuboid(number):
    table = number.table
    seconds = rospy.get_time()
    objects_marker_pub = rospy.Publisher("/table/objects_cuboid", Marker, queue_size=4)
    persons_marker_pub = rospy.Publisher("/table/persons_cuboid", Marker, queue_size=4)

    if latest_pose is not None and (seconds - latest_pose.header.stamp.secs < 1.0):

        tr = get_transform_to_marker("aruco_marker_frame", "map")

        # THESE POINTS ARE IN MARKER COORDINATE FRAME [LOCATION OF ARUCO MARKER IS (0,0) IN THIS FRAME]
        local_corner_1 = [0, 0] 
        local_corner_2 = [TABLE_LONG_SIDE, 0] if table >= 0 else [WAITING_AREA_LONG_SIDE, 0]
        local_corner_3 = [TABLE_LONG_SIDE, TABLE_SHORT_SIDE] if table >= 0 else [WAITING_AREA_LONG_SIDE, WAITING_AREA_SHORT_SIDE]
        local_corner_4 = [0, TABLE_SHORT_SIDE] if table >= 0 else [0, WAITING_AREA_SHORT_SIDE]


        # TRANSFORM TO MAP FRAME
        corner_1 = get_map_frame_pose(local_corner_1, tr)
        corner_2 = get_map_frame_pose(local_corner_2, tr)
        corner_3 = get_map_frame_pose(local_corner_3, tr)
        corner_4 = get_map_frame_pose(local_corner_4, tr)

        objects_marker_pub.publish(create_marker_msg(corner_1, 0))
        objects_marker_pub.publish(create_marker_msg(corner_2, 1))
        objects_marker_pub.publish(create_marker_msg(corner_3, 2))
        objects_marker_pub.publish(create_marker_msg(corner_4, 3))

        if table >= 0: 

            local_padded_corner_1 = [local_corner_1[0] - PADDING, local_corner_1[1] - PADDING]
            local_padded_corner_2 = [local_corner_2[0] + PADDING, local_corner_2[1] - PADDING]
            local_padded_corner_3 = [local_corner_3[0] + PADDING, local_corner_3[1] + PADDING]
            local_padded_corner_4 = [local_corner_4[0] - PADDING, local_corner_4[1] + PADDING]

            padded_corner_1 = get_map_frame_pose(local_padded_corner_1, tr)
            padded_corner_2 = get_map_frame_pose(local_padded_corner_2, tr)
            padded_corner_3 = get_map_frame_pose(local_padded_corner_3, tr)
            padded_corner_4 = get_map_frame_pose(local_padded_corner_4, tr)

            persons_marker_pub.publish(create_marker_msg(padded_corner_1, 0))
            persons_marker_pub.publish(create_marker_msg(padded_corner_2, 1))
            persons_marker_pub.publish(create_marker_msg(padded_corner_3, 2))
            persons_marker_pub.publish(create_marker_msg(padded_corner_4, 3))

            rospy.set_param("/tables/table" + str(table) + "/objects_cuboid", [corner_1, corner_2, corner_3, corner_4])
            rospy.set_param("/tables/table" + str(table) + "/persons_cuboid", [padded_corner_1, padded_corner_2, padded_corner_3, padded_corner_4])
            now = str(datetime.datetime.now())
            rospy.set_param("/tables/table" + str(table) + "/last_updated", now)
            rospy.loginfo("Cuboid for table %d saved to parameter server", table)

        elif table == -1:
            rospy.set_param("/counter/cuboid", [corner_1, corner_2, corner_3, corner_4])
            now = str(datetime.datetime.now())
            rospy.set_param("/counter/last_updated", now)
            rospy.loginfo("Cuboid for the counter saved to parameter server")

        elif table == -2:
            rospy.set_param("/wait/cuboid", [corner_1, corner_2, corner_3, corner_4])
            now = str(datetime.datetime.now())
            rospy.set_param("/wait/last_updated", now)
            rospy.loginfo("Cuboid for the waiting area saved to parameter server")

        else:
            rospy.logerr("Invalid table number %d", table)
            return TableNumberResponse(False)

        # Dump rosparams to file
        data = {
            'tables': rosparam.get_param('/tables'),
            'counter': rosparam.get_param('/counter'),
            'wait': rosparam.get_param('/wait')
        }

        with open(FILENAME, 'w') as file:
            yaml.dump(data, file)

        return TableNumberResponse(True)
    else:
        rospy.logerr("No pose data available to generate cuboid for table %d, please check if the marker is on the table", table)
        return TableNumberResponse(False)

def get_latest_pose(msg):
    global latest_pose
    latest_pose = msg

if __name__ == "__main__":

    rospy.init_node("generate_table_cuboid")
    sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, get_latest_pose)
    s = rospy.Service("generate_table_cuboid", TableNumber, generate_cuboid)

    els = rosparam.load_file(FILENAME)
    for param, ns in els:
        rosparam.upload_params(ns, param)
    
    rospy.loginfo("Cuboid Generator Service Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
