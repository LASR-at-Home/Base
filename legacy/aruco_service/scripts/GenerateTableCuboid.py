#!/usr/bin/env python

import yaml
import rospy
import datetime
from aruco_service.srv import GenerateTableCuboid, GenerateTableCuboidResponse
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
import rosparam
import tf2_ros
import tf2_geometry_msgs
from math import sqrt

# Units are in meters
# The marker should be placed on the bottom left corner of the table, with the x axis pointing to the right and the y axis pointing up
# Use 0 to inf for tables, use -1 for counter, use -2 for waiting area

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

def generate_cuboid(msg):
    table = msg.table
    long_side = msg.long_side
    short_side = msg.short_side
    padding = msg.padding
    is_rect = msg.is_rect
    radius = msg.radius
    seconds = rospy.get_time()
    objects_marker_pub = rospy.Publisher("/table/objects_cuboid", Marker, queue_size= 4 if is_rect else 8)
    persons_marker_pub = rospy.Publisher("/table/persons_cuboid", Marker, queue_size=4 if is_rect else 8)

    if latest_pose is not None and (seconds - latest_pose.header.stamp.secs < 1.0):

        tr = get_transform_to_marker("aruco_marker_frame", "map")

        if is_rect:

            # THESE POINTS ARE IN MARKER COORDINATE FRAME [LOCATION OF ARUCO MARKER IS (0,0) IN THIS FRAME]
            local_corner_1 = [0, 0] 
            local_corner_2 = [long_side, 0]
            local_corner_3 = [long_side, short_side]
            local_corner_4 = [0, short_side]


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

                local_padded_corner_1 = [local_corner_1[0] - padding, local_corner_1[1] - padding]
                local_padded_corner_2 = [local_corner_2[0] + padding, local_corner_2[1] - padding]
                local_padded_corner_3 = [local_corner_3[0] + padding, local_corner_3[1] + padding]
                local_padded_corner_4 = [local_corner_4[0] - padding, local_corner_4[1] + padding]

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
                return GenerateTableCuboidResponse(False)

            # Dump rosparams to file
            data = {
                'tables': rosparam.get_param('/tables'),
                'counter': rosparam.get_param('/counter'),
                'wait': rosparam.get_param('/wait')
            }

            with open(rosparam.get_param('/config_path'), 'w') as file:
                yaml.dump(data, file)
        else:
            # assume circular, and aruco marker is placed in the centre of the table
            local_r1 = [-radius, 0]
            local_r2 = [(sqrt(2.)*-radius)/2., (sqrt(2.)*+radius)/2.]
            local_r3 = [0, +radius]
            local_r4 = [(sqrt(2.)*+radius)/2., (sqrt(2.)*+radius)/2.]
            local_r5 = [+radius, 0]
            local_r6 = [(sqrt(2.)*+radius)/2., (sqrt(2.)*-radius)/2.]
            local_r7 = [0, -radius]
            local_r8 = [(sqrt(2.)*-radius)/2., (sqrt(2.)*-radius)/2.]

            r1 = get_map_frame_pose(local_r1, tr)
            r2 = get_map_frame_pose(local_r2, tr)
            r3 = get_map_frame_pose(local_r3, tr)
            r4 = get_map_frame_pose(local_r4, tr)
            r5 = get_map_frame_pose(local_r5, tr)
            r6 = get_map_frame_pose(local_r6, tr)
            r7 = get_map_frame_pose(local_r7, tr)
            r8 = get_map_frame_pose(local_r8, tr)

            objects_marker_pub.publish(create_marker_msg(r1, 0))
            objects_marker_pub.publish(create_marker_msg(r2, 1))
            objects_marker_pub.publish(create_marker_msg(r3, 2))
            objects_marker_pub.publish(create_marker_msg(r4, 3))
            objects_marker_pub.publish(create_marker_msg(r5, 4))
            objects_marker_pub.publish(create_marker_msg(r6, 5))
            objects_marker_pub.publish(create_marker_msg(r7, 6))
            objects_marker_pub.publish(create_marker_msg(r8, 7))

            local_padded_r1 = [local_r1[0] - padding, 0]
            local_padded_r2 = [local_r2[0] - padding, local_r2[1] + padding]
            local_padded_r3 = [0, local_r3[1] + padding]
            local_padded_r4 = [local_r4[0] + padding, local_r4[1] + padding]
            local_padded_r5 = [local_r5[0] + padding, 0]
            local_padded_r6 = [local_r6[0] + padding, local_r6[1] - padding]
            local_padded_r7 = [0, local_r7[1] - padding]
            local_padded_r8 = [local_r8[0] - padding, local_r8[1] - padding]

            r1p = get_map_frame_pose(local_padded_r1, tr)
            r2p = get_map_frame_pose(local_padded_r2, tr)
            r3p = get_map_frame_pose(local_padded_r3, tr)
            r4p = get_map_frame_pose(local_padded_r4, tr)
            r5p = get_map_frame_pose(local_padded_r5, tr)
            r6p = get_map_frame_pose(local_padded_r6, tr)
            r7p = get_map_frame_pose(local_padded_r7, tr)
            r8p = get_map_frame_pose(local_padded_r8, tr)

            persons_marker_pub.publish(create_marker_msg(r1p, 0))
            persons_marker_pub.publish(create_marker_msg(r2p, 1))
            persons_marker_pub.publish(create_marker_msg(r3p, 2))
            persons_marker_pub.publish(create_marker_msg(r4p, 3))
            persons_marker_pub.publish(create_marker_msg(r5p, 4))
            persons_marker_pub.publish(create_marker_msg(r6p, 5))
            persons_marker_pub.publish(create_marker_msg(r7p, 6))
            persons_marker_pub.publish(create_marker_msg(r8p, 7))


            rospy.set_param("/tables/table" + str(table) + "/objects_cuboid", [r1, r2, r3, r4, r5, r6, r7, r8])
            rospy.set_param("/tables/table" + str(table) + "/persons_cuboid", [r1p, r2p, r3p, r4p, r5p, r6p, r7p, r8p])
            now = str(datetime.datetime.now())
            rospy.set_param("/tables/table" + str(table) + "/last_updated", now)
            rospy.loginfo("Cuboid for table %d saved to parameter server", table)

        return GenerateTableCuboidResponse(True)
    else:
        rospy.logerr("No pose data available to generate cuboid for table %d, please check if the marker is on the table", table)
        return GenerateTableCuboidResponse(False)

def get_latest_pose(msg):
    global latest_pose
    latest_pose = msg

if __name__ == "__main__":

    rospy.init_node("generate_table_cuboid")
    sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, get_latest_pose)
    s = rospy.Service("generate_table_cuboid", GenerateTableCuboid, generate_cuboid)

    if rospy.has_param("/config_path"):
        els = rosparam.load_file(rosparam.get_param('/config_path'))
        for param, ns in els:
            rosparam.upload_params(ns, param)
    
    rospy.loginfo("Cuboid Generator Service Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)