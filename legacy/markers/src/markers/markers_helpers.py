#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def create_point_marker(x, y, z, idx, g=1.0):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = x
    marker_msg.pose.position.y = y
    marker_msg.pose.position.z = z
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.0
    marker_msg.color.g = g
    marker_msg.color.b = 0.0
    return marker_msg

if __name__ == '__main__':
    rospy.init_node("viz")

    wait_pos = rospy.Publisher("/wait_pos_debug", Marker, queue_size=100)
    wait_points = rospy.get_param("/wait_position_points")
    for i, point in enumerate(wait_points):
        print(i, point, "wait")
        rospy.sleep(1)
        wait_pos.publish(create_point_marker(point[0], point[1], 0, i,g=0.5))


    # wait_points_cen = rospy.get_param("/wait_position_center_point")
    # wait_pos_cen = rospy.Publisher("/wait_position_center_point_debugger", Marker, queue_size=100)
    # for point in wait_points_cen:
    #     print(point)
    #     wait_pos_cen.publish(create_point_marker(point[0], point[1], 0, 0))


    # lift_pos = rospy.Publisher("/lift_pos_debugger", Marker, queue_size=100)
    # lift_pose = rospy.get_param("/lift_position_points")
    # for i, point in enumerate(lift_pose):
    #      rospy.sleep(1)
    #      lift_pos.publish(create_point_marker(point[0], point[1], 0, i))

    # lift_center = rospy.get_param("/lift_position_center_point")
    # print(lift_center)
    # lift_cen = rospy.Publisher("/lift_pos_center_debugger", Marker, queue_size=100)
    # # for i, point in enumerate(lift_center):
    # #     print(i, point,"center lift")
    # lift_cen.publish(create_point_marker(lift_center[0], lift_center[1], 0, 100, g=0.1))

    while not rospy.is_shutdown():
        rospy.spin()
