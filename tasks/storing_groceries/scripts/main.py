#!/usr/bin/env python3

import rospy
from storing_groceries.state_machine import StoringGroceries
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.validation import explain_validity

from std_msgs.msg import Header

def polygon_from_coords(coords):
    return Polygon(points=[Point(x=x, y=y, z=0.0) for x, y in coords])

def get_pose_param(param_name):
    if not rospy.has_param(param_name):
        rospy.logerr(f"Missing parameter: {param_name}")
        rospy.signal_shutdown(f"Missing required parameter: {param_name}")
        return None
    try:
        param = rospy.get_param(param_name)
        pose = Pose(
            position=Point(**param["position"]),
            orientation=Quaternion(**param["orientation"])
        )
        rospy.loginfo(f"Loaded pose for {param_name}: {param}")
        return pose
    except KeyError as e:
        rospy.logerr(f"Malformed pose parameter: {param_name}. Missing key: {e}")
        rospy.signal_shutdown(f"Bad parameter: {param_name}")
        return None

if __name__ == "__main__":
    rospy.init_node("storing_groceries_robocup")

    # Publishers
    wait_area_pub = rospy.Publisher("/storing_groceries/wait_area", PolygonStamped, queue_size=1, latch=True)
    table_area_pub = rospy.Publisher("/storing_groceries/table_area", PolygonStamped, queue_size=1, latch=True)
    cabinet_area_pub = rospy.Publisher("/storing_groceries/cabinet_area", PolygonStamped, queue_size=1, latch=True)

    # Load poses safely
    wait_pose = get_pose_param("/storing_groceries/wait_pose")
    table_pose = get_pose_param("/storing_groceries/table_pose")
    cabinet_pose = get_pose_param("/storing_groceries/cabinet_pose")

    # Load areas
    try:
        wait_area_param = rospy.get_param("/storing_groceries/wait_area")
        table_area_param = rospy.get_param("/storing_groceries/table_area")
        cabinet_area_param = rospy.get_param("/storing_groceries/cabinet_area")
    except KeyError as e:
        rospy.logerr(f"Missing area parameter: {e}")
        rospy.signal_shutdown("Area parameter missing")
        exit(1)

    wait_area = ShapelyPolygon(wait_area_param)
    table_area = ShapelyPolygon(table_area_param)
    cabinet_area = ShapelyPolygon(cabinet_area_param)

    for name, poly in [("wait_area", wait_area), ("table_area", table_area), ("cabinet_area", cabinet_area)]:
        if not poly.is_valid:
            rospy.logerr(f"{name} polygon is invalid: {explain_validity(poly)}")
            rospy.signal_shutdown(f"Invalid polygon: {name}")
            exit(1)
        else:
            rospy.loginfo(f"{name} polygon is valid.")

    # Publish areas
    wait_area_pub.publish(PolygonStamped(header=Header(frame_id="map"), polygon=polygon_from_coords(wait_area.exterior.coords)))
    table_area_pub.publish(PolygonStamped(header=Header(frame_id="map"), polygon=polygon_from_coords(table_area.exterior.coords)))
    cabinet_area_pub.publish(PolygonStamped(header=Header(frame_id="map"), polygon=polygon_from_coords(cabinet_area.exterior.coords)))

    # Load shelf point
    try:
        shelf_point_param = rospy.get_param("/storing_groceries/shelf_point")
        shelf_point = Point(**shelf_point_param)
    except KeyError:
        rospy.logerr("Missing or malformed /storing_groceries/shelf_point")
        rospy.signal_shutdown("Bad shelf_point param")
        exit(1)

    sweep = rospy.get_param("/storing_groceries/sweep", False)

    # Launch state machine
    storing_groceries = StoringGroceries(
        wait_pose=wait_pose,
        wait_area=wait_area,
        table_pose=table_pose,
        table_area=table_area,
        cabinet_pose=cabinet_pose,
        cabinet_area=cabinet_area,
        shelf_area=None,  # Replace with actual param if available
        shelf_point=shelf_point,
    )

    outcome = storing_groceries.execute()
    rospy.loginfo(f"Storing Groceries finished with outcome: {outcome}")

    rospy.spin()
