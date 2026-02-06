#!/usr/bin/env python3
import rospy
import rosparam
import sys
import os
import rospkg
from aruco_service.srv import SaveNavigationPoint, GenerateTableCuboid

if len(sys.argv) < 1:
    print("Usage: rosrun coffee_shop prep.py <config_name>")

rospy.init_node("coffee_shop_prep")

save_navigation_point = rospy.ServiceProxy(
    "/save_navigation_points", SaveNavigationPoint
)
save_table_cuboid = rospy.ServiceProxy("/generate_table_cuboid", GenerateTableCuboid)

PKG_ROOT = rospkg.RosPack().get_path("coffee_shop")
CONFIG_PATH = os.path.join(PKG_ROOT, "config")
OUTPUT_PATH = os.path.join(CONFIG_PATH, f"{sys.argv[1]}.yaml")

if not os.path.exists(OUTPUT_PATH):
    open(OUTPUT_PATH, "w").close()

rosparam.set_param("/config_path", OUTPUT_PATH)

n_tables = int(input("Enter the number of tables: "))
rospy.loginfo(f"There are {n_tables} tables.")

for i in range(n_tables):
    while True:
        long_side_length = short_side_length = radius = 0.0
        input(
            f"Place an Aruco marker in the corner of table {i}, adjust me so that I can see it, then press Enter."
        )
        is_rect = bool(int(input("Is rect (0/1): ")))
        if is_rect:
            long_side_length = float(input(f"Long side length: "))
            short_side_length = float(input(f"Short side length: "))
        else:
            radius = float(input("Radius: "))
        padding_sz = float((input("Padding: ")))
        try:
            save_table_cuboid(
                i, long_side_length, short_side_length, padding_sz, is_rect, radius
            )
            break
        except rospy.service.ServiceException:
            rospy.logwarn("Something went wrong - is the Aruco marker in view?")

    input(f"Take me to the search position for table {i}, then press Enter.")
    save_navigation_point(i)

while True:
    padding_sz = 0.0
    input(
        f"Place an Aruco marker in the corner of counter, adjust me so that I can see it, then press Enter."
    )
    is_rect = bool(int(input("Is rect (0/1): ")))
    if is_rect:
        long_side_length = float(input(f"Long side length: "))
        short_side_length = float(input(f"Short side length: "))
    else:
        radius = float(input("Radius: "))
    try:
        save_table_cuboid(
            -1, long_side_length, short_side_length, padding_sz, is_rect, radius
        )
        break
    except rospy.service.ServiceException:
        rospy.logwarn("Something went wrong - is the Aruco marker in view?")

input("Take me to the search position for the counter, then press Enter.")
save_navigation_point(-1)

while True:
    input(
        f"Place an Aruco marker to indicate the waiting area, adjust me so that I can see it, then press Enter."
    )
    try:
        long_side_length = float(input(f"Long side length: "))
        short_side_length = float(input(f"Short side length: "))
        save_table_cuboid(-2, long_side_length, short_side_length, 0.0, 1, 0.0)
        break
    except rospy.service.ServiceException:
        rospy.logwarn("Something went wrong - is the Aruco marker in view?")

input("Take me to the search position for the waiting area, then press Enter.")
save_navigation_point(-2)

rospy.loginfo("ALL OK.")
els = rosparam.load_file(OUTPUT_PATH)
for param, ns in els:
    rosparam.upload_params(ns, param)
