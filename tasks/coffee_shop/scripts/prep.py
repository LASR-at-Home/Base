#!/usr/bin/env python3
import rospy
import rosparam
import sys
import os
import rospkg
from aruco_service.srv import TableNumber

if len(sys.argv) < 1:
    print("Usage: rosrun coffee_shop prep.py <config_name>")


rospy.init_node("coffee_shop_prep")

save_navigation_point = rospy.ServiceProxy("/save_navigation_points", TableNumber)
save_load_point = rospy.ServiceProxy("/save_load_points", TableNumber)
save_table_cuboid = rospy.ServiceProxy("/generate_table_cuboid", TableNumber)


PKG_ROOT = rospkg.RosPack().get_path("coffee_shop")
CONFIG_PATH = os.path.join(PKG_ROOT, "config")
OUTPUT_PATH = os.path.join(CONFIG_PATH, f"{sys.argv[1]}.yaml")

if not os.path.exists(OUTPUT_PATH):
    open(OUTPUT_PATH, "w").close()

rosparam.set_param("/config_path", OUTPUT_PATH)
els = rosparam.load_file(OUTPUT_PATH)
for param, ns in els:
    rosparam.upload_params(ns, param)

n_tables = int(input("Enter the number of tables: "))
rospy.loginfo(f"There are {n_tables} tables.")

for i in range(n_tables):

    input(f"Take me to the search position for table {i}, then press Enter.")
    save_navigation_point(i)
    input(f"Take me to the unload position for table {i}, then press Enter.")
    save_load_point(i)

    while True:
        input(f"Place an Aruco marker in the corner of table {i}, adjust me so that I can see it, then press Enter.")
        try:
            save_table_cuboid(i)
            break
        except rospy.service.ServiceException:
            rospy.logwarn("Something went wrong - is the Aruco marker in view?")


input("Take me to the search position for the counter, then press Enter.")
save_navigation_point(-1)
input("Take me to the load position for table counter, then press Enter.")
save_load_point(-1)

input("Take me to the search position for the waiting area, then press Enter.")
save_navigation_point(-2)
while True:
    input(f"Place an Aruco marker to indicate the waiting area, adjust me so that I can see it, then press Enter.")
    try:
        save_table_cuboid(-2)
        break
    except rospy.service.ServiceException:
        rospy.logwarn("Something went wrong - is the Aruco marker in view?")

rospy.loginfo("ALL OK.")
