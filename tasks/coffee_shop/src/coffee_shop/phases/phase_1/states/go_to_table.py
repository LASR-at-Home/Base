#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class GoToTable(smach.State):
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
    def execute(self, userdata):
        tables = rospy.get_param("/tables")
        robot_x, robot_y = self.base_controller.get_pose()
        unvisited = [(label, table) for label, table in tables.items() if table["status"] == "unvisited"]
        closest_table = min(unvisited, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))
        label, next_table = closest_table
        position, orientation = next_table["location"]["position"], next_table["location"]["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        rospy.set_param(f"/tables/{label}/status", "currently_visiting")
        rospy.set_param("current_table", label)
        return 'done'
