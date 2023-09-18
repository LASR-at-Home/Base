#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class GoToTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        robot_x, robot_y = self.context.base_controller.get_pose()
        unvisited = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "unvisited"]
        closest_table = min(unvisited, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))
        label, next_table = closest_table
        self.context.voice_controller.async_tts(f"I am going to {label}")
        position, orientation = next_table["location"]["position"], next_table["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.tables[label]["status"] = "currently visiting"
        self.context.current_table = label
        return 'done'