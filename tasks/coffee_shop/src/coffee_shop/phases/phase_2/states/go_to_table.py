#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class GoToTable(smach.State):
    
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done', 'skip'])
        self.context = context

    def execute(self, userdata):
        robot_x, robot_y = self.context.base_controller.get_pose()
        tables_need_serving = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "needs serving"]
        if not tables_need_serving:
            return 'skip'
        closest_table = min(tables_need_serving, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))
        label, table = closest_table
        if label == self.context.current_table:
            self.context.voice_controller.sync_tts("Lucky me, I am already at the table which needs serving!")
        else:
            self.context.voice_controller.sync_tts(f"I am going to {label}, which needs serving")
            position = table["location"]["position"]
            orientation = table["location"]["orientation"]
            self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.current_table = label
        return 'done'
