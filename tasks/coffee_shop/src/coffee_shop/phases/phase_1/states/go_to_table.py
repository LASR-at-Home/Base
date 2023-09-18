#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from collections import defaultdict

class GoToTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done', 'not done', 'skip'])
        self.context = context
        self.prev_tried_tables = defaultdict(int) 
        self.prev_tried_table = None

    def execute(self, userdata):
        robot_x, robot_y = self.context.base_controller.get_pose()
        unvisited = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "unvisited" and label != self.prev_tried_table]
        if self.prev_tried_tables'
        closest_table = min(unvisited, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))
        label, next_table = closest_table
        self.context.voice_controller.async_tts(f"I am going to {label}")
        position, orientation = next_table["location"]["position"], next_table["location"]["orientation"]
        succeeded = False
        for i in range(5):
            succeeded = self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
            if succeeded:
                break
            rospy.loginfo(f"Failed to get to {label}, attempt {i}")
        if not succeeded:
            self.context.voice_controller.async_tts(f"I am struggling to go to {label}, I'll come back later")
            self.prev_tried_tables[label] += 1
            self.prev_tried_table = label
            return 'not done'
        self.context.tables[label]["status"] = "currently visiting"
        self.context.current_table = label
        return 'done'