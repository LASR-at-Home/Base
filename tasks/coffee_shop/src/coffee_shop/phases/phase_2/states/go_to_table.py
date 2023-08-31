#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToTable(smach.State):
    
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done', 'skip'])
        self.context = context

    def execute(self, userdata):
        tables_need_serving = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "needs serving"]
        if not tables_need_serving:
            return 'skip'
        label, table = tables_need_serving[0]
        self.context.voice_controller.sync_tts(f"I am going to {table}, which needs serving")
        position = table["location"]["position"]
        orientation = table["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.tables[label]["status"] = "currently serving"
        self.context.current_table = label
        return 'done'