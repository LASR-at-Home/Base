#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToTable(smach.State):
    
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done', 'skip'])
        self.context = context

    def execute(self, userdata):
        tables_need_serving = [(table, data) for table, data in rospy.get_param("/tables").items() if data["status"] == "needs serving"]
        if not tables_need_serving:
            return 'skip'
        table, data = tables_need_serving[0]
        self.context.voice_controller.sync_tts(f"I am going to {table}, which needs serving")
        position = data["location"]["position"]
        orientation = data["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        rospy.set_param("/current_table", table)
        return 'done'