#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice
from geometry_msgs.msg import Pose, Point, Quaternion

class GuidePerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        tables = rospy.get_param("/tables")
        empty_tables = [(label, table) for label, table in tables.items() if table["status"] == "ready"]
        table, data = empty_tables[0]
        position, orientation = data["location"]["position"], data["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.voice_controller.sync_tts("Please be seated!")
        rospy.set_param(f"/tables/{table}/status", "needs serving")
        return 'done'