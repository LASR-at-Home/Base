#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice
from geometry_msgs.msg import Pose, Point, Quaternion

class GuidePerson(smach.State):
    def __init__(self, base_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        tables = rospy.get_param("/tables")
        empty_tables = [(label, table) for label, table in tables.items() if table["status"] == "ready"]
        target_table = empty_tables[0][1]
        position, orientation = target_table["location"]["position"], target_table["location"]["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.voice_controller.sync_tts("Please be seated!")
        return 'done'