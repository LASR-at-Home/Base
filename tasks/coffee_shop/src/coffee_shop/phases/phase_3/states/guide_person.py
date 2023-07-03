#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice

class GuidePerson(smach.State):
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice = Voice()
    def execute(self, userdata):
        tables = rosparam.get("/tables")
        empty_tables = [(label, table) for label, table in tables.items() if table["status"] == ready]
        target_table = empty_tables[0]
        location, orientation = target_table["location"]["position"], target_table["location"]["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.voice.sync_tts("Please be seated!")
        return 'done'

