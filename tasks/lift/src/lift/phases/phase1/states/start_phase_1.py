import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param

class StartPhase1(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice
        self.controllers = controllers

    def execute(self, userdata):
        if rospy.get_param("/is_simulation"):
            rospy.loginfo("Hi, my name is Tiago and I am starting Phase 1.")
        else:
            self.voice.sync_tts("starting Phase 1.")
            # self.voice.sync_tts("Hi, my name is Tiago and I am starting Phase 1.")


        self.controllers.base_controller.sync_to_pose(get_pose_from_param('/start/pose'))

        return 'success'