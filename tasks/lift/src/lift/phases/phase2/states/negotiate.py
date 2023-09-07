#!/usr/bin/env python3
import smach
import rospy
import numpy as np

from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from common_math.math_ import euclidian_distance
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object
class Negotiate(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice
        self.speech = rospy.ServiceProxy("/interaction_module", AudioAndTextInteraction)

    def execute(self, userdata):
        # call and count the people objects
        self.voice.speak("Let's negotiate who is going out first")

        is_closer_to_door = is_close_to_object(object_name='/door/pose', min_dist=0.5)
        if is_closer_to_door:
            self.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            self.voice.speak("I will wait by the lift for you.")
            res = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/start/pose'))
        else:
            self.voice.speak("I am not the closest to the door.")
            self.voice.speak("I will wait for you to exit first")
            rospy.sleep(1)

        self.voice.speak("should I wait more for you?")
        result = 'yes'
        count = 0
        while result != 'no' or count < 5:
            req = AudioAndTextInteractionRequest()
            req.action = "BUTTON_PRESSED"
            req.subaction = "confirm_button"
            req.query_text = "SOUND:PLAYING:PLEASE"
            resp = self.speech(req)
            clear_costmap()
            rospy.logwarn('the response of input to loc srv is: {}'.format(resp))
            if resp.result == 'no':
                self.voice.speak("i am done with waiting")
                break
            self.voice.speak("i will wait more")
            count += 1
            rospy.sleep(0.5)

        if is_closer_to_door:
            # clear costmap
            clear_costmap()
            # maybe take the lift info again
            # if there are no clusters
            self.voice.speak("Exiting the lift")

        return 'success'

