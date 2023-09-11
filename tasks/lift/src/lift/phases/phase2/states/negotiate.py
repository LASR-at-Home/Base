#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import json
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object


class Negotiate(smach.State):
    def __init__(self, controllers, voice, speech):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice
        self.speech = speech

    def listen(self):
        resp = self.speech()
        if not resp.success:
            self.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def hear_wait(self):
        resp = self.listen()

        if resp["intent"]["name"] == "negotiate_lift":
            # I'm going to wait
            wait = resp["entities"].get("wait_command", [])
            if not wait:
                self.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_wait()
            else:
                return True
        else:

            return False

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
            # TODO: here maybe switch to the wait_centre
            res = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/start/pose'))
        else:
            self.voice.speak("I am not the closest to the door.")
            self.voice.speak("I will wait for you to exit first")
            rospy.sleep(1)

        self.voice.speak("Should I wait more for you?")
        self.voice.speak("Just say 'Tiago, wait' if you need more time.")
        hear_wait = True
        count = 0
        while hear_wait or count < 5:
            if RASA:
                hear_wait = self.hear_wait()
                if hear_wait:
                    self.voice.speak("I will wait more")
                    rospy.sleep(5)
                else:
                    self.voice.speak("i am done with waiting")
                    break

            else:
                req = AudioAndTextInteractionRequest()
                req.action = "BUTTON_PRESSED"
                req.subaction = "confirm_button"
                req.query_text = "SOUND:PLAYING:PLEASE"
                resp = self.speech(req)
                rospy.logwarn('the response of input to loc srv is: {}'.format(resp))
                if resp.result == 'yes':
                    self.voice.speak("I will wait more")
                    rospy.sleep(5)
                else:
                    self.voice.speak("i am done with waiting")
                    break

            count += 1
            rospy.sleep(0.5)

        if count >= 5:
            return 'failed'


        if is_closer_to_door:
            # clear costmap
            clear_costmap()
            # maybe take the lift info again
            self.voice.speak("Exiting the lift")

        return 'success'
