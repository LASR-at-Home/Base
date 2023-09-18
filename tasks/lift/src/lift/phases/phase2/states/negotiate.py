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
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object, rank
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA


class Negotiate(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default
        # self.voice = voice
        # self.speech = speech

    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def affirm(self):
        # Listen to person:
        resp = self.listen()
        # Response in intent can either be yes or no.
        # Making sure that the response belongs to "affirm", not any other intent:
        if resp['intent']['name'] != 'affirm':
            self.default.voice.speak("Sorry, I didn't get that, please say yes or no")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"].lower()
        if choice not in ["yes", "no"]:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        return choice

    def hear_wait(self):
        resp = self.listen()

        if resp["intent"]["name"] == "negotiate_lift":
            # I'm going to wait
            wait = resp["entities"].get("wait_command", [])
            if not wait:
                self.default.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_wait()
            else:
                return True
        else:

            return False

    def execute(self, userdata):
        # call and count the people objects
        self.default.voice.speak("Let's negotiate who is going out first")

        is_closer_to_door = rank()
        if is_closer_to_door:
            self.default.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            self.default.voice.speak("I will wait by the lift for you.")
            # TODO: here maybe switch to the wait_centre
            res = self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/start/pose'))
        else:
            self.default.voice.speak("I am not the closest to the door.")
            self.default.voice.speak("I will wait for you to exit first")
            rospy.sleep(1)

        self.default.voice.speak("Should I wait more for you?")
        self.voice.speak("Please say yes or no.")
        # self.default.voice.speak("Just say 'Tiago, wait' if you need more time.")
        hear_wait = True
        count = 0
        while hear_wait or count < 5:
            if RASA:
                hear_wait = self.hear_wait()
                if hear_wait:
                    self.default.voice.speak("I will wait more")
                    rospy.sleep(5)
                else:
                    self.default.voice.speak("i am done with waiting")
                    break

        # untested
        # hear_wait = "yes"
        # count = 0
        # while (hear_wait == "yes") or count < 5:
        #     if RASA:
        #         hear_wait = self.affirm()
        #         if hear_wait == "yes":
        #             self.voice.speak("I will wait more")
        #             rospy.sleep(5)
        #         else:
        #             self.voice.speak("i am done with waiting")
        #             break

            else:
                req = AudioAndTextInteractionRequest()
                req.action = "BUTTON_PRESSED"
                req.subaction = "confirm_button"
                req.query_text = "SOUND:PLAYING:PLEASE"
                resp = self.default.speech(req)
                rospy.logwarn('the response of input to loc srv is: {}'.format(resp))
                if resp.result == 'yes':
                    self.default.voice.speak("I will wait more")
                    rospy.sleep(5)
                else:
                    self.default.voice.speak("i am done with waiting")
                    break

            count += 1
            rospy.sleep(0.5)

        if count >= 5:
            return 'failed'


        if is_closer_to_door:
            # clear costmap
            clear_costmap()
            # maybe take the lift info again
            self.default.voice.speak("Exiting the lift")

        return 'success'
