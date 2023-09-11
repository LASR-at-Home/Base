#!/usr/bin/env python3
import smach
import rospy
import json 

class Negotiate(smach.State):
    def __init__(self, controllers, voice,speech):
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


    def hear_if_wait(self):
        resp = self.listen()

        if resp["intent"]["name"] == "negotiate_lift":
            #I'm going to wait 
            wait = resp["entities"].get("wait_command",[])
            if not wait:
                self.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_if_wait()
            else:
                return True 
        else: 

            return False




    def execute(self, userdata):
        # call and count the people objects
        self.voice.speak(" I can see {} people in front of me".format(2))
        closer_to_door = True
        if closer_to_door:
            self.voice.speak("I am the closest to the door so I have to exit first")
            self.voice.speak("I can wait for you to exit the lift. Just say 'Tiago, wait' if you need more time.")
            rospy.sleep(5)
            # hear
            hear_wait = True
            hear_wait = self.hear_if_wait()

            if hear_wait:
                self.voice.speak("I will wait more")
                rospy.sleep(5)
            else:
                # goint back to the door
                pass

            return 'success'


        else:
            counter = 0
            people_in_front_of_door = True
            while ready_to_exit or counter < 3:
                self.voice.speak("I am not the closest to the door so I will wait for you to exit first")
                self.voice.speak("I will tell you a joke while I wait")
                # joke
                # clear costmap
                # wait for the person to exit
                rospy.sleep(5)
                # check if people are still in front of the door
                if not people_in_front_of_door:
                    ready_to_exit = True
                    break
                else:
                    counter += 1
                    ready_to_exit = False

            if ready_to_exit:
                return 'success'
            elif counter == 3:
                self.voice.speak("I will wait more")
                return 'failed'
            else:
                return 'failed'
