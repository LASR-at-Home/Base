#!/usr/bin/env python3
import smach
import rospy

class ScheduleGoingOut(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice

    def execute(self, userdata):
        self.voice.speak("How many people are thinking to go out of the lift?")

        closer_to_door = True
        if closer_to_door:
            self.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            # go to centre waiting area
            # turn around
            # wait for the person to exit
            rospy.sleep(5)
            # hear
            hear_wait = True
            if hear_wait:
                self.voice.speak("I will wait more")
                rospy.sleep(5)
            else:
                # goint back to the door
                pose = rospy.get_param("/wait_centre")
                self.controllers.base_controller.sync_to_pose(pose[0], pose[1], radius=2.5)

                pass

            return 'success'
        else:
            self.voice.speak("I am not the closest to the door so I will wait for you to exit first")
            counter = 0
            people_in_front_of_door = True
            while ready_to_exit or counter < 3:
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



        return 'success'
