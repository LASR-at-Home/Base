#!/usr/bin/env python3
import numpy as np
import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object, rank
import json
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug
from sensor_msgs.msg import PointCloud2


class ScheduleGoingOut(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default

    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
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
                self.default.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_wait()
            else:
                return True
        else:

            return False

    def is_anyone_in_front_of_me(self):
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        polygon = rospy.get_param('arena')
        detections, im = perform_detection(self.default, pcl_msg, polygon, filter)
        return len(detections) > 0


    def execute(self, userdata):
        self.default.voice.speak("I know there are {} people in the lift".format(rospy.get_param("/lift/num_clusters")))

        is_robot_closest_rank = rank(points_name="/lift/pos_persons") and self.is_anyone_in_front_of_me()
        print("is robot closest rank->>> {}".format(is_robot_closest_rank))

        is_closer_to_door = is_robot_closest_rank
        if is_closer_to_door:
            self.default.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            state = self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre/pose'))
            if not state:
                return 'failed'
            # turn around
            self.default.voice.speak("Just minding my own business!")
            self.default.controllers.base_controller.rotate_to_face_object(object_name='/door/pose')
            # wait for the person to exit
            rospy.sleep(2)
            self.default.voice.speak("Should I wait more for you?")
            # hear
            hear_wait = True
            count = 0
            while hear_wait and count < 5:
                if RASA:
                    hear_wait = self.hear_wait()
                    if hear_wait:
                        self.default.voice.speak("I will wait more")
                        rospy.sleep(5)
                    else:
                        self.default.voice.speak("i am done with waiting")
                        break
                    count += 1
            return 'success'
        else:
            self.default.voice.speak("I am not the closest to the door.")
            self.default.voice.speak("I will wait inside the lift because this is not my floor")
            return 'success'
