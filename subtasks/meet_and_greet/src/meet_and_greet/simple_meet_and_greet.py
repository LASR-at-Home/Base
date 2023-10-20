#!/usr/bin/env python3
import rospy
import os
import rospkg
from lasr_voice.voice import Voice
from tiago_controllers.helpers import get_pose_from_param, is_running
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge, cv2
from lasr_perception_server.srv import DetectImage
from tiago_controllers.controllers import Controllers

IMAGES = 1

class SimpleMeetGreet:
    def __init__(self):
        self.controllers = Controllers()

        if rospy.get_published_topics(namespace='/xtion'):
            print('in xtion')
            self.topic = '/xtion/rgb/image_raw'
        else:
            print('in usb')
            self.topic = '/usb_cam/image_raw'


        self.voice = Voice()
        self.map_points = ['/point1', '/point2']  # pos on map

    def find_person(self):
        imgs = []
        rate = rospy.Rate(3)
        for i in range(IMAGES):
            im = rospy.wait_for_message('/usb_cam/image_raw', Image)
            imgs.append(im)
            # imgs.append(rospy.wait_for_message(self.topic, Image))
            rate.sleep()

        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
        resp = det(imgs, "coco", 0.7, 0.3, ["person"], 'known_people').detected_objects
        print(resp)
        return resp

    def handle_findings(self, detections):
        names = []
        counter = 0
        if len(detections)> 0:
            for human in detections:
                print('human name', human.name)
                if human.name == 'person':
                    counter = counter + 1
                else:
                    self.voice.sync_tts("Hi" +str(human.name))

                    print('hi, ', human.name)
            if counter > 0:
                # self.voice.sync_tts(' there are new people. I have not met ' + str(counter) + 'people')
                if counter == 1:
                    # self.voice.sync_tts("there are new people. I have not met you before")
                    print(' there are new people. I have not met you before')
                else:
                    # self.voice.sync_tts(' there are new people. I have not met ' + str(counter) + 'people')
                    print(' there are new people. I have not met ' + str(counter) + 'people')



    def main(self):
        # pos = get_pose_from_param('/door')
        # self.controllers.base_controller.sync_to_pose(pos)
        for i in range(4):
            resp = self.find_person()
            if len(resp) > 0:
                self.handle_findings(resp)
                return

        # self.voice.sync_tts("Hi, i don't know anyone")
        print("Hi, i don't know anyone")
        print('I dont known anyone')




if __name__ == '__main__':
    rospy.init_node("simple_meet_and_greet", anonymous=True)
    test = SimpleMeetGreet()
    test.main()
