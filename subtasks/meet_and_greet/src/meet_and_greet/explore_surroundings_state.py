#!/usr/bin/env python3
# coding=utf-8
import os
import smach
import rospy
from sensor_msgs.msg import Image
import rospkg
from cv_bridge3 import CvBridge
from cv_bridge3 import cv2
import random
# import random
# import string
from tiago_controllers.base_controller import BaseController
from tiago_controllers.head_controller import HeadController
from lasr_perception_server import looking_around_and_taking_imgs as looking_around

from lasr_perception_server.srv import DetectImages, DetectImage



def find_faces():
    rospy.init_node('test')

    imgs = []
    rate = rospy.Rate(3)
    for i in range(2):
        imgs.append(rospy.wait_for_message("/usb_cam/image_raw", Image))
        rate.sleep()
    print('img len', len(imgs))

    # # * show the output image
    # print("I GOT HERE!!! ------------------------------")
    # bridge = CvBridge()
    # cv_image = bridge.imgmsg_to_cv2(imgs[len(imgs)-1], desired_encoding='passthrough')
    # # cv2.imshow("Image", cv_image)
    # path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")
    # cv2.imwrite(path_output + "/images/random" + str(random.random())+".jpg", cv_image)
    # cv2.waitKey(0)
    # print("DETECTION TIME!!! ------------------------------")
    # # * show the output image

    det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
    resp = det(imgs, "coco", 0.7, 0.3, ['person'],'known_people').detected_objects
    print(resp)
    # return resp if len(resp) > 0 else None


class ExploreSurroundingsState(smach.State):
    '''
        Wonders around and detects people
    '''

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished_scan'],
                             )

        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.map_points = [] # pos on map


    def execute(self, userdata):
        while not rospy.shutdown:
            for i in random(self.search_points):
                imgs = looking_around(self.head_controller)
                print('images -> ', len(imgs))
                if find_faces():
                    return 'finished_scan'
                self.base_controller.sync_to_pose(i)
        return 

if __name__ == '__main__':
    find_faces()
    # rospy.init_node('smach_example_state_machine')
    # sm = smach.StateMachine(outcomes=['success'])
    # with sm:
    #     smach.StateMachine.add('EXPLORE_SURROUNDINGS', ExploreSurroundingsState(),
    #                            transitions={'finished_scan':'success'},
    #                            remapping={'dataset_path':'dataset_path'})
    # outcome = sm.execute()
    # rospy.loginfo('I have completed execution with outcome: ')
    # rospy.loginfo(outcome)
