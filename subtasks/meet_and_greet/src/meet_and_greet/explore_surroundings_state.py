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



def test():
    # rospy.init_node('test')
    # head_controller = HeadController()
    # imgs = looking_around(head_controller)
    # print('images -> ', len(imgs))
    # perception = rospy.ServiceProxy('lasr_perception_server/detect_object_image', DetectImages)
    # print(perception(imgs[0],'coco', 0.7, 0.3,['person'],'known_people'))

    rospy.init_node("test")
    im = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
    imgs = []
    imgs.append(im)
    im = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
    imgs.append(im)
    print(len(imgs))
    # * show the output image
    print("I GOT HERE!!! ------------------------------")
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(im, desired_encoding='passthrough')
    cv2.imshow("Image", cv_image)
    path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")
    cv2.imwrite(path_output + "/images/random" + str(random.random())+".jpg", cv_image)
    cv2.waitKey(0)
    print("DETECTION TIME!!! ------------------------------")
    # * show the output image

    det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
    resp = det(im, "coco", 0.7, 0.3, ["person"],'open_cv').detected_objects


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
        self.search_points = [(-1, 0.5), (1, 0.5), (1, -0.5), (-1, -0.5), (0, 0)]


    def execute(self, userdata):
        imgs = looking_around(self.head_controller)
        print('images -> ', len(imgs))
        self.perception = rospy.ServiceProxy('lasr_perception_server/detect_objects_image', DetectImages)
        print(self.perception(imgs,'coco', 0.7, 0.3, '','known_people'))




        return 'finished_scan'


if __name__ == '__main__':
    test()
    # rospy.init_node('smach_example_state_machine')
    # sm = smach.StateMachine(outcomes=['success'])
    # with sm:
    #     smach.StateMachine.add('EXPLORE_SURROUNDINGS', ExploreSurroundingsState(),
    #                            transitions={'finished_scan':'success'},
    #                            remapping={'dataset_path':'dataset_path'})
    # outcome = sm.execute()
    # rospy.loginfo('I have completed execution with outcome: ')
    # rospy.loginfo(outcome)
