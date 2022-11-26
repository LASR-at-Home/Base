#!/usr/bin/env python3
import os
import smach
import rospy, rospkg
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge, cv2
from lasr_perception_server.srv import DetectImage
from std_msgs.msg import Empty

class LookAroundState(smach.State):
    '''
        Looks aroudn and tries to detect people
    '''

    def __init__(self, controllers):
        smach.State.__init__(self,
                             outcomes=['finished_without_info', 'finished_with_known_ppl', 'finished_with_unknown_ppl'])

        self.controllers = controllers
        if self.controllers.head_controller:
            self.rotate_head = self.controllers.head_controller.sync_reach_to
        else:
            self.rotate_head = lambda *args, **kwargs : None

        self.pub = rospy.Publisher('/sm_reset', Empty, queue_size=1)
    def find_faces(self):
        print('in find faces')
        imgs = []
        rate = rospy.Rate(3)
        for i in range(2):
            imgs.append(rospy.wait_for_message("/xtion/rgb/image_raw", Image))
            rate.sleep()

        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
        resp = det(imgs, "coco", 0.7, 0.3, ["person"],'known_people').detected_objects
        return resp

    def execute(self, userdata):
        direction = 0.2
        for i in range(2):
            direction *= -1
            self.rotate_head(direction, -0.2, velocities=[0.1, 0.0])
            rospy.sleep(15)
            if True:
                # if self.find_faces():
                print(self.find_faces())
                self.pub.publish(Empty())
                return 'finished_with_known_ppl'
                # elif len(self.find_faces()) > 2:
                # return 'finished_with_unknown_ppl'

        # return 'finished_without_info'


# if __name__ == '__main__':
#     rospy.init_node('look_around_state')
#     sm = smach.StateMachine(outcomes=['success'])
#     with sm:
#         smach.StateMachine.add('LOOK_AROUND_STATE', LookAroundState(),
#                                transitions={'found_people':'finished_looking_around'})
#     outcome = sm.execute()
#     rospy.loginfo('I have completed execution with outcome: ')
#     rospy.loginfo(outcome)
