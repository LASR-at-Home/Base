#!/usr/bin/env python3
import smach
from lift.phases import Phase1, Phase2, Phase3
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice
from lasr_object_detection_yolo.srv import YoloDetection
import rospy

class Lift(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        self.controllers = Controllers()
        # self.voice = Voice()
        self.voice = "hello"

        if not rospy.get_published_topics(namespace='/pal_head_manager'):
            rospy.set_param("/is_simulation", True)

        with self:
            smach.StateMachine.add('PHASE1', Phase1(self.controllers, self.voice), transitions={'success' : 'PHASE2'})
            smach.StateMachine.add('PHASE2', Phase2(self.controllers, self.voice, self.yolo), transitions={'success' : 'PHASE3'})
            smach.StateMachine.add('PHASE3', Phase3(self.controllers, self.voice), transitions={'success' : 'success'})