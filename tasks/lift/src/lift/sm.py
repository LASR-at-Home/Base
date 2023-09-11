#!/usr/bin/env python3
import smach
from lift.phases import Phase1, Phase2, Phase3
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice
from lasr_object_detection_yolo.srv import YoloDetection
import rospy, actionlib
from tiago_controllers.controllers.base_controller import CmdVelController
from interaction_module.srv import AudioAndTextInteraction
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction

class Lift(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.speech = rospy.ServiceProxy("/interaction_module", AudioAndTextInteraction)
        self.pm = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

        self.controllers = Controllers()
        self.cmd = CmdVelController()
        self.voice = Voice()

        if not rospy.get_published_topics(namespace='/pal_head_manager'):
            rospy.set_param("/is_simulation", True)
        else:
            rospy.set_param("/is_simulation", False)

        with self:
            smach.StateMachine.add('PHASE1', Phase1(self.controllers, self.voice, self.cmd, self.pm), transitions={'success' : 'PHASE2'})
            smach.StateMachine.add('PHASE2', Phase2(self.controllers, self.voice, self.yolo, self.cmd, self.speech, self.pm), transitions={'success' : 'PHASE3'})
            smach.StateMachine.add('PHASE3', Phase3(self.controllers, self.voice), transitions={'success' : 'success'})