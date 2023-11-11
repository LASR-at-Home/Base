#!/usr/bin/env python3
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice
from lasr_vision_msgs.srv import YoloDetection
import rospy, actionlib
from tiago_controllers.controllers.base_controller import CmdVelController
from interaction_module.srv import AudioAndTextInteraction
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from lasr_speech.srv import Speech
from tf_module.srv import BaseTransformRequest, ApplyTransformRequest, LatestTransformRequest, BaseTransform, \
    LatestTransform, ApplyTransform, TfTransform, TfTransformRequest

from cv_bridge3 import CvBridge
from lasr_shapely import LasrShapely
from std_msgs.msg import Int16, Empty


rasa = True

class Default:
    def __init__(self):
        rospy.loginfo("YOLO is here")
        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        rospy.loginfo("PM is here")
        self.pm = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Controllers is here")
        self.voice = Voice()
        self.bridge = CvBridge()
        rospy.loginfo("CV Bridge")
        self.shapely = LasrShapely()
        rospy.loginfo("Got shapely")
        self.controllers = Controllers()
        rospy.loginfo("CMD is here")
        self.cmd = CmdVelController()
        rospy.loginfo("Voice is here")

        self.tf = rospy.ServiceProxy('tf_transform', TfTransform)
        print("TF is here")
        self.tf_base = rospy.ServiceProxy('base_transform', BaseTransform)
        self.tf_latest = rospy.ServiceProxy('latest_transform', LatestTransform)
        self.tf_apply = rospy.ServiceProxy('apply_transform', ApplyTransform)
        if rasa:
            rospy.wait_for_service("/lasr_speech/transcribe_and_parse")
            rospy.loginfo("SPEECH RASA is here")
            self.speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)
        else:
            pass
            rospy.loginfo("SPEECH Dialogflow is here")
            self.speech = rospy.ServiceProxy("/interaction_module", AudioAndTextInteraction)

        if not rospy.get_published_topics(namespace='/pal_head_manager'):
            rospy.loginfo("Is SIM ---> True")
            rospy.set_param("/is_simulation", True)
        else:
            rospy.loginfo("Is SIM ---> FALSE")
            rospy.set_param("/is_simulation", False)

