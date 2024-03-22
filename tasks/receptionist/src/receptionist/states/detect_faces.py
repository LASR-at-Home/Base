import sys
import smach
import rospy

from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import Recognise, RecogniseRequest
from lasr_voice import Voice
import re



class DetectFaces(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['unrecognised', 'recognised'])
        self.default = default
        self.dataset = '/home/rexy/Documents/robotclub/robocup_ws/src/base_zoe_fork/common/vision/lasr_vision_deepface/datasets'
        self.people_in_frame = {}  # dict of people in frame and the time they were detected
        self.listen_topic = "/xtion/rgb/image_raw"
        self.max_attempts: int = 5
        self.current_attempts = 0

    def execute(self, userdata):
        self.default.voice.speak("I'm about to guess who you are")
        result = self.detect()
        print("DETECT PHASES RESULT: ", result)
        if result == "recognised":
            self.greet()
        return result
        
        

    def detect(self):
        rospy.wait_for_service("/recognise")
        try:
            for attempt in range(self.max_attempts):
                image = rospy.wait_for_message(self.listen_topic, Image)
                rospy.loginfo("Received Image from xtion")
                detect_service = rospy.ServiceProxy("/recognise", Recognise)
                req = RecogniseRequest()
                req.image_raw = image
                req.dataset = self.dataset
                req.confidence = 0.4
                resp = detect_service(req)
                for detection in resp.detections:
                    self.people_in_frame[detection.name] = rospy.Time.now()
                    print(detection)
                if len(resp.detections) > 0:
                    return 'recognised'
            return 'unrecognised'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def greet(self):
        voice = Voice()
        voice.speak(f"Hello, {' '.join(self.people_in_frame)}")
        guestcount = rospy.get_param("guestcount/count", 0)
        drink = rospy.get_param(f"guest{guestcount}/drink", "Orange")
        voice.speak(f"I know your favourite drink is: {drink}")
