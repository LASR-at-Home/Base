import sys
import smach
import rospy

from sensor_msgs.msg import Image

from common.vision.lasr_vision_msgs.srv import Recognise, RecogniseRequest
from lasr_voice import Voice

class DetectFaces(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default
        self.dataset = sys.argv[2]
        self.people_in_frame = {} # dict of people in frame and the time they were detected
        self.listen_topic = sys.argv[1]

    def execute(self, userdata):
<<<<<<< HEAD
        
        # Here we will detect faces
=======
        self.listener()
>>>>>>> aaliyah_fork/main
        return 'succeeded'
    
    def listener(self):
        rospy.init_node("image_listener", anonymous=True)
        rospy.wait_for_service("/recognise")
        rospy.Subscriber(self.listen_topic, Image, self.image_callback, queue_size=1)
        rospy.spin()

    def detect(self, image):
        rospy.loginfo("Received image message")
        try:
            detect_service = rospy.ServiceProxy("/recognise", Recognise)
            req = RecogniseRequest()
            req.image_raw = image
            req.dataset = self.dataset
            req.confidence = 0.4
            resp = detect_service(req)
            for detection in resp.detections:
                self.people_in_frame[detection.name] = rospy.Time.now()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    
    def greet():
        voice = Voice()
        voice.speak(f"Hello, {' '.join(people_in_frame)}")
    
    def image_callback(self, image):
        global people_in_frame
        prev_people_in_frame = list(people_in_frame.keys())
        # remove detections from people_in_frame that are older than 5 seconds long
        self.detect(image)
        for person in list(people_in_frame.keys()):
            if rospy.Time.now() - people_in_frame[person] > rospy.Duration(10):
                del people_in_frame[person]
        if (
            list(people_in_frame.keys()) != prev_people_in_frame
            and len(people_in_frame) > 0
        ) or (len(prev_people_in_frame) == 0 and len(people_in_frame) > 0):
            self.greet()

