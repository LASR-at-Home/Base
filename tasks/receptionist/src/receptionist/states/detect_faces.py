"""This is no longer used"""

import sys
import smach
import rospy

from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import Recognise, RecogniseRequest

from lasr_skills import Say


class DetectFaces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["recognised", "unrecognised"])
        # self.dataset = sys.argv[2]
        self.people_in_frame = (
            {}
        )  # dict of people in frame and the time they were detected
        self.listen_topic = "/xtion/rgb/image_raw"
        self.detected_people = False

    def execute(self, userdata):
        print("I'm about to guess who you are")
        self.listener()
        if self.subscriber:
            self.subscriber.unregister()
        if not self.detected_people:
            self.say_nobody_detected()
            rospy.sleep(2)
            return "unrecognised"
        else:
            self.greet()
            rospy.sleep(3)
            return "recognised"

    def listener(self):
        # rospy.init_node("image_listener", anonymous=True)
        rospy.wait_for_service("/recognise")
        self.subscriber = rospy.Subscriber(
            self.listen_topic, Image, self.image_callback, queue_size=1
        )
        rospy.sleep(7)

    def detect(self, image):
        rospy.loginfo("Received image message")
        try:
            detect_service = rospy.ServiceProxy("/recognise", Recognise)
            req = RecogniseRequest()
            req.image_raw = image
            req.dataset = "receptionist"
            req.confidence = 0.4
            resp = detect_service(req)
            for detection in resp.detections:
                self.people_in_frame[detection.name] = rospy.Time.now()
            if len(resp.detections) == 0:
                return "unrecognised"
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def greet(self):
        sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with sm:
            smach.StateMachine.add(
                "SAY_DETECTIONS_IN_SEATING_AREA",
                Say(
                    text=f"I have detected people. Hello, {' '.join(self.people_in_frame.keys())}"
                ),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "aborted",
                    "preempted": "preempted",
                },
            )

        outcome = sm.execute()

    def say_nobody_detected(self):
        sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with sm:
            smach.StateMachine.add(
                "SAY_NOBODY_DETECTED_IN_SEATING_AREA",
                Say(text="I didn't detect anyone that I know."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "aborted",
                    "preempted": "preempted",
                },
            )

        outcome = sm.execute()

    def image_callback(self, image):

        prev_people_in_frame = list(self.people_in_frame.keys())
        # remove detections from people_in_frame that are older than 5 seconds long
        self.detect(image)
        for person in list(self.people_in_frame.keys()):
            if rospy.Time.now() - self.people_in_frame[person] > rospy.Duration(10):
                del self.people_in_frame[person]
        if (
            list(self.people_in_frame.keys()) != prev_people_in_frame
            and len(self.people_in_frame) > 0
        ) or (len(prev_people_in_frame) == 0 and len(self.people_in_frame) > 0):
            self.detected_people = True
