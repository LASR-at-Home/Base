#!/usr/bin/env python3
import smach, os, rospy
from sensor_msgs.msg import Image
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
import json
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from sensor_msgs.msg import PointCloud2
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug

class WaitForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default


    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp


    def get_people_number(self):
        resp = self.listen()
        if resp["intent"]["name"] != "negotiate_lift":
            self.default.voice.speak("Sorry, I misheard you, could you say again how many people?")
            return self.get_people_number()
        people = resp["entities"].get("people",[])
        if not people: 
            self.default.voice.speak("Sorry, could you say again how many people?")
            return self.get_people_number()
        people_number = int(people[0]["value"])        
        self.default.voice.speak("I hear that there are {} people".format(people_number))
        return people_number

    def safe_seg_info(self, detections):

        # detections = np.array(detections)

        pos_people = []
        for i, person in detections:
            person = person.tolist()
            pos_people.append([person[0], person[1]])

        num_people = len(detections)

        rospy.set_param("/lift/num_people", num_people)
        rospy.set_param("/lift/pos_persons", pos_people)

        # if DEBUG > 3:
        #     print("num clusters in safe")
        #     print(rospy.get_param("/lift/num_people"))
        #     print(pos_people)
        #     print(type(pos_people))
        #     print("centers in safe")
        #     print(rospy.get_param("/lift/pos_persons"))

    def affirm(self):
        # Listen to person:
        resp = self.listen()
        # Response in intent can either be yes or no.
        # Making sure that the response belongs to "affirm", not any other intent:
        if resp['intent']['name'] != 'affirm':
            self.default.voice.speak("Sorry, I didn't get that, please say yes or no")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"].lower()
        if choice not in ["yes", "no"]:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        return choice


    def execute(self, userdata):
        # wait and ask

        rospy.set_param("/from_schedule", False)
        self.default.voice.speak("Exciting stuff, we are going to the lift! But let me ask you something first.")
        self.default.voice.speak("How many people are going in the lift?")
        self.default.voice.speak("Please answer with a number of people.")

        count = 2
        if RASA:
            try:
                count = self.get_people_number()
            except Exception as e:
                print(e)
                count = 2
                self.default.voice.speak("I couldn't hear how many people, so I'm going to guess 2")
        else:
            req = AudioAndTextInteractionRequest()
            req.action = "ROOM_REQUEST"
            req.subaction = "ask_location"
            req.query_text = "SOUND:PLAYING:PLEASE"
            resp = self.default.speech(req)
            print("The response of asking the people is {}".format(resp.result))
            # count = resp.result

        self.default.voice.speak("Thank you for your answer!")
        self.default.voice.speak("Please go inside the lift. You see, I am a very very good robot!")
        # self.default.voice.speak("I will give way to the people now, because I am a very very good robot!")
        rospy.sleep(5)

        self.default.voice.speak("I will now move to the center of the lift waiting area")
        state = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_in_front_lift_centre/pose'))
        rospy.loginfo("State of the robot in wait for people is {}".format(state))
        rospy.sleep(0.5)

        # prev start   only yolo
        # send request - image, dataset, confidence, nms
        # image = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        # detections = self.default.yolo(image, "yolov8n-seg.pt", 0.3, 0.3)
        # prev end

        polygon = rospy.get_param('test_lift_points')
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        detections, im = perform_detection(self.default, pcl_msg, None, ["person"], "yolov8n-seg.pt")
        print("len detections")
        print(len(detections))


        self.safe_seg_info(detections)
        # debug(im, detections)
        # people = detect_objects(["person"])
        # count_people = 0
        # count_people = sum(1 for det in detections.detected_objects if det.name == "person")


        # segment them as well and count them
        count_people = len(detections)

        self.default.voice.speak("I can see beautiful people around. Only {} of them to be exact.".format(count_people))

        # lab dev
        # if count_people < count:
        #     return 'failed'
        # else:
        #     return 'success'

        # new things
        if count_people < count:
            return 'failed'
        else:
            self.default.voice.speak("Are you ready for me to enter the lift?")
            self.default.voice.speak("Please answer with a yes or no")
            answer = 'no'
            if RASA:
                answer = self.affirm()
                print("Answer from Speech: ", answer)
                if answer == 'yes':
                    self.default.voice.speak("Good stuff!")
                    return 'success'
                else:
                    return 'failed'


