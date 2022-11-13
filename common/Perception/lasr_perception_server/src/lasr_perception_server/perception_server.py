#!/usr/bin/env python3
import rospy

from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionResponse

from lasr_perception_server.srv import DetectImage, DetectImages, DetectImageResponse, DetectImagesResponse
from face_detection.srv import FaceDetection, FaceDetectionResponse, \
    FaceDetectionRequest

from recognise_people.srv import RecognisePeople, RecognisePeopleResponse, RecognisePeopleRequest
from lasr_perception_server.msg import Detection

# for debugging

from cv_bridge3 import CvBridge
from cv_bridge3 import cv2

import os, random
import rospkg


# TODO: extend msg for pcl
# TODO: a func that takes yolo detections and returns with a ceratin param for instance person in the form of Detection
# TODO: what do we do when there is nothing on the photo -> in an optimasided way see if the task is executable with tiny face
# and then give it to everything else! or move the head in another pos
# decide pcl or not
# TODO: refactor to call an outside function
# def rotate(self, head_controller):
class PerceptionServer():

    def __init__(self):

        self.yolo_detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.face_detect = rospy.ServiceProxy("face_detection_server", FaceDetection)
        self.recogniser = rospy.ServiceProxy('recognise_people_server', RecognisePeople)

        self.handler = rospy.Service("lasr_perception_server/detect_objects_image", DetectImage,
                                     self.handle_task)

    def detect_image(self, req):
        print(req.task, 'the task')
        if len(req.filter):
            return DetectImageResponse(
                [det for det in self.yolo_detect(req.image[0], req.dataset, req.confidence, req.nms).detected_objects if
                 det.name in req.filter])
        else:
            return DetectImageResponse(
                self.yolo_detect(req.image[0], req.dataset, req.confidence, req.nms).detected_objects)

    def face_detection(self, req):
        if len(req.image) > 0:
            detected_obj = []
            for i in req.image:
                detected_obj.append(self.face_detect(i).detected_objects)
                print('printing detected obj', detected_obj)
            flat_list = [item for sublist in detected_obj for item in sublist]
            return DetectImageResponse(flat_list)

        else:
            return DetectImageResponse(self.face_detect(req.image[0]).detected_objects)

    # returns bbox of known and unknown people
    # check if the lists ar eempty and handle that
    def face_and_yolo(self, req):
        if isinstance(req.image, list):
            detected_obj_yolo = []
            detected_obj_open_cv = []
            for i in req.image:
                # take opencv detection
                detected_obj_open_cv.append(self.face_detect(i).detected_objects)
                # take yolo detection
                detected_obj_yolo.append(self.yolo_detect(i, req.dataset, req.confidence, req.nms).detected_objects)
            # flat_list = [item for sublist in detected_obj for item in sublist]
            # print(type(detected_obj_yolo))
            print('i am breaking here')
            print(detected_obj_yolo)
            req = RecognisePeopleRequest()
            req.detected_objects_yolo = detected_obj_yolo
            req.detected_objects_opencv = detected_obj_open_cv

            return DetectImageResponse([])
            # print(len(self.recogniser(req).detected_objects))
        else:
            resp = self.face_detect(req.image).detected_objects
            return DetectImageResponse(resp)

    def save_images_debugger(self, imgs):
        # * show the output image
        bridge = CvBridge()
        for im in imgs:
            cv_image = bridge.imgmsg_to_cv2(im, desired_encoding='passthrough')
            path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")
            cv2.imwrite(path_output + "/images/random" + str(random.random()) + ".jpg", cv_image)
            print("next image ------------------------------")

    def handle_task(self, req):
        resp = None
        print("IA AM IN HANDLING TASKS")
        if req.task == 'open_cv':
            resp = self.face_detection(req)
        elif req.task == 'known_people':
            print('tea time')
            resp = self.face_and_yolo(req)
        else:
            resp = self.detect_image(req)
        print(resp, 'printing resp')
        return resp


if __name__ == "__main__":
    rospy.init_node("lasr_perception_server")
    rospy.loginfo("initialising the perception server")
    perception_server = PerceptionServer()
    rospy.spin()
