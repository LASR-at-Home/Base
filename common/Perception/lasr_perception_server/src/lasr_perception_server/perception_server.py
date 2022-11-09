#!/usr/bin/env python3
import rospy

from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionResponse

from lasr_perception_server.srv import DetectImage, DetectImages, DetectImageResponse, DetectImagesResponse
from face_detection.srv import FaceDetection, FaceDetectionResponse,  \
    FaceDetectionRequest

from recognise_people.srv import RecognisePeople, RecognisePeopleResponse


#TODO: extend msg for pcl
#TODO: a func that takes yolo detections and returns with a ceratin param for instance person in the form of Detection
#TODO: what do we do when there is nothing on the photo -> in an optimasided way see if the task is executable with tiny face
# and then give it to everything else! or move the head in another pos
# decide pcl or not
#TODO: refactor to call an outside function
# def rotate(self, head_controller):
class PerceptionServer():

    def __init__(self):

        self.yolo_detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.face_detect = rospy.ServiceProxy("face_detection_server", FaceDetection)
        self.recogniser = rospy.ServiceProxy('recognise_people', RecognisePeople)

        self.handler = rospy.Service("lasr_perception_server/detect_images", DetectImages,
                                     self.handle_task)

        self.detect_objects_image = rospy.Service("lasr_perception_server/detect_objects_image", DetectImage,
                                                  self.handle_task)

    def detect_image(self, req):
        print(req.task, 'the task')
        if len(req.filter):
            return DetectImageResponse(
                [det for det in self.yolo_detect(req.image, req.dataset, req.confidence, req.nms).detected_objects if
                 det.name in req.filter])
        else:
            return DetectImageResponse(
                self.yolo_detect(req.image, req.dataset, req.confidence, req.nms).detected_objects)


    def face_detection(self, req):
        return DetectImageResponse(self.face_detect(req.image).detected_objects)

    # returns bbox of known and unknown people
    def face_and_yolo(self, req):
        print(len(req), 'hi')
        # take opencv detection
        # face_detection_resp = self.face_detect(req).detected_objects
        # take the yolo detection of people
        # yolo_resp = self.yolo_detect(req.image, req.dataset, req.confidence, req.nms).detected_objects
        #recogniser
        return DetectImageResponse(
            self.yolo_detect(req.images[0], req.dataset, req.confidence, req.nms).detected_objects)

    def handle_task(self, req):
        resp = None
        if req.task == 'open_cv':
            resp = self.face_detection(req)
        elif req.task == 'known_people':
            resp = self.face_and_yolo(req)
        else:
            resp = self.detect_image(req)
        # resp = self.detect_image(req)
        print(resp, 'printing resp')
        return resp


if __name__ == "__main__":
    rospy.init_node("lasr_perception_server")
    rospy.loginfo("initialising the perception server")
    perception_server = PerceptionServer()
    rospy.spin()
