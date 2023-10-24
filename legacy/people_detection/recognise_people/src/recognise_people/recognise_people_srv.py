#!/usr/bin/env python3

from recognise_people.srv import RecognisePeople,  RecognisePeopleResponse
import rospy
from cv_bridge3 import CvBridge
from lasr_vision_msgs.msg import Detection

from cv_bridge3 import cv2
import random, os, rospkg

class RecognisePeopleServer():
    """
    A Server for recognising known people using yolo and opencv
    """

    def __init__(self):

        self.face_detections = []
        self.yolo_detections = []
        self.bridge = CvBridge()


     #todo: get known people form rosparam or another funct

    def recogniser(self, req):
        response = RecognisePeopleResponse()

        self.yolo_detections = req.detected_objects_yolo
        self.face_detections = req.detected_objects_opencv
        detection_map = {}
        # print(self.face_detections[0].name)
        # print('-'*40)
        # print(self.yolo_detections[0].name)
        i = 0
        if len(self.yolo_detections) < 1 and len(self.face_detections) < 1:
            print('empty resp')
            return response

        # get bb of person using face bb and body bb
        # maps name -> face(name, confidence, xywh) and yolo(name, conf, xywh)
        for face in self.face_detections:
            for yolo in self.yolo_detections:
                x1, y1, x2, y2 = face.xywh
                print(yolo, 'the yoloooo ')

                head_x = (x1 + x2) / 2
                body_x = (yolo.xywh[0] + yolo.xywh[2]) / 2 # center of the body in the x axis
                # self.image_show(yolo.name, yolo.confidence,yolo.xywh, i)
                i = i+1

                if abs(head_x - body_x) < 20:
                    if not face.name in detection_map.keys():
                        print('detection map', face, yolo)
                        detection_map[face.name] = (face, yolo)
                        print("yolo detection here, bb overlapped.")
                        break
                    # if newer detection has bigger confidence
                    if yolo.confidence > detection_map[face.name][1].confidence:
                        detection_map[face.name] = (face, yolo)
                else:
                    print(face.name, face.xywh, yolo.xywh)

        print('-'* 40)
        print(detection_map, 'detection map\n')

        if len(self.face_detections) > len(self.yolo_detections) and len(self.face_detections) > 0:
            for face in self.face_detections:
                # Append detection
                response.detected_objects.append(
                    Detection(
                        name=face.name,
                        confidence=face.confidence,
                        xywh=face.xywh
                    )
                )
                print('the face recognitons are', face)
        elif len(self.face_detections) < 1 and len(self.yolo_detections) > 0:
            for person in self.yolo_detections:
                # Append detection.
                response.detected_objects.append(
                    Detection(
                        name=person.name,
                        confidence=person.confidence,
                        xywh=person.xywh
                    )
                )
        else:
            response = []

        print(response)
        return response

    def image_show(self, name, proba, dim, i):
        x1, y1, x2, y2 = dim
        path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")
        image = cv2.imread(path_output + "/images/random.jpg",0)
        # draw the bounding box of the face along with the associated
        # probability
        text = "{}: {:.2f}%".format(name, proba * 100)
        y = y1 - 10 if y1 - 10 > 10 else y1 + 10
        cv2.rectangle(image, (x1, y1), (x2+x1, y2+y1),
                      (0, 0, 255), 2)
        cv2.putText(image, text, (x1, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        # show the output image

        bridge = CvBridge()
        # cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv2.imwrite(path_output + "/images/random" + str(i+1)+".jpg", image)


if __name__ == "__main__":
    rospy.init_node("recognise_people_server")
    server = RecognisePeopleServer()
    service = rospy.Service('recognise_people_server', RecognisePeople, server.recogniser)
    rospy.loginfo("Recognise People Service initialised")
    rospy.spin()
