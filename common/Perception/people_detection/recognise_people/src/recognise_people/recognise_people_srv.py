#!/usr/bin/env python3

from recognise_people.srv import RecognisePeople, RecognisePeoplePCL,\
    RecognisePeopleResponse, RecognisePeoplePCLResponse

import rospy
from cv_bridge3 import CvBridge

from lasr_perception_server.msg import Detection



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
        detection_map = {}

        # receive yolo resp
        yolo_resp = req.detected_objects_yolo
        print(yolo_resp, 'len yolo ', len(yolo_resp) )
        # receive the opencv resp
        face_detection_resp = req.detected_objects_opencv
        print(face_detection_resp, 'len face ', len(face_detection_resp))

        # if no people return if yes continue
        # assuming that there is a person/ people

        # for _ in range(4):
        #
        #     # face_detection_resp = self.face_detection(pcl_msg)
        #     # yolo_detections.extend(detect_person())
        #
        #     for detection in face_detection_resp.detections:
        #         try:
        #             n = [d.name for d in face_detections].index(detection.name)
        #             if detection.confidence > face_detections[n].confidence:
        #                 face_detections[n] = detection
        #         except ValueError:
        #             face_detections.append(detection)

        # get bb of person using face bb and body bb
        # maps name -> face(name, confidence, xywh) and yolo(name, conf, xywh)
        for face in face_detection_resp:
            for detection in yolo_resp:
                x1, y1, x2, y2 = face.bb

                head_x = (x1 + x2) / 2  # center of head in x-axis
                body_x = (detection.xywh[0] + detection.xywh[2]) / 2  # center of body in x-axis

                if abs(head_x - body_x) < 20:
                    if not face.name in detection_map.keys():
                        detection_map[face.name] = (face, detection)
                        print("yolo detection here, bb overlapped.")
                        break
                    if detection.confidence > detection_map[face.name][1].confidence:
                        detection_map[face.name] = (face, detection)
                else:
                    print(face.name, face.bb, detection.xywh)
        print(detection_map)

        response = RecognisePeopleResponse()
        if len(face_detection_resp) > len(yolo_resp) and len(face_detection_resp) > 0:
            for face in face_detection_resp:
                # Append detection.
                response.detected_objects.append(
                    Detection(
                        name=face.name,
                        confidence=face.confidence,
                        xywh=face.xywh
                    )
                )
        elif len(face_detection_resp) < 1 and len(yolo_resp) > 0:
            for person in yolo_resp:
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


        # intersect = []
        # if len(detection_map.keys()):
        #     for face_detection, yolo_detection in detection_map.values():
        #         print(f"Checking {face_detection.name}")
        #         try:
        #             n = [g.name for g in userdata.guest_list].index(face_detection.name)
        #             userdata.guest_list[n].last_known_pose = yolo_detection.centroid
        #             intersect.append(userdata.guest_list[n])
        #         except ValueError:
        #             continue
        #         print(intersect)



if __name__ == "__main__":
    rospy.init_node("recognise_people_server")
    server = RecognisePeopleServer()
    service = rospy.Service('recognise_people', RecognisePeople, server)
    rospy.loginfo("Recognise People Service initialised")
    rospy.spin()


























































    # TODO: you are given the image and you run it through the
    # self.face_detection = rospy.ServiceProxy("people_detection", FaceDetection)

    def recognise(self, req, dataset='coco', confidence=0.5, nms=0.5):
        pass
        # you are given an image
        # check for pcl
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        face_detection_response = self.face_detection(req.image,dataset, confidence, nms).detected_objects


        # you send it to the perception server
        # get bb of person using face bb and body bb
        # Create list of combinations of pairs of guests.
        # Insert the reverse of each pair, ensuring the sorting criteria is maintained.

























    def test(self, im, confidence=50):
        direction = 0.2

        face_detections = []

        yolo_detections = []

        detection_map = {}

        for _ in range(4):
            direction *= -1

            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            face_detection_resp = self.face_detection(pcl_msg)
            yolo_detections.extend(detect_person())

            for detection in face_detection_resp.detections:
                try:
                    n = [d.name for d in face_detections].index(detection.name)
                    if detection.confidence > face_detections[n].confidence:
                        face_detections[n] = detection
                except ValueError:
                    face_detections.append(detection)

        # get bb of person using face bb and body bb
        for face in face_detections:
            for detection in yolo_detections:
                x1, y1, x2, y2 = face.bb

                head_x = (x1 + x2) / 2  # center of head in x-axis
                body_x = (detection.xywh[0] + detection.xywh[2]) / 2  # center of body in x-axis

                if abs(head_x - body_x) < 20:
                    if not face.name in detection_map.keys():
                        detection_map[face.name] = (face, detection)
                        print("yolo detection here, bb overlapped.")
                        break
                    if detection.confidence > detection_map[face.name][1].confidence:
                        detection_map[face.name] = (face, detection)
                else:
                    print(face.name, face.bb, detection.xywh)
        intersect = []
        if len(detection_map.keys()):
            for face_detection, yolo_detection in detection_map.values():
                print(f"Checking {face_detection.name}")
                try:
                    n = [g.name for g in userdata.guest_list].index(face_detection.name)
                    userdata.guest_list[n].last_known_pose = yolo_detection.centroid
                    intersect.append(userdata.guest_list[n])
                except ValueError:
                    continue
                print(intersect)

                # Create list of combinations of pairs of guests.
                comb = list(itertools.combinations([g for g in intersect], 2))

                # Sort the combinations.
                comb = sorted(comb, key=lambda element: (element[0].name, element[1].name))

                # Insert the reverse of each pair, ensuring the sorting criteria is maintained.
                comb = [a for pair in [(c, c[::-1]) for c in comb] for a in pair]

                print(comb)

                return comb

























    def __call__(self, req):
        print(req)
        return RecognisePeopleResponse(self.recognise_people(req.image_raw))



if __name__ == "__main__":
    rospy.init_node("recognise_people_server")
    perception_server = RecognisePeopleServer()
    rospy.spin()
