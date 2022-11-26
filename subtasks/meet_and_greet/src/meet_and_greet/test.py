#!/usr/bin/env python3
import rospy
from tiago_controllers.base_controller import BaseController
from tiago_controllers.head_controller import HeadController
from lasr_voice.voice import Voice
from tiago_controllers.helpers import get_pose_from_param
from lasr_object_detection_yolo.person_detection import detect_person
from sensor_msgs.msg import Image, PointCloud2

from face_detection.srv import FaceDetection


class GreetKnownPeople:
    def __init__(self):
        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.head_controller.sync_reach_to(0, 0)  # start by centering the head
        # self.voice = Voice()
        self.search_points = [(-1, 0.5), (1, 0.5), (1, -0.5), (-1, -0.5), (0, 0)]
        self.face_detection = rospy.ServiceProxy("face_detection", FaceDetection)
        self.topic = '/usb_cam/image_raw'
        self.dtype = Image

    def search_for_person(self):
        face_detections = []
        yolo_detections = []
        detection_map = {}
        direction = 0.2
        for _ in range(4):
            direction *= -1
            self.head_controller.sync_reach_to(direction, -0.2, velocities=[0.1, 0.0])

            pcl_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            face_detection_resp = self.face_detection(pcl_msg)
            print(face_detection_resp)
            yolo_detections.extend(detect_person())
            print(yolo_detections)

            for detection in face_detection_resp.detected_objects:
                try:
                    n = [d.name for d in face_detections].index(detection.name)
                    if detection.confidence > face_detections[n].confidence:
                        face_detections[n] = detection
                except ValueError:
                    face_detections.append(detection)

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

        return detection_map

    def main(self):
        door_pos = get_pose_from_param('/door_simu')
        self.base_controller.sync_to_pose(door_pos)
        check_people = self.search_for_person()

        print(check_people, 'check')


if __name__ == '__main__':
    rospy.init_node("find_person_and_ask_open_door_node", anonymous=True)
    greet_people = GreetKnownPeople()
    greet_people.main()
