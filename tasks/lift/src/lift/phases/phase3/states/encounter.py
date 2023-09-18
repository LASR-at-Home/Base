#!/usr/bin/env python3
import os
import smach
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice
import rospy
from object_interest_tracking.srv import  Tdr
from math import acos
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug, estimate_pose

HORIZONTAL = 0.8
VERTICAL = 0.3
class Encounter(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])

        self.default = default

        # stop head manager
        if "/pal_startup_control/stop" in rosservice.get_service_list():
            self.stop_head_manager = rospy.ServiceProxy("/pal_startup_control/stop", StartupStop)
            self.start_head_manager = rospy.ServiceProxy("/pal_startup_control/start", StartupStart)


    def toNPArray(self, a):
        print(a)
        print(type(a))
        return np.array(a)


    def orderIndices(self, current, previous):
        if len(current) == 1 and len(previous) == 1:
            return [0]

        indices = []
        if len(current) == len(previous):
            for person_i in range(len(current)):
                diffs = list(map(lambda lo: np.linalg.norm(np.array(current[person_i]) - np.array(lo)), previous))
                indices.append(diffs.index(min(diffs)))

        print("indices")
        print(indices)
        return indices


    def getPoseDiff(self,lastRecorded, x, y):
        distances = []
        for i in lastRecorded:
            distances.append(np.linalg.norm(self.toNPArray(i) - np.array([x, y])))
        return distances

    def execute(self, userdata):
        # result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift/pose'))
        # get the point from zoe

        self.stop_head_manager()

        self.default.voice.speak("Hi")
        # rospy.wait_for_service('/v2')
        # rospy.ServiceProxy('/v2', Tdr)()
        # self.default.voice.speak("hi, i need to speak with you")
        # self.default.voice.speak("Hi mate, nice to meet you, you Look great today. I have just arrived at the second floor.")

        pose = get_pose_from_param("/phase3_lift/pose")
        polygon = rospy.get_param("/corners_arena")
        headPoint = None
        found = False
        ANGLE_THRESHOLD = 1
        REACHING_THRESHOLDS = 2

        while not found:
            rospy.sleep(2)
            locs = []
            for i in range(2):
                img_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
                pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

                detections, im = perform_detection(self.default, pcl_msg, polygon, ['person'])
                print(detections)


                # currentLocs = list(filter(lambda i: i.name == "person",
                #                           objectRecognitionService(img_msg, 'yolov8n-seg.pt', 0.7, 0.3).detected_objects))

                rospy.logwarn("0")

                # cm = []
                # for i2 in range(len(currentLocs)):
                #     # Assume 2 ppl!
                #     pose = estimate_pose(self.default.tf, self.default.bridge.imgmsg_to_cv2_np(img_msg), detections)
                #     if (shapley.is_point_in_polygon_2d(corners, pose.x, pose.y)):
                #         cm.append(pose)
                pos_people = []
                for j, person in detections:
                    person = person.tolist()
                    pos_people.append([person[0], person[1]])
            # end

                locs.append(pos_people)
                rospy.logwarn(len(locs[0]))

                print("vecs11")

                if i == 1 and len(locs[0]) > 0 and len(locs[0]) == len(locs[1]):
                    # swap if needed
                    match_indices = self.orderIndices(locs[1], locs[0])
                    print("-----MATCH")
                    print(match_indices)

                    if not (match_indices == [0]):
                        locs[1] = [locs[1][match_i] for match_i in match_indices]

                    # CALC VECS
                    vectors = []
                    for loc_vect in range(len(locs[1])):
                        rospy.logerr(locs[1][loc_vect])
                        print("hello")
                        print(locs[1][loc_vect])
                        a2 = self.toNPArray(locs[1][loc_vect]) - self.toNPArray(locs[0][loc_vect])
                        vectors.append(a2)
                    print("vecs")

                    print(vectors)
                    print(len(vectors))

                    # CALC ANGLES
                    [x, y, q] = self.default.controllers.base_controller.get_current_pose()

                    angles = []
                    for vec_i in range(len(vectors)):
                        print("---")
                        print(np.array([x, y]))
                        print(np.linalg.norm(np.array(vectors[vec_i])) * np.linalg.norm(np.array([x, y])))
                        angles.append(
                            acos(
                                (np.dot(np.array(vectors[vec_i]) ,  np.array([x, y])))
                             / (
                                        np.linalg.norm(np.array(vectors[vec_i])) * np.linalg.norm(np.array([x, y]))
                                )
                            )
                        )


                    distances = self.getPoseDiff(locs[1], x, y)
                    rospy.logwarn("dis ANG")
                    rospy.logwarn(min(angles))
                    rospy.logwarn(distances[angles.index(min(angles))])

                    # FACE PERSON
                    if min(angles) < ANGLE_THRESHOLD and distances[angles.index(min(angles))] < REACHING_THRESHOLDS:
                        headPoint = locs[angles.index(min(angles))]
                        found = True

        print(headPoint)
        self.default.controllers.base_controller.sync_face_to(headPoint[0][0], headPoint[0][1])

        return 'success'


if __name__ == '__main__':
    rospy.init_node("encounter", anonymous=True)
