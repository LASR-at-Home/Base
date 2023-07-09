#!/usr/bin/env python3
import smach
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point
from cv_bridge3 import CvBridge, cv2
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_voice.voice import Voice
from pcl_segmentation.srv import SegmentCuboid
from common_math import pcl_msg_to_cv2
import numpy as np
OBJECTS = ["cup", "mug"]

class CheckTable(smach.State):
    def __init__(self, head_controller, voice_controller):
        smach.State.__init__(self, outcomes=['not_finished', 'finished'])
        self.head_controller = head_controller
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        self.detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.voice_controller = voice_controller
        self.mask_from_cuboid = rospy.ServiceProxy("/pcl_segmentation_server/segment_cuboid", SegmentCuboid)
        self.bridge = CvBridge()
        self.detections_objects = []
        self.detections_people = []
        

    def perform_detection(self, pcl_msg, min_xyz, max_xyz, filter):
        mask = self.mask_from_cuboid(pcl_msg, Point(*min_xyz), Point(*max_xyz)).mask
        cv_im = pcl_msg_to_cv2(pcl_msg)
        cv_mask = self.bridge.imgmsg_to_cv2_np(mask)
        img_msg = self.bridge.cv2_to_imgmsg(cv2.bitwise_and(cv_im, cv_im, mask=cv_mask))
        detections = self.detect(img_msg, "coco", 0.5, 0.3)
        detections = [det for det in detections.detected_objects if det.name in filter]
        return detections, cv2.bitwise_and(cv_im, cv_im, mask=cv_mask)

    def check(self, pcl_msg):
        detections_objects_, objects_mask = self.perform_detection(pcl_msg, self.min_xyz_objects, self.max_xyz_objects, OBJECTS)
        detections_people_, people_mask = self.perform_detection(pcl_msg, self.min_xyz_people, self.max_xyz_people, ["person"])
        self.detections_objects.extend(detections_objects_)
        self.detections_people.extend(detections_people_)
        #stack = np.hstack((objects_mask, people_mask))
        #cv2.imshow("Masks", stack)
        #cv2.waitKey(1)

    def execute(self, userdata):
        current_table = rospy.get_param("current_table")
        rospy.loginfo(current_table)
        self.min_xyz_objects, self.max_xyz_objects = rospy.get_param(f"/tables/{current_table}/objects_cuboid/min"), rospy.get_param(f"/tables/{current_table}/objects_cuboid/max")
        self.min_xyz_people, self.max_xyz_people = rospy.get_param(f"/tables/{current_table}/persons_cuboid/min"), rospy.get_param(f"/tables/{current_table}/persons_cuboid/max")
        self.detections_objects = []
        self.detections_people = []
        self.pcl_sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.check)

        motions = ["check_table", "check_table_low", "look_left", "look_down_left", "look_right",  "look_down_right", "back_to_default"]

        for motion in motions:
            pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.play_motion_client.send_goal_and_wait(pm_goal)
            rospy.sleep(1)

        self.pcl_sub.unregister()

        if len(self.detections_objects) > 0 and len(self.detections_people) == 0:
            status = "needs cleaning"
        elif len(self.detections_objects) > 0 and len(self.detections_people) > 0:
            status = "served"
        elif len(self.detections_objects) == 0 and len(self.detections_people) > 0:
            status = "needs serving"
        elif len(self.detections_objects) == 0 and len(self.detections_people) == 0:
            status = "ready"

        rospy.set_param(f"/tables/{current_table}/status/", status)
        self.voice_controller.sync_tts("The status of this table is")
        self.voice_controller.sync_tts(str(status))
        cv2.destroyAllWindows()
        return 'finished' if len([(label, table) for label, table in rospy.get_param("/tables").items() if table["status"] == "unvisited"]) == 0 else 'not_finished'
