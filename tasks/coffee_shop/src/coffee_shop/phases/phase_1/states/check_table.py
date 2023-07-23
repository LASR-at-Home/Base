#!/usr/bin/env python3
import smach
import rospy
import rospkg
import os
import shutil
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point, Point
from visualization_msgs.msg import Marker
from cv_bridge3 import CvBridge, cv2
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_voice.voice import Voice
from pcl_segmentation.srv import SegmentCuboid, Centroid, MaskFromCuboid, SegmentBB
from common_math import pcl_msg_to_cv2
import numpy as np
from actionlib_msgs.msg import GoalStatus

OBJECTS = ["cup", "mug"]

class CheckTable(smach.State):
    def __init__(self, head_controller, voice_controller, debug=False):
        smach.State.__init__(self, outcomes=['not_finished', 'finished'])
        self.head_controller = head_controller
        self.debug = debug
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        self.detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.voice_controller = voice_controller
        self.mask_from_cuboid = rospy.ServiceProxy("/pcl_segmentation_server/mask_from_cuboid", MaskFromCuboid)
        self.segment_bb = rospy.ServiceProxy("/pcl_segmentation_server/segment_bb", SegmentBB)
        self.centroid = rospy.ServiceProxy("/pcl_segmentation_server/centroid", Centroid)
        self.bridge = CvBridge()
        self.detections_objects = []
        self.detections_people = []
        self.num_detections = 0
        self.person_location_pub = rospy.Publisher("/person_locations", Marker, queue_size=69)

    def perform_detection(self, pcl_msg, min_xyz, max_xyz, filter):
        mask = self.mask_from_cuboid(pcl_msg, Point(*min_xyz), Point(*max_xyz)).mask
        cv_im = pcl_msg_to_cv2(pcl_msg)
        cv_mask = self.bridge.imgmsg_to_cv2_np(mask)
        masked_im = cv2.bitwise_and(cv_im, cv_im, mask=cv_mask)
        #cv2.imshow("mask", masked_im)
        #cv2.waitKey(1)
        img_msg = self.bridge.cv2_to_imgmsg(masked_im)
        detections = self.detect(img_msg, "coco", 0.25, 0.3)
        detections = [det for det in detections.detected_objects if det.name in filter]
        self.num_detections += 1
        return detections, cv_im, cv2.bitwise_and(cv_im, cv_im, mask=cv_mask)

    def check_table(self, pcl_msg):
        detections_objects_, raw_im, objects_mask = self.perform_detection(pcl_msg, self.min_xyz_objects, self.max_xyz_objects, OBJECTS)
        self.detections_objects.extend(detections_objects_)
        if self.debug:
            cv2.imwrite(os.path.join(self.debug_path, f"objects_{self.num_detections}_mask.png"), objects_mask)
            cv2.imwrite(os.path.join(self.debug_path, f"objects_{self.num_detections}.png"), raw_im)
            with open(os.path.join(self.debug_path, f"objects_{self.num_detections}.txt"), "w+") as fp:
                for detection in detections_objects_:
                    fp.write(f"{detection.name}\n")

    def check_people(self, pcl_msg):
        detections_people_, raw_im, people_mask = self.perform_detection(pcl_msg, self.min_xyz_people, self.max_xyz_people, ["person"])
        self.detections_people.extend(detections_people_)
        if self.debug:
            cv2.imwrite(os.path.join(self.debug_path, f"people_{self.num_detections}_mask.png"), people_mask)
            cv2.imwrite(os.path.join(self.debug_path, f"people_{self.num_detections}.png"), raw_im)
            with open(os.path.join(self.debug_path, f"people_{self.num_detections}.txt"), "w+") as fp:
                fp.write(f"{len(detections_people_)}")

    def execute(self, userdata):
        self.current_table = rospy.get_param("current_table")
        if self.debug:
            if not os.path.exists(os.path.join(rospkg.RosPack().get_path("coffee_shop"), "debug")):
                os.mkdir(os.path.join(rospkg.RosPack().get_path("coffee_shop"), "debug"))
            self.debug_path = os.path.join(rospkg.RosPack().get_path("coffee_shop"), "debug", self.current_table)
            if os.path.exists(self.debug_path):
                shutil.rmtree(self.debug_path)
            os.mkdir(self.debug_path)
        self.num_detections = 0

        rospy.loginfo(self.current_table)
        self.min_xyz_objects, self.max_xyz_objects = rospy.get_param(f"/tables/{self.current_table}/objects_cuboid/min"), rospy.get_param(f"/tables/{self.current_table}/objects_cuboid/max")
        self.min_xyz_people, self.max_xyz_people = rospy.get_param(f"/tables/{self.current_table}/persons_cuboid/min"), rospy.get_param(f"/tables/{self.current_table}/persons_cuboid/max")
        self.detections_objects = []
        self.detections_people = []

        motions = ["back_to_default", "check_table", "check_table_low", "look_left", "look_down_left",  "look_down_right", "look_right", "back_to_default"]

        for motion in motions:
            pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.play_motion_client.send_goal_and_wait(pm_goal)
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            self.check_table(pcl_msg)
            self.check_people(pcl_msg)

        if len(self.detections_objects) > 0 and len(self.detections_people) == 0:
            status = "needs cleaning"
        elif len(self.detections_objects) > 0 and len(self.detections_people) > 0:
            status = "served"
        elif len(self.detections_objects) == 0 and len(self.detections_people) > 0:
            status = "needs serving"
        elif len(self.detections_objects) == 0 and len(self.detections_people) == 0:
            status = "ready"

        rospy.set_param(f"/tables/{self.current_table}/status/", status)
        self.voice_controller.sync_tts(f"The status of this table is {status}. There were {len(self.detections_objects)} objects and {len(self.detections_people)} people")
        return 'finished' if len([(label, table) for label, table in rospy.get_param("/tables").items() if table["status"] == "unvisited"]) == 0 else 'not_finished'
