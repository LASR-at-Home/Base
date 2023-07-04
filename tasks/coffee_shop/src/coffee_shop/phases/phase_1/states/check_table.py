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
OBJECTS = ["cup", "mug"]

class CheckTable(smach.State):
    def __init__(self, head_controller, voice_controller):
        smach.State.__init__(self, outcomes=['not_finished', 'finished'])
        self.head_controller = head_controller
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.voice_controller = voice_controller
        self.mask_from_cuboid = rospy.ServiceProxy("/pcl_segmentation_server/segment_cuboid", SegmentCuboid)
        self.bridge = CvBridge()

    def perform_detection(self, min_xyz, max_xyz):
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        mask = self.mask_from_cuboid(pcl_msg, Point(*min_xyz), Point(*max_xyz)).mask
        cv_im = pcl_msg_to_cv2(pcl_msg)
        cv_mask = self.bridge.imgmsg_to_cv2_np(mask)
        
        img_msg = self.bridge.cv2_to_imgmsg(cv2.bitwise_and(cv_im, cv_im, mask=cv_mask))
        detections = self.detect(img_msg, "coco", 0.5, 0.3)
        detections = [det for det in detections.detected_objects if det.name in OBJECTS]
        return detections

    def execute(self, userdata):

        current_table = rospy.get_param("current_table")
        min_xyz, max_xyz = rospy.get_param(f"/tables/{current_table}/objects_cuboid/min"), rospy.get_param(f"/tables/{current_table}/objects_cuboid/max")
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        

        pm_goal = PlayMotionGoal(motion_name="check_table", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)

        detections = self.perform_detection(min_xyz, max_xyz)
        pm_goal = PlayMotionGoal(motion_name="look_down_left", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)
        detections.extend(self.perform_detection(min_xyz, max_xyz))

        pm_goal = PlayMotionGoal(motion_name="look_down_right", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)
        detections.extend(self.perform_detection(min_xyz, max_xyz))

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)

        status = "needs cleaning" if len(detections) > 0 else "ready"
        rospy.set_param(f"/tables/{current_table}/status/", status)
        self.voice_controller.sync_tts("The status of this table is")
        self.voice_controller.sync_tts(str(status))
        return 'finished' if len([(label, table) for label, table in rospy.get_param("/tables").items() if table["status"] == "unvisited"]) == 0 else 'not_finished'
