#!/usr/bin/env python3
import smach
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import PointCloud2, Image
import cv2
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_voice.voice import Voice

OBJECTS = ["cup", "mug"]

class CheckTable(smach.State):
    def __init__(self, head_controller):
        smach.State.__init__(self, outcomes=['not_finished', 'finished'])
        self.head_controller = head_controller
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
        self.voice = Voice()
        self.voice.sync_tts("I am checking the table")
    def execute(self, userdata):

        """
        Look down at table, segment pointcloud by the defined cuboid and then perform object detection.
        Then return to home and look left and right, looking for people in the other defined cuboid
        """
        current_table = rospy.get_param("current_table")
        # min_xyz, max_xyz = rospy.get_param(f"/tables/{current_table}/objects_cuboid/min"), rospy.get_param(f"/tables/{current_table}/objects_cuboid/max")
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        pm_goal = PlayMotionGoal(motion_name="check_table", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)

        """
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        segmented = pcl_segment_cuboid_to_frame(pcl, min_xyz, max_xyz)
        cv2.imshow("segmented", segmented)
        cv2.waitKey(0)
        """
        im = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        detections = self.detect(im, "coco", 0.7, 0.3)
        detections = [det for det in detections.detected_objects if det.name in OBJECTS]
        print(detections)
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)

        status = "needs cleaning" if len(detections) > 0 else "ready"
        rospy.set_param(f"/tables/{current_table}/status/", status)
        self.voice.sync_tts("The status of this table is")
        self.voice.sync_tts(str(status))
        return 'finished' if len([(label, table) for label, table in rospy.get_param("/tables").items() if table["status"] == "unvisited"]) == 0 else 'not_finished'
