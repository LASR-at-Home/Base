#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import PointCloud2
from tiago_controllers.utils import activate_robot_navigation
from face_detection.srv import FaceDetectionPCL


class Scan(smach.State):
    def __init__(self, head_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['prev'],
                             output_keys=['location', 'prev'])
        self.head_controller = head_controller
        rospy.wait_for_service("pcl_face_detection_server")
        self.face_detection = rospy.ServiceProxy("pcl_face_detection_server", FaceDetectionPCL)

    def execute(self, userdata):
        userdata.prev = 'Scan'

        def rotate_head(direction):
            self.head_controller.sync_reach_to(direction, 0.0, velocities=[0.1, 0.])

        # deactivate navigation on the robot to move the head around
        activate_robot_navigation(False)

        # run detection at most 4 times
        direction = 0.2
        for _ in range(4):
            direction *= -1
            rotate_head(direction)
            print(f'rotating head in direction {direction}')
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)

            resp = self.face_detection(pcl_msg)
            print(resp, "my response")
            if resp.detections:
                for detection in resp.detections:
                    # print(detection.name)
                    # if detection.name == "female_doll":
                    print(detection.centroid.point)
                    userdata.location = Pose(detection.centroid.point, Quaternion(0, 0, 0, 1))
                    return 'succeeded'
        return 'failed'
