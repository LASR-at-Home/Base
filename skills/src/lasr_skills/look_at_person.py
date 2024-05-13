#!/usr/bin/env python3
import smach_ros
from geometry_msgs.msg import PointStamped
import smach
from vision import GetPointCloud
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from lasr_skills import LookToPoint, DetectFaces
import cv2
import rospy
from sensor_msgs.msg import Image
from cv2_img import cv2_img_to_msg, msg_to_cv2_img
from markers import create_and_publish_marker
from visualization_msgs.msg import Marker
from cv2_pcl import pcl_to_img_msg
import ros_numpy as rnp
import rosservice
from smach import CBState
from std_msgs.msg import String
from geometry_msgs.msg import Point

PUBLIC_CONTAINER = False

try:
    from pal_startup_msgs.srv import (
        StartupStart,
        StartupStop,
        StartupStartRequest,
        StartupStopRequest,
    )
except ModuleNotFoundError:
    PUBLIC_CONTAINER = True

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal


class LookAtPerson(smach.StateMachine):
    class CheckEyes(smach.State):
        def __init__(self, debug=True):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "no_detection"],
                input_keys=["bbox_eyes", "pcl_msg", "masks", "detections"],
                output_keys=["pointstamped"],
            )
            self.DEBUG = debug
            self.img_pub = rospy.Publisher("/debug/image", Image, queue_size=1)
            self.marker_pub = rospy.Publisher("eyes", Marker, queue_size=1)
            self.look_at_pub = actionlib.SimpleActionClient(
                "/head_controller/point_head_action", PointHeadAction
            )

        def execute(self, userdata):
            rospy.loginfo("Checking eyes")
            print("bbox eyes", userdata.bbox_eyes)
            rospy.sleep(3)
            if len(userdata.bbox_eyes) < 1 and len(userdata.detections.detections) > 0:
                return "succeeded"
            elif (
                len(userdata.bbox_eyes) < 1 and len(userdata.detections.detections) < 1
            ):
                return "no_detection"

            for det in userdata.bbox_eyes:
                left_eye = det["left_eye"]
                right_eye = det["right_eye"]
                eye_point = (left_eye[0] + right_eye[0]) / 2, (
                    left_eye[1] + right_eye[1]
                ) / 2
                print("EYE POINT", eye_point)

                if self.DEBUG:
                    img = msg_to_cv2_img(pcl_to_img_msg(userdata.pcl_msg))
                    cv2.circle(img, (left_eye[0], left_eye[1]), 5, (0, 255, 0), -1)
                    cv2.circle(img, (right_eye[0], right_eye[1]), 5, (0, 255, 0), -1)
                    img_msg1 = cv2_img_to_msg(img)
                    self.img_pub.publish(img_msg1)

                pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
                    userdata.pcl_msg, remove_nans=False
                )
                eye_point_pcl = pcl_xyz[int(eye_point[1]), int(eye_point[0])]
                print("EYE POINT PCL", eye_point_pcl)
                # check if the list contains nan, if it does then take one of the eyes
                if any([True for i in eye_point_pcl if i != i]):
                    eye_point_pcl = pcl_xyz[int(left_eye[1]), int(left_eye[0])]
                    eye_point_pcl = pcl_xyz[int(right_eye[1]), int(right_eye[0])]
                    print("EYE POINT PCL", eye_point_pcl)

                look_at = PointStamped()
                look_at.header = userdata.pcl_msg.header
                look_at.point.x = eye_point_pcl[0]
                look_at.point.y = eye_point_pcl[1]
                look_at.point.z = eye_point_pcl[2]

                if self.DEBUG:
                    create_and_publish_marker(self.marker_pub, look_at, r=0, g=1, b=0)

                userdata.bbox_eyes.remove(det)
                print("LOOK AT POINT", look_at)
                userdata.pointstamped = look_at

                self.look_at_pub.wait_for_server()
                goal = PointHeadGoal()
                goal.pointing_frame = "head_2_link"
                goal.pointing_axis = Point(1.0, 0.0, 0.0)
                goal.max_velocity = 1.0
                goal.target = look_at
                self.look_at_pub.send_goal(goal)

                return "succeeded"

            return "failed"

    @smach.cb_interface(input_keys=["poses", "detections"], output_keys=["bbox_eyes"])
    def match_poses_and_detections(ud):
        # a map for bbox : list of left eye and right eye keypoints
        bbox_eyes = []
        for pose in ud.poses:
            for detection in ud.detections.detections:
                temp = {
                    "bbox": detection.xywh,
                }
                for keypoint in pose.keypoints:
                    if (
                        keypoint.part == "leftEye"
                        and detection.xywh[0]
                        < keypoint.xy[0]
                        < detection.xywh[0] + detection.xywh[2]
                        and detection.xywh[1]
                        < keypoint.xy[1]
                        < detection.xywh[1] + detection.xywh[3]
                    ):
                        print("LEFT EYE", keypoint.xy)
                        temp["left_eye"] = keypoint.xy
                    if (
                        keypoint.part == "rightEye"
                        and detection.xywh[0]
                        < keypoint.xy[0]
                        < detection.xywh[0] + detection.xywh[2]
                        and detection.xywh[1]
                        < keypoint.xy[1]
                        < detection.xywh[1] + detection.xywh[3]
                    ):
                        print("RIGHT EYE", keypoint.xy)
                        temp["right_eye"] = keypoint.xy

                    if "left_eye" in temp and "right_eye" in temp:
                        bbox_eyes.append(temp)

        print("BBOX EYES")
        print(bbox_eyes)
        ud.bbox_eyes = bbox_eyes

        return "succeeded"

    def __init__(self):
        super(LookAtPerson, self).__init__(
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["masks", "poses", "pointstamped"],
        )
        self.DEBUG = rospy.get_param("/debug", True)
        IS_SIMULATION = (
            "/pal_startup_control/start" not in rosservice.get_service_list()
        )

        with self:
            if not IS_SIMULATION:
                if PUBLIC_CONTAINER:
                    rospy.logwarn(
                        "You are using a public container. The head manager will not be stopped during navigation."
                    )
                else:
                    smach.StateMachine.add(
                        "DISABLE_HEAD_MANAGER",
                        smach_ros.ServiceState(
                            "/pal_startup_control/stop",
                            StartupStop,
                            request=StartupStopRequest("head_manager"),
                        ),
                        transitions={
                            "succeeded": "GET_IMAGE",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )
            smach.StateMachine.add(
                "GET_IMAGE",
                GetPointCloud("/xtion/depth_registered/points"),
                transitions={
                    "succeeded": "SEGMENT_PERSON",
                },
                remapping={"pcl_msg": "pcl_msg"},
            )

            eyes = BodyPixMaskRequest()
            eyes.parts = ["left_eye", "right_eye"]
            masks = [eyes]

            smach.StateMachine.add(
                "SEGMENT_PERSON",
                smach_ros.ServiceState(
                    "/bodypix/detect",
                    BodyPixDetection,
                    request_cb=lambda ud, _: BodyPixDetectionRequest(
                        pcl_to_img_msg(ud.pcl_msg), "resnet50", 0.7, masks
                    ),
                    response_slots=["masks", "poses"],
                    input_keys=["pcl_msg"],
                    output_keys=["masks", "poses"],
                ),
                transitions={
                    "succeeded": "DETECT_FACES",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "DETECT_FACES",
                DetectFaces(),
                transitions={
                    "succeeded": "MATCH_POSES_AND_DETECTIONS",
                    "failed": "failed",
                },
                remapping={"pcl_msg": "pcl_msg", "detections": "detections"},
            )

            # match the poses of the eyes and the bbox of the detections of the person
            smach.StateMachine.add(
                "MATCH_POSES_AND_DETECTIONS",
                CBState(
                    self.match_poses_and_detections,
                    input_keys=["poses", "detections"],
                    output_keys=["poses"],
                    outcomes=["succeeded", "failed"],
                ),
                transitions={"succeeded": "CHECK_EYES", "failed": "failed"},
                remapping={"bbox_eyes": "bbox_eyes"},
            )
            smach.StateMachine.add(
                "CHECK_EYES",
                self.CheckEyes(self.DEBUG),
                transitions={
                    "succeeded": "LOOP",
                    "failed": "failed",
                    "no_detection": "succeeded",
                },
                remapping={
                    "pcl_msg": "pcl_msg",
                    "bbox_eyes": "bbox_eyes",
                    "pointstamped": "pointstamped",
                },
            )

            smach.StateMachine.add(
                "LOOK_TO_POINT",
                LookToPoint(),
                transitions={
                    "succeeded": "LOOP",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"pointstamped": "pointstamped"},
            )
            # if the len of bbox_eyes is not 0 then loop back to check eyes
            smach.StateMachine.add(
                "LOOP",
                smach.CBState(
                    lambda ud: "succeeded" if len(ud.bbox_eyes) > 0 else "finish",
                    input_keys=["bbox_eyes"],
                    output_keys=["bbox_eyes"],
                    outcomes=["succeeded", "finish"],
                ),
                transitions={"succeeded": "CHECK_EYES", "finish": "succeeded"},
            )
            # TODO: do sweeping maybe with iterator

            if not IS_SIMULATION:
                if PUBLIC_CONTAINER:
                    rospy.logwarn(
                        "You are using a public container. The head manager will not be start following navigation."
                    )
                else:
                    smach.StateMachine.add(
                        "ENABLE_HEAD_MANAGER",
                        smach_ros.ServiceState(
                            "/pal_startup_control/start",
                            StartupStart,
                            request=StartupStartRequest("head_manager", ""),
                        ),
                        transitions={
                            "succeeded": "succeeded",
                            "preempted": "failed",
                            "aborted": "failed",
                        },
                    )


if __name__ == "__main__":
    rospy.init_node("look_at_person")
    sm = LookAtPerson()
    outcome = sm.execute()
    rospy.loginfo("Outcome: " + outcome)
    rospy.spin()
