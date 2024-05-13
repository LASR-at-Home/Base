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

        def execute(self, userdata):
            # check the detections from yolo and choose the bbox with the highest confidence
            # if len(userdata.detections.detections) == 0 or userdata.poses is None:
            #     return "no_detection"

            # max_confidence_det = None
            # for detection in userdata.detections.detections:
            #     if (
            #         max_confidence_det is None
            #         or detection.confidence > max_confidence_det.confidence
            #     ):
            #         max_confidence_det = detection

            # left_eye, right_eye, eye_point = [], [], []

            # for pose in userdata.poses:
            #     for keypoint in pose.keypoints:
            #         if (
            #             keypoint.part == "leftEye"
            #             and max_confidence_det.xywh[0]
            #             < keypoint.xy[0]
            #             < max_confidence_det.xywh[0] + max_confidence_det.xywh[2]
            #             and max_confidence_det.xywh[1]
            #             < keypoint.xy[1]
            #             < max_confidence_det.xywh[1] + max_confidence_det.xywh[3]
            #         ):
            #             left_eye = keypoint.xy
            #
            #         if (
            #             keypoint.part == "rightEye"
            #             and max_confidence_det.xywh[0]
            #             < keypoint.xy[0]
            #             < max_confidence_det.xywh[0] + max_confidence_det.xywh[2]
            #             and max_confidence_det.xywh[1]
            #             < keypoint.xy[1]
            #             < max_confidence_det.xywh[1] + max_confidence_det.xywh[3]
            #         ):
            #             right_eye = keypoint.xy

            # if self.DEBUG:
            #     print("Eyes found:", left_eye, right_eye)
            #     print("Number of detections:")
            #     print(len(userdata.detections.detections))
            #     img = msg_to_cv2_img(pcl_to_img_msg(userdata.pcl_msg))
            #     for det in userdata.detections.detections:
            #         cv2.rectangle(
            #             img,
            #             (det.xywh[0], det.xywh[1]),
            #             (det.xywh[0] + det.xywh[2], det.xywh[1] + det.xywh[3]),
            #             (0, 255, 0),
            #             2,
            #         )
            #         cv2.putText(
            #             img,
            #             f"{det.confidence:.2f}",
            #             (det.xywh[0], det.xywh[1]),
            #             cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5,
            #             (0, 250, 0),
            #             2,
            #         )
            #
            #     if len(left_eye) == 2:
            #         cv2.circle(img, (left_eye[0], left_eye[1]), 5, (0, 255, 0), -1)
            #     if len(right_eye) == 2:
            #         cv2.circle(img, (right_eye[0], right_eye[1]), 5, (0, 255, 0), -1)
            #
            #     img_msg1 = cv2_img_to_msg(img)
            #
            #     self.img_pub.publish(img_msg1)
            #
            # if len(left_eye) == 0 or len(right_eye) == 0:
            #     return "failed"
            #
            # if len(left_eye) == 2 and len(right_eye) == 2:
            #     eye_point = (left_eye[0] + right_eye[0]) / 2, (
            #         left_eye[1] + right_eye[1]
            #     ) / 2
            #
            # if len(left_eye) == 2 and len(right_eye) == 0:
            #     eye_point = left_eye
            #
            # elif len(left_eye) == 0 and len(right_eye) == 2:
            #     eye_point = right_eye
            #
            # pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
            #     userdata.pcl_msg, remove_nans=False
            # )
            # eye_point_pcl = pcl_xyz[int(eye_point[1]), int(eye_point[0])]
            #
            # look_at = PointStamped()
            # look_at.header = userdata.pcl_msg.header
            # look_at.point.x = eye_point_pcl[0]
            # look_at.point.y = eye_point_pcl[1]
            # look_at.point.z = eye_point_pcl[2]
            #
            # if self.DEBUG:
            #     create_and_publish_marker(self.marker_pub, look_at, r=0, g=1, b=0)
            #
            # userdata.pointstamped = look_at
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
                    "succeeded": "LOOK_TO_POINT",
                    "failed": "failed",
                    "no_detection": "CHECK_EYES",
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
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"pointstamped": "pointstamped"},
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
