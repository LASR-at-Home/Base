import smach_ros
from geometry_msgs.msg import PointStamped
import smach
from .vision import GetPointCloud
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
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
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point
from copy import copy

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
        def __init__(self, debug=True, filter=False):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "no_detection"],
                input_keys=[
                    "bbox_eyes",
                    "pcl_msg",
                    "masks",
                    "detections",
                    "deepface_detection",
                ],
                output_keys=["pointstamped", "bbox_eyes"],
            )
            self.DEBUG = debug
            self.img_pub = rospy.Publisher("/debug/image", Image, queue_size=1)
            self.marker_pub = rospy.Publisher("eyes", Marker, queue_size=1)
            self.look_at_pub = actionlib.SimpleActionClient(
                "/head_controller/point_head_action", PointHeadAction
            )
            self._filter = filter

        def execute(self, userdata):
            rospy.sleep(0.1)
            ######### BUG HERE REQUIRES RESOLVING ###############
            if len(userdata.bbox_eyes) == 0 and len(userdata.detections.detections) > 0:
                # userdata.pointstamped = PointStamped()
                return "succeeded"
            elif (
                len(userdata.bbox_eyes) < 1 and len(userdata.detections.detections) < 1
            ):
                return "no_detection"
            print("THE DEEPFACE")
            print(userdata.deepface_detection)

            if self._filter:
                if userdata.deepface_detection:
                    deepface = userdata.deepface_detection[0]
                    for bbox in userdata.bbox_eyes:
                        if bbox["bbox"] == deepface:
                            userdata.bbox_eyes = [bbox]
                            break
                else:
                    return "failed"

            if self._filter:
                if userdata.deepface_detection:
                    deepface = userdata.deepface_detection[0]
                    for bbox in userdata.bbox_eyes:
                        if bbox["bbox"] == deepface:
                            userdata.bbox_eyes = [bbox]
                            break
                else:
                    return "failed"

            bbox_eyes = copy(userdata.bbox_eyes)

            for det in bbox_eyes:
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
                try:
                    eye_point_pcl = pcl_xyz[int(eye_point[1]), int(eye_point[0])]
                except IndexError:
                    eye_point_pcl = pcl_xyz[
                        int(eye_point[1]) - 1, int(eye_point[0]) - 1
                    ]
                except Exception as e:
                    rospy.logerr(f"Unexpected Exception: {e}")
                    return "failed"
                if any([True for i in eye_point_pcl if i != i]):
                    eye_point_pcl = pcl_xyz[int(right_eye[1]), int(right_eye[0])]

                look_at = PointStamped()
                look_at.header = userdata.pcl_msg.header
                look_at.point.x = eye_point_pcl[0]
                look_at.point.y = eye_point_pcl[1]
                look_at.point.z = eye_point_pcl[2]

                if self.DEBUG:
                    create_and_publish_marker(self.marker_pub, look_at, r=0, g=1, b=0)

                userdata.bbox_eyes.remove(det)
                userdata.pointstamped = look_at

                self.look_at_pub.wait_for_server()
                if any(
                    [
                        True
                        for i in [look_at.point.x, look_at.point.y, look_at.point.z]
                        if i != i
                    ]
                ):
                    look_at.point.x = 0.0
                    look_at.point.y = 0.0
                    look_at.point.z = 0.0

                # goal = PointHeadGoal()
                # goal.pointing_frame = "head_2_link"
                # goal.pointing_axis = Point(1.0, 0.0, 0.0)
                # goal.max_velocity = 1.0
                # goal.target = look_at
                # rospy.loginfo(
                #     f"LOOKING AT POINT {look_at.point.x}, {look_at.point.y}, {look_at.point.z}"
                # )
                # self.look_at_pub.send_goal(goal)

                print(self.look_at_pub.get_state())

                return "succeeded"

            return "failed"

    @smach.cb_interface(
        input_keys=["keypoints", "detections"], output_keys=["bbox_eyes"]
    )
    def match_poses_and_detections(ud):
        bbox_eyes = []
        for keypoint in ud.keypoints:
            for detection in ud.detections.detections:
                temp = {
                    "bbox": detection.xywh,
                }
                if (
                    keypoint.keypoint_name == "leftEye"
                    and detection.xywh[0]
                    < keypoint.x
                    < detection.xywh[0] + detection.xywh[2]
                    and detection.xywh[1]
                    < keypoint.y
                    < detection.xywh[1] + detection.xywh[3]
                ):
                    temp["left_eye"] = keypoint.x, keypoint.y
                if (
                    keypoint.keypoint_name == "rightEye"
                    and detection.xywh[0]
                    < keypoint.x
                    < detection.xywh[0] + detection.xywh[2]
                    and detection.xywh[1]
                    < keypoint.y
                    < detection.xywh[1] + detection.xywh[3]
                ):
                    temp["right_eye"] = keypoint.x, keypoint.y

                if "left_eye" in temp and "right_eye" in temp:
                    bbox_eyes.append(temp)

        ud.bbox_eyes = bbox_eyes

        return "succeeded"

    def __init__(self, filter=False):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed", "no_detection"],
            input_keys=["pcl_msg", "deepface_detection"],
            output_keys=[],
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
                            "succeeded": "SEGMENT_PERSON",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                    )

            smach.StateMachine.add(
                "SEGMENT_PERSON",
                smach_ros.ServiceState(
                    "/bodypix/keypoint_detection",
                    BodyPixKeypointDetection,
                    request_cb=lambda ud, _: BodyPixKeypointDetectionRequest(
                        pcl_to_img_msg(ud.pcl_msg), "resnet50", 0.2
                    ),
                    response_slots=["keypoints"],
                    input_keys=["pcl_msg"],
                    output_keys=["keypoints"],
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

            smach.StateMachine.add(
                "MATCH_POSES_AND_DETECTIONS",
                CBState(
                    self.match_poses_and_detections,
                    input_keys=["keypoints", "detections"],
                    output_keys=["keypoints"],
                    outcomes=["succeeded", "failed"],
                ),
                transitions={"succeeded": "CHECK_EYES", "failed": "failed"},
                remapping={"bbox_eyes": "bbox_eyes"},
            )
            # smach.StateMachine.add(
            #     "CHECK_EYES",
            #     self.CheckEyes(self.DEBUG, filter=filter),
            #     transitions={
            #         "succeeded": "LOOK_TO_POINT",
            #         "failed": "failed",
            #         "no_detection": "no_detection",
            #     },
            #     remapping={
            #         "pcl_msg": "pcl_msg",
            #         "bbox_eyes": "bbox_eyes",
            #         "pointstamped": "pointstamped",
            #         "deepface_detection": "deepface_detection",
            #     },
            # )
            smach.StateMachine.add(
                "CHECK_EYES",
                self.CheckEyes(self.DEBUG, filter=filter),
                transitions={
                    "succeeded": "LOOK_TO_POINT",
                    "failed": "failed",
                    "no_detection": "no_detection",
                },
                remapping={
                    "pcl_msg": "pcl_msg",
                    "bbox_eyes": "bbox_eyes",
                    "pointstamped": "pointstamped",
                    "deepface_detection": "deepface_detection",
                },
            )

            smach.StateMachine.add(
                "LOOK_TO_POINT",
                LookToPoint(),
                transitions={
                    "succeeded": "LOOP",
                    "aborted": "failed",
                    "timed_out": "LOOP",
                },
                remapping={"pointstamped": "pointstamped"},
            )
            smach.StateMachine.add(
                "LOOP",
                smach.CBState(
                    lambda ud: "succeeded" if len(ud.bbox_eyes) > 0 else "finish",
                    input_keys=["bbox_eyes"],
                    output_keys=["bbox_eyes"],
                    outcomes=["succeeded", "finish"],
                ),
                transitions={
                    "succeeded": "CHECK_EYES",
                    "finish": "succeeded",
                },
            )
