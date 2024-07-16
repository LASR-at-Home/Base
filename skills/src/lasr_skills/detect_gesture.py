import smach
import rospy
import cv2
import cv2_img
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PointStamped
from std_msgs.msg import Header
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import tf2_ros as tf
from visualization_msgs.msg import Marker
from markers import create_and_publish_marker
from typing import Tuple, Union


class DetectGesture(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(
        self,
        gesture_to_detect: Union[str, None] = None,
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.1,
        buffer_width: int = 50,
        debug_publisher: str = "/skills/gesture_detection/debug",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "missing_keypoints", "failed"],
            input_keys=["img_msg", "detection"],
            output_keys=["gesture_detected", "person_point"],
        )
        self.gesture_to_detect = gesture_to_detect
        self.bodypix_client = rospy.ServiceProxy(
            "/bodypix/keypoint_detection", BodyPixKeypointDetection
        )
        self.bodypix_model = bodypix_model
        self.bodypix_confidence = bodypix_confidence
        self.debug_publisher = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self.buffer_width = buffer_width
        self.required_keypoints = [
            "leftWrist",
            "leftShoulder",
            "rightWrist",
            "rightShoulder",
        ]
        # publish a marker
        self.person_point_pub = rospy.Publisher("/person_point", Marker, queue_size=1)

    def execute(self, userdata):

        req = BodyPixKeypointDetectionRequest()
        req.image_raw = userdata.img_msg
        req.dataset = self.bodypix_model
        req.confidence = self.bodypix_confidence
        req.keep_out_of_bounds = False

        try:
            res = self.bodypix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        detected_keypoints = res.keypoints

        detected_gesture = "none"

        keypoint_info = {
            keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        detected_gesture = "none"

        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if (
                self.gesture_to_detect == "raising_left_arm"
                or self.gesture_to_detect is None
            ):
                if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                    detected_gesture = "raising_left_arm"
            if (
                self.gesture_to_detect == "pointing_to_the_left"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["leftWrist"]["x"] - self.buffer_width
                    > keypoint_info["leftShoulder"]["x"]
                ):
                    detected_gesture = "pointing_to_the_left"

        if (
            "rightShoulder" in keypoint_info
            and "rightWrist" in keypoint_info
            and self.gesture_to_detect is None
        ):
            print(keypoint_info["rightShoulder"]["x"], keypoint_info["rightWrist"]["x"])
            if (
                self.gesture_to_detect == "raising_right_arm"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["rightWrist"]["y"]
                    < keypoint_info["rightShoulder"]["y"]
                ):
                    detected_gesture = "raising_right_arm"
            if (
                self.gesture_to_detect == "pointing_to_the_right"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["rightShoulder"]["x"] - self.buffer_width
                    > keypoint_info["rightWrist"]["x"]
                ):
                    detected_gesture = "pointing_to_the_right"

        rospy.loginfo(f"Detected gesture: {detected_gesture}")
        userdata.gesture_detected = detected_gesture

        if not userdata.detection.point:
            # take it a meter away from the robot position if no keypoints are detected
            robot_pose = rospy.wait_for_message("/robot_pose", Pose)
            userdata.person_point = Point(
                x=robot_pose.position.x + 1, y=robot_pose.position.y
            )
        else:
            _buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
            _listener = tf.TransformListener(_buffer)
            person_pose = PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=userdata.detection.point,
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            )
            trans = _buffer.lookup_transform(
                "odom", person_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
            )
            pose = do_transform_pose(person_pose, trans)
            # userdata.person_point = pose.pose.position
            create_and_publish_marker(
                self.person_point_pub,
                PointStamped(header=pose.header, point=pose.pose.position),
                name="person_point_odom",
                r=0.0,
                g=0.0,
                b=1.0,
            )
            userdata.person_point = pose.pose.position

        cv2_gesture_img = cv2_img.msg_to_cv2_img(userdata.img_msg)
        # Add text to the image
        cv2.putText(
            cv2_gesture_img,
            detected_gesture,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        # Publish the image
        self.debug_publisher.publish(cv2_img.cv2_img_to_msg(cv2_gesture_img))

        if self.gesture_to_detect is not None:
            return (
                "succeeded" if detected_gesture == self.gesture_to_detect else "failed"
            )
        else:
            return "succeeded"
