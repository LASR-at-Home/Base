import smach
import rospy
import cv2
import cv2_img
from lasr_vision_msgs.srv import (
    YoloPoseDetection,
    YoloPoseDetectionRequest,
)
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PointStamped
from std_msgs.msg import Header
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import tf2_ros as tf
from visualization_msgs.msg import Marker
from markers import create_and_publish_marker

from typing import Union


class DetectGesture(smach.State):
    """
    State for detecting gestures using YOLO pose detection.
    Notice that the input image will probably needed to be cropped or masked to keep only one person in the image.
    """

    def __init__(
            self,
            gesture_to_detect: Union[str, None] = None,
            yolo_model: str = "yolov8n-pose.pt",
            yolo_confidence: float = 0.5,
            buffer_width: int = 50,
            debug_publisher: str = "/skills/gesture_detection/debug",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detected_gesture"],
        )
        self.gesture_to_detect = gesture_to_detect

        # Initialize YOLO pose detection service client
        self.yolo_client = rospy.ServiceProxy("/yolo/detect_pose", YoloPoseDetection)

        self.yolo_model = yolo_model
        self.yolo_confidence = yolo_confidence
        self.debug_publisher = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self.buffer_width = buffer_width

        # YOLO keypoint names mapping (different from BodyPix)
        self.required_keypoints = [
            "left_wrist",
            "left_shoulder",
            "right_wrist",
            "right_shoulder",
        ]

        # Publish a marker for person detection
        self.person_point_pub = rospy.Publisher("/person_point", Marker, queue_size=1)

    def execute(self, userdata):
        # Prepare YOLO pose detection request
        req = YoloPoseDetectionRequest()
        req.image_raw = userdata.img_msg
        req.model = self.yolo_model
        req.confidence = self.yolo_confidence

        try:
            res = self.yolo_client(req)
        except Exception as e:
            rospy.logerr(f"YOLO service call failed: {e}")
            return "failed"

        # Check if any person detected
        if not res.detections:
            rospy.logwarn("No person detected in the image")
            userdata.detected_gesture = "none"
            return "failed"

        # Use the first detected person (assuming single person scenario)
        detected_keypoints = res.detections[0].keypoints

        detected_gesture = "none"

        # Convert keypoints to dictionary for easier access
        keypoint_info = {
            keypoint.name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        # Gesture detection logic using YOLO keypoint names

        # Raising left arm - check if left wrist is above left shoulder
        if "left_shoulder" in keypoint_info and "left_wrist" in keypoint_info:
            if keypoint_info["left_wrist"]["y"] < keypoint_info["left_shoulder"]["y"]:
                detected_gesture = "raising_left_arm"

        # Pointing to the left - check if left wrist is significantly to the right of left shoulder
        if "left_shoulder" in keypoint_info and "left_wrist" in keypoint_info:
            if (
                    keypoint_info["left_wrist"]["x"] - self.buffer_width
                    > keypoint_info["left_shoulder"]["x"]
            ):
                detected_gesture = "pointing_to_the_left"

        # Raising right arm - check if right wrist is above right shoulder
        if "right_shoulder" in keypoint_info and "right_wrist" in keypoint_info:
            if keypoint_info["right_wrist"]["y"] < keypoint_info["right_shoulder"]["y"]:
                detected_gesture = "raising_right_arm"

        # Pointing to the right - check if right wrist is significantly to the left of right shoulder
        if "right_shoulder" in keypoint_info and "right_wrist" in keypoint_info:
            if (
                    keypoint_info["right_shoulder"]["x"] - self.buffer_width
                    > keypoint_info["right_wrist"]["x"]
            ):
                detected_gesture = "pointing_to_the_right"

        # Handle waving gesture (combination of raising arms)
        if self.gesture_to_detect == "waving":
            if detected_gesture in ["raising_left_arm", "raising_right_arm"]:
                detected_gesture = "waving"

        rospy.loginfo(f"Detected gesture: {detected_gesture}")
        userdata.detected_gesture = detected_gesture

        # Create debug image with gesture annotation
        cv2_gesture_img = cv2_img.msg_to_cv2_img(userdata.img_msg)

        # Draw keypoints on the image for debugging
        self._draw_keypoints(cv2_gesture_img, keypoint_info)

        # Add gesture text to the image
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

        # Publish the debug image
        self.debug_publisher.publish(cv2_img.cv2_img_to_msg(cv2_gesture_img))

        # Return success based on whether target gesture was detected
        if self.gesture_to_detect is not None:
            return (
                "succeeded" if detected_gesture == self.gesture_to_detect else "failed"
            )
        else:
            return "succeeded"
