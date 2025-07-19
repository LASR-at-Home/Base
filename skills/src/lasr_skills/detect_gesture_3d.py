import smach
import rospy
import cv2
import cv2_img
import numpy as np
from lasr_vision_msgs.srv import (
    YoloPoseDetection3D,
    YoloPoseDetection3DRequest,
)
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PointStamped
from std_msgs.msg import Header
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import tf2_ros as tf
from visualization_msgs.msg import Marker
from markers import create_and_publish_marker
from lasr_skills.vision import GetImageAndDepthImage
from lasr_vision_msgs.msg import Detection3D

from typing import Union, List, Dict


class DetectGesture3D(smach.State):
    """
    State for detecting gestures using YOLO 3D pose detection.
    Detects multiple people and returns a list sorted by distance (far to near).
    Each person's center point is calculated using median of valid keypoints.
    """

    def __init__(
        self,
        yolo_model: str = "yolo11n-pose.pt",
        yolo_confidence: float = 0.5,
        buffer_width: int = 50,
        target_frame: str = "base_footprint",
        debug_publisher: str = "/skills/gesture_detection_3d/debug",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg", "depth_msg", "camera_info"],
            output_keys=["detected_people"],  # List of dictionaries
        )

        # Initialize YOLO 3D pose detection service client
        self.yolo_client = rospy.ServiceProxy(
            "/yolo/detect3d_pose", YoloPoseDetection3D
        )

        self.yolo_model = yolo_model
        self.yolo_confidence = yolo_confidence
        self.debug_publisher = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self.buffer_width = buffer_width
        self.target_frame = target_frame

        # Required keypoints for gesture detection
        self.required_keypoints = [
            "left_wrist",
            "left_shoulder",
            "right_wrist",
            "right_shoulder",
        ]

        # TF buffer for coordinate transformations
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Publisher for person markers
        self.person_markers_pub = rospy.Publisher(
            "/detected_people_markers", Marker, queue_size=10
        )

        rospy.loginfo("DetectGesture3D initialized")

    def execute(self, userdata):
        """
        Main execution function that processes the input image and depth data
        to detect people and their gestures in 3D space.
        """

        # Prepare YOLO 3D pose detection request
        req = YoloPoseDetection3DRequest()
        req.image_raw = userdata.img_msg
        req.depth_image = userdata.depth_msg
        req.depth_camera_info = userdata.camera_info
        req.model = self.yolo_model
        req.confidence = self.yolo_confidence
        req.target_frame = self.target_frame

        try:
            res = self.yolo_client(req)
        except Exception as e:
            rospy.logerr(f"YOLO 3D service call failed: {e}")
            return "failed"

        # Check if any people detected
        if not res.detections:
            rospy.logwarn("No people detected in the image")
            userdata.detected_people = []
            return "succeeded"

        detected_people = []

        # Process each detected person
        for person_idx, detected_keypoints in enumerate(res.detections):

            # Convert keypoints to dictionary for easier access
            keypoint_dict = {
                keypoint.keypoint_name: keypoint.point
                for keypoint in detected_keypoints.keypoints
            }

            # Calculate center point using median of all valid keypoints
            center_point = self._calculate_center_point(keypoint_dict)
            if center_point is None:
                rospy.logwarn(
                    f"Could not calculate center point for person {person_idx}"
                )
                continue

            # Calculate distance from robot base (assuming base_footprint at origin)
            distance = np.sqrt(
                center_point.x**2 + center_point.y**2 + center_point.z**2
            )

            # Detect gesture for this person
            detected_gesture = self._detect_person_gesture(keypoint_dict)

            # Create Pose object for medium point
            medium_pose = Pose()
            medium_pose.position = center_point
            medium_pose.orientation = Quaternion(
                0.0, 0.0, 0.0, 1.0
            )  # Default orientation

            # Create person dictionary
            person = {
                "medium_point": medium_pose,
                "gesture": detected_gesture,
                "distance": distance,
            }
            detected_people.append(person)

            rospy.loginfo(
                f"Person {person_idx}: gesture={detected_gesture}, "
                f"distance={distance:.2f}m, position=({center_point.x:.2f}, "
                f"{center_point.y:.2f}, {center_point.z:.2f})"
            )

        # Sort people by distance (far to near)
        detected_people.sort(key=lambda p: p["distance"], reverse=True)

        # Store results
        userdata.detected_people = detected_people

        # Publish debug visualization
        self._publish_debug_info(userdata.img_msg, detected_people)
        self._publish_person_markers(detected_people)

        rospy.loginfo(f"Successfully detected {len(detected_people)} people")
        return "succeeded"

    def _calculate_center_point(
        self, keypoint_dict: Dict[str, Point]
    ) -> Union[Point, None]:
        """
        Calculate the center point of a person using median of all valid keypoints.

        Args:
            keypoint_dict: Dictionary mapping keypoint names to 3D points

        Returns:
            Point representing the center of the person, or None if insufficient keypoints
        """

        # Collect all valid 3D points
        valid_points = []
        for name, point in keypoint_dict.items():
            # Check if point has valid coordinates (not NaN or zero depth)
            if not (
                np.isnan(point.x)
                or np.isnan(point.y)
                or np.isnan(point.z)
                or point.z <= 0
            ):
                valid_points.append([point.x, point.y, point.z])

        if (
            len(valid_points) < 0
        ):  # Need at least 3 points for reliable center calculation
            return None

        if len(valid_points) > 1:
            # Calculate median position
            points_array = np.array(valid_points)
            median_x = np.median(points_array[:, 0])
            median_y = np.median(points_array[:, 1])
            median_z = np.median(points_array[:, 2])
        elif len(valid_points) == 1:
            median_x, median_y, median_z = valid_points[0]
        else:
            return None
        return Point(median_x, median_y, median_z)

    def _detect_person_gesture(self, keypoint_dict: Dict[str, Point]) -> str:
        """
        Detect gesture for a single person based on their 3D keypoints.

        Args:
            keypoint_dict: Dictionary mapping keypoint names to 3D points

        Returns:
            String representing the detected gesture
        """

        detected_gesture = "none"

        # Check if required keypoints are available and valid
        required_points = {}
        for kp_name in self.required_keypoints:
            if kp_name in keypoint_dict:
                point = keypoint_dict[kp_name]
                if not (
                    np.isnan(point.x)
                    or np.isnan(point.y)
                    or np.isnan(point.z)
                    or point.z <= 0
                ):
                    required_points[kp_name] = point

        # Gesture detection logic using 3D coordinates

        # Raising left arm - check if left wrist is above left shoulder (higher z in base_footprint)
        if "left_shoulder" in required_points and "left_wrist" in required_points:
            if required_points["left_wrist"].z > required_points["left_shoulder"].z:
                detected_gesture = "raising_left_arm"

        # Raising right arm - check if right wrist is above right shoulder
        if "right_shoulder" in required_points and "right_wrist" in required_points:
            if required_points["right_wrist"].z > required_points["right_shoulder"].z:
                detected_gesture = "raising_right_arm"

        # we will not do the rest two for now, since we really just care about raising the arms
        # # Pointing to the left - check if left wrist is significantly to the left of left shoulder
        # if "left_shoulder" in required_points and "left_wrist" in required_points:
        #     left_offset = required_points["left_wrist"].y - required_points["left_shoulder"].y
        #     if left_offset > self.buffer_width / 1000.0:  # Convert buffer_width to meters
        #         detected_gesture = "pointing_to_the_left"
        #
        # # Pointing to the right - check if right wrist is significantly to the right of right shoulder
        # if "right_shoulder" in required_points and "right_wrist" in required_points:
        #     right_offset = required_points["right_shoulder"].y - required_points["right_wrist"].y
        #     if right_offset > self.buffer_width / 1000.0:  # Convert buffer_width to meters
        #         detected_gesture = "pointing_to_the_right"
        #
        # # Waving gesture - check if either arm is raised
        # if detected_gesture in ["raising_left_arm", "raising_right_arm"]:
        #     # Could add temporal logic here for true waving detection
        #     # For now, treat raised arms as potential waving
        #     pass

        return detected_gesture

    def _publish_debug_info(self, img_msg: Image, detected_people: List[Dict]):
        """
        Publish debug image with gesture annotations.

        Args:
            img_msg: Original input image message
            detected_people: List of detected people dictionaries
        """

        # Convert image for annotation
        img = cv2_img.msg_to_cv2_img(img_msg)

        # Add text annotations for each person
        for i, person in enumerate(detected_people):
            text = f"Person {i + 1}: {person['gesture']} ({person['distance']:.1f}m)"
            y_pos = 30 + i * 25
            cv2.putText(
                img,
                text,
                (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # Publish the debug image
        self.debug_publisher.publish(cv2_img.cv2_img_to_msg(img))

    def _publish_person_markers(self, detected_people: List[Dict]):
        """
        Publish visualization markers for detected people.

        Args:
            detected_people: List of detected people dictionaries
        """

        for i, person in enumerate(detected_people):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position from Pose
            marker.pose = person["medium_point"]

            # Set size
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Set color based on gesture
            if person["gesture"] == "raising_left_arm":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif person["gesture"] == "raising_right_arm":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif person["gesture"] in ["pointing_to_the_left", "pointing_to_the_right"]:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5

            marker.color.a = 1.0

            # Add text with gesture and distance info
            marker.text = f"{person['gesture']}\n{person['distance']:.1f}m"

            self.person_markers_pub.publish(marker)


class DetectHandUp3D(DetectGesture3D):
    """
    State for detecting hand-up gestures using YOLO 3D pose detection.
    Inherits from DetectGesture3D but returns Detection3D messages instead of dictionaries.
    """

    def __init__(
        self,
        yolo_model: str = "yolo11n-pose.pt",
        yolo_confidence: float = 0.5,
        buffer_width: int = 50,
        target_frame: str = "base_footprint",
        debug_publisher: str = "/skills/hand_up_detection_3d/debug",
    ):
        # Initialize parent class
        super().__init__(
            yolo_model=yolo_model,
            yolo_confidence=yolo_confidence,
            buffer_width=buffer_width,
            target_frame=target_frame,
            debug_publisher=debug_publisher,
        )

        # Override state definition for different output
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg", "depth_msg", "camera_info"],
            output_keys=["detected_people"],  # List of Detection3D objects
        )

    def execute(self, userdata):
        """
        Main execution function that processes the input image and depth data
        to detect people with hand-up gestures and returns Detection3D messages.
        """

        # Prepare YOLO 3D pose detection request
        req = YoloPoseDetection3DRequest()
        req.image_raw = userdata.img_msg
        req.depth_image = userdata.depth_msg
        req.depth_camera_info = userdata.camera_info
        req.model = self.yolo_model
        req.confidence = self.yolo_confidence
        req.target_frame = self.target_frame

        try:
            res = self.yolo_client(req)
        except Exception as e:
            rospy.logerr(f"YOLO 3D service call failed: {e}")
            return "failed"

        # Check if any people detected
        if not res.detections:
            rospy.logwarn("No people detected in the image")
            userdata.detected_people = []
            return "succeeded"

        detected_people = []

        # Process each detected person
        for person_idx, detected_keypoints in enumerate(res.detections):

            # Convert keypoints to dictionary for easier access
            keypoint_dict = {
                keypoint.keypoint_name: keypoint.point
                for keypoint in detected_keypoints.keypoints
            }

            # Calculate center point using median of all valid keypoints
            center_point = self._calculate_center_point(keypoint_dict)
            if center_point is None:
                rospy.logwarn(
                    f"Could not calculate center point for person {person_idx}"
                )
                continue

            # Calculate distance from robot base
            distance = np.sqrt(
                center_point.x**2 + center_point.y**2 + center_point.z**2
            )

            # Detect gesture for this person
            detected_gesture = self._detect_person_gesture(keypoint_dict)

            # Only process people with hand-up gestures
            if detected_gesture in ["raising_left_arm", "raising_right_arm"]:
                # Calculate confidence based on wrist-shoulder relative distance
                confidence = self._calculate_hand_up_confidence(
                    keypoint_dict, detected_gesture
                )

                # Create Detection3D object
                detection = Detection3D()
                detection.name = f"{distance:.2f}m"  # Distance as string
                detection.confidence = confidence
                detection.xywh = []  # Empty bounding box
                detection.xyseg = []  # Empty segmentation mask
                detection.point = center_point

                detected_people.append(detection)

                rospy.loginfo(
                    f"Hand-up person {person_idx}: gesture={detected_gesture}, "
                    f"confidence={confidence:.3f}, distance={distance:.2f}m, "
                    f"position=({center_point.x:.2f}, {center_point.y:.2f}, {center_point.z:.2f})"
                )

        # Sort people by distance (far to near)
        detected_people.sort(
            key=lambda d: float(d.name.replace("m", "")), reverse=False
        )

        # Store results
        userdata.detected_people = detected_people

        # Publish debug visualization (modified for Detection3D)
        self._publish_debug_info_detection3d(userdata.img_msg, detected_people)
        self._publish_person_markers_detection3d(detected_people)

        rospy.loginfo(
            f"Successfully detected {len(detected_people)} people with hand-up gestures"
        )
        return "succeeded"

    def _calculate_hand_up_confidence(
        self, keypoint_dict: Dict[str, Point], gesture: str
    ) -> float:
        """
        Calculate confidence based on wrist-shoulder relative distance and shoulder height.

        Args:
            keypoint_dict: Dictionary mapping keypoint names to 3D points
            gesture: The detected gesture ("raising_left_arm" or "raising_right_arm")

        Returns:
            Float confidence value between 0 and 1
        """

        if gesture == "raising_left_arm":
            wrist_key = "left_wrist"
            shoulder_key = "left_shoulder"
        elif gesture == "raising_right_arm":
            wrist_key = "right_wrist"
            shoulder_key = "right_shoulder"
        else:
            return 0.0

        # Check if required keypoints are available
        if wrist_key not in keypoint_dict or shoulder_key not in keypoint_dict:
            return 0.0

        wrist_point = keypoint_dict[wrist_key]
        shoulder_point = keypoint_dict[shoulder_key]

        # Validate points
        if (
            np.isnan(wrist_point.x)
            or np.isnan(wrist_point.y)
            or np.isnan(wrist_point.z)
            or np.isnan(shoulder_point.x)
            or np.isnan(shoulder_point.y)
            or np.isnan(shoulder_point.z)
            or wrist_point.z <= 0
            or shoulder_point.z <= 0
        ):
            return 0.0

        # Calculate vertical distance (wrist above shoulder)
        vertical_distance = wrist_point.z - shoulder_point.z

        # Use shoulder height as reference (assume shoulder is at reasonable height)
        # Normalize by shoulder height from ground (approximate)
        shoulder_height = shoulder_point.z

        if shoulder_height <= 0:
            return 0.0

        # Calculate confidence as ratio of vertical lift to shoulder height
        # Clamp between 0 and 1
        confidence = min(max(vertical_distance / shoulder_height, 0.0), 1.0)

        return confidence

    def _publish_debug_info_detection3d(
        self, img_msg: Image, detected_people: List[Detection3D]
    ):
        """
        Publish debug image with hand-up gesture annotations.

        Args:
            img_msg: Original input image message
            detected_people: List of Detection3D objects
        """

        # Convert image for annotation
        img = cv2_img.msg_to_cv2_img(img_msg)

        # Add text annotations for each person
        for i, detection in enumerate(detected_people):
            text = f"Hand-up {i + 1}: conf={detection.confidence:.2f} dist={detection.name}"
            y_pos = 30 + i * 25
            cv2.putText(
                img,
                text,
                (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # Publish the debug image
        self.debug_publisher.publish(cv2_img.cv2_img_to_msg(img))

    def _publish_person_markers_detection3d(self, detected_people: List[Detection3D]):
        """
        Publish visualization markers for detected hand-up people.

        Args:
            detected_people: List of Detection3D objects
        """

        for i, detection in enumerate(detected_people):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position from Detection3D point
            marker.pose.position = detection.point
            marker.pose.orientation.w = 1.0

            # Set size based on confidence
            marker.scale.x = 0.1 + detection.confidence * 0.2  # Scale with confidence
            marker.scale.y = 0.1 + detection.confidence * 0.2
            marker.scale.z = 0.1 + detection.confidence * 0.2

            # Set color based on confidence (green for high confidence)
            marker.color.r = 1.0 - detection.confidence
            marker.color.g = detection.confidence
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add text with confidence and distance info
            marker.text = (
                f"Hand-up\nconf: {detection.confidence:.2f}\ndist: {detection.name}"
            )

            self.person_markers_pub.publish(marker)


if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("test_gesture_detection_3d")

    # Create a simple state machine for testing both detection states
    sm = smach.StateMachine(outcomes=["finished"])

    with sm:
        # Add states to the state machine
        smach.StateMachine.add(
            "GET_CAMERA_DATA",
            GetImageAndDepthImage(camera="xtion"),
            transitions={
                "succeeded": "DETECT_GESTURES",
                "failed": "GET_CAMERA_DATA",  # Retry on failure
            },
        )

        smach.StateMachine.add(
            "DETECT_GESTURES",
            DetectGesture3D(
                yolo_confidence=0.3,  # Lower confidence for better detection
                target_frame="base_footprint",
            ),
            transitions={
                "succeeded": "DETECT_HAND_UP",  # Test hand-up detection after general detection
                "failed": "GET_CAMERA_DATA",  # Retry on failure
            },
        )

        smach.StateMachine.add(
            "DETECT_HAND_UP",
            DetectHandUp3D(
                yolo_confidence=0.3,  # Lower confidence for better detection
                target_frame="base_footprint",
            ),
            transitions={
                "succeeded": "GET_CAMERA_DATA",  # Loop back to get new camera data
                "failed": "GET_CAMERA_DATA",  # Retry on failure
            },
        )

    # Execute the state machine in a loop
    rospy.loginfo("Starting continuous gesture and hand-up detection...")
    try:
        # Wait for YOLO service to be available
        rospy.loginfo("Waiting for YOLO 3D pose detection service...")
        rospy.wait_for_service("/yolo/detect3d_pose", timeout=10.0)

        # Execute state machine (will run continuously)
        outcome = sm.execute()
        rospy.loginfo(f"State machine finished with outcome: {outcome}")

    except rospy.ROSException as e:
        rospy.logerr(f"Failed to run test: {e}")
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")

    rospy.loginfo("Test completed")
