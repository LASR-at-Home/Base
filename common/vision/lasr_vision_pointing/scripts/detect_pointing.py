#!/usr/bin/env python3

import os
import cv2
import rospkg
import rospy
import mediapipe as mp
from cv_bridge import CvBridge
from lasr_vision_msgs.srv import (
    YoloDetectionRequest,
    YoloDetection,
    PointingDirection,
    PointingDirectionResponse,
    PointingDirectionRequest,
)


class PointingDetector:
    def __init__(self):
        self.detect_service = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self.service = rospy.Service(
            "/pointing_detection_service", PointingDirection, self.excute
        )
        self.counter = 0

        # Load MediaPipe Pose model
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

    def excute(self, req: PointingDirectionRequest) -> PointingDirectionResponse:
        img = req.image_raw
        resp = PointingDirectionResponse()
        people_bounding_box = self.detection(img)

        if people_bounding_box:
            for person in people_bounding_box:
                keypoints = self.detect_keypoints(
                    img
                )  # Detect keypoints using MediaPipe
                direction = self.determine_pointing_direction(
                    keypoints, buffer_width=25
                )
                rospy.loginfo(f"Person detected pointing: {direction}")

                # Visualize pointing direction with landmarks
                image_path = os.path.join(
                    rospkg.RosPack().get_path("lasr_vision_pointing"),
                    "images",
                    f"image{self.counter}.jpg",
                )
                self.visualize_pointing_direction_with_landmarks(
                    image_path, person, direction, keypoints
                )

                resp.direction = direction
        else:
            resp.direction = "NONE"

        self.counter += 1
        return resp

    def store_image(self, img):
        package_path = rospkg.RosPack().get_path("lasr_vision_pointing")
        os.chdir(os.path.abspath(os.path.join(package_path, "images")))
        cv2.imwrite(f"image{self.counter}.jpg", img)

    def detect_keypoints(self, img):
        img_bridge = CvBridge().imgmsg_to_cv2(img, desired_encoding="passthrough")
        img_rgb = cv2.cvtColor(img_bridge, cv2.COLOR_BGR2RGB)

        # store the image
        self.store_image(img_rgb)

        results = self.pose.process(img_rgb)

        keypoints = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                x = int(landmark.x * img.width)
                y = int(landmark.y * img.height)
                keypoints.append((x, y))

        return keypoints

    def detection(self, img):
        request = YoloDetectionRequest()
        request.image_raw = img  # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"  # YOLOv8 model, auto-downloads
        request.confidence = 0.7  # minimum confidence to include in results
        request.nms = 0.4  # non maximal supression

        # send request
        response = self.detect_service(request)

        result = []
        for detection in response.detected_objects:
            if detection.name == "person":
                # cords of person in image
                result.append(detection.xywh)

        return result

    def determine_pointing_direction(self, keypoints, buffer_width=50):
        # Ensure keypoints are available
        if (
            len(keypoints) >= 7
        ):  # Ensure we have at least 7 keypoints for the upper body
            # Extract relevant keypoints for shoulders and wrists
            left_shoulder = keypoints[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
            right_shoulder = keypoints[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
            left_wrist = keypoints[self.mp_pose.PoseLandmark.LEFT_WRIST.value]
            right_wrist = keypoints[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]

            # Ensure all keypoints are detected
            if left_shoulder and right_shoulder and left_wrist and right_wrist:
                # Calculate the x-coordinate difference between shoulders and wrists
                left_diff = left_wrist[0] - left_shoulder[0]
                right_diff = right_shoulder[0] - right_wrist[0]

                # Determine pointing direction based on the difference in x-coordinates
                if abs(left_diff - right_diff) < buffer_width:
                    return "FORWARDS"
                elif abs(left_diff) > buffer_width and abs(left_diff) > abs(right_diff):
                    return "LEFT" if left_diff > 0 else "RIGHT"
                elif abs(right_diff) > buffer_width and abs(right_diff) > abs(
                    left_diff
                ):
                    return "RIGHT" if right_diff > 0 else "LEFT"

        # Default: Determine direction based on the relative position to the center of the image
        return "NONE"

    def visualize_pointing_direction_with_landmarks(
        self, image_path, person_bbox, pointing_direction, keypoints
    ):
        # Load image
        img = cv2.imread(image_path)

        # Extract person bbox coordinates
        x, y, w, h = person_bbox
        # Calculate center of bbox
        center_x = x + w // 2
        center_y = y + h // 2

        # Calculate endpoint of arrow based on pointing direction
        arrow_length = min(w, h) // 2
        if pointing_direction == "LEFT":
            endpoint = (center_x - arrow_length, center_y)
        elif pointing_direction == "RIGHT":
            endpoint = (center_x + arrow_length, center_y)
        elif pointing_direction == "FORWARDS":
            endpoint = (center_x, center_y)
        else:
            return  # No pointing direction detected

        # Draw arrow on image
        color = (0, 255, 0)  # Green color
        thickness = 2
        cv2.arrowedLine(img, (center_x, center_y), endpoint, color, thickness)

        # Draw landmarks (keypoints) on the image
        for keypoint in keypoints:
            if keypoint:
                cv2.circle(img, keypoint, 3, (0, 0, 255), -1)  # Red color for landmarks

        # Store image with pointing direction visualization and landmarks
        output_image_path = f"landmark_image{self.counter}.jpg"  # Change the output image path as needed
        cv2.imwrite(output_image_path, img)


if __name__ == "__main__":
    rospy.init_node("pointing_detector")
    pointer = PointingDetector()
    rospy.loginfo("Pointing Detector is running")
    rospy.spin()
