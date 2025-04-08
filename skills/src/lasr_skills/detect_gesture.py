from ros_state import RosState
import rclpy
import cv2
import cv2_img
from lasr_vision_interfaces.srv import BodyPixKeypointDetection
from sensor_msgs.msg import Image
import tf2_ros as tf
from visualization_msgs.msg import Marker
from .vision.get_image import GetImage
from typing import Union


class DetectGesture(RosState):
    """
    State for detecting gestures.
    """

    def __init__(
        self,
        node,
        gesture_to_detect: Union[str, None] = None,
        bodypix_confidence: float = 0.1,
        buffer_width: int = 50,
        debug_publisher: str = "/skills/gesture_detection/debug",
    ):
        super().__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detected_gesture"],
        )
        self.gesture_to_detect = gesture_to_detect
        self.bodypix_client = self.node.create_client(
            BodyPixKeypointDetection, "/bodypix/keypoint_detection"
        )
        self.bodypix_client.wait_for_service()
        self.bodypix_confidence = bodypix_confidence
        self.debug_publisher = self.node.create_publisher(
            Image, debug_publisher, queue_size=1
        )
        self.buffer_width = buffer_width
        self.required_keypoints = [
            "leftWrist",
            "leftShoulder",
            "rightWrist",
            "rightShoulder",
        ]
        # publish a marker
        self.person_point_pub = self.node.create_publisher(
            Marker, "/person_point", queue_size=1
        )

    def execute(self, userdata):
        req = BodyPixKeypointDetection.Request()
        req.image_raw = userdata.img_msg
        req.confidence = self.bodypix_confidence
        req.keep_out_of_bounds = False
        # self.node.get_logger().info(f"before")

        try:
            future = self.bodypix_client(req)
            # self.node.get_logger().info(f"start")
            rclpy.spin_until_future_complete(self.node, future)
            # self.node.get_logger().info(f"end")
            res = future.result()

        except Exception as e:
            self.node.get_logger().error(f"{e}")
            return "failed"

        detected_keypoints = res.keypoints

        detected_gesture = "none"

        keypoint_info = {
            keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        # raising left arm
        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                detected_gesture = "raising_left_arm"
        # pointing to the left
        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if (
                keypoint_info["leftWrist"]["x"] - self.buffer_width
                > keypoint_info["leftShoulder"]["x"]
            ):
                detected_gesture = "pointing_to_the_left"
        # raising right arm
        if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
            if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
                detected_gesture = "raising_right_arm"
        # pointing to the right
        if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
            if (
                keypoint_info["rightShoulder"]["x"] - self.buffer_width
                > keypoint_info["rightWrist"]["x"]
            ):
                detected_gesture = "pointing_to_the_right"

        if self.gesture_to_detect == "waving":
            if detected_gesture in ["raising_left_arm", "raising_right_arm"]:
                detected_gesture = "waving"

        self.node.get_logger().info(f"Detected gesture: {detected_gesture}")
        userdata.detected_gesture = detected_gesture

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


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("detect_gesture")

    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "GetImage",
            GetImage(node),
            transitions={"succeeded": "DetectGesture", "failed": "failed"},
        )
        smach.StateMachine.add(
            "DetectGesture",
            DetectGesture(node),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    outcome = sm.execute()
    rclpy.shutdown()


if __name__ == "__main__":
    import smach

    main()
