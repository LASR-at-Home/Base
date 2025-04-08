from typing import Union
import rclpy
from ros_state import RosState
from std_msgs.msg import Header
from cv2_pcl import pcl_to_img_msg
from lasr_vision_interfaces.srv import BodyPixKeypointDetection
import ros2_numpy as rnp
import numpy as np
from geometry_msgs.msg import PointStamped, Point

# pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
# cv_im = cv2_pcl.pcl_to_cv2(pcl_msg)
# img_msg = cv2_img.cv2_img_to_msg(cv_im)


class DetectWave(RosState):
    def __init__(self, node, confidence: float):
        super().__init__(
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["pcl_msg"],
            output_keys=["wave_detected", "wave_position"],
        )
        self.confidence = confidence

        # Set up the service client
        self.detect_service_client = self.create_client(
            BodyPixKeypointDetection, "/bodypix/keypoint_detection"
        )
        while not self.detect_service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, waiting...")
        self.keypoint_response = None

    def detect_callback(self, future):
        try:
            self.keypoint_response = future.result()
            self.node.get_logger().info(f"Detection response: {self.keypoint_response}")
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
        finally:
            self.processing = False

    def execute(self, userdata):
        if not "pcl_msg" in userdata:
            self.node.get_logger().error(f"PCL message doesn't exist!")
        pcl_msg = userdata.pcl_msg

        try:
            # Prepare request for keypoint detection
            bp_req = BodyPixKeypointDetection.Request()
            bp_req.image_raw = pcl_to_img_msg(pcl_msg)
            # bp_req.image_raw = request.image_raw
            bp_req.confidence = self.confidence

            # Call BodyPix keypoint detection
            future = self.detect_service_client.call_async(bp_req)
            future.add_done_callback(self.detect_callback)

            # take out the detected keypoints
            detected_keypoints = self.detect_callback.keypoints

            gesture_to_detect = None
            keypoint_info = {
                keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
                for keypoint in detected_keypoints
            }

            if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
                if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                    gesture_to_detect = "raising_left_arm"
            if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
                if (
                    keypoint_info["rightWrist"]["y"]
                    < keypoint_info["rightShoulder"]["y"]
                ):
                    gesture_to_detect = "raising_right_arm"

            if gesture_to_detect is not None:
                self.node.get_logger().info(f"Detected gesture: {gesture_to_detect}")

            # Process wave point in point cloud
            wave_point = keypoint_info.get(
                "leftShoulder"
                if gesture_to_detect == "raising_left_arm"
                else "rightShoulder"
            )
            pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
                pcl_msg, remove_nans=False
            )

            wave_position = np.zeros(3)
            for i in range(-5, 5):
                for j in range(-5, 5):
                    if np.any(
                        np.isnan(
                            pcl_xyz[int(wave_point["y"]) + i][int(wave_point["x"]) + j]
                        )
                    ):
                        self.node.get_logger().warn("NaN point in PCL")
                        continue
                    wave_position += pcl_xyz[int(wave_point["y"]) + i][
                        int(wave_point["x"]) + j
                    ]
            wave_position /= 100
            wave_position_msg = PointStamped(
                point=Point(*wave_position),
                header=Header(frame_id=pcl_msg.header.frame_id),
            )
            self.node.get_logger().info(f"Wave point: {wave_position_msg}")

            is_waving = gesture_to_detect is not None

            userdata.wave_detected = is_waving
            userdata.wave_position = wave_position_msg

        except Exception as e:
            self.node.get_logger().error(f"Error detecting keypoints: {e}")
            userdata.wave_detected = False
            userdata.wave_position = PointStamped()
            return "failed"

        return "succeeded"
