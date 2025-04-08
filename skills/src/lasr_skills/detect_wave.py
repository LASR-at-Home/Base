from typing import Union
import rclpy
import smach
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from lasr_skills import AccessNode


# pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
# cv_im = cv2_pcl.pcl_to_cv2(pcl_msg)
# img_msg = cv2_img.cv2_img_to_msg(cv_im)


class DetectWave(smach.State):
    def __init__(self, confidence: float):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys = ["pcl_msg"],
            output_keys = ["wave_detected", "wave_position"],
        )
        self.node = AccessNode.get_node()
        self.confidence = confidence

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
            detected_keypoints = bodypix.detect_keypoints(
                bp_req, debug_publisher=self.debug_publisher, logger=self.node.get_logger()
            ).keypoints

            gesture_to_detect = None
            keypoint_info = {
                keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
                for keypoint in detected_keypoints
            }

            if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
                if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                    gesture_to_detect = "raising_left_arm"
            if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
                if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
                    gesture_to_detect = "raising_right_arm"

            if gesture_to_detect is not None:
                self.get_logger().info(f"Detected gesture: {gesture_to_detect}")

            # Process wave point in point cloud
            wave_point = keypoint_info.get(
                "leftShoulder"
                if gesture_to_detect == "raising_left_arm"
                else "rightShoulder"
            )
            pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
                request.pcl_msg, remove_nans=False
            )
        except Exception as e:
            self.node.get_logger().error(f"Error detecting keypoints: {e}")
            return BodyPixWaveDetection.Response()

        return (
            "succeeded"
            if self.navigator.getResult() == TaskResult.SUCCEEDED
            else "failed"
        )
