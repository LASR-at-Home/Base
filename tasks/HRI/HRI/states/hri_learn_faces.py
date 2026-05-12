"""The HRI version of learn faces uses userdata for the name of the guest instead"""

import rclpy
from rclpy.node import Node

import smach
from smach_ros import RosState

from lasr_vision_interfaces.srv import (
    AddFace,
    YoloPoseDetection,
)
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv_bridge import CvBridge


class HRILearnFaces(smach.StateMachine):
    class CheckEyes(RosState):
        """Checks if eyes are present in a given RGB image"""

        # _yolo_service: rospy.ServiceProxy

        def __init__(self, node: Node):
            RosState.__init__(
                self,
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["image_raw"],
            )

            self._yolo_service = self.node.create_client(
                YoloPoseDetection, "/yolo/detect_pose"
            )
            while not self._yolo_service.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(
                    "'YoloPoseDetection' service is not available... Waiting."
                )

        def execute(self, userdata):
            image_raw = userdata.image_raw

            req = YoloPoseDetection.Request()
            req.image_raw = image_raw
            req.model = "yolo11n-pose.pt"
            req.confidence = 0.5
            # may need req.target_frame

            try:
                future = self._yolo_service.call_async(req)
                rclpy.spin_until_future_complete(self.node, future)

                response = future.result()

                if not response.detections:
                    result = "failed"
                else:
                    for keypoint_detection in response.detections:
                        for keypoint in keypoint_detection.keypoints:
                            if "eye" in keypoint.keypoint_name.lower():
                                result = "succeeded"
                                break
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {e}")
                return "failed"

            return result

    class LearnFaceState(RosState):
        def __init__(self, guest_id: str, node: Node):
            RosState.__init__(
                self,
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["cropped_images", "num_images"],
                output_keys=["num_images"],
            )

            self._guest_id = guest_id
            self._bridge = CvBridge()

            self._learn_face = self.node.create_client(
                AddFace, "/lasr_vision_reid/add_face"
            )
            while not self._learn_face.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(
                    "'AddFace' service is not available... Waiting."
                )

        def execute(self, userdata):
            try:
                request = AddFace.Request()
                request.image_raw = self._bridge.cv2_to_imgmsg(
                    userdata.cropped_images["person"], encoding="rgb8"
                )
                request.name = self._guest_id

                future = self._learn_face.call_async(request)
                rclpy.spin_until_future_complete(self.node, future)
                response = future.result()

                if response.success:
                    userdata.num_images += 1
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {e}")
                return "failed"

            return "succeeded"

    class CheckDoneState(RosState):
        def __init__(self, dataset_size: int, node: Node):
            RosState.__init__(
                self,
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["num_images"],
            )
            self._dataset_size = dataset_size

        def execute(self, userdata):
            if userdata.num_images >= self._dataset_size:
                self.node.get_logger().info("Collected enough images for the guest.")
                return "succeeded"
            else:
                self.node.get_logger().warn(
                    f"Not enough images collected for the guest: {userdata.num_images}/{self._dataset_size}."
                )
                return "failed"

    def __init__(self, node: Node, guest_id: str, dataset_size: int = 3):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id
        self._dataset_size = dataset_size
        # TODO: Should add a check for detecting eyes in image befor learning face.
        with self:
            self.userdata.num_images = 0
            smach.StateMachine.add(
                "DETECT_3D",
                Detect3D(filter=["person"], node=node),
                transitions={"succeeded": "CHECK_EYES", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_EYES",
                self.CheckEyes(node=node),
                transitions={"succeeded": "CROP_IMAGE_3D", "failed": "DETECT_3D"},
            )
            smach.StateMachine.add(
                "CROP_IMAGE_3D",
                CropImage3D(
                    filters=["person"],
                    crop_logic="nearest",
                    crop_type="masked",
                    node=node,
                ),
                transitions={"succeeded": "LEARN_FACE", "failed": "failed"},
                remapping={"cropped_images": "cropped_images"},
            )

            smach.StateMachine.add(
                "LEARN_FACE",
                self.LearnFaceState(self._guest_id, node=node),
                transitions={"succeeded": "CHECK_DONE", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_DONE",
                self.CheckDoneState(self._dataset_size, node=node),
                transitions={"succeeded": "succeeded", "failed": "DETECT_3D"},
            )
