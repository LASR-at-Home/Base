import rclpy
import smach
from lasr_skills import AccessNode


from lasr_vision_interfaces.msg import CDRequest
from lasr_vision_interfaces.srv import (
    CroppedDetection,
    CroppedDetectionRequest,
    CroppedDetectionResponse,
)


class GetCroppedImage(smach.State):
    """
    This state calls CroppedDetection service instead of running on its own.
    THis is a much faster version than the older one.
    """

    def __init__(
        self,
        object_name: str,
        method: str = "closest",
        use_mask: bool = True,
        yolo_model: str = "yolov8x-seg.pt",
        yolo_model_confidence: float = 0.5,
        yolo_nms_threshold: float = 0.3,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["img_msg", "detection"],
        )
        self.node = AccessNode.get_node()
        self.object_name = object_name
        self.method = method
        self.use_mask = use_mask
        self.yolo_model = yolo_model
        self.yolo_model_confidence = yolo_model_confidence
        self.yolo_nms_threshold = yolo_nms_threshold

        self._cropped_detection = self.create_client(
            CroppedDetection, "/vision/cropped_detection"
        )
        self._cropped_detection.wait_for_service()

    def execute(self, userdata) -> str:
        req = CDRequest()
        req.method = self.method
        req.use_mask = self.use_mask
        req.yolo_model = self.yolo_model
        req.yolo_model_confidence = self.yolo_model_confidence
        req.yolo_nms_threshold = self.yolo_nms_threshold
        req.object_names = [self.object_name]

        cropped_detection_req = CroppedDetectionRequest()
        cropped_detection_req.requests = [req]

        try:
            future = self._cropped_detection.call_async(cropped_detection_req)
            rclpy.spin_until_future_complete(self.node, future)
            cropped_detection_resp = future.result()

            cropped_image = cropped_detection_resp.responses[0].cropped_imgs[0]
            # cropped_msg = cv2_img_to_msg(cropped_image)
            # self._debug_pub.publish(cropped_msg)
            userdata.img_msg = cropped_image
            userdata.detection = cropped_detection_resp.responses[0].detections_3d[0]
            return "succeeded"
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")
        #     return "failed"
        except Exception as e:  # Got some errors that is not rospy.
            self.node.get_logger().error(f"Service call failed: {e}")
            return "failed"


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("get_cropped_image")

    while rclpy.ok():
        get_cropped_image = GetCroppedImage(
            node,
            "person",
            crop_method="closest",
            use_mask=True,
        )
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "GET_CROPPED_IMAGE",
                get_cropped_image,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
        input("Press Enter to continue...")
