#!/usr/bin/env python3
import rospy
import smach
from cv2_img import cv2_img_to_msg

from lasr_vision_msgs.msg import CDRequest
from lasr_vision_msgs.srv import (
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
        method: str = "centered",
        use_mask: bool = True,
        yolo_model: str = "yolov8x-seg.pt",
        yolo_model_confidence: float = 0.5,
        yolo_nms_threshold: float = 0.3,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["img_msg"],
        )

        self.object_name = object_name
        self.method = method
        self.use_mask = use_mask
        self.yolo_model = yolo_model
        self.yolo_model_confidence = yolo_model_confidence
        self.yolo_nms_threshold = yolo_nms_threshold

        rospy.wait_for_service("/vision/cropped_detection")
        self._cropped_detection = rospy.ServiceProxy(
            "/vision/cropped_detection", CroppedDetection
        )

    def execute(self, userdata) -> str:
        req = CDRequest()
        req.method=self.method
        req.use_mask=self.use_mask
        req.yolo_model=self.yolo_model
        req.yolo_model_confidence=self.yolo_model_confidence
        req.yolo_nms_threshold=self.yolo_nms_threshold
        req.object_names=[self.object_name]
        cropped_detection_req: CroppedDetectionRequest = CroppedDetectionRequest()
        cropped_detection_req.requests = [req]

        try:
            cropped_detection_resp: CroppedDetectionResponse = self._cropped_detection(
                cropped_detection_req
            )
            cropped_image = cropped_detection_resp.responses[0].cropped_imgs[0]
            # cropped_msg = cv2_img_to_msg(cropped_image)
            # self._debug_pub.publish(cropped_msg)
            userdata.img_msg = cropped_image
            return "succeeded"
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")
        #     return "failed"
        except Exception as e:  # Got some errors that is not rospy.
            rospy.logerr(f"Service call failed: {e}")
            return "failed"
        

if __name__ == "__main__":
    rospy.init_node("get_cropped_image")
    while not rospy.is_shutdown():
        get_cropped_image = GetCroppedImage(
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
