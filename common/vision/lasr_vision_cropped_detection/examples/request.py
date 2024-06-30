#!/usr/bin/env python3
import rospy
from lasr_vision_msgs.msg import CDRequest
from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest


if __name__ == "__main__":
    service = rospy.ServiceProxy("/vision/cropped_detection", CroppedDetection)
    service.wait_for_service()
    while not rospy.is_shutdown():
        request_srv = CroppedDetectionRequest()
        request = CDRequest()
        request.method = "centered"
        request.use_mask = True
        request.yolo_model = "yolov8x-seg.pt"
        request.yolo_model_confidence = 0.5
        request.yolo_nms_threshold = 0.3
        request.object_names = ["person", "bottle", "chair"]
        request_srv.requests = [request]
        response = service(request_srv)
