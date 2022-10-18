#!/usr/bin/env python3
import rospy

from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionResponse

from lasr_perception_server.srv import DetectImage, DetectImageResponse

class PerceptionServer():

    def __init__(self):
        
        self.yolo_detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)

        self.detect_objects_image = rospy.Service("lasr_perception_server/detect_objects_image", DetectImage, self.detect_image)

    def detect_image(self, req):
        if len(req.filter):
            return DetectImageResponse([det for det in self.yolo_detect(req.image, req.dataset, req.confidence, req.nms).detected_objects if det.name in req.filter])
        else:
            return DetectImageResponse(self.yolo_detect(req.image, req.dataset, req.confidence, req.nms).detected_objects)

if __name__ == "__main__":
    rospy.init_node("lasr_perception_server")
    perception_server = PerceptionServer()
    rospy.spin()