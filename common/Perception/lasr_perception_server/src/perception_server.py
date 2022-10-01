#!/usr/bin/env python3
import rospy

from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionResponse

from lasr_perception_server.srv import DetectImage, DetectImageResponse

class PerceptionServer():

    def __init__(self):
        
        self.yolo_detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)

        self.detect_objects_image = rospy.Service("lasr_perception_server/detect_objects_image", DetectImage, self.detect_image)

    def detect_image(self, image, dataset, confidence=0.7, nms=0.3, filter=[]):
        if filter:
            return DetectImageResponse([det for det in self.yolo_detect(image, dataset, confidence, nms).detected_objects if det.name in filter])
        else:
            return DetectImageResponse(self.yolo_detect(image, dataset, confidence, nms).detected_objects)

if __name__ == "__main__":
    rospy.init_node("lasr_perception_server")
    perception_server = PerceptionServer()
    rospy.spin()