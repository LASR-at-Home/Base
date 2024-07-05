#!/usr/bin/env python3
import rospy
import smach
import numpy as np

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from lasr_vision_msgs.srv import YoloDetection3D
from markers import create_and_publish_marker

from typing import List, Union


class Detect3D(smach.State):
    def __init__(
        self,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8x-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        debug_publisher: str = "/skills/detect3d/debug",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )
        self.depth_topic = depth_topic
        self.model = model
        self.filter = filter if filter is not None else []
        self.confidence = confidence
        self.nms = nms
        self.yolo = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()
        self.debug_pub = rospy.Publisher(debug_publisher, Marker, queue_size=1)

    def execute(self, userdata):
        pcl_msg = rospy.wait_for_message(self.depth_topic, PointCloud2)
        try:
            result = self.yolo(pcl_msg, self.model, self.confidence, self.nms)
            if len(self.filter) > 0:
                result.detected_objects = [
                    det for det in result.detected_objects if det.name in self.filter
                ]
            userdata.detections_3d = result

            for det in result.detected_objects:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "map"
                point_stamped.point = det.point
                rospy.loginfo(f"Detected point: {point_stamped}")
                if np.isnan(det.point.x).any():
                    rospy.loginfo(f"No depth detected, object likely too far away")
                    continue
                create_and_publish_marker(self.debug_pub, point_stamped, name=det.name)

            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"


if __name__ == "__main__":
    rospy.init_node("detect")
    while not rospy.is_shutdown():
        detect = Detect3D()
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "DETECT",
                detect,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
