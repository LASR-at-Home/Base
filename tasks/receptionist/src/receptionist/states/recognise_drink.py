#!/usr/bin/env python3
import smach
import rospy
from lasr_vision_msgs.srv import Recognise
from sensor_msgs.msg import Image

class RecogniseDrink(smach.State):
    """
    Calls the /recognise_objects service on the latest camera image,
    looks for the current guest's requested drink, and outputs a
    geometry_msgs/PointStamped if found.
    """
    def __init__(self, dataset: str, confidence: float = 0.4):
        super().__init__(
            outcomes=["available", "not_available", "failed"],
            input_keys=["guest_data"],
            output_keys=["drink_location"],
        )
        self._dataset    = dataset
        self._confidence = confidence
        self._svc        = rospy.ServiceProxy("/recognise_objects", Recognise)

    def execute(self, ud):
        try:
            img = rospy.wait_for_message("/xtion/rgb/image_raw", Image, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr(f"No image: {e}")
            return "failed"

        want = ud.guest_data["drink"].lower()

        try:
            resp = self._svc(
                image_raw=img,
                dataset=self._dataset,
                confidence=self._confidence,
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service failed: {e}")
            return "failed"

        for det in resp.detections:
            if det.label.lower() == want:
                ud.drink_location = det.centre
                return "available"

        return "not_available"