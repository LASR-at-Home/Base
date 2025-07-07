#!/usr/bin/env python3
import smach
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DetectShirtColor(smach.State):
    """
    Takes userdata.cropped_image (sensor_msgs/Image) and outputs
    userdata.shirt_color ∈ {'dark_green','green','light_green','unknown'}.
    """
    def __init__(self):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["cropped_image"],
            output_keys=["shirt_color"],
        )
        self.bridge = CvBridge()

        # --- green thresholds in HSV ---
        # --- purple thresholds in HSV ---
        self.dark_purple_lower  = np.array([125, 100,  50])
        self.dark_purple_upper  = np.array([145, 255, 150])

        self.purple_lower       = np.array([125, 100, 150])
        self.purple_upper       = np.array([145, 255, 255])

        self.light_purple_lower = np.array([125,  40, 200])
        self.light_purple_upper = np.array([145, 100, 255])


    def execute(self, userdata):
        # convert ROS image → OpenCV BGR → HSV
        cv_img = self.bridge.imgmsg_to_cv2(userdata.cropped_image, "bgr8")
        hsv    = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # count pixels in each range
        counts = {}
        for name, (lo, hi) in [
            ("dark_purple",  (self.dark_purple_lower,  self.dark_purple_upper)),
            ("purple",       (self.purple_lower,       self.purple_upper)),
            ("light_purple", (self.light_purple_lower, self.light_purple_upper)),
        ]:
            mask = cv2.inRange(hsv, lo, hi)
            counts[name] = int(mask.sum() / 255)

        # pick the most‐populous bin, but require a minimum pixel count
        winner, pix = max(counts.items(), key=lambda kv: kv[1])
        userdata.shirt_color = winner if pix > 500 else "unknown"

        return "succeeded"
