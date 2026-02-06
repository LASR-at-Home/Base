import rospy

import smach

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

from lasr_skills import Say


class SelectAndVisualiseObject(smach.StateMachine):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["selected_object", "selected_object_name"],
        )

        self._referee_pub = rospy.Publisher(
            "/referee_view", Image, latch=True, queue_size=1
        )
        self._bridge = CvBridge()

        with self:
            smach.StateMachine.add(
                "SELECT_OBJECT",
                smach.CBState(
                    self._select_object,
                    output_keys=["selected_object", "selected_object_name"],
                    outcomes=["succeeded", "failed"],
                    input_keys=["detected_objects"],
                ),
                transitions={"succeeded": "SAY_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_OBJECT",
                Say(
                    text="I have selected an object, and it is displayed on my screen. Please take a look."
                ),
                transitions={
                    "succeeded": "VIS_OBJECT",
                    "preempted": "VIS_OBJECT",
                    "aborted": "VIS_OBJECT",
                },
            )

            smach.StateMachine.add(
                "VIS_OBJECT",
                smach.CBState(
                    self._vis_object,
                    outcomes=["succeeded"],
                    input_keys=["selected_object", "selected_object_name"],
                ),
                transitions={"succeeded": "succeeded"},
            )

    def _select_object(self, userdata):
        if not userdata.detected_objects:
            return "failed"
        userdata.selected_object = userdata.detected_objects[0]
        userdata.selected_object_name = userdata.detected_objects[0][0].name
        return "succeeded"

    def _vis_object(self, userdata):
        label, xywh, confidence = (
            userdata.selected_object[0].name,
            userdata.selected_object[0].xywh,
            userdata.selected_object[0].confidence,
        )
        cv_im = self._bridge.imgmsg_to_cv2(
            userdata.selected_object[2], desired_encoding="rgb8"
        )
        cv2.rectangle(
            cv_im,
            (int(xywh[0]), int(xywh[1])),
            (int(xywh[0] + xywh[2]), int(xywh[1] + xywh[3])),
            (0, 255, 0),
            2,
        )
        cv2.putText(
            cv_im,
            f"{label} {confidence:.2f}",
            (int(xywh[0]), int(xywh[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
        image_msg = self._bridge.cv2_to_imgmsg(cv_im, encoding="rgb8")
        self._referee_pub.publish(image_msg)
        return "succeeded"
