#!/usr/bin/env python3
import smach
import rospy
from lasr_vision_msgs.srv import Recognise, RecogniseRequest


class RecognisePeople(smach.State):
    def __init__(
        self,
        dataset: str,
        confidence: float = 0.5,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["named_detections"],
            input_keys=["people_detections"],
        )
        self._dataset = dataset
        self._confidence = confidence
        self._recognise = rospy.ServiceProxy("/recognise", Recognise)

    def execute(self, userdata):
        try:
            named_detections = []
            for person_detection in userdata.people_detections:
                img_msg = person_detection[1]
                result = self._recognise(img_msg, self._dataset, self._confidence)
                named_detections.append(
                    [person_detection[0], result.detections[0].name]
                )
            userdata.named_detections = named_detections
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
        return "succeeded"
