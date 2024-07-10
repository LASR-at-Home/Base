import smach

from typing import Optional


class DetectClothing(smach.State):

    _clothing_to_detect: Optional[str]

    def __init__(self, clothing_to_detect: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["img_msg"]
        )

        self._clothing_to_detect = clothing_to_detect

    def execute(self, userdata):
        return "succeeded"
