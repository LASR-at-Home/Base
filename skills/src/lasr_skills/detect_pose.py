import smach

from typing import Optional


class DetectPose(smach.State):

    _pose_to_detect: Optional[str]

    def __init__(self, pose_to_detect: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["img_msg"]
        )

        self._pose_to_detect = pose_to_detect

    def execute(self, userdata):
        return "succeeded"
