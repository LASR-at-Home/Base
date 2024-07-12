import smach

from typing import Optional, List, Dict

from lasr_skills import QueryImage


class DetectPose(smach.StateMachine):

    _pose_dict: Dict[str, str] = {
        "a person standing": "standing",
        "a person sitting": "sitting",
        "a person laying down": "lying_down",
    }

    _possible_answers: List[str] = [
        "a person standing",
        "a person sitting",
        "a person laying down",
    ]
    _pose_to_detect: Optional[str]

    class HandleResponse(smach.State):

        _pose_dict: Dict[str, str]
        _pose_to_detect: Optional[str]

        def __init__(
            self, pose_dict: Dict[str, str], pose_to_detect: Optional[str] = None
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["answer"],
                output_keys=[] if pose_to_detect is None else ["detected_pose"],
            )

            self._pose_dict = pose_dict
            self._pose_to_detect = pose_to_detect

        def execute(self, userdata):
            if self._pose_to_detect is None:
                userdata.detected_pose = self._pose_dict[userdata.answer]
                return "succeeded"
            else:
                if self._pose_dict[userdata.answer] == self._pose_to_detect:
                    return "succeeded"
                return "failed"

    def __init__(self, pose_to_detect: Optional[str] = None):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["img_msg"]
        )

        self._pose_to_detect = pose_to_detect

        with self:
            smach.StateMachine.add(
                "QUERY_IMAGE",
                QueryImage(possible_answers=self._possible_answers),
                transitions={"succeeded": "HANDLE_RESPONSE", "failed": "failed"},
            )
            smach.StateMachine.add(
                "HANDLE_RESPONSE",
                self.HandleResponse(self._pose_dict, self._pose_to_detect),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
