from ros_state import RosState
import smach
from lasr_skills import Detect
from lasr_skills.vision import GetImage
from lasr_skills import AccessNode


class WaitForPerson(smach.StateMachine):

    def __init__(
        self,
        node,
        image_topic: str = "/xtion/rgb/image_raw",
    ):
        super().__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections"],
        )

        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetImage(node, topic=image_topic),
                transitions={"succeeded": "DETECT_PEOPLE", "failed": "failed"},
            )
            smach.StateMachine.add(
                "DETECT_PEOPLE",
                Detect(node, filter=["person"]),
                transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_FOR_PERSON",
                CheckForPerson(node),
                transitions={"done": "succeeded", "not_done": "GET_IMAGE"},
            )


class CheckForPerson(RosState):
    def __init__(
        self,
        node,
    ):
        super().__init__(
            self, node, outcomes=["done", "not_done"], input_keys=["detections"]
        )

    def execute(self, userdata):
        if len(userdata.detections.detected_objects):
            return "done"
        else:
            return "not_done"
