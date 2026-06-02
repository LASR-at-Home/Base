from yasmin import StateMachine, State

from lasr_skills.detect import Detect
from lasr_skills.vision import GetImage

class WaitForPerson(StateMachine):

    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
            handle_sigint=True,
        )
        self.add_output_key("detections")

        self.add_state(
            "GET_IMAGE",
            GetImage(topic=image_topic),
            transitions={"succeeded": "DETECT_PEOPLE", "failed": "failed"},
        )
        self.add_state(
            "DETECT_PEOPLE",
            Detect(filter=["person"]),
            transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
        )
        self.add_state(
            "CHECK_FOR_PERSON",
            CheckForPerson(),
            transitions={"done": "succeeded", "not_done": "GET_IMAGE"},
        )


class CheckForPerson(State):
    def __init__(self):
        super().__init__(outcomes=["done", "not_done"])
        self.add_input_key("detections")

    def execute(self, blackboard):
        if len(blackboard["detections"].detected_objects):
            return "done"
        else:
            return "not_done"
