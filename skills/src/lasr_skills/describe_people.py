import rclpy
import yasmin
import yasmin_ros
from lasr_vision_interfaces.srv import Vqa
from .vision import GetImage


class DescribePeople(yasmin.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )

        self.add_output_key("clip_detection_dict")

        self.add_state(
            "GET_IMAGE",
            GetImage(),
            transitions={"succeeded": "GET_CLIP_ATTRIBUTES", "failed": "failed"},
        )

        loop_state = yasmin.CbState(
            outcomes=["succeeded", "continue"], callback=self._get_attr
        )

        loop_state.add_input_key("clip_index")
        loop_state.add_output_key("clip_index")

        self.add_state(
            "LOOP_ATTR_STATE",
            loop_state,
            transitions={"succeeded": "succeeded", "continue": "GET_CLIP_ATTRIBUTES"},
        )

        self.add_state(
            "GET_CLIP_ATTRIBUTES",
            GetClipAttributes(),
            transitions={"succeeded": "LOOP_ATTR_STATE", "aborted": "failed"},
        )

    def _get_attr(self, blackboard):
        if blackboard["clip_index"] is None:
            blackboard["clip_index"] = 0
            return "continue"
        elif blackboard["clip_index"] < 3:
            blackboard["clip_index"] += 1
            return "continue"
        else:
            return "succeeded"


class GetClipAttributes(yasmin_ros.ServiceState):
    def __init__(self):
        super().__init__(
            srv_name="/clip_vqa/query_service",
            srv_type=Vqa,
            create_request_handler=self._create_request,
            response_handler=self._handle_resp,
        )

        self.add_input_key("img_raw")
        self.add_output_key("clip_detection_dict")

        self.glasses_questions = [
            "a person wearing glasses",
            "a person not wearing glasses",
        ]
        self.hat_questions = [
            "a person wearing a hat",
            "a person not wearing a hat",
        ]
        self.hair_questions = [
            "a person with long hair",
            "a person with short hair",
        ]
        self.t_shirt_questions = [
            "a person wearing a short-sleeve t-shirt",
            "a person wearing a long-sleeve t-shirt",
        ]

    def _create_request(self, blackboard):
        if blackboard["clip_index"] == 0:
            glasses_request = Vqa.Request()
            glasses_request.possible_answers = self.glasses_questions
            glasses_request.image_raw = blackboard["img_raw"]
            return glasses_request
        elif blackboard["clip_index"] == 1:
            hat_request = Vqa.Request()
            hat_request.possible_answers = self.hat_questions
            hat_request.image_raw = blackboard["img_raw"]
            return hat_request
        elif blackboard["clip_index"] == 2:
            hair_request = Vqa.Request()
            hair_request.possible_answers = self.hair_questions
            hair_request.image_raw = blackboard["img_raw"]
            return hair_request
        elif blackboard["clip_index"] == 3:
            t_shirt_request = Vqa.Request()
            t_shirt_request.possible_answers = self.t_shirt_questions
            t_shirt_request.image_raw = blackboard["img_raw"]
            return t_shirt_request

    def _handle_resp(self, blackboard, response):
        if blackboard["clip_index"] == 0:
            yasmin.YASMIN_LOG_INFO(f"Glasses: {response.answer}")
            glasses_bool = response.answer == "a person wearing glasses"
            blackboard["clip_detection_dict"].update({"glasses": glasses_bool})
            return "succeeded"
        elif blackboard["clip_index"] == 1:
            yasmin.YASMIN_LOG_INFO(f"Hat: {response.answer}")
            hat_bool = response.answer == "a person wearing a hat"
            blackboard["clip_detection_dict"].update({"hat": hat_bool})
            return "succeeded"
        elif blackboard["clip_index"] == 2:
            yasmin.YASMIN_LOG_INFO(f"Hair: {response.answer}")
            hair_bool = response.answer == "a person with long hair"
            blackboard["clip_detection_dict"].update({"long_hair": hair_bool})
            return "succeeded"
        elif blackboard["clip_index"] == 3:
            yasmin.YASMIN_LOG_INFO(f"T-shirt: {response.answer}")
            t_shirt_bool = response.answer == "a person wearing a short-sleeve t-shirt"
            blackboard["clip_detection_dict"].update(
                {"short_sleeve_t_shirt": t_shirt_bool}
            )
            attributes = blackboard["clip_detection_dict"]
            yasmin.YASMIN_LOG_INFO(f"Detected attributes: {attributes}")
            return "succeeded"
