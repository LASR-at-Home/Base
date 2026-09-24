import rclpy
import yasmin
import yasmin_ros
from lasr_vision_interfaces.srv import VlmDescribePeople
from .vision import GetImage


class DescribePeople(yasmin.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )

        self.add_output_key("attributes")

        self.add_state(
            "GET_IMAGE",
            GetImage(),
            transitions={"succeeded": "GET_ATTRIBUTES", "failed": "failed"},
        )

        self.add_state(
            "GET_ATTRIBUTES",
            GetVlmAttributes(),
            transitions={"succeeded": "succeeded", "aborted": "failed"},
        )


class GetVlmAttributes(yasmin_ros.ServiceState):
    def __init__(self):
        super().__init__(
            srv_name="/vlm/describe_people",
            srv_type=VlmDescribePeople,
            create_request_handler=self._create_request,
            response_handler=self._handle_resp,
        )

        self.add_input_key("image_raw")
        self.add_output_key("attributes")

    def _create_request(self, blackboard):
        request = VlmDescribePeople.Request()
        request.image_raw = blackboard["image_raw"]

        return request

    def _handle_resp(self, blackboard, response):

        dict = {
            "hair_color": response.hair_color,
            "hair_length": response.hair_length,
            "glasses": response.glasses,
            "hat": response.hat,
            "shirt_color": response.shirt_color,
        }

        blackboard["attributes"] = dict

        return "succeeded"
