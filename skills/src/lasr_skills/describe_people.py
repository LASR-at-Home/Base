import rclpy
import smach
from ros_state import RosState
from lasr_vision_interfaces.srv import Vqa
from .vision import GetImage


class DescribePeople(smach.StateMachine):
    def __init__(self, node):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["clip_detection_dict"],
        )

        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetImage(node),
                transitions={"succeeded": "GET_CLIP_ATTRIBUTES", "failed": "failed"},
                remapping={"img_msg": "img_raw"},
            )

            smach.StateMachine.add(
                "GET_CLIP_ATTRIBUTES",
                GetClipAttributes(node),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


class GetClipAttributes(RosState):
    def __init__(
        self,
        node,
    ):
        super().__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["img_raw"],
            output_keys=["clip_detection_dict"],
        )
        self.clip_client = self.node.create_client(Vqa, "/clip_vqa/query_service")

        while not self.clip_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /clip_vqa/query_service...")

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

    def execute(self, userdata):
        try:
            glasses_request = Vqa.Request()
            glasses_request.possible_answers = self.glasses_questions
            glasses_request.image_raw = userdata.img_raw

            hat_request = Vqa.Request()
            hat_request.possible_answers = self.hat_questions
            hat_request.image_raw = userdata.img_raw

            hair_request = Vqa.Request()
            hair_request.possible_answers = self.hair_questions
            hair_request.image_raw = userdata.img_raw

            t_shirt_request = Vqa.Request()
            t_shirt_request.possible_answers = self.t_shirt_questions
            t_shirt_request.image_raw = userdata.img_raw

            future_glasses = self.clip_client.call_async(glasses_request)
            future_hat = self.clip_client.call_async(hat_request)
            future_hair = self.clip_client.call_async(hair_request)
            future_t_shirt = self.clip_client.call_async(t_shirt_request)

            rclpy.spin_until_future_complete(self.node, future_glasses)
            rclpy.spin_until_future_complete(self.node, future_hat)
            rclpy.spin_until_future_complete(self.node, future_hair)
            rclpy.spin_until_future_complete(self.node, future_t_shirt)

            glasses_response = future_glasses.result()
            hat_response = future_hat.result()
            hair_response = future_hair.result()
            t_shirt_response = future_t_shirt.result()

            self.node.get_logger().info("RESPONSES")
            self.node.get_logger().info(f"Glasses: {glasses_response.answer}")
            self.node.get_logger().info(f"Hat: {hat_response.answer}")
            self.node.get_logger().info(f"Hair: {hair_response.answer}")
            self.node.get_logger().info(f"T-shirt: {t_shirt_response.answer}")

            glasses_bool = glasses_response.answer == "a person wearing glasses"
            hat_bool = hat_response.answer == "a person wearing a hat"
            hair_bool = hair_response.answer == "a person with long hair"
            t_shirt_bool = (
                t_shirt_response.answer == "a person wearing a short-sleeve t-shirt"
            )

            clip_detection_dict = {
                "glasses": glasses_bool,
                "hat": hat_bool,
                "long_hair": hair_bool,
                "short_sleeve_t_shirt": t_shirt_bool,
            }

            self.node.get_logger().info(f"DETECTED ATTRIBUTES: {clip_detection_dict}")
            userdata.clip_detection_dict = clip_detection_dict
        except Exception as e:
            self.node.get_logger().error(f"Failed to get clip attributes: {e}")
            return "failed"
        return "succeeded"
