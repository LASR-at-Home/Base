import rclpy
from rclpy.node import Node

import smach
from smach_ros import RosState

from lasr_skills.say import Say
from lasr_llm_interfaces.srv import (
    SentenceEmbedding,
    ReceptionistQueryLlm,
)  # HRITaskQueryLlm


class GetInterest(RosState):
    """Prompts an LLM to find a common interest between the guests and the host."""

    # _sentence_embed_srv: rospy.ServiceProxy

    def __init__(self, node: Node):
        RosState.__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["interest_message", "guest_data"],
        )
        self._sentence_embed_srv = self.node.create_client(
            SentenceEmbedding, "/lasr_sentence_embedding/sentence_embedding"
        )
        while not self._sentence_embed_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                " Sentence Embedding service not available, waiting again..."
            )

        self._llm_srv = self.node.create_client(
            ReceptionistQueryLlm, "/receptionist/query_llm"
        )
        while not self._llm_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("LLM service not available, waiting again...")

    def execute(self, userdata):
        try:
            guest_ids = ["host", "guest1", "guest2"]
            interests = [
                userdata.guest_data["host"]["interest"],
                userdata.guest_data["guest1"]["interest"],
                userdata.guest_data["guest2"]["interest"],
            ]

            embed_request = SentenceEmbedding.Request(sentences=interests)

            embed_future = self._sentence_embed_srv.call_async(embed_request)
            rclpy.spin_until_future_complete(self.node, embed_future)
            most_similar_1, most_similar_2 = embed_future.result().most_similar

            most_similar_1_name = guest_ids[interests.index(most_similar_1)]
            most_similar_2_name = guest_ids[interests.index(most_similar_2)]

            # TODO: Update and test this
            llm_request = ReceptionistQueryLlm.Request()
            llm_request.llm_input = f"{most_similar_1}, {most_similar_2}"
            llm_request.task = "interest_commonality"

            llm_future = self._llm_srv.call_async(llm_request)
            rclpy.spin_until_future_complete(self.node, llm_future)
            llm_response = llm_future.result()

            commonality = llm_response.response.interest_commonality.strip()
            if commonality:
                commonality = commonality.split(" ", 1)[0]
                commonality = "in " + commonality

            userdata.interest_message = (
                "To break the ice, I thought you'd like to know that "
                f"{userdata.guest_data[most_similar_1_name]['name']} and {userdata.guest_data[most_similar_2_name]['name']} have a common interest {commonality} as they both like "
                f"{most_similar_1} and {most_similar_2}."
            )
        except Exception as e:
            self.node.get_logger().error(f"Error in GetInterest state: {e}")
            userdata.interest_message = (
                "I couldn't find a common interest between the guests."
            )
            return "failed"

        return "succeeded"


class GetCommonInterest(smach.StateMachine):
    """State machine to find a common interest between the two guests
    and the host."""

    def __init__(self, node):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data", "interest_message"],
        )
        with self:
            smach.StateMachine.add(
                "GET_INTEREST_MESSAGE",
                GetInterest(node),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={
                    "guest_data": "guest_data",
                    "interest_message": "interest_message",
                },
            )
