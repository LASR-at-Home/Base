import rclpy
from rclpy.node import Node

import smach
from smach_ros import RosState

from lasr_skills import Say
from lasr_llm_msgs.srv import SentenceEmbedding, Llm, LlmRequest


"""
    TODO: update after LLM package is done (need Llm service )
"""

class GetInterest(RosState):
    """Prompts an LLM to find a common interest between the guests and the host."""

    #_sentence_embed_srv: rospy.ServiceProxy

    def __init__(self, node:Node):
        smach.State.__init__(
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
            self.node.get_logger().info('service not available, waiting again...')

        self._llm_srv = self.node.create_client(Llm, "/lasr_llm/llm")
        while not self._llm_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

    #TODO
    def execute(self, userdata):
        try:
            guest_ids = ["host", "guest1", "guest2"]
            interests = [
                userdata.guest_data["host"]["interest"],
                userdata.guest_data["guest1"]["interest"],
                userdata.guest_data["guest2"]["interest"],
            ]
            response = self._sentence_embed_srv(interests)  
            most_similar_1, most_similar_2 = response.most_similar

            most_similar_1_name = guest_ids[interests.index(most_similar_1)]
            most_similar_2_name = guest_ids[interests.index(most_similar_2)]

            llm_request = LlmRequest(
                system_prompt="Please give a single world to describe the similarities between two interests. For example, you may be given 'football, tennis', and you should output something like 'sports'. Please output only one word.",
                prompt=f"{most_similar_1}, {most_similar_2}",
                max_tokens=5,
            )
            llm_response = self._llm_srv(llm_request)
            commonality = llm_response.output.strip()
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
