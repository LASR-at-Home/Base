import rospy
import smach

from lasr_skills import Say
from lasr_llm_msgs.srv import SentenceEmbedding


class GetInterest(smach.State):
    """Prompts an LLM to find a common interest between the guests and the host."""

    _sentence_embed_srv: rospy.ServiceProxy

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["interest_message"],
        )
        self._sentence_embed_srv = rospy.ServiceProxy(
            "/lasr_sentence_embedding/sentence_embedding", SentenceEmbedding
        )
        self._sentence_embed_srv.wait_for_service()

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

            userdata.interest_message = (
                "To break the ice, I thought you'd like to know that "
                f"{most_similar_1_name} and {most_similar_2_name} have a common interest in "
                f"'{most_similar_1}'."
            )
        except:
            return "failed"

        return "succeeded"


class GetCommonInterest(smach.StateMachine):
    """State machine to find a common interest between the two guests
    and the host."""

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )
        with self:
            smach.StateMachine.add(
                "GET_INTEREST_MESSAGE",
                GetInterest(),
                transitions={
                    "succeeded": "SAY_INTEREST",
                    "failed": "SAY_INTEREST",
                },
                remapping={
                    "guest_data": "guest_data",
                    "interest_message": "interest_message",
                },
            )
            smach.StateMachine.add(
                "SAY_INTEREST",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "interest_message"},
            )
