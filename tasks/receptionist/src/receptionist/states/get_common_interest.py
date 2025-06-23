import rospy
import smach

from lasr_skills import Say
from lasr_llm_msgs.srv import Llm, LlmRequest


class GetInterest(smach.State):
    """Prompts an LLM to find a common interest between the guests and the host."""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["interest_message"],
        )
        self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
        self._llm.wait_for_service()

    def execute(self, userdata):
        try:
            request = LlmRequest()
            request.system_prompt = f"You are a robot acting as a party host. You are tasked with finding a common interest between two people, out of a group of three people. You will receive input such as 'Name: James, Interest: football. Name: Chloe, Interest: baking. Name: John, Interest: tennis'. You should output exactly three words, the names of the two people who share a common interest, and the common interest itself. In the example, this might be 'James, John, sports'. If you cannot find any common interest, output 'none'."
            request.prompt = f"Name: {userdata.guest_data['guest1']['name']}, Interest: {userdata.guest_data['guest1']['interest']}. Name: {userdata.guest_data['guest2']['name']}, Interest: {userdata.guest_data['guest2']['interest']}. Name: {userdata.guest_data['host']['name']}, Interest: {userdata.guest_data['host']['interest']}."
            response = self._llm(request)
            commonality = response.output.lower()
            # Remove punctuation and split the response
            commonality = commonality.replace(".", "").replace(",", "").split()
            if len(commonality) != 3 or commonality[0] == "none":
                rospy.logerr("Failed to get common interest from LLM.")
                userdata.interest_message = "I'm afraid I couldn't find a common interest between you all, you'll have to make do with small talk."
                return "failed"
            name_1, name_2, common_interest = commonality
            userdata.interest_message = f"To get the conversation going, you might like to know that {name_1} and {name_2} both share an interest in {common_interest}. Do with that what you will."
        except:
            rospy.logerr("Failed to call LLM service.")
            userdata.interest_message = "I'm afraid I couldn't find a common interest between you all, you'll have to make do with small talk."
            return "failed"


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
