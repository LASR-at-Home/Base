from typing import Optional

import smach
import rospy

from lasr_skills import Say
from lasr_llm_msgs.srv import (
    Llm,
    LlmRequest,
)


class WelcomeGuest(smach.StateMachine):
    """Class to welcome guest 2 to the party at the door, where we
    tell guest 2 the interests they have in common with guest 1."""

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        """
        Rough plan:
        - Say Hello {name} and welcome them to the party.
        - Guest {name} is already here. You'll recognise them as
          {describe}. I'd thought you might like to know
          that you share a common interest in {interest} with them.
         
        
        """

        with self:
            smach.StateMachine.add(
                "GET_SIMILARITY",
                self.GetSimilarity(),
                transitions={
                    "succeeded": "SAY_WELCOME",
                    "failed": "SAY_WELCOME",
                },
                remapping={
                    "guest_data": "guest_data",
                    "common_interest": "common_interest",
                },
            )
            smach.StateMachine.add(
                "SAY_WELCOME",
                self.GetWelcomeMessage(),
                transitions={
                    "succeeded": "SAY_WELCOME_MESSAGE",
                    "failed": "failed",
                },
                remapping={
                    "guest_data": "guest_data",
                    "common_interest": "common_interest",
                    "welcome_message": "welcome_message",
                },
            )
            smach.StateMachine.add(
                "SAY_WELCOME_MESSAGE",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "welcome_message"},
            )

    class GetSimilarity(smach.State):
        """State to get the similarity between two guests."""

        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data", "common_interest"],
            )
            self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
            self._llm.wait_for_service()

        def execute(self, userdata):

            request = LlmRequest()
            request.system_prompt = f"You are a robot acting as a party host. You are tasked with reasoning about interests between two guests. You will receive input such as 'Person 1 interest: football. Person 2 interest: tennis'. You should reason about what these guests have in common, if anything, and output a single word describing this commonality. In the example, this might be 'sports' or 'exercise'. If you cannot find any commanality, output 'none'."
            request.prompt = f"Person 1 interest: {userdata.guest_data['guest1']['interest']}. Person 2 interests: {userdata.guest_data['guest2']['interest']}."
            response = self._llm(request)
            commonality = response.output.lower()

            if commonality == "none":
                rospy.logerr("Failed to get similarity from LLM.")
                userdata.common_interest = "unknown"
                return "failed"
            else:
                userdata.common_interest = commonality
                return "succeeded"

    class GetWelcomeMessage(smach.State):
        """State to get the welcome message for the guest."""

        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "common_interest"],
                output_keys=["welcome_message"],
            )

        def execute(self, userdata):
            guest_1_name = userdata.guest_data["guest1"]["name"]
            guest_2_name = userdata.guest_data["guest2"]["name"]
            common_interest = userdata.common_interest

            userdata.welcome_message = (
                f"Hello {guest_2_name}, welcome to the party! "
                f"You'll get on well with {guest_1_name} who is already here as they share a common interest in {common_interest}."
            )
            return "succeeded"
