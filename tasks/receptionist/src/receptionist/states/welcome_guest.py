from typing import Optional

import smach
import rospy

from lasr_skills import Say
from lasr_llm_msgs.srv import (
    ReceptionistQueryLlm,
    ReceptionistQueryLlmRequest,
    ReceptionistQueryLlmResponse,
)


class WelcomeGuest(smach.StateMachine):
    """Class to welcome guest 2 to the party at the door, where we
    tell guest 2 the interests they have in common with guest 1."""

    def __init__(self, llm_service: str = "/receptionist/query_llm/"):
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
                self.GetSimilarity(llm_service=llm_service),
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

        def __init__(self, llm_service: str = "/receptionist/query_llm/"):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data", "common_interest"],
            )
            self._llm_client = rospy.ServiceProxy(llm_service, ReceptionistQueryLlm)
            self._llm_client.wait_for_service()

        def execute(self, userdata):

            request = ReceptionistQueryLlmRequest()
            request.task = "interest_commonality"
            request.llm_input = f"Person 1 interest: {userdata.guest_data['guest1']['interest']}. Person 2 interests: {userdata.guest_data['guest2']['interest']}."

            response = self._llm_client(request)
            if response.response:
                userdata.common_interest = response.response.interest_commonality
                return "succeeded"
            else:
                rospy.logerr("Failed to get similarity from LLM.")
                userdata.common_interest = "unknown"
                return "failed"

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
