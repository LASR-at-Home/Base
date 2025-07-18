"""
State Machine for parsing the transcription of the operators request and using it to create a pose 
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from give_me_a_hand import HandleRequestLLM
from lasr_llm_msgs.srv import Llm, LlmRequest
import string
from lasr_skills import AskAndListen, GoToLocation




class CommunicateOperator(smach.StateMachine):
    def __init__(
        self, last_resort: bool, param_key: str = "/give_me_a_hand/priors"
    ):

        self._param_key = param_key
        self._last_resort = last_resort
        self.request_location_pose = None

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
        )
        with self:
            smach.StateMachine.add(
                f"GET_REQUEST",
                AskAndListen(
                    "Please say 'Hi Tiago' for me to begin listening. What is your request?"
                ),
                transitions={
                    "succeeded": f"HANDLE_REQUEST_LLM",
                    "failed": f"HANDLE_REQUEST_LLM",
                },
            )
            smach.StateMachine.add(
                f"HANDLE_REQUEST_LLM",
                HandleRequestLLM(),
                transitions={
                    "succeeded": f"GO_TO_PUT_LOCATION",
                    "failed": f"GO_TO_PUT_LOCATION",
                },

            )
            smach.StateMachine.add(
                f"GO_TO_PUT_LOCATION",
                GoToLocation(self.request_location_pose),
                transitions={
                    "succeeded": f"succeeded",
                    "failed": f"GO_TO_PUT_LOCATION",
                },
            )



            # smach.StateMachine.add(
            #     "PARSE_INTEREST",
            #     self.ParseInterest(
            #         guest_id=self._guest_id,
            #         last_resort=self._last_resort,
            #         param_key=self._param_key,
            #     ),
            #     transitions={"succeeded": "succeeded", "failed": "RECOVER_INTEREST"},
            # )
            # smach.StateMachine.add(
            #     "RECOVER_INTEREST",
            #     self.RecoverInterest(
            #         guest_id=self._guest_id,
            #         last_resort=self._last_resort,
            #         param_key=self._param_key,
            #     ),
            #     transitions={"failed": "failed", "succeeded": "succeeded"},
            # )

 


# ASK - "Where should I put the object"

# e.g - "Where should I put the object?" "put the object by the microwave"

#

# AskAndListen - output transcription 



# Pass transcription into HandleRequestLLM 


# WHich should output POSE. 



#Pass Pose to GoToLocation 



# DONE


