import yasmin

from lasr_skills import AskAndListen

class VerbalConfirm(yasmin.StateMachine):
    def __init__(self, pretext: str= "Please confirm the following"):
        
        super().__init__(self, outcomes=["yes", "no"])
        
        self.add_state(
            "ASK_CONFIRMATION",
            AskAndListen(
                tts_phrase=f"{pretext}, say yes or no?",
            ),
            transitions={
                "succeeded": "CHECK_RESPONSE",
                "failed": "ASK_CONFIRMATION",
            },
        )

        self.add_state(
            "CHECK_RESPONSE",
            yasmin.CbState(
                outcomes=["yes", "no", "unknown"], 
                callback=self.parseResponse),
            transitions={
                "yes":"yes",
                "no": "no",
                "unknown": "ASK_CONFIRMATION"
            }
        )   

    def parse_arrival_confirmation(self, blackboard):
        response = str(blackboard["transcribed_speech"]).lower()
        yasmin.YASMIN_LOG_INFO(f"Recieved response: {response}")

        if "yes" in response:
            return "yes"
        elif "no" in response:
            return "no"
        else:
            yasmin.YASMIN_LOG_WARN(f"No confirmation in {response}")
            return "unknown"