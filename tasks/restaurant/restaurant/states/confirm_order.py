import yasmin
from lasr_skills import AskAndListen

CONFIRM_KEYWORDS = ["yes", "correct", "right", "yeah", "yep", "sure", "ok", "okay"]
REJECT_KEYWORDS = ["no", "wrong", "incorrect", "nope", "not"]


class ConfirmOrder(yasmin.StateMachine):
    """
    Sub state machine that confirms the first item with the customer.
    Says the order back and listens for yes/no response.

    Inputs (from blackboard):
        order (list[str]): current order list e.g. ["coffee"]

    Outputs (to blackboard):
        none

    Outcomes:
        confirmed — customer said yes/correct
        retry     — customer was unclear, ask again
        re_ask    — customer said no/wrong, retake order
        failed    — technical error
    """

    def __init__(self):
        super().__init__(outcomes=["confirmed", "retry", "re_ask", "failed"])
        self.add_input_key("order")

        # 1. Build the phrase and set it on blackboard
        self.add_state(
            "BUILD_PHRASE",
            self.BuildPhrase(),
            transitions={
                "succeeded": "ASK_AND_LISTEN",
                "failed": "failed",
            },
        )

        # 2. Say the order back and listen for response
        self.add_state(
            "ASK_AND_LISTEN",
            AskAndListen(
                tts_phrase_format_str="You ordered {}. Is that correct? Please say yes or no."
            ),
            transitions={
                "succeeded": "CHECK_RESPONSE",
                "failed": "failed",
            },
            remappings={"transcribed_speech": "transcribed_speech"},
        )

        # 3. Check the response
        self.add_state(
            "CHECK_RESPONSE",
            self.CheckResponse(),
            transitions={
                "confirmed": "confirmed",
                "retry": "retry",
                "re_ask": "re_ask",
                "failed": "failed",
            },
        )

    class BuildPhrase(yasmin.State):
        """Builds the confirmation phrase from the order list."""

        def __init__(self):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_input_key("order")
            self.add_output_key("tts_phrase_placeholders")

        def execute(self, blackboard):
            order = blackboard["order"]
            if len(order) == 1:
                order_str = order[0]
            else:
                order_str = " and ".join(order)
            blackboard["tts_phrase_placeholders"] = order_str
            return "succeeded"

    class CheckResponse(yasmin.State):
        """Keyword matches the customer's response."""

        def __init__(self):
            super().__init__(outcomes=["confirmed", "retry", "re_ask", "failed"])
            self.add_input_key("transcribed_speech")

        def execute(self, blackboard):
            transcription = blackboard["transcribed_speech"].lower().strip()
            print(f"[ConfirmOrder] heard: '{transcription}'")

            for word in CONFIRM_KEYWORDS:
                if word in transcription:
                    print(f"[MATCH] Confirmed with: '{word}'")
                    return "confirmed"

            for word in REJECT_KEYWORDS:
                if word in transcription:
                    print(f"[MATCH] Rejected with: '{word}'")
                    return "re_ask"

            print(f"[UNCLEAR] Could not match: '{transcription}'")
            return "retry"
