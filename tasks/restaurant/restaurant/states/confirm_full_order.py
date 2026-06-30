import yasmin
from lasr_skills import AskAndListen

CONFIRM_KEYWORDS = ["yes", "correct", "right", "yeah", "yep", "sure", "ok", "okay"]
REJECT_KEYWORDS = ["no", "wrong", "incorrect", "nope", "not"]


class ConfirmFullOrder(yasmin.StateMachine):
    """
    Sub state machine that confirms the full order (both items) with the customer.
    Says the full order back and listens for yes/no response.

    Inputs (from blackboard):
        order (list[str]): full order list e.g. ["coffee", "cola"]

    Outputs (to blackboard):
        none

    Outcomes:
        confirmed — customer confirmed full order
        retry     — customer was unclear, confirm again
        re_ask    — customer said no, redo second item only (back to ADD_DISH)
        failed    — technical error
    """

    def __init__(self):
        super().__init__(outcomes=["confirmed", "retry", "re_ask", "failed"])
        self.add_input_key("order")

        # 1. Build the full order phrase
        self.add_state(
            "BUILD_PHRASE",
            self.BuildPhrase(),
            transitions={
                "succeeded": "ASK_AND_LISTEN",
                "failed": "failed",
            },
        )

        # 2. Say the full order back and listen for response
        self.add_state(
            "ASK_AND_LISTEN",
            AskAndListen(
                tts_phrase_format_str="You ordered {}. Is that correct, please say yes or no?"
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
        """Builds the full order confirmation phrase."""

        def __init__(self):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_input_key("order")
            self.add_output_key("tts_phrase_placeholders")

        def execute(self, blackboard):
            order = blackboard["order"]
            order_str = " and ".join(order)
            blackboard["tts_phrase_placeholders"] = order_str
            return "succeeded"

    class CheckResponse(yasmin.State):
        """
        Keyword matches the customer's response.
        If rejected, removes second item so ADD_DISH starts fresh.
        """

        def __init__(self):
            super().__init__(outcomes=["confirmed", "retry", "re_ask", "failed"])
            self.add_input_key("transcribed_speech")
            self.add_input_key("order")
            self.add_output_key("order")

        def execute(self, blackboard):
            transcription = blackboard["transcribed_speech"].lower().strip()
            print(f"[ConfirmFullOrder] heard: '{transcription}'")

            for word in CONFIRM_KEYWORDS:
                if word in transcription:
                    print(f"[MATCH] Full order confirmed with: '{word}'")
                    return "confirmed"

            for word in REJECT_KEYWORDS:
                if word in transcription:
                    print(
                        f"[MATCH] Full order rejected with: '{word}' — redoing second item"
                    )
                    # Remove second item so ADD_DISH starts fresh
                    blackboard["order"] = [blackboard["order"][0]]
                    return "re_ask"

            print(f"[UNCLEAR] Could not match: '{transcription}'")
            return "retry"
