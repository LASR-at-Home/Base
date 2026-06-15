import yasmin

CONFIRM_KEYWORDS = ["yes", "correct", "right", "yeah", "yep", "sure", "ok", "okay"]
REJECT_KEYWORDS = ["no", "wrong", "incorrect", "nope", "not"]

# MOCK response for testing
# TODO: replace with real Listen when speech recognition is available
# Options to test different flows:
#   "yes"  → confirmed → succeeded
#   "no"   → re_ask   → back to ADD_DISH (redo second item only)
#   "hmm"  → retry    → back to CONFIRM_FULL_ORDER
MOCK_RESPONSE = "no"

class ConfirmFullOrder(yasmin.State):
    """
    Confirms the full order (both items) with the customer.

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

    def execute(self, blackboard):
        # Build full order string e.g. ["coffee", "cola"] → "coffee and cola"
        order = blackboard["order"]
        order_str = " and ".join(order)

        # MOCK: print instead of speaking
        # TODO: replace with AskAndListen when TTS is working:
        #   from lasr_skills import AskAndListen
        #   ask_and_listen = AskAndListen(
        #       tts_phrase=f"You ordered {order_str}. Is that correct? Please say yes or no."
        #   )
        #   outcome = ask_and_listen(blackboard)
        #   if outcome != "succeeded": return "failed"
        #   transcription = blackboard["transcribed_speech"].lower().strip()
        print(f"[MOCK] ConfirmFullOrder says: 'You ordered {order_str}. Is that correct? Please say yes or no.'")

        # MOCK: hardcoded response for testing
        transcription = MOCK_RESPONSE
        print(f"[MOCK] ConfirmFullOrder heard: '{transcription}'")

        # Keyword matching
        for word in CONFIRM_KEYWORDS:
            if word in transcription:
                print(f"[MATCH] Full order confirmed with: '{word}'")
                return "confirmed"

        for word in REJECT_KEYWORDS:
            if word in transcription:
                print(f"[MATCH] Full order rejected with: '{word}' — redoing second item")
                # Remove second item from order so ADD_DISH starts fresh
                blackboard["order"] = [blackboard["order"][0]]
                return "re_ask"

        # Unclear response
        print(f"[UNCLEAR] Could not match response: '{transcription}'")
        return "retry"