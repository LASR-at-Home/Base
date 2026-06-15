import rclpy
import yasmin
from lasr_llm_interfaces.srv import RestaurantQueryLlm

# MOCK sentence for testing (replace with AskAndListen when TTS is working)
MOCK_TRANSCRIPTION = "I would also like a cola please"


class AddDish(yasmin.State):
    """
    Asks the customer for their second item, listens to their
    response, and uses the LLM to parse it into the second item.

    Inputs (from blackboard):
        order (list[str]): current order list e.g. ["coffee"]

    Outputs (to blackboard):
        order (list[str]): updated order list e.g. ["coffee", "cola"]
        transcribed_speech (str): raw speech from the customer

    Outcomes:
        succeeded — second item parsed and appended to order
        failed    — could not understand or LLM failed
    """

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_output_key("order")
        self.add_output_key("transcribed_speech")

        self._node = node

        # Get the list of possible items from the config
        self._possible_items = node.get_parameter("priors.items").value or []

        # Create the restaurant LLM service client
        self._llm_client = node.create_client(
            RestaurantQueryLlm, "/restaurant/query_llm"
        )
        self._llm_client.wait_for_service()

    def execute(self, blackboard):
        # MOCK: skip TTS and listening, use hardcoded sentence for testing
        # TODO: replace with AskAndListen when TTS is working:
        #   from lasr_skills import AskAndListen
        #   ask_and_listen = AskAndListen(tts_phrase="What else would you like?")
        #   outcome = ask_and_listen(blackboard)
        #   if outcome != "succeeded": return "failed"
        #   transcription = blackboard["transcribed_speech"].lower()
        transcription = MOCK_TRANSCRIPTION.lower()
        print(f"[MOCK] AddDish says: 'What else would you like?'")
        print(f"[MOCK] AddDish heard: '{transcription}'")

        # Step 1: keyword matching first
        for item in self._possible_items:
            if item.lower() in transcription:
                print(f"[KEYWORD] AddDish matched: '{item}'")
                blackboard["transcribed_speech"] = transcription
                blackboard["order"] = blackboard["order"] + [item.lower()]
                return "succeeded"

        # Step 2: fallback to LLM
        print("[LLM] No keyword match, calling LLM...")
        request = RestaurantQueryLlm.Request()
        request.llm_input = transcription
        request.possible_items = self._possible_items

        future = self._llm_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if future.result() is None:
            return "failed"

        item = future.result().item.strip().lower()
        print(f"[LLM] AddDish parsed: '{item}'")

        if not item:
            return "failed"

        blackboard["transcribed_speech"] = transcription
        blackboard["order"] = blackboard["order"] + [item]
        return "succeeded"