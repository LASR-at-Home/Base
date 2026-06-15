import rclpy
import yasmin
from lasr_llm_interfaces.srv import RestaurantQueryLlm

# MOCK sentence for testing (replace with AskAndListen when TTS is working)
MOCK_TRANSCRIPTION = "I would like a lemonade please"

class AskForOrder(yasmin.State):
    """
    Asks the customer what they would like to order, listens to their
    response, and uses the LLM to parse it into the first item.

    Inputs (from blackboard):
        none

    Outputs (to blackboard):
        order (list[str]): list containing the first parsed item e.g. ["coffee"]
        transcribed_speech (str): raw speech from the customer

    Outcomes:
        succeeded — first item parsed and stored in order
        failed    — could not understand or LLM failed
    """

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("order")
        self.add_output_key("transcribed_speech")

        self._node = node

        # Get the list of possible items from the config
        self._possible_items = node.get_parameter("priors.items").value or []
        print(f"[DEBUG] possible_items loaded: {self._possible_items}")

        # Create the restaurant LLM service client
        self._llm_client = node.create_client(
            RestaurantQueryLlm, "/restaurant/query_llm"
        )
        self._llm_client.wait_for_service()

    def execute(self, blackboard):
        # MOCK: skip TTS and listening, use hardcoded sentence for testing
        # TODO: replace with AskAndListen when TTS is working:
        #   from lasr_skills import AskAndListen
        #   self._ask_and_listen = AskAndListen(tts_phrase="What would you like to order?")
        #   outcome = self._ask_and_listen(blackboard)
        #   if outcome != "succeeded": return "failed"
        #   transcription = blackboard["transcribed_speech"].lower()
        transcription = MOCK_TRANSCRIPTION.lower()
        print(f"[MOCK] AskForOrder heard: '{transcription}'")

        # Step 1: keyword matching first — fast, no LLM needed
        for item in self._possible_items:
            if item.lower() in transcription:
                print(f"[KEYWORD] AskForOrder matched: '{item}'")
                blackboard["transcribed_speech"] = transcription
                blackboard["order"] = [item.lower()]
                return "succeeded"

        # Step 2: fallback to LLM if keyword matching fails
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
        print(f"[LLM] AskForOrder parsed: '{item}'")

        if not item:
            return "failed"

        blackboard["transcribed_speech"] = transcription
        blackboard["order"] = [item]
        return "succeeded"