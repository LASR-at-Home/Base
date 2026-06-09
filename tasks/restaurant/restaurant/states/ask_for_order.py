import rclpy
import yasmin
from lasr_llm_interfaces.srv import Llm

# MOCK sentence for testing (replace with AskAndListen when TTS is working)
MOCK_TRANSCRIPTION = "I would like a coffee please"


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
        self._possible_items = node.get_parameter("priors.items").value

        # Create the LLM service client
        self._llm_client = node.create_client(Llm, "/lasr_llm/llm")
        self._llm_client.wait_for_service()

    def execute(self, blackboard):
        # MOCK: skip TTS and listening, use hardcoded sentence for testing
        # TODO: replace with AskAndListen when TTS is working:
        #   from lasr_skills import AskAndListen
        #   self._ask_and_listen = AskAndListen(tts_phrase="What would you like to order?")
        #   outcome = self._ask_and_listen(blackboard)
        #   if outcome != "succeeded": return "failed"
        #   transcription = blackboard["transcribed_speech"].lower()
        transcription = MOCK_TRANSCRIPTION
        print(f"[MOCK] AskForOrder heard: '{transcription}'")

        # Call the LLM to parse the item
        request = Llm.Request()
        request.system_prompt = (
            "You are a robot acting as a waiter in a restaurant. "
            "You are tasked with identifying a single item from a customer's order. "
            f"The possible items are: {', '.join(self._possible_items)}. "
            "You will receive input such as 'I would like a coffee please'. "
            "You should output only the single item name, for example: 'coffee'. "
            "Do not output anything else. "
            "If you cannot identify an item, output 'none'."
        )
        request.prompt = transcription
        request.max_tokens = 20

        # Send the request — use spin_once loop (safer inside YASMIN state)
        future = self._llm_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if future.result() is None:
            return "failed"

        response = future.result().output.strip().lower()
        print(f"[LLM] AskForOrder parsed: '{response}'")

        if response == "none":
            return "failed"

        # Store results on blackboard
        blackboard["transcribed_speech"] = transcription
        blackboard["order"] = [response]

        return "succeeded"