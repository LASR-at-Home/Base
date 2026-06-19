import rclpy
import yasmin
from lasr_skills import AskAndListen
from lasr_llm_interfaces.srv import RestaurantQueryLlm
import time

class AskForOrder(yasmin.StateMachine):
    """
    Sub state machine that asks the customer what they would like to order,
    listens to their response, and parses it into the first item.

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

        # 1. Ask and listen — say phrase and listen for response
        self.add_state(
            "ASK_AND_LISTEN",
            AskAndListen(tts_phrase="What would you like to order?"),
            transitions={
                "succeeded": "PARSE_ORDER",
                "failed":    "failed",
            },
            remappings={"transcribed_speech": "transcribed_speech"},
        )

        # 2. Parse order — keyword match or LLM fallback
        self.add_state(
            "PARSE_ORDER",
            self.ParseOrder(node=node),
            transitions={
                "succeeded": "succeeded",
                "failed":    "failed",
            },
        )

    class ParseOrder(yasmin.State):
        """
        Parses the transcribed speech into the first ordered item.
        Uses keyword matching first, falls back to LLM.
        """

        def __init__(self, node):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_input_key("transcribed_speech")
            self.add_output_key("order")

            self._node = node
            self._possible_items = node.get_parameter("priors.items").value or []
            print(f"[DEBUG] possible_items loaded: {self._possible_items}")

            self._llm_client = node.create_client(
                RestaurantQueryLlm, "/restaurant/query_llm"
            )
            self._llm_client.wait_for_service()

        def execute(self, blackboard):
            transcription = blackboard["transcribed_speech"].lower()
            print(f"[AskForOrder] heard: '{transcription}'")

            # Step 1: keyword matching first
            for item in self._possible_items:
                if item.lower() in transcription:
                    print(f"[KEYWORD] matched: '{item}'")
                    blackboard["order"] = [item.lower()]
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
            print(f"[LLM] parsed: '{item}'")

            if not item:
                return "failed"

            blackboard["order"] = [item]
            return "succeeded"