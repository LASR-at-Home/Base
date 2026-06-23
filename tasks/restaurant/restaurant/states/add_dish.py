import rclpy
import yasmin
from lasr_skills import AskAndListen
from lasr_llm_interfaces.srv import RestaurantQueryLlm
import yasmin_ros


class AddDish(yasmin.StateMachine):
    """
    Sub state machine that asks for the second item, listens,
    and appends it to the order list.

    Inputs (from blackboard):
        order (list[str]): current order list e.g. ["coffee"]

    Outputs (to blackboard):
        order (list[str]): updated order list e.g. ["coffee", "cola"]
        transcribed_speech (str): raw speech from the customer

    Outcomes:
        succeeded — second item parsed and appended to order
        failed    — could not understand or LLM failed
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_output_key("order")
        self.add_output_key("transcribed_speech")

        # 1. Ask and listen — say phrase and listen for response
        self.add_state(
            "ASK_AND_LISTEN",
            AskAndListen(tts_phrase="What else would you like?"),
            transitions={
                "succeeded": "PARSE_DISH",
                "failed": "failed",
            },
            remappings={"transcribed_speech": "transcribed_speech"},
        )

        # 2. Parse dish — keyword match or LLM fallback
        self.add_state(
            "PARSE_DISH",
            self.ParseDish(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )

    class ParseDish(yasmin.State):
        """
        Parses the transcribed speech into the second ordered item
        and appends it to the order list.
        """

        def __init__(self):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_input_key("transcribed_speech")
            self.add_input_key("order")
            self.add_output_key("order")

            self._node = yasmin_ros.logger_node
            self._possible_items = self._node.get_parameter("priors.items").value or []

            self._llm_client = self._node.create_client(
                RestaurantQueryLlm, "/restaurant/query_llm"
            )
            self._llm_client.wait_for_service()

        def execute(self, blackboard):
            transcription = blackboard["transcribed_speech"].lower()
            print(f"[AddDish] heard: '{transcription}'")

            # Step 1: keyword matching first
            for item in self._possible_items:
                if item.lower() in transcription:
                    print(f"[KEYWORD] matched: '{item}'")
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
            print(f"[LLM] parsed: '{item}'")

            if not item:
                return "failed"

            blackboard["order"] = blackboard["order"] + [item]
            return "succeeded"
