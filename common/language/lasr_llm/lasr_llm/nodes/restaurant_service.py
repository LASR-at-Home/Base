#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lasr_llm_interfaces.srv import RestaurantQueryLlm
from lasr_llm import LLMInference, ModelConfig


class RestaurantLlmService(Node):
    """
    Service for handling restaurant order queries to the LLM.
    Extracts the ordered item from the customer's speech.
    """

    def __init__(self):
        super().__init__("restaurant_query_llm_service")
        self.create_service(
            RestaurantQueryLlm, "/restaurant/query_llm", self.restaurant_query_llm
        )
        config = ModelConfig(model_name="Qwen/Qwen2.5-1.5B", model_type="llm")
        self.llm_inference = LLMInference(config)
        self.get_logger().info("Restaurant Query LLM service started")

    def restaurant_query_llm(self, request, response):
        self.get_logger().info(f"Received query: {request.llm_input}")

        import re

        items_str = (
            ", ".join(request.possible_items)
            if request.possible_items
            else "any food or drink"
        )

        query = (
            f"You are a waiter robot. Extract only the food or drink item from this sentence. "
            f"The possible items are: {items_str}. "
            f"Output only the item name from the list, nothing else. "
            f"For example: 'I would like a coffee please' -> coffee. "
            f"Sentence: {request.llm_input}. "
            f"Item:"
        )

        llm_output = self.llm_inference.run_inference(query, max_tokens=10)
        self.get_logger().info(f"LLM output: {llm_output}")

        item = llm_output.strip().lower()
        item = re.split(r"[\n\.,!?]", item)[0].strip()
        # for filler in ["the", "a", "an", "is", "item:", "output:", "field:"]:
        #     item = item.replace(filler, "").strip()

        self.get_logger().info(f"Parsed item: {item}")
        response.item = item
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantLlmService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
