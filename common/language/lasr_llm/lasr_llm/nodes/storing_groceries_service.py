#!/usr/bin/env python3
import rclpy
import re
import torch
from rclpy.node import Node
from lasr_llm_interfaces.srv import StoringGroceriesQueryLlm
from lasr_llm import (
    LLMInference,
    ModelConfig,
)


class StoringGroceriesQueryLlmService(Node):
    """
    Service for handling storing_groceries queries to the LLM.
    """

    def __init__(self):
        super().__init__("storing_groceries_query_llm_service")
        self.create_service(
            StoringGroceriesQueryLlm,
            "/storing_groceries/query_llm",
            self.storing_groceries_query_llm,
        )
        config = ModelConfig(model_name="Qwen/Qwen2.5-1.5B", model_type="llm")
        self.llm_inference = LLMInference(config)
        self.get_logger().info("StoringGroceries Query LLM service started")

    def storing_groceries_query_llm(self, request, response):
        """
        Handle the storing_groceries query to the LLM.
        """
        llm_input = request.llm_input
        task = request.task
        self.get_logger().info(f"Received query: {llm_input} (task: {task})")

        # Construct the query
        if task == "ClassifyObject":
            query = f"What category does the object '{llm_input[0]}' belong to? Respond with only one word."
        elif task == "ClassifyCabinet":
            query = (
                "Respond with only one word. "
                f"Classify the following objects as a group: {', '.join(llm_input)}."
            )
        elif task == "LinkCategory":
            object_name = llm_input[0]
            categories = ", ".join(llm_input[1:])
            query = (
                "Respond with only one word. "
                f"What does '{object_name}' belong to the most or none: nothing, {categories}?"
            )
        else:
            self.get_logger().error(f"Unsupported task: {task}")
            raise ValueError(f"Unsupported task: {task}")

        self.get_logger().info(f"[QUERY SENT TO LLM]\n{query}")
        try:
            llm_output = self.llm_inference.run_inference(query)
        except Exception as e:
            self.get_logger().error(f"LLM inference failed: {e}")
            response.category = "error"
            return response

        self.get_logger().info(f"[RAW OUTPUT FROM LLM]\n{llm_output}")

        # Clean and extract category
        llm_output_clean = llm_output.strip().lower()
        match = re.search(r"(?:category(?: is| of|:)?\s*)(\w+)", llm_output_clean)
        if match:
            predicted_category = match.group(1)
        else:
            predicted_category = llm_output_clean.split()[-1].strip(",.\"':")

        if task == "LinkCategory":
            valid_categories = [cat.lower() for cat in llm_input[1:]]
            if predicted_category not in valid_categories:
                predicted_category = "new"

        # Build response
        response.category = predicted_category
        self.get_logger().info(f"[RETURNING RESPONSE]: {response.category}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StoringGroceriesQueryLlmService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
