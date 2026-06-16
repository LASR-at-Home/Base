#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lasr_llm_interfaces.srv import HRITaskQueryLlm
from lasr_llm import (
    LLMInference,
    ModelConfig,
    create_query,
    parse_llm_output_to_dict,
    truncate_llm_output,
)


class HRITaskLLMService(Node):
    """
    Service for handling the HRI task queries to the LLM.
    This service will process incoming requests and return responses.
    """

    def __init__(self):
        super().__init__("hri_task_query_llm_service")
        self.create_service(HRITaskQueryLlm, "/hri_task/query_llm", self.query_llm)
        config = ModelConfig(
            model_name="Qwen/Qwen2.5-1.5B", model_type="llm", quantize=False
        )
        self.llm_inference = LLMInference(config)
        self.get_logger().info("HRI Task Query LLM service started")

    def query_llm(self, request, response):
        """
        Handle the query to the LLM.
        This function processes the request and returns a response.
        """
        self.get_logger().info(f"Received query: {request.llm_input}, and task is {request.task}")
        task = request.task

        if task == "name":
            query = create_query(
                text=request.llm_input,
                task="extract_fields",
                fields=["Name"],
            )
        elif task == "drink":
            query = create_query(
                text=request.llm_input,
                task="extract_fields",
                fields=["Favourite drink"],
            )
        else:
            self.get_logger().warning(
                f"Unsupported task: {task}, defaulting to extracting name"
            )
            task = "name"
            query = create_query(
                text=request.llm_input,
                task="extract_fields",
                fields=["Name"],
            )
            # raise ValueError(f"Unsupported task: {task}")

        llm_output = self.llm_inference.run_inference(query)
        self.get_logger().info(f"LLM output: {llm_output}")

        response.response.llm_response = llm_output
        if task == "name":
            parsed_output = parse_llm_output_to_dict(llm_output, fields=["Name"])
        else:  # task == "drink"
            parsed_output = parse_llm_output_to_dict(
                llm_output, fields=["Favourite drink"]
            )

        for key in parsed_output:
            if parsed_output[key] is None:
                parsed_output[key] = ""

        response.response.name = parsed_output.get("Name", "")
        response.response.favourite_drink = parsed_output.get("Favourite drink", "")

        self.get_logger().info(f"Returning response: {response.response}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HRITaskLLMService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
