#!/usr/bin/env python3.10
import rospy
import re
import torch

from lasr_llm_msgs.srv import (
    StoringGroceriesQueryLlm,
    StoringGroceriesQueryLlmRequest,
    StoringGroceriesQueryLlmResponse,
)
from lasr_llm import (
    LLMInference,
    ModelConfig,
)


class StoringGroceriesQueryLlmService:
    """
    Service for handling storing_groceries queries to the LLM.
    """

    def __init__(self):
        rospy.init_node("storing_groceries_query_llm_service")
        rospy.Service(
            "/storing_groceries/query_llm",
            StoringGroceriesQueryLlm,
            self.storing_groceries_query_llm
        )
        config = ModelConfig(model_name="Qwen/Qwen2.5-1.5B", model_type="llm")
        self.llm_inference = LLMInference(config)
        rospy.loginfo("StoringGroceries Query LLM service started")

    def storing_groceries_query_llm(
        self,
        request: StoringGroceriesQueryLlmRequest,
    ) -> StoringGroceriesQueryLlmResponse:
        """
        Handle the storing_groceries query to the LLM.
        """
        llm_input = request.llm_input
        task = request.task
        rospy.loginfo(f"Received query: {llm_input} (task: {task})")

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
            categories = ', '.join(llm_input[1:])
            query = (
                "Respond with only one word. "
                f"What does '{object_name}' belong to the most or none: nothing, {categories}?"
            )
        else:
            rospy.logerr(f"Unsupported task: {task}")
            raise ValueError(f"Unsupported task: {task}")

        rospy.loginfo(f"[QUERY SENT TO LLM]\n{query}")
        try:
            llm_output = self.llm_inference.run_inference(query)
        except Exception as e:
            rospy.logerr(f"LLM inference failed: {e}")
            return StoringGroceriesQueryLlmResponse(category="error")

        rospy.loginfo(f"[RAW OUTPUT FROM LLM]\n{llm_output}")

        # Clean and extract category
        llm_output_clean = llm_output.strip().lower()
        match = re.search(r"(?:category(?: is| of|:)?\s*)(\w+)", llm_output_clean)
        if match:
            predicted_category = match.group(1)
        else:
            predicted_category = llm_output_clean.split()[-1].strip(",.\"\':")

        if task == "LinkCategory":
            valid_categories = [cat.lower() for cat in llm_input[1:]]
            if predicted_category not in valid_categories:
                predicted_category = "new"

        # Build response
        response = StoringGroceriesQueryLlmResponse()
        response.category = predicted_category
        rospy.loginfo(f"[RETURNING RESPONSE]: {response.category}")
        return response


if __name__ == "__main__":
    storing_groceries_query_llm = StoringGroceriesQueryLlmService()
    rospy.spin()
