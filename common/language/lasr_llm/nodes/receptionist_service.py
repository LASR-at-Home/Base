#!/usr/bin/env python3.10
import rospy

from lasr_llm_msgs.srv import (
    ReceptionistQueryLlm,
    ReceptionistQueryLlmRequest,
    ReceptionistQueryLlmResponse,
)
from lasr_llm import (
    LLMInference,
    ModelConfig,
    create_query,
    parse_llm_output_to_dict,
    truncate_llm_output,
)


class ReceptionistLlmService:
    """
    Service for handling receptionist queries to the LLM.
    This service will process incoming requests and return responses.
    """

    def __init__(self):
        rospy.init_node("receptionist_query_llm_service")
        rospy.Service(
            "/receptionist/query_llm", ReceptionistQueryLlm, self.receptionist_query_llm
        )
        config = ModelConfig(model_name="Qwen/Qwen2.5-1.5B", model_type="llm")
        self.llm_inference = LLMInference(config)
        rospy.loginfo("Receptionist Query LLM service started")

    def receptionist_query_llm(
        self,
        request: ReceptionistQueryLlmRequest,
    ) -> ReceptionistQueryLlmResponse:
        """
        Handle the receptionist query to the LLM.
        This function processes the request and returns a response.
        """
        rospy.loginfo(f"Received query: {request.llm_input}")
        task = request.task

        if task == "name_and_interest":
            query = create_query(
                text=request.llm_input,
                task="extract_fields",
                fields=["Name", "Interests"],
            )
        elif task == "drink":
            query = create_query(
                text=request.llm_input,
                task="extract_fields",
                fields=["Favourite drink"],
            )
        elif task == "interest_commonality":
            query = create_query(text=request.llm_input, task="interest_commonality")
        else:
            rospy.logerr(f"Unsupported task: {task}")
            raise ValueError(f"Unsupported task: {task}")

        llm_output = self.llm_inference.run_inference(query)
        rospy.loginfo(f"LLM output: {llm_output}")

        response = ReceptionistQueryLlmResponse()
        response.response.llm_response = llm_output
        if task != "interest_commonality":
            parsed_output = parse_llm_output_to_dict(
                llm_output, fields=["Name", "Favourite drink", "Interests"]
            )
            for key in parsed_output:
                if parsed_output[key] is None:
                    parsed_output[key] = ""
            response.response.name = parsed_output.get("Name", "")
            response.response.favourite_drink = parsed_output.get("Favourite drink", "")
            response.response.interests = parsed_output.get("Interests", "")
        else:
            parsed_output = truncate_llm_output(llm_output)
            response.response.interest_commonality = parsed_output

        rospy.loginfo(f"Returning response: {response.response}")
        return response


rospy.init_node("receptionist_query_llm_service")
receptionist_query_llm = ReceptionistLlmService()
rospy.spin()
