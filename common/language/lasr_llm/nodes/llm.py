#!/usr/bin/env python3

from typing import Dict, Any

import rospy

from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline
from transformers.pipelines.text_generation import TextGenerationPipeline

from lasr_llm_msgs.srv import Llm, LlmRequest, LlmResponse


class LLMService:

    _generation_args: Dict[str, Any] = {
        "max_new_tokens": 500,
        "return_full_text": False,
    }

    _model: AutoModelForCausalLM
    _tokenizer: AutoTokenizer
    _pipeline: TextGenerationPipeline

    def __init__(self):
        self._model = AutoModelForCausalLM.from_pretrained(
            "microsoft/Phi-3-mini-4k-instruct",
            device_map="cpu",
            torch_dtype="auto",
            trust_remote_code=True,
        )
        self._tokenizer = AutoTokenizer.from_pretrained(
            "microsoft/Phi-3-mini-4k-instruct"
        )
        self._pipeline = pipeline(
            "text-generation",
            model=self._model,
            tokenizer=self._tokenizer,
        )

        self._service = rospy.Service("/lasr_llm/llm", Llm, self._llm)
        rospy.loginfo("/lasr_llm/llm service is ready!")

    def _llm(self, request: LlmRequest) -> LlmResponse:
        response = LlmResponse()

        llm_input = [
            {"role": "system", "content": request.system_prompt},
            {"role": "user", "content": request.prompt},
        ]
        full_prompt = self._tokenizer.apply_chat_template(
            llm_input, tokenize=False, add_generation_prompt=True
        )
        rospy.loginfo(f"Prompting LLM with prompt:\n {full_prompt}")
        llm_output = self._pipeline(llm_input, **self._generation_args)
        rospy.loginfo(f"LLM Output:\n {llm_input}")
        response.output = llm_output[0]["generated_text"]

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_llm")
    llm_service = LLMService()
    rospy.spin()
