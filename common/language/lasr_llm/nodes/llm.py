#!/usr/bin/env python3

from typing import Dict, Any

import rospy

from llama_cpp import Llama


from lasr_llm_msgs.srv import Llm, LlmRequest, LlmResponse


class LLMService:

    _model: Llama

    def __init__(self):
        self._model = Llama.from_pretrained(
            repo_id="microsoft/Phi-3-mini-4k-instruct-gguf",
            verbose=False,
            filename="*fp16.gguf",
            n_ctx=4096,
            device="cpu",
        )

        self._service = rospy.Service("/lasr_llm/llm", Llm, self._llm)
        rospy.loginfo("/lasr_llm/llm service is ready!")

    def _llm(self, request: LlmRequest) -> LlmResponse:
        response = LlmResponse()

        prompt = f"{request.system_prompt} The user says: {request.prompt}"
        rospy.loginfo(f"Prompting LLM with prompt:\n {prompt}")
        llm_output = self._model(
            f"<|user|>\n{prompt}<|end|>\n<|assistant|>",
            max_tokens=256,
            stop=["<|end|>"],
            echo=False,
        )
        rospy.loginfo(f"LLM Output:\n {llm_output}")
        response.output = llm_output["choices"][0]["text"])

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_llm")
    llm_service = LLMService()
    rospy.spin()
