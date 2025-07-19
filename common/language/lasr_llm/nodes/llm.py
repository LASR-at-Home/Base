from typing import Dict, Any

import rospy

from llama_cpp import Llama

from lasr_llm_msgs.srv import Llm, LlmRequest, LlmResponse

import timeit


# From https://stackoverflow.com/questions/14452145/how-to-measure-time-taken-between-lines-of-code-in-python
class CodeTimer:
    def __init__(self, name=None):
        self.name = " '" + name + "'" if name else ""

    def __enter__(self):
        self.start = timeit.default_timer()

    def __exit__(self, exc_type, exc_value, traceback):
        self.took = (timeit.default_timer() - self.start) * 1000.0
        print("Code block" + self.name + " took: " + str(self.took) + " ms")


class LLMService:

    _model: Llama

    def __init__(self):
        # self._model = Llama.from_pretrained(
        #     repo_id="microsoft/Phi-3-mini-4k-instruct-gguf",
        #     verbose=False,
        #     filename="*q4.gguf",
        #     n_ctx=4096,  # Context length
        #     n_gpu_layers=-1,  # Use all available GPU layers
        # )

        self._model = Llama(
            # repo_id="microsoft/Phi-3-mini-4k-instruct-gguf",
            # verbose=False,
            # filename="*q4.gguf",
            model_path="/home/jared/.cache/huggingface/hub/models--microsoft--Phi-3-mini-4k-instruct-gguf/snapshots/999f761fe19e26cf1a339a5ec5f9f201301cbb83/./Phi-3-mini-4k-instruct-q4.gguf",
            n_ctx=4096,  # Context length
            n_gpu_layers=-1,  # Use all available GPU layers
            # local_files_only=True,  # Use local files only
        )

        # Warm up the model
        with CodeTimer("LLM Warmup"):
            self._model(
                "You are a robot acting as a party host. You are tasked with identifying the name and interest belonging to a guest. The possible names are John, Charlie, Axel, Matt, Jared, Ben,, Siyao, Albert, Robert, Grace, Freya, George, Siyao. You will receive input such as my name is john and I like robotics. Output only the name and interest, e.g., john, robotics. Make sure that the interest is only one or two words. If you cant identify the name or interest output unkown, e.g. john, unkown. The user says:",
                max_tokens=10,
                stop=["<|end|>"],
                echo=False,
            )

        self._service = rospy.Service("/lasr_llm/llm", Llm, self._llm)
        rospy.loginfo("/lasr_llm/llm service is ready!")

    def _llm(self, request: LlmRequest) -> LlmResponse:
        response = LlmResponse()
        with CodeTimer("LLM Request"):
            prompt = f"{request.system_prompt} The user says: {request.prompt}"
            rospy.loginfo(f"Prompting LLM with prompt:\n {prompt}")
            llm_output = self._model(
                f"<|user|>\n{prompt}<|end|>\n<|assistant|>",
                max_tokens=request.max_tokens,
                stop=["<|end|>"],
                echo=False,
            )
        rospy.loginfo(f"LLM Output:\n {llm_output}")
        response.output = llm_output["choices"][0]["text"]

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_llm")
    llm_service = LLMService()
    rospy.spin()
