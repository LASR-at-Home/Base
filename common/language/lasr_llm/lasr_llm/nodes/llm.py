# from typing import Dict, Any

# import rclpy
# from rclpy.node import Node
# from llama_cpp import Llama
# from lasr_llm_interfaces.srv import Llm
# import timeit


# # From https://stackoverflow.com/questions/14452145/how-to-measure-time-taken-between-lines-of-code-in-python
# class CodeTimer:
#     def __init__(self, name=None):
#         self.name = " '" + name + "'" if name else ""

#     def __enter__(self):
#         self.start = timeit.default_timer()

#     def __exit__(self, exc_type, exc_value, traceback):
#         self.took = (timeit.default_timer() - self.start) * 1000.0
#         print("Code block" + self.name + " took: " + str(self.took) + " ms")


# class LLMService(Node):

#     _model: Llama

#     def __init__(self):
#         super().__init__("lasr_llm")

#         self._model = Llama.from_pretrained(
#             repo_id="microsoft/Phi-3-mini-4k-instruct-gguf",
#             verbose=False,
#             filename="*q4.gguf",
#             n_ctx=4096,  # Context length
#             n_gpu_layers=-1,  # Use all available GPU layers
#         )

#         # Warm up the model
#         with CodeTimer("LLM Warmup"):
#             self._model(
#                 "You are a robot acting as a party host. You are tasked with identifying the name and interest belonging to a guest. The possible names are John, Charlie, Axel, Matt, Jared, Ben,, Siyao, Albert, Robert, Grace, Freya, George, Siyao. You will receive input such as my name is john and I like robotics. Output only the name and interest, e.g., john, robotics. Make sure that the interest is only one or two words. If you cant identify the name or interest output unkown, e.g. john, unkown. The user says:",
#                 max_tokens=10,
#                 stop=["<|end|>"],
#                 echo=False,
#             )

#         self._service = self.create_service(Llm, "/lasr_llm/llm", self._llm)
#         self.get_logger().info("/lasr_llm/llm service is ready!")

#     def _llm(self, request, response):
#         with CodeTimer("LLM Request"):
#             prompt = f"{request.system_prompt} The user says: {request.prompt}"
#             self.get_logger().info(f"Prompting LLM with prompt:\n {prompt}")
#             llm_output = self._model(
#                 f"<|user|>\n{prompt}<|end|>\n<|assistant|>",
#                 max_tokens=request.max_tokens,
#                 stop=["<|end|>"],
#                 echo=False,
#             )
#         self.get_logger().info(f"LLM Output:\n {llm_output}")
#         response.output = llm_output["choices"][0]["text"]

#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = LLMService()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
