# Script to perform LLM inference using a pre-trained model from HuggingFace

from dataclasses import dataclass
from typing import Optional, List, Dict
import re
import logging
import numpy as np

import os
import ollama
from ollama import Client
import socket

import torch

import json
from datetime import datetime

from .utils import (
    create_query,
    truncate_llm_output,
    parse_llm_output_to_dict,
)

import os

here = os.path.dirname(os.path.abspath(__file__))


@dataclass
class ModelConfig:
    """
    Configuration class for the LLM or pipeline models.
    """

    model_name: str
    model_type: str = (
        "pipeline"  # 'pipeline' (NER, QA, classification) or 'llm' (chat/instruction)
    )
    task: Optional[str] = None  # For pipeline models
    quantize: bool = True
    ollama_host: str = "http://127.0.0.1:11434"  # Ollama host for LLMs


models = {
    # "BERT-ner": "dslim/bert-base-NER",  # NER so mainly names
    "Qwen": "qwen2.5:3b",
    # "QCode": "Qwen/Qwen2.5-Coder-1.5B",  # perfect
    # "Mistral": "mistralai/Mistral-7B-v0.1",  # empty output
    # "Gemma": "google/gemma-2b",  # nope
    # "DeepSeekQwen": "deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B",  # terrible lol
}

class LLMInference:
    def __init__(self, model_config: ModelConfig):
        self.config = model_config
        self.logger = logging.getLogger(__name__)
        self.model_name = models["Qwen"]

        self.num_gpu = self._detect_gpu_layers()
        self.client = Client(host=self.config.ollama_host)
        self._ensure_model_available()

    def run_inference(
        self, query: str, context: Optional[str] = None, max_tokens: int = 56
    ) -> str:
        messages = []

        if context:
            messages.append({"role": "system", "content": context})

        messages.append({"role": "user", "content": query})

        try:
            response = self.client.chat(
                model=self.model_name,
                messages=messages,
                options={
                    "num_predict": max_tokens,
                    "num_gpu": self.num_gpu,
                },
            )
            result = response.message.content.strip()
            # Strip the input query from the result if echoed back, matching original behaviour
            generated_text = re.sub(re.escape(query), "", result).strip()
            return generated_text

        except Exception as e:
            raise RuntimeError(f"[LLMInference] Inference failed: {e}")

    def _is_online(self) -> bool:
        """Check internet connectivity."""
        try:
            socket.setdefaulttimeout(3)
            socket.create_connection(("8.8.8.8", 53))
            return True
        except OSError:
            return False

    def _model_is_local(self) -> bool:
        """Return True if the model is already pulled locally."""
        local_models = [m.model for m in self.client.list().models]
        return self.model_name.split(":")[0] in [m.split(":")[0] for m in local_models]

    def _detect_gpu_layers(self) -> int:
        try:
            if torch.cuda.is_available():
                free_vram_gb = torch.cuda.mem_get_info(0)[0] / 1e9
                self.logger.info(f"[LLMInference] Free VRAM: {free_vram_gb:.1f} GB.")
                if free_vram_gb >= 4:
                    return -1  # full GPU offload
                else:
                    return 8  # Partial offload, rest to CPU
            else:
                self.logger.info("[LLMInference] No GPU detected — running on CPU.")
                return 0
        except ImportError:
            self.logger.warning("[LLMInference] torch not available — defaulting to CPU.")
            return 0

    def _ensure_model_available(self):
        """
        Ensure the model is available locally, pulling it if necessary.
        If no internet connection is available, raise an error.
        """
        if self._model_is_local():
            self.logger.info(f"[LLMInference] Model '{self.model_name}' found locally.")
            return

        # Model not local — need internet to pull it
        if not self._is_online():
            raise RuntimeError(
                f"[LLMInference] Model '{self.model_name}' is not available locally "
                f"and there is no internet connection to pull it. "
                f"Run with connectivity first so the model can be downloaded and cached."
            )

        self.logger.info(f"[LLMInference] Pulling '{self.model_name}' (this only happens once)...")
        self.client.pull(self.model_name)
        self.logger.info(f"[LLMInference] '{self.model_name}' saved locally — offline use enabled.")


    def infer_task(self) -> str:
        name = self.model_name.lower()
        if "ner" in name or "token" in name:
            return "token-classification"
        elif "qa" in name or "question" in name:
            return "question-answering"
        else:
            raise ValueError(
                f"Cannot infer task from model name: {self.model_name}. Please specify manually in model config."
            )

    def serialise_output(self, output):
        """
        Serialise the output to a format that can be saved to JSON.
        """
        if isinstance(output, dict):
            return {k: self.serialise_output(v) for k, v in output.items()}
        elif isinstance(output, list):
            return [self.serialise_output(v) for v in output]
        elif isinstance(output, np.float32) or isinstance(output, torch.Tensor):
            return float(output)
        else:
            return output

    def log_output(self, generated_text):
        # LOG the output
        log_filename = "LLMinference.json"

        # If the log file does not exist, create it
        if not os.path.exists(log_filename):
            with open(log_filename, "w+") as file:
                json.dump({"logs": []}, file, indent=4)

        try:
            with open(log_filename, "r") as file:
                logs = json.load(file)
        except FileNotFoundError:
            logs = {"logs": []}

        log_entry = {
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ"),
            "model": self.model_name,
            "query": self.query_list,
            "model_type": self.config.model_type,
            # "task": self.task,
            "generated_output": self.serialise_output(generated_text),
        }

        # Add the new entry to the logs
        logs["logs"].append(log_entry)

        with open(log_filename, "w+") as file:
            json.dump(logs, file, indent=4)


def interest_commonality_llm(interests: list[str]) -> str:
    """
    Find common interests between a list of interests.
    :param interests: a list of interests
    :return: a sentence describing the commonalities
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)
    sentence = ", ".join(interests)
    query = create_query(
        sentence,
        "Create a sentence that find and introduce commeness of interests given",
    )
    inference = LLMInference(config, query)
    response = inference.run_inference()
    # print(response)
    parsed_response = truncate_llm_output(response[0])
    return parsed_response


def introduce_llm(name: str, drink: str, interests: str) -> str:
    """
    Create a sentence introducing a person using the given name, drink, and interests.
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)
    input_summary = f"Name: {name}, Favorite drink: {drink}, Interests: {interests}"
    prompt = f"Create a sentence that introduces a person named {name}, mentioning their favorite drink ({drink}) and their interest in {interests}."

    query = create_query(input_summary, prompt)
    inference = LLMInference(config, query)
    response = inference.run_inference()
    parsed_response = truncate_llm_output(response[0])

    return parsed_response


def classify_category(objects: List[str]) -> str:
    """
    Classify category between a list of objects.
    :param objects: a list of interests
    :return: category
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)
    sentence = ", ".join(objects)
    query = create_query(
        sentence, "Detect which category these or a object belongs to."
    )
    inference = LLMInference(config)
    response = inference.run_inference(query)
    # print(response)
    parsed_response = truncate_llm_output(response[0])
    return parsed_response


def link_category(object: str, categories: list[str]) -> str:
    """
    Classify category between a list of objects.
    :param objects: a list of interests
    :return: category
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)
    categories_str = ", ".join(categories)
    query = create_query(
        f"Detect which category {object} belongs to the most from the following categories: {categories_str}. If not appropriate category to go return 'new'",
    )
    inference = LLMInference(config)
    response = inference.run_inference(query)
    # print(response)
    parsed_response = truncate_llm_output(response[0])
    return parsed_response


def extract_fields_llm(text: str, fields: List[str]) -> Dict:
    """
    Extracts structured information from a sentence using an LLM.
    Returns a dictionary with all fields — missing ones are filled with 'Unknown'.
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)

    if fields is None:
        fields = ["Name", "Favourite drink", "Interests"]

    query = create_query(text, "extract_fields", fields)
    inference = LLMInference(config)
    response = inference.run_inference(query)

    # Parse the model response
    parsed = parse_llm_output_to_dict(response, fields)

    # Fill missing or empty fields with "Unknown"
    result = {field: parsed.get(field, "Unknown") or "Unknown" for field in fields}

    return result


def main():
    # # Examples for testing
    # config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)

    # # sentence = "My name is John, my favourite drink is green tea, and my interests are robotics."
    # # sentence = "I am John, I usually drink green tea, and I really like robotics. I also like to play chess and watch movies."
    # # sentence = "Oh hi yeah, I'm John erm I drink tea usually green, and I am a robotics enthusiast. I also like to play chess and watch movies when I can."
    # sentence = "I would like a vegan hamburger, no cheese please, and a large fries. I also want a large coke and a small salad."
    # # query = f"Extract the following fields from the sentence:\n- Name\n- Favorite drink\n- Interests\n\nSentence: {sentence}"
    # query = f"Extract the following fields from the sentence:\n -Food\n -Requests\n -Drink\n\nSentence: {sentence}."
    # inference = LLMInference(config, query)
    # response = inference.run_inference()
    # print(response)
    # inference.log_output(response)

    # print("\n🔍 TEST: extract_fields_llm")
    # sentence = "Oh hi yeah, I'm John erm and I drink tea usually green."
    sentence = "Hi Tiago, My name is Hayeong and my favourite drink is matcha."
    print(f"Input sentence: {sentence}")
    extracted = extract_fields_llm(sentence, ["Name", "Favourite drink"])
    print("Extracted fields:", extracted)

    # print("\n🔍 TEST: interest_commonality_llm")
    # interests = ["robotics", "chess"]
    # commonality = interest_commonality_llm(interests)
    # print("Commonality summary:", commonality)
    #
    # print("\n🔍 TEST: introduce_llm")
    # intro = introduce_llm(name="Eunice", drink="green tea", interests="swimming")
    # print("Introduction:", intro)
    #
    # print("\n🔍 TEST: classify_category")
    # category = classify_category(["apple"])  # FIXED: must pass a list
    # print("Classified category:", category)
    #
    # print("\n🔍 TEST: link_category for 'apple'")
    # linked_apple = link_category("apple", ["fruit", "drink", "new"])
    # print("Linked category:", linked_apple)
    #
    # print("\n🔍 TEST: link_category for 'basketball'")
    # linked_basketball = link_category("basketball", ["fruit", "drink", "new"])
    # print("Linked category:", linked_basketball)


if __name__ == "__main__":
    main()
    # extract_fields_llm("Oh hi yeah, I'm John erm I drink tea usually green, and I am a robotics enthusiast. I also like to play chess and watch movies when I can.")
    # interest_commonality_llm(["robotics", "chess", "movies"])
    # interest_commonality_llm(["tennis", "football", "basketball"])