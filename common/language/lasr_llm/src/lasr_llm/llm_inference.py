# Script to perform LLM inference using a pre-trained model from HuggingFace

from dataclasses import dataclass
from typing import Optional, List, Dict
import re
import rospy

import numpy as np

print(f"Numpy version: {np.__version__}")  # For debugging purposes

from transformers import (
    pipeline,
    AutoModelForCausalLM,
    AutoTokenizer,
    BitsAndBytesConfig,
    AutoModelForTokenClassification,
    AutoModelForQuestionAnswering,
)
import torch

import os
import json
from datetime import datetime

from .utils import (
    create_query,
    truncate_llm_output,
    parse_llm_output_to_dict,
)


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


models = {
    "BERT-ner": "dslim/bert-base-NER",  # NER so mainly names
    "Qwen": "Qwen/Qwen2.5-1.5B",  # perfect
    "QCode": "Qwen/Qwen2.5-Coder-1.5B",  # perfect
    "Mistral": "mistralai/Mistral-7B-v0.1",  # empty output
    "Gemma": "google/gemma-2b",  # nope
    "DeepSeekQwen": "deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B",  # terrible lol
}


class LLMInference:
    def __init__(self, model_config: ModelConfig):
        self.config = model_config
        if self.config.quantize:
            # Quantize the model - for using 1/4 (or 1/8) of the GPU RAM. Full example in the models' HugginFace docs
            self.quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.bfloat16,
            )
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model_name = self.config.model_name
        self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)

        if self.config.model_type == "pipeline":
            if self.config.task:
                self.task = self.config.task
            else:
                self.infer_task()
            self.model = self.load_pipeline_model()
            self.pipe = pipeline(
                task=self.task, model=self.model, tokenizer=self.tokenizer
            )  # , device=self.device)

        elif self.config.model_type == "llm":
            self.model = self.load_llm_model()
        else:
            raise ValueError(
                f"Model type {self.config.model_type} is unknown or not supported."
            )

    def process_query(self, query):
        """
        Process the query to ensure it is in the correct format.
        param query: str, list, or file path
        """
        if isinstance(query, str):
            # If file path is provided, read the file
            if os.path.isfile(query):
                with open(query, "r") as file:
                    query = file.read()
            return [query]
        elif isinstance(query, list):
            return query
        else:
            raise ValueError(
                "Unsupported query type. Use 'string', 'list' or provide a file to a path containing text."
            )

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

    def load_pipeline_model(self):
        # device_map = "auto" if self.device != "cpu" else "cpu"
        torch_dtype = torch.bfloat16 if self.device != "cpu" else torch.float32
        kwargs = {"torch_dtype": torch_dtype}
        if self.config.quantize:
            kwargs["quantization_config"] = self.quantization_config

        if self.task == "token-classification":
            # Bert models for token classification do not support device_map
            model_class = AutoModelForTokenClassification
        elif self.task == "question-answering":
            kwargs["device_map"] = "auto" if self.device != "cpu" else "cpu"
            model_class = AutoModelForQuestionAnswering
        else:
            raise ValueError(f"Unsupported task: {self.task}")

        return model_class.from_pretrained(self.model_name, **kwargs)

    def load_llm_model(self):
        from transformers import BitsAndBytesConfig

        if self.device == torch.device("cpu"):
            rospy.logwarn("[LLMInference] CPU detected — skipping quantization.")
            return AutoModelForCausalLM.from_pretrained(self.model_name)

        # Try quantized load if requested and CUDA is available
        try:
            if self.config.quantize:
                quant_config = BitsAndBytesConfig(
                    load_in_4bit=True,
                    bnb_4bit_quant_type="nf4",
                    bnb_4bit_compute_dtype=torch.bfloat16,
                )
                return AutoModelForCausalLM.from_pretrained(
                    self.model_name,
                    device_map="auto",
                    quantization_config=quant_config
                )
            else:
                return AutoModelForCausalLM.from_pretrained(
                    self.model_name,
                    device_map="auto",
                    torch_dtype=torch.bfloat16
                )
        except Exception as e:
            rospy.logerr(f"[LLMInference] Quantized model load failed: {e}")
            rospy.logwarn("[LLMInference] Falling back to full-precision model.")
            return AutoModelForCausalLM.from_pretrained(self.model_name)

    def run_inference(self, query: str, context: Optional[str] = None) -> str:

        result = None
        if self.config.model_type == "pipeline":
            if self.task == "question-answering":
                if not context:
                    raise ValueError("Question Answering task requires context.")
                result = self.pipe(question=query, context=context)
            else:
                result = self.pipe(query)
        elif self.config.model_type == "llm":
            input_ids = self.tokenizer(query, return_tensors="pt").to(self.model.device)
            output_ids = self.model.generate(**input_ids, max_new_tokens=128)
            result = self.tokenizer.decode(output_ids[0], skip_special_tokens=True)
            print(f"LLM output: {result}")
        generated_text = re.sub(re.escape(query), "", result).strip()

        return generated_text

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
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)
    sentence = ", ".join(interests)
    query = create_query(
        sentence,
        "Create a sentence that introduces a person named Eunice, mentioning her interest in green tea and swimming.",
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
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)
    input_summary = f"Name: {name}, Favorite drink: {drink}, Interests: {interests}"
    prompt = f"Create a sentence that introduces a person named {name}, mentioning their favorite drink ({drink}) and their interest in {interests}."

    query = create_query(input_summary, prompt)
    inference = LLMInference(config, query)
    response = inference.run_inference()
    parsed_response = truncate_llm_output(response[0])

    return parsed_response


def extract_fields_llm(text: str, fields: List[str]) -> Dict:
    """
    Extracts structured information from a sentence using an LLM.
    Returns a dictionary with all fields — missing ones are filled with 'Unknown'.
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)

    if fields is None:
        fields = ["Name", "Favourite drink", "Interests"]

    query = create_query(text, "extract_fields", fields)
    inference = LLMInference(config, query)
    response = inference.run_inference()

    # Parse the model response
    parsed = parse_llm_output_to_dict(response[0], fields)

    # Fill missing or empty fields with "Unknown"
    result = {field: parsed.get(field, "Unknown") or "Unknown" for field in fields}

    return result


def main():
    # Examples for testing
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)

    # sentence = "My name is John, my favourite drink is green tea, and my interests are robotics."
    # sentence = "I am John, I usually drink green tea, and I really like robotics. I also like to play chess and watch movies."
    # sentence = "Oh hi yeah, I'm John erm I drink tea usually green, and I am a robotics enthusiast. I also like to play chess and watch movies when I can."
    sentence = "I would like a vegan hamburger, no cheese please, and a large fries. I also want a large coke and a small salad."
    # query = f"Extract the following fields from the sentence:\n- Name\n- Favorite drink\n- Interests\n\nSentence: {sentence}"
    query = f"Extract the following fields from the sentence:\n -Food\n -Requests\n -Drink\n\nSentence: {sentence}."
    inference = LLMInference(config, query)
    response = inference.run_inference()
    print(response)
    inference.log_output(response)


if __name__ == "__main__":
    # main()
    # extract_fields_llm("Oh hi yeah, I'm John erm I drink tea usually green, and I am a robotics enthusiast. I also like to play chess and watch movies when I can.")
    # interest_commonality_llm(["robotics", "chess", "movies"])
    interest_commonality_llm(["tennis", "football", "basketball"])
