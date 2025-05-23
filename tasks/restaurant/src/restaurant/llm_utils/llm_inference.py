# Script to perform LLM inference using a pre-trained model from HuggingFace

from dataclasses import dataclass
from typing import Optional, List
import re

import numpy as np
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

from utils import (
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
    def __init__(self, model_config: ModelConfig, query):
        self.config = model_config
        if self.config.quantize:
            # Quantize the model - for using 1/4 (or 1/8) of the GPU RAM. Full example in the models' HugginFace docs
            self.quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.bfloat16,
            )
        self.device = "cpu"

        self.model_name = self.config.model_name
        self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
        self.model = None
        self.query_list = self.process_query(query)

        if self.config.model_type == "pipeline":
            if self.config.task:
                self.task = self.config.task
            else:
                self.infer_task()
            self.model = self.load_pipeline_model()
            self.pipe = pipeline(
                task=self.task,
                model=self.model,
                tokenizer=self.tokenizer,
                device=self.device,
            )
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
        device_map = "auto" if self.device != "cpu" else "cpu"
        torch_dtype = torch.bfloat16 if self.device != "cpu" else torch.float32
        kwargs = {"device_map": device_map, "torch_dtype": torch_dtype}
        if self.config.quantize:
            kwargs["quantization_config"] = self.quantization_config

        return AutoModelForCausalLM.from_pretrained(self.model_name, **kwargs)

    def run_inference(self, context: Optional[str] = None) -> List:
        queries = self.query_list
        outputs = []

        for q in queries:
            result = None
            if self.config.model_type == "pipeline":
                if self.task == "question-answering":
                    if not context:
                        raise ValueError("Question Answering task requires context.")
                    result = self.pipe(question=q, context=context)
                else:
                    result = self.pipe(q)
            elif self.config.model_type == "llm":
                input_ids = self.tokenizer(q, return_tensors="pt").to(self.model.device)
                output_ids = self.model.generate(**input_ids, max_new_tokens=128)
                result = self.tokenizer.decode(output_ids[0], skip_special_tokens=True)
            generated_text = re.sub(re.escape(q), "", result).strip()
            outputs.append(generated_text)
        return outputs

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


def interest_commonality_llm(interests: List[str]) -> str:
    """
    Find common interests between a list of interests.
    :param interests: a list of interests
    :return: a sentence describing the commonalities
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)
    sentence = ", ".join(interests)
    query = create_query(sentence, "interest_commonality")
    inference = LLMInference(config, query)
    response = inference.run_inference()
    # print(response)
    parsed_response = truncate_llm_output(response[0])
    return parsed_response


def restaurant_llm(text: str, items: List[str]):
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)
    # bar_query = f"You are a robot taking an order in a restaurant. The menu consists of the following items: {','.join(items)}. The customer says: {text}. Output a JSON dict mapping items to quantities."
    bar_query = f"Extract the following fields from the sentence:\n -Food\n \nDrinks \nSentence: {text}."
    bar_inference = LLMInference(config, bar_query)
    bar_response = bar_inference.run_inference()
    parsed_response = parse_llm_output_to_dict(bar_response[0], ["Food", "Drinks"])
    return parsed_response


def extract_fields_llm(text: str, fields: List[str] = None):
    """
    A simple receptionist LLM inference function.
    :param text: The input sentence to process.
    :param fields: A list of fields to return.
    """
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=True)
    if fields is None:
        fields = ["Name", "Favourite drink", "Interests"]  # all for receptionist

    sentence = text
    query = create_query(sentence, "extract_fields", fields)
    inference = LLMInference(config, query)
    response = inference.run_inference()
    parsed_response = parse_llm_output_to_dict(response[0], fields)
    return parsed_response


def main():
    # Examples for testing
    config = ModelConfig(model_name=models["Qwen"], model_type="llm", quantize=False)

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
    # interest_commonality_llm(["tennis", "football", "basketball"])
    restaurant_llm(
        "Get me a black tea please, and an apple and some iced tea and another apple",
        ["black tea", "iced tea", "apple"],
    )
