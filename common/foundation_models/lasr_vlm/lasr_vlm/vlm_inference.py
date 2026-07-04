# Script to perform VLM inference using a pre-trained model from Ollama
import os

import ollama
from dataclasses import dataclass
from typing import Optional
import re

import base64
from pathlib import Path


@dataclass
class ModelConfig:
    """
    Configuration class for the LLM or pipeline models.
    """

    model_name: str = "gemma3:4b"  #  qwen2.5vl:3b, llama3.2
    system_prompt: Optional[str] = None
    host: str = "http://localhost:11434"


def ensure_model(model: str):
    available = [m.model for m in ollama.list().models]
    if model not in available:
        print(f"Pulling model: {model}")
        for chunk in ollama.pull(model, stream=True):
            if chunk.status == "pulling manifest" or chunk.completed:
                pct = f"{chunk.completed/chunk.total*100:.1f}%" if chunk.total else ""
                print(f"\r{chunk.status} {pct}", end="", flush=True)
        print(f"\nDone.")


class VLMInference:
    def __init__(self, model_config: ModelConfig, new_model=True):
        self.model_name = model_config.model_name
        self.system_prompt = model_config.system_prompt
        self.client = ollama.Client(host=model_config.host)

        if new_model:
            ensure_model(self.model_name)

    def query_text(
        self, prompt: str, system_prompt: Optional[str] = None, force_json=False
    ) -> str:
        """Send a text prompt to the VLM and return the response as text.

        Args:
            prompt: The user prompt.
            system_prompt: Override the class system prompt for this call.
            force_json: Force JSON encoding of the response (if supported by the model).

        Returns:
            The model's response as a string.
        """
        messages = []
        sp = system_prompt or self.system_prompt
        if sp:
            messages.append({"role": "system", "content": sp})
        messages.append({"role": "user", "content": prompt})

        kwargs = {"model": self.model_name, "messages": messages}
        if force_json:
            kwargs["format"] = "json"

        response = self.client.chat(**kwargs)
        return response["message"]["content"]

    def query_vision(
        self,
        prompt: str,
        image_path: str,
        system_prompt: Optional[str] = None,
        force_json=False,
    ) -> str:
        """
        Send a prompt and an image to the VLM and return the response as text.
        Args:
            prompt: The user prompt.
            image_path: Path to the image file to be sent to the model.
            system_prompt: Override the class system prompt for this call.
            force_json: Force JSON encoding of the response (if supported by the model).
        """
        messages = []
        sp = system_prompt or self.system_prompt
        if sp:
            messages.append({"role": "system", "content": sp})

        image_data = base64.b64encode(Path(image_path).read_bytes()).decode("utf-8")
        messages.append(
            {
                "role": "user",
                "content": prompt,
                "images": [image_data],  # list of base64 strings
            }
        )

        kwargs = {"model": self.model_name, "messages": messages}
        if force_json:
            kwargs["format"] = "json"

        response = self.client.chat(**kwargs)
        return response["message"]["content"]


def extract_name_and_drink(input_sentence, inference: VLMInference):
    """
    Test VLM's ability to extract the name and drink from the sentence.
    """
    fields = ["name", "drink"]
    results = {}
    for field in fields:
        # system_query = f"Extract the following fields from the sentence:\n{field}\n\n For example, the sentence ' my favourite drink is coca cola' should have the field drink matched to 'coca cola'."
        user_query = f"Extract the following fields from the sentence:\n{field}\n\n For example, the sentence 'my favourite drink is coca cola' should have the field drink matched to 'coca cola'. Sentence: {input_sentence}"

        response = inference.query_text(prompt=user_query)
        results[field] = response

    return results


def test_vlm_text_query(inference: VLMInference):
    first_sentence = "My name is Eloise and my favourite drink is coca cola."
    second_sentence = "Oh hi yeah, I'm John erm I drink tea usually green, and I am a robotics enthusiast."

    first_result = extract_name_and_drink(first_sentence, inference)
    second_result = extract_name_and_drink(second_sentence, inference)

    print(f"First result: {first_result}")
    print(f"Second result: {second_result}")


def visually_describe_people(input_image, inference: VLMInference) -> dict[str, list]:
    """
    Test VLM's ability to visually describe the person in the image.
    """
    attributes = ["hair_color", "hair_length", "glasses", "hat", "shirt color"]

    user_query = build_prompt(attributes)

    print("Running VLM inference query")
    response = inference.query_vision(prompt=user_query, image_path=input_image)
    print(f"Raw VLM response: {response}")

    return parse_vlm_response(response, attributes)


def build_prompt(attributes):
    template = ", ".join(f"{attr}: _" for attr in attributes)

    boolean_attrs = {"glasses", "hat"}  # extend as needed
    has_boolean = any(a.lower() in boolean_attrs for a in attributes)

    prompt = (
        "Describe the person in the image using ONLY this exact format, "
        "replacing each underscore with your answer. Do not add any other text.\n\n"
        f"{template}\n"
    )

    if has_boolean:
        prompt += (
            "\nFor glasses and hat: answer true only if clearly visible, "
            "otherwise answer false.\n"
        )

    return prompt

def parse_vlm_response(response: str, attributes: list[str]) -> dict[str, list]:
    """
    Parse the VLM response string into a dictionary of attribute values.
    """
    result = {}
    for attr in attributes:
        value = parse_attribute(response.lower(), attr)
        result[attr] = value

    return result


def parse_attribute(response: str, attr: str) -> list:
    """
    Extract and parse values for a single attribute from the response.
    """
    if attr not in response:
        return []
    # print(f"Parsing {attr} response: {response}")

    raw = response.split(attr)[1].split(",")[0].lstrip(":").strip()
    # print(f"Parsing attribute '{attr}' with raw value: '{raw}'")
    values = raw.split(" and ") if " and " in raw else [raw]
    return [postprocess_value(v) for v in values]


def postprocess_value(value: str):
    """
    Postprocess the value by stripping whitespace and converting to lowercase.
    """
    STRIP_CHARS = str.maketrans("", "", " :.'")
    NO_PATTERN = re.compile(r"\bno\s+\w+", re.IGNORECASE)

    TRUE = {"true", "yes", "1"}
    FALSE = {"false", "no", "0"}

    if NO_PATTERN.match(value.strip()):
        return False

    cleaned = value.translate(STRIP_CHARS).strip()
    if cleaned in TRUE:
        return True
    if cleaned in FALSE:
        return False

    return cleaned


def test_vlm_vision_query():
    model_name = "gemma3:4b"  # "gemma3:4b", "qwen2.5vl:3b", "llama3.2"
    model_config = ModelConfig(model_name=model_name)
    ensure_model(model_name)
    inference = VLMInference(model_config, new_model=False)

    image_dir = f"{os.getcwd()}/test_images/more"
    for image_file in os.listdir(image_dir):
        image_path = f"{image_dir}/{image_file}"
        response: dict[str, list] = visually_describe_people(
            input_image=image_path, inference=inference
        )
        print(f"Vision response for {image_file}: {response}")


if __name__ == "__main__":
    # model_config = ModelConfig(model_name="moondream") # gemma3:4b
    # # ensure_model(model_config.model_name)
    # vlm_inference = VLMInference(model_config, new_model=False)

    # Test name and drink
    # test_vlm_text_query(vlm_inference)

    # Test visually describing people
    test_vlm_vision_query()

    ### Generic testing
    # text_response = vlm_inference.query_text("Please print Hello World")
    # print(text_response)
    #
    # image_dir = f"{os.getcwd()}/test_images"
    # image_path = f"{image_dir}/person2.jpg"
    # vision_response = vlm_inference.query_vision("What is in this image?", image_path)
    # print(vision_response)
