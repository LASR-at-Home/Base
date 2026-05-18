# Script to perform VLM inference using a pre-trained model from Ollama
import os

import ollama
from dataclasses import dataclass
from typing import Optional

import base64
from pathlib import Path

@dataclass
class ModelConfig:
    """
    Configuration class for the LLM or pipeline models.
    """
    model_name: str = "gemma3:4b" #  qwen2.5vl:3b, llama3.2
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
    def __init__(self, model_config: ModelConfig):
        self.model_name = model_config.model_name
        self.system_prompt = model_config.system_prompt
        self.client = ollama.Client(host=model_config.host)

        ensure_model(self.model_name)

    def query_text(self, prompt: str, system_prompt: Optional[str] = None, force_json=False) -> str:
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


    def query_vision(self, prompt: str, image_path: str, system_prompt: Optional[str] = None, force_json=False) -> str:
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
        messages.append({
            "role": "user",
            "content": prompt,
            "images": [image_data]  # list of base64 strings
        })

        kwargs = {"model": self.model_name, "messages": messages}
        if force_json:
            kwargs["format"] = "json"

        response = self.client.chat(**kwargs)
        return response["message"]["content"]


if __name__ == "__main__":
    vlm_inference = VLMInference(ModelConfig(model_name="moondream")) # gemma3:4
    text_response = vlm_inference.query_text("Please print Hello World")
    print(text_response)

    image_dir = f"{os.getcwd()}/test_images"
    image_path = f"{image_dir}/person2.jpg"
    vision_response = vlm_inference.query_vision("What is in this image?", image_path)
    print(vision_response)
