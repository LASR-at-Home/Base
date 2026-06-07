#!/usr/bin/env python3
"""LLM agent using Ollama for inference."""

from typing import Optional

import ollama


class Agent:
    def __init__(
        self,
        model: str = "llama3.2",
        system_prompt: Optional[str] = None,
        host: str = "http://localhost:11434",
    ):
        self.model = model
        self.system_prompt = system_prompt
        self.client = ollama.Client(host=host)

    def query(self, prompt: str, system_prompt: Optional[str] = None) -> str:
        messages = []
        sp = system_prompt or self.system_prompt
        if sp:
            messages.append({"role": "system", "content": sp})
        messages.append({"role": "user", "content": prompt})

        response = self.client.chat(model=self.model, messages=messages)
        return response["message"]["content"]

    def query_json(self, prompt: str, system_prompt: Optional[str] = None) -> str:
        messages = []
        sp = system_prompt or self.system_prompt
        if sp:
            messages.append({"role": "system", "content": sp})
        messages.append({"role": "user", "content": prompt})

        response = self.client.chat(
            model=self.model,
            messages=messages,
            format="json",
        )
        return response["message"]["content"]

    def query_vision(self, prompt: str, image_path: str) -> str:
        raise NotImplementedError("VLM support not yet implemented")
