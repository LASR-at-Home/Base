#!/usr/bin/env python3
"""LLM agent using Ollama for inference.

Usage:
    agent = Agent(model="llama3.2")
    response = agent.query("What should I do next?")
"""

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
        """Send a prompt to the LLM and return the response text.

        Args:
            prompt: The user prompt.
            system_prompt: Override the instance system prompt for this call.

        Returns:
            The model's response as a string.
        """
        messages = []
        sp = system_prompt or self.system_prompt
        if sp:
            messages.append({"role": "system", "content": sp})
        messages.append({"role": "user", "content": prompt})

        response = self.client.chat(model=self.model, messages=messages)
        return response["message"]["content"]

    def query_json(self, prompt: str, system_prompt: Optional[str] = None) -> str:
        """Like query() but forces JSON output mode."""
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

    # VLM placeholder
    def query_vision(self, prompt: str, image_path: str) -> str:
        raise NotImplementedError("VLM support not yet implemented")

