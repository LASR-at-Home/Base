#!/usr/bin/env python3
"""LLM proxy — receives prompt + optional system_prompt, returns text."""

import json
import os
import socketserver
import sys

from openai import OpenAI

HOST = os.environ.get("LLM_SERVER_HOST", "0.0.0.0")
PORT = int(os.environ.get("LLM_SERVER_PORT", "8765"))
OPENAI_URL = os.environ.get("ENDPOINT_URL") or os.environ.get("OPENAI_URL", "")
API_KEY = os.environ.get("API_KEY") or os.environ.get("OPENAI_API_KEY", "")
MODEL = os.environ.get("LLM_MODEL") or os.environ.get("OPENAI_MODEL", "")

_client: OpenAI | None = None


def _send(wfile, msg: dict) -> None:
    wfile.write((json.dumps(msg) + "\n").encode())
    wfile.flush()


def _query(prompt: str, system_prompt: str = "") -> str:
    messages = []
    if system_prompt:
        messages.append({"role": "system", "content": system_prompt})
    messages.append({"role": "user", "content": prompt})
    response = _client.chat.completions.create(model=MODEL, messages=messages)
    return response.choices[0].message.content or ""


class Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        print(
            f"Client connected: {self.client_address[0]}:{self.client_address[1]}",
            flush=True,
        )
        while True:
            line = self.rfile.readline()
            if not line:
                break
            try:
                msg = json.loads(line.decode().strip())
            except json.JSONDecodeError:
                _send(self.wfile, {"type": "error", "text": "invalid json"})
                continue
            if msg.get("type") != "query":
                _send(self.wfile, {"type": "error", "text": "expected type=query"})
                continue
            prompt = (msg.get("prompt") or "").strip()
            if not prompt:
                _send(self.wfile, {"type": "error", "text": "empty prompt"})
                continue
            system_prompt = (msg.get("system_prompt") or "").strip()
            print(f"Query ({len(prompt)} chars)", flush=True)
            try:
                text = _query(prompt, system_prompt)
            except Exception as e:
                print(f"LLM error: {e}", flush=True)
                _send(self.wfile, {"type": "error", "text": str(e)})
                continue
            print(f"Answer ({len(text)} chars)", flush=True)
            _send(self.wfile, {"type": "answer", "text": text})


class Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    global _client
    if not API_KEY or not OPENAI_URL or not MODEL:
        print("Set ENDPOINT_URL, API_KEY, LLM_MODEL", file=sys.stderr)
        sys.exit(1)
    _client = OpenAI(base_url=OPENAI_URL, api_key=API_KEY, timeout=300.0)
    with Server((HOST, PORT), Handler) as server:
        print(f"LLM proxy on {HOST}:{PORT} | model={MODEL}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
