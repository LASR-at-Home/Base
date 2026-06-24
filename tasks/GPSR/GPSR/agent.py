"""Local LLM + cloud LLM dispatch."""

import asyncio
import json
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from typing import Callable, Optional

import ollama

from GPSR.planner import run_planner, run_announce


def _cloud_query(
    prompt: str,
    system_prompt: str,
    host: str,
    port: int,
    timeout_sec: float = 10.0,
) -> str:
    async def _go():
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port),
            timeout=timeout_sec,
        )
        try:
            msg = {"type": "query", "prompt": prompt}
            if system_prompt:
                msg["system_prompt"] = system_prompt
            writer.write((json.dumps(msg) + "\n").encode())
            await writer.drain()
            while True:
                line = await asyncio.wait_for(reader.readline(), timeout=timeout_sec)
                if not line:
                    raise ConnectionError("server closed connection")
                resp = json.loads(line.decode().strip())
                if resp.get("type") == "answer":
                    return resp["text"]
                if resp.get("type") == "error":
                    raise RuntimeError(resp.get("text", "cloud error"))
        finally:
            writer.close()
            await writer.wait_closed()

    return asyncio.run(_go())


class Agent:
    def __init__(
        self,
        model: str = "llama3.2",
        host: str = "http://localhost:11434",
        cloud_host: str = "",
        cloud_port: int = 8765,
        cloud_timeout_sec: float = 20.0,
        system_prompt: Optional[str] = None,
        use_cloud: bool = False,
    ):
        self._model = model
        self._host = host
        self._system_prompt = system_prompt
        self._cloud_host = cloud_host.strip()
        self._cloud_port = cloud_port
        self._cloud_timeout = cloud_timeout_sec
        self._use_cloud = use_cloud
        self._client = None if use_cloud else ollama.Client(host=host, timeout=300.0)

    @property
    def cloud_enabled(self) -> bool:
        return bool(self._cloud_host)

    @classmethod
    def from_node(cls, node, log: Optional[Callable[[str], None]] = None) -> "Agent":
        agent = cls(
            model=node.get_parameter("llm_model").value,
            host=node.get_parameter("llm_host").value,
            cloud_host=node.get_parameter("cloud_host").value,
            cloud_port=node.get_parameter("cloud_port").value,
            cloud_timeout_sec=node.get_parameter("cloud_timeout_sec").value,
        )
        if log:
            cloud = "off" if not agent.cloud_enabled else agent._cloud_host
            log(f"Agent ready | local={agent._model} | cloud={cloud}")
        return agent

    def query_json(self, prompt: str, system_prompt: Optional[str] = None) -> str:
        sp = system_prompt or self._system_prompt or ""
        if self._use_cloud:
            return _cloud_query(
                prompt, sp, self._cloud_host, self._cloud_port, self._cloud_timeout
            )
        response = self._client.chat(
            model=self._model,
            messages=[{"role": "user", "content": prompt}],
            format="json",
        )
        return response["message"]["content"]

    def _clone(self, use_cloud: bool) -> "Agent":
        return Agent(
            model=self._model,
            host=self._host,
            cloud_host=self._cloud_host,
            cloud_port=self._cloud_port,
            cloud_timeout_sec=self._cloud_timeout,
            system_prompt=self._system_prompt,
            use_cloud=use_cloud,
        )

    def plan(
        self,
        world: dict,
        command: str,
        log: Optional[Callable[[str], None]] = None,
    ) -> dict:
        if not self.cloud_enabled:
            plan = run_planner(self._clone(False), world, command)
            plan["source"] = "local"
            return plan

        deadline = time.monotonic() + self._cloud_timeout
        t0 = time.monotonic()
        pool = ThreadPoolExecutor(max_workers=2)
        local_f = pool.submit(run_planner, self._clone(False), world, command)
        cloud_f = pool.submit(run_planner, self._clone(True), world, command)
        local_plan = None
        cloud_failed = False

        try:
            pending = {local_f, cloud_f}
            while pending:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                done, pending = wait(
                    pending, timeout=remaining, return_when=FIRST_COMPLETED
                )
                for future in done:
                    if future is cloud_f:
                        try:
                            plan = future.result()
                        except Exception as e:
                            cloud_failed = True
                            if log:
                                log(f"Cloud unavailable ({e}) — using local")
                            if local_plan is not None:
                                local_plan["source"] = "local"
                                return local_plan
                            break
                        plan["source"] = "cloud"
                        if log:
                            log(
                                f"Cloud planner finished in {time.monotonic() - t0:.1f}s"
                            )
                        return plan
                    try:
                        local_plan = future.result()
                        if cloud_f.done():
                            try:
                                plan = cloud_f.result()
                                plan["source"] = "cloud"
                                if log:
                                    log(
                                        f"Cloud planner finished in {time.monotonic() - t0:.1f}s"
                                    )
                                return plan
                            except Exception as e:
                                cloud_failed = True
                                if log:
                                    log(f"Cloud unavailable ({e}) — using local")
                                local_plan["source"] = "local"
                                return local_plan
                        if cloud_failed:
                            if log:
                                log("Using local plan (cloud unavailable)")
                            local_plan["source"] = "local"
                            return local_plan
                        if log:
                            log(
                                "Local planner finished first — waiting for cloud until timeout"
                            )
                    except Exception as e:
                        if log:
                            log(f"Local planner failed: {e}")

            if local_plan is not None:
                local_plan["source"] = "local"
                return local_plan

            plan = local_f.result()
            plan["source"] = "local"
            if log:
                log("Using local plan")
            return plan
        finally:
            pool.shutdown(wait=False, cancel_futures=False)

    def announce(
        self,
        command: str,
        plan_description: str,
        steps: list,
        source: str = "local",
        log: Optional[Callable[[str], None]] = None,
    ) -> str:
        backend = self._clone(use_cloud=(source == "cloud"))
        try:
            return run_announce(backend, command, plan_description, steps)
        except Exception as e:
            if log:
                log(f"Announce failed ({e})")
            return ""
