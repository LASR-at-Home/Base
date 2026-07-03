"""Local LLM + cloud LLM dispatch."""

import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from typing import Callable, Optional

from openai import OpenAI

from GPSR.planner import run_planner, run_announce


def _normalize_url(host: str, port: int | None = None) -> str:
    host = (host or "").strip()
    if not host:
        return ""
    if host.startswith("http://") or host.startswith("https://"):
        return host
    if ":" in host and host.count(":") == 1:
        return f"http://{host}"
    if port is None:
        return f"http://{host}"
    return f"http://{host.rstrip('/')}:{port}"


def _make_client(base_url: str, timeout_sec: float = 300.0) -> OpenAI:
    return OpenAI(base_url=base_url, timeout=timeout_sec)


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
        self._cloud_host = cloud_host.strip()
        self._cloud_port = cloud_port
        self._local_url = _normalize_url(host)
        self._cloud_url = _normalize_url(cloud_host, cloud_port)
        self._system_prompt = system_prompt
        self._cloud_timeout = cloud_timeout_sec
        self._use_cloud = use_cloud
        self._client = _make_client(
            self._cloud_url if use_cloud else self._local_url,
            timeout_sec=300.0,
        )

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
            cloud = "off" if not agent.cloud_enabled else agent._cloud_url
            log(f"Agent ready | local={agent._model} | cloud={cloud}")
        return agent

    def query_json(self, prompt: str, system_prompt: Optional[str] = None) -> str:
        sp = system_prompt or self._system_prompt or ""
        messages = []
        if sp:
            messages.append({"role": "system", "content": sp})
        messages.append({"role": "user", "content": prompt})

        response = self._client.chat.completions.create(
            model=self._model,
            messages=messages,
        )
        choice = response.choices[0]
        message = getattr(choice, "message", None) or choice["message"]
        return (message.content or message["content"]).strip()

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
