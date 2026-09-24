"""Local LLM + cloud LLM dispatch."""

from typing import Callable, Optional

from openai import OpenAI

from GPSR.planner import run_planner, run_cloud_planner, run_announce


def _normalize_url(host: str, port: int | None = None) -> str:
    port = None
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


def _make_client(
    base_url: str, api_key: str = "none", timeout_sec: float = 300.0
) -> OpenAI:
    return OpenAI(base_url=base_url, api_key=api_key or "none", timeout=timeout_sec)


class Agent:
    def __init__(
        self,
        model: str = "gemma3:4b",
        host: str = "http://localhost:11434",
        cloud_model: str = "qwen3.5-9b",
        cloud_host: str = "",
        cloud_port: int = 8000,
        cloud_api_key: str = "none",
        cloud_timeout_sec: float = 40.0,
        system_prompt: Optional[str] = None,
        use_cloud: bool = False,
    ):
        self._model = cloud_model if use_cloud else model
        self._local_model = model
        self._cloud_model = cloud_model
        self._host = host
        self._cloud_host = cloud_host.strip()
        self._cloud_port = cloud_port
        self._cloud_api_key = cloud_api_key or "none"
        self._local_url = _normalize_url(host)
        self._cloud_url = _normalize_url(cloud_host, cloud_port)
        self._system_prompt = system_prompt
        self._cloud_timeout = cloud_timeout_sec
        self._use_cloud = use_cloud
        self._client = _make_client(
            self._cloud_url if use_cloud else self._local_url,
            api_key=self._cloud_api_key if use_cloud else "none",
            timeout_sec=300.0,
        )

    @property
    def cloud_enabled(self) -> bool:
        return self._use_cloud

    @classmethod
    def from_node(cls, node, log: Optional[Callable[[str], None]] = None) -> "Agent":
        agent = cls(
            model=node.get_parameter("llm_model").value,
            host=node.get_parameter("llm_host").value,
            cloud_model=node.get_parameter("cloud_model").value,
            cloud_host=node.get_parameter("cloud_host").value,
            cloud_port=node.get_parameter("cloud_port").value,
            cloud_api_key=node.get_parameter("cloud_api_key").value,
            cloud_timeout_sec=node.get_parameter("cloud_timeout_sec").value,
            use_cloud=node.get_parameter("use_cloud").value,
        )
        if log:
            cloud = "off" if not agent.cloud_enabled else agent._cloud_url
            log(
                f"Agent ready | local={agent._local_model} | cloud={agent._cloud_model} @ {cloud}"
            )
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
            model=self._local_model,
            host=self._host,
            cloud_model=self._cloud_model,
            cloud_host=self._cloud_host,
            cloud_port=self._cloud_port,
            cloud_api_key=self._cloud_api_key,
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
        if self.cloud_enabled:
            try:
                plan = run_cloud_planner(self._clone(True), world, command)
                plan["source"] = "cloud"
                if log:
                    log("Using cloud plan")
                return plan
            except Exception as e:
                if log:
                    log(f"Cloud unavailable ({e}) — using local")

        plan = run_planner(self._clone(False), world, command)
        plan["source"] = "local"
        return plan

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
