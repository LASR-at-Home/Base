import re
import time

import yasmin

from GPSR.agent import Agent
from GPSR.planner import PLAN_FAILED_TOKEN, SkillSelectorError, clean_transcription
from GPSR.world import build_world


class QueryLLM(yasmin.State):
    """Skill selector → refiner → planner (+ optional announce)."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("transcribed_speech")
        self.add_output_key("steps")
        self.node = node
        self.world = build_world(node)
        self.agent = Agent.from_node(node, log=self.node.get_logger().info)
        self.node.get_logger().info(
            "QueryLLM ready (skill selector + refiner + planner)."
        )

    def execute(self, blackboard):
        t0 = time.perf_counter()
        raw_command = blackboard["transcribed_speech"].strip()
        if self.agent.cloud_enabled:
            command = raw_command
        else:
            command, original = clean_transcription(self.agent, self.world, raw_command)
            if command != original:
                self.node.get_logger().info(
                    f"Transcription cleaned: '{original}' → '{command}'"
                )
        self.node.get_logger().info(f"Query: '{command}'")

        try:
            plan = self.agent.plan(
                self.world,
                command,
                log=lambda msg: self.node.get_logger().info(msg),
            )
        except SkillSelectorError as e:
            self.node.get_logger().error(f"Planner failed: {e}")
            return "failed"

        source = plan.get("source", "local")
        steps = plan["steps"]

        # For local: separate announce call. For cloud: announcement already in steps.
        if source != "cloud":
            announcement = self.agent.announce(
                command,
                plan["plan_description"],
                steps,
                source,
                log=lambda msg: self.node.get_logger().info(msg),
            )
            if announcement:
                announcement = re.sub(r'[{}\[\]"]', "", announcement).strip()
                matches = list(re.finditer(r"Step \d+:[^.]+\.", announcement))
                if matches:
                    announcement = announcement[: matches[-1].end()].strip()
                steps = [{"skill": "say", "args": {"text": announcement}}] + steps

        blackboard["steps"] = steps

        elapsed = time.perf_counter() - t0
        label = "CLOUD" if source == "cloud" else "LOCAL"
        is_failed = (
            len(steps) == 1
            and steps[0].get("skill") == "say"
            and steps[0].get("args", {}).get("text") == PLAN_FAILED_TOKEN
        )
        if is_failed:
            self.node.get_logger().warn(
                f"=== PLAN FAILED [{label}] {elapsed:.1f}s === {plan['plan_description']}"
            )
        else:
            self.node.get_logger().info(
                f"=== PLAN [{label}] {elapsed:.1f}s === {plan['plan_description']}"
            )
            for i, step in enumerate(steps, 1):
                args = ", ".join(f"{k}={v}" for k, v in step.get("args", {}).items())
                self.node.get_logger().info(f"  [{i}] {step['skill']}({args})")
        return "succeeded"
