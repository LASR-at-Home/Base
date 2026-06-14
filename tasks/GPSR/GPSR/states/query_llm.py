import time

import yasmin

from GPSR.agent import Agent
from GPSR.planner import SkillSelectorError
from GPSR.world import build_world


class QueryLLM(yasmin.State):
    """Skill selector → refiner → planner (+ optional announce)."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("sequence")
        self.add_output_key("steps")
        self.node = node
        self.world = build_world(node)
        self.agent = Agent.from_node(node, log=self.node.get_logger().info)
        self.node.get_logger().info("QueryLLM ready (skill selector + refiner + planner).")

    def execute(self, blackboard):
        t0 = time.perf_counter()
        command = blackboard["sequence"].strip()
        self.node.get_logger().info(f"Query: '{command}'")

        # Stage 1 — skill selector
        # Stage 2 — skill refiner
        # Stage 3 — planner
        # (agent.plan runs all three; cloud + local in parallel)
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

        # Announce plan: LLM generates spoken summary as first say step
        if not (len(steps) == 1 and steps[0].get("skill") == "say"):
            announcement = self.agent.announce(
                command, plan["plan_description"], steps, source,
                log=lambda msg: self.node.get_logger().info(msg),
            )
            if announcement:
                steps = [{"skill": "say", "args": {"text": announcement}}] + steps

        blackboard["steps"] = steps

        label = "CLOUD" if source == "cloud" else "LOCAL"
        self.node.get_logger().info(
            f"=== PLAN SOURCE: {label} === | {plan['plan_description']} | "
            f"{len(steps)} steps | {time.perf_counter() - t0:.1f}s"
        )
        return "succeeded"
