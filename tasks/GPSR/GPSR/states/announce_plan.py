import yasmin
from GPSR.tts import say


class AnnouncePlan(yasmin.State):
    """Say the plan description before executing it."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded"])
        self.add_input_key("plan_description")
        self.add_input_key("steps")
        self.add_output_key("plan_description")
        self.add_output_key("steps")
        self.node = node

    def execute(self, blackboard):
        desc = blackboard["plan_description"]
        steps = blackboard["steps"] or []

        parts = []
        if desc:
            parts.append(desc)

        if len(steps) > 1:
            step_texts = []
            for i, step in enumerate(steps, 1):
                skill = step.get("skill", "")
                args = step.get("args", {})
                text = _step_to_text(i, skill, args)
                if text:
                    step_texts.append(text)
            if step_texts:
                parts.append("Here is my plan: " + ". ".join(step_texts) + ".")

        announcement = " ".join(parts)
        if not announcement:
            return "succeeded"

        self.node.get_logger().info(f"Announcing plan: {announcement}")
        say(self.node, announcement)
        return "succeeded"


def _step_to_text(i: int, skill: str, args: dict) -> str:
    if skill == "say":
        return f"Step {i}: say '{args.get('text', '')}'"
    if skill == "go_to_location":
        return f"Step {i}: go to {args.get('location', '')}"
    if skill == "find_object":
        return f"Step {i}: find the {args.get('object', '')} in the {args.get('location', '')}"
    if skill == "pick_up":
        return f"Step {i}: pick up the {args.get('object', '')}"
    if skill == "place_object":
        return f"Step {i}: place it on the {args.get('location', '')}"
    if skill == "give_to_person":
        return f"Step {i}: give the {args.get('object', '')} to the person"
    if skill == "find_person":
        desc = args.get("name") or args.get("pose") or args.get("gesture") or "person"
        loc = args.get("location", "")
        return f"Step {i}: find the {desc}{' in the ' + loc if loc else ''}"
    if skill == "guide_person":
        return f"Step {i}: guide {args.get('name', 'the person')} from {args.get('start', '')} to {args.get('end', '')}"
    if skill == "follow_person":
        return f"Step {i}: follow the person{' to ' + args['destination'] if args.get('destination') else ''}"
    if skill == "count_objects":
        return f"Step {i}: count {args.get('object', 'objects')} at {args.get('location', '')}"
    if skill == "count_people":
        desc = args.get("pose") or args.get("gesture") or "people"
        return f"Step {i}: count {desc} in the {args.get('location', '')}"
    if skill == "get_person_info":
        return f"Step {i}: get the {args.get('info', 'info')} of the person at {args.get('location', '')}"
    if skill == "find_object_by_property":
        return f"Step {i}: find the {args.get('property', '')} object at {args.get('location', '')}"
    if skill == "answer_question":
        return f"Step {i}: answer the question"
    return f"Step {i}: {skill}"
