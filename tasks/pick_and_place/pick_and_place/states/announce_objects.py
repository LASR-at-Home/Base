"""After detection: say what is on the table and where each object goes.
The first time: the full speech. Back at the table later: "I can still see ..."."""
import yasmin
import yasmin_ros

from lasr_skills import Say
from pick_and_place.categories import announcement, _join


class AnnounceObjects(yasmin.State):
    """
    "I can see a coke can, a paper cup ... The paper cup and the tea spoon are tableware,
    so they go in the dishwasher. ..." Objects are named left to right in the image.
    The routing rule is DecideDestination's (pick_and_place.categories).
    Later rounds (back at the table): "I can still see a paper cup and a tea spoon.";
    nothing when the table is empty (FINISH speaks then).

    Blackboard inputs:
        detected_objects : List[Detection3D]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded"])
        self.add_input_key("detected_objects")
        self.node = yasmin_ros.logger_node
        self._rounds = 0

    def execute(self, blackboard) -> str:
        self._rounds += 1
        dets = list(blackboard["detected_objects"] or [])
        left_to_right = [d.name for d in sorted(dets, key=lambda d: d.xywh[0] if len(d.xywh) else 0)]
        if self._rounds == 1:
            try:
                trash = self.node.get_parameter("pick_and_place.trash_category").value or ""
            except Exception:
                trash = ""
            text = announcement(left_to_right, str(trash))
        elif left_to_right:
            names = list(dict.fromkeys(left_to_right))
            art = lambda n: ("an " if n[0].lower() in "aeiou" else "a ") + n
            text = "I can still see " + _join([art(n) for n in names]) + "."
        else:
            return "succeeded"
        yasmin.YASMIN_LOG_INFO(f"Announcing: {text}")
        Say(text=text).execute(blackboard)
        return "succeeded"
