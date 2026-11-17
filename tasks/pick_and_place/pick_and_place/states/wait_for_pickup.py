"""Announce-only mode: after "Please grab the X ...", wait until X has really left the table."""
import time
from types import SimpleNamespace

import yasmin
import yasmin_ros

from lasr_skills import Say
from pick_and_place import referee_view


class WaitForPickup(yasmin.State):
    """
    The robot has not moved since it detected the table, so every CHECK_PERIOD s it detects
    again on a new frame (same detector as DETECT_OBJECTS) and shows that frame on
    /referee_view, the selected object in orange while it is still there.

    X counts as taken when no detection named X is where X was, GONE_CHECKS times in a row
    (one missed frame, e.g. a hand in front of it, is not enough). While X is still seen, says
    so after FIRST_REMIND s and then every REMIND_EVERY s; after TIMEOUT s it goes on anyway,
    so the demo never hangs.
    Skipped when pick_and_place.grasp.enable is true (the robot grasps itself).

    Blackboard inputs:
        selected_object : Detection3D
    """

    CHECK_PERIOD = 1.0
    GONE_CHECKS = 2
    FIRST_REMIND = 5.0    # s into the wait (InstructPick already waited 3 s after speaking)
    REMIND_EVERY = 15.0
    TIMEOUT = 60.0

    def __init__(self, detector):
        super().__init__(outcomes=["succeeded"])
        self.add_input_key("selected_object")
        self._det = detector
        self.node = yasmin_ros.logger_node

    def _grasp_enabled(self):
        try:
            v = self.node.get_parameter("pick_and_place.grasp.enable").value
            return True if v is None else bool(v)   # GraspObject's default is on
        except Exception:
            return True

    def _same_place(self, old, box):   # both (cx, cy, w, h)
        if self._det._iou(old, box) > 0.2:
            return True
        dist = ((old[0] - box[0]) ** 2 + (old[1] - box[1]) ** 2) ** 0.5
        return dist < 0.5 * max(old[2], old[3], 40.0)

    def _say(self, text, blackboard):
        yasmin.YASMIN_LOG_INFO(f"Saying: {text}")
        try:
            Say(text=text).execute(blackboard)
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"say failed: {e}")

    def execute(self, blackboard) -> str:
        if self._grasp_enabled():
            return "succeeded"
        sel = blackboard["selected_object"]
        if sel is None or len(sel.xywh) < 4:
            return "succeeded"
        name = sel.name
        x, y, w, h = (float(v) for v in sel.xywh[:4])
        old = (x + w / 2, y + h / 2, w, h)
        yasmin.YASMIN_LOG_INFO(f"Waiting for the {name} to be picked up…")

        t0 = time.time()
        next_remind = t0 + self.FIRST_REMIND
        gone = 0
        while time.time() - t0 < self.TIMEOUT:
            time.sleep(self.CHECK_PERIOD)
            frame, dets = self._det.detect_once()
            if frame is None:
                continue
            objs = [SimpleNamespace(name=n, confidence=c,
                                    xywh=[int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)])
                    for n, c, (cx, cy, bw, bh) in dets]
            here = [o for o, (n, c, b) in zip(objs, dets) if n == name and self._same_place(old, b)]
            try:
                referee_view.show(self.node, frame, objs)
                if here:
                    referee_view.select(self.node, here[0])
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f"referee view: {e}")
            if here:
                gone = 0
            else:
                gone += 1
                if gone >= self.GONE_CHECKS:
                    yasmin.YASMIN_LOG_INFO(f"The {name} has been picked up.")
                    self._say("Thank you. Please follow me.", blackboard)
                    return "succeeded"
            if here and time.time() >= next_remind:   # still seen where it was
                self._say(f"I can see the {name} is still on the table. Please pick it up.", blackboard)
                next_remind = time.time() + self.REMIND_EVERY

        yasmin.YASMIN_LOG_WARN(f"The {name} still looks to be on the table after "
                               f"{self.TIMEOUT:.0f} s; going on anyway.")
        return "succeeded"
