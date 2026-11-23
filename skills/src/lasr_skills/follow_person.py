"""
Follow a person (the host) until they stop and confirm, by voice, that we have arrived.

The heavy lifting runs continuously in the `lasr_person_following` nodes (launch
`lasr_person_following person_following.launch.py`, plus the YOLO service and Nav2):

    person_tracker   YOLO detections -> Kalman multi-person tracker -> which one is the host
                     (never switches to someone else), keeps the head pointed at the host
    follow_manager   streams Nav2 NavigateToPose goals to stay behind the host (no cmd_vel)

This state machine only makes the high-level decisions:

    lock onto the person in front -> follow -> host stationary -> "have we arrived?"
        yes -> succeeded
        no  -> keep following
    host lost -> drive to where they were last seen and wait for them to be seen again
              -> not seen: ask them to come back and stand in front of the robot

Outcomes: "succeeded" (arrived), "failed" (could not find / keep the host).
"""

import json
import re
import threading
import time
from typing import Optional

import rclpy
import yasmin
import yasmin_ros
from yasmin import Blackboard, State, StateMachine
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from lasr_skills import AskAndListen, PlayMotion, Say

TRACKER = "/person_tracker"
MANAGER = "/follow_manager"


class FollowInterface:
    """Subscriptions and clients for the following nodes, shared by every state (one set
    per process, so building several FollowPerson machines costs nothing extra)."""

    _instance = None

    @classmethod
    def get(cls) -> "FollowInterface":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        node = yasmin_ros.logger_node
        self.node = node
        self._lock = threading.Lock()
        self.host = None  # latest person_tracker/host_state
        self.host_time = 0.0
        self.follow = None  # latest follow_manager/status
        node.create_subscription(String, f"{TRACKER}/host_state", self._host_cb, 10)
        node.create_subscription(String, f"{MANAGER}/status", self._follow_cb, 10)
        self.clients = {
            "enable": node.create_client(SetBool, f"{TRACKER}/enable"),
            "head_tracking": node.create_client(SetBool, f"{TRACKER}/head_tracking"),
            "lock_host": node.create_client(Trigger, f"{TRACKER}/lock_host"),
            "release_host": node.create_client(Trigger, f"{TRACKER}/release_host"),
            "new_leg": node.create_client(Trigger, f"{TRACKER}/new_leg"),
            "start": node.create_client(Trigger, f"{MANAGER}/start"),
            "stop": node.create_client(Trigger, f"{MANAGER}/stop"),
        }

    def _host_cb(self, msg):
        with self._lock:
            self.host = json.loads(msg.data)
            self.host_time = time.monotonic()

    def _follow_cb(self, msg):
        with self._lock:
            self.follow = json.loads(msg.data)

    def host_state(self, max_age: float = 2.0) -> Optional[dict]:
        with self._lock:
            if self.host is None or time.monotonic() - self.host_time > max_age:
                return None
            return dict(self.host)

    def follow_state(self) -> Optional[dict]:
        with self._lock:
            return None if self.follow is None else dict(self.follow)

    def call(self, name: str, data: Optional[bool] = None, timeout: float = 3.0, attempts: int = 3):
        """Calls a Trigger/SetBool service; returns the response or None.

        Retried: right after a server appears, DDS can drop the first response."""
        cli = self.clients[name]
        if not cli.wait_for_service(timeout_sec=10.0):
            yasmin.YASMIN_LOG_ERROR(f"FollowPerson: service {cli.srv_name} not available")
            return None
        for attempt in range(attempts):
            req = SetBool.Request(data=data) if data is not None else Trigger.Request()
            fut = cli.call_async(req)
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            if fut.done():
                return fut.result()
            cli.remove_pending_request(fut)
            yasmin.YASMIN_LOG_WARN(f"FollowPerson: no response from {cli.srv_name}, retrying")
        return None


class CallService(State):
    """Calls one or more following services in order. Always "succeeded" unless
    `required`, in which case a failed call gives "failed"."""

    def __init__(self, calls, required: bool = False):
        super().__init__(outcomes=["succeeded", "failed"])
        self.calls = calls  # list of (name, data or None)
        self.required = required

    def execute(self, blackboard):
        api = FollowInterface.get()
        for name, data in self.calls:
            res = api.call(name, data)
            ok = res is not None and res.success
            if not ok:
                yasmin.YASMIN_LOG_WARN(f"FollowPerson: {name} failed: {getattr(res, 'message', 'no response')}")
                if self.required:
                    return "failed"
        return "succeeded"


class WaitForHost(State):
    """Locks onto the person standing in front of the robot. "timeout" if nobody is
    there after `timeout` seconds."""

    def __init__(self, timeout: float = 15.0):
        super().__init__(outcomes=["succeeded", "timeout"])
        self.timeout = timeout

    def execute(self, blackboard):
        api = FollowInterface.get()
        end = time.monotonic() + self.timeout
        while time.monotonic() < end and not self.is_canceled():
            res = api.call("lock_host")
            if res is not None and res.success:
                yasmin.YASMIN_LOG_INFO(f"FollowPerson: {res.message}")
                return "succeeded"
            time.sleep(0.5)
        return "timeout"


class MonitorFollow(State):
    """Watches the host while follow_manager drives.

    arrived: host walked at least `min_travel` m, then stood still for `stationary_time`
             s, and the robot has closed in (or `settle_timeout` s more have passed)
    lost:    the tracker lost the host's track (it already coasted for a few seconds)
    failed:  the tracker stopped publishing
    """

    def __init__(self, stationary_time=4.0, min_travel=1.0, settle_timeout=6.0):
        super().__init__(outcomes=["arrived", "lost", "failed"])
        self.stationary_time = stationary_time
        self.min_travel = min_travel
        self.settle_timeout = settle_timeout

    def execute(self, blackboard):
        api = FollowInterface.get()
        no_data_since = None
        last_status = None
        while not self.is_canceled():
            time.sleep(0.1)
            host = api.host_state()
            if host is None:
                no_data_since = no_data_since or time.monotonic()
                if time.monotonic() - no_data_since > 5.0:
                    yasmin.YASMIN_LOG_ERROR("FollowPerson: person_tracker is not publishing")
                    return "failed"
                continue
            no_data_since = None
            if host["status"] != last_status:
                yasmin.YASMIN_LOG_INFO(f"FollowPerson: host {host['status']}")
                last_status = host["status"]
            if host["status"] == "lost":
                return "lost"
            if host["status"] == "no_host":
                return "lost"
            # a standing host's prediction is reliable: briefly unseen still counts
            if host["status"] == "coasting" and (host.get("last_seen_age") or 99.0) > 3.0:
                continue
            if host["travelled"] < self.min_travel or host["still_for"] < self.stationary_time:
                continue
            follow = api.follow_state() or {}
            if follow.get("settled") or host["still_for"] >= self.stationary_time + self.settle_timeout:
                yasmin.YASMIN_LOG_INFO(
                    f"FollowPerson: host stationary for {host['still_for']:.1f}s after "
                    f"{host['travelled']:.1f}m, robot {follow.get('distance')}m away"
                )
                return "arrived"
        return "failed"


class WaitForReturn(State):
    """The tracker lost the host. follow_manager keeps driving to where they were last
    seen; the tracker takes them back as soon as it is sure it sees them again.
    "found" if that happens within `timeout` seconds."""

    def __init__(self, timeout: float = 8.0):
        super().__init__(outcomes=["found", "not_found"])
        self.timeout = timeout

    def execute(self, blackboard):
        api = FollowInterface.get()
        yasmin.YASMIN_LOG_WARN("FollowPerson: host lost, waiting to see them again")
        end = time.monotonic() + self.timeout
        while time.monotonic() < end and not self.is_canceled():
            host = api.host_state()
            if host is not None and host["status"] in ("tracking", "coasting"):
                yasmin.YASMIN_LOG_INFO("FollowPerson: host found again")
                return "found"
            time.sleep(0.1)
        return "not_found"


class FollowPerson(StateMachine):
    """
    Wait for the host to stand in front of the robot, follow them until they stop and
    confirm (by voice) that we have arrived.

    Args:
        stationary_time: seconds the host must stand still before we ask if we arrived.
        max_lost_attempts: times to ask the host to come back before giving up.
        max_ask_attempts: unanswered/unclear arrival questions before assuming arrival.
        max_wait_attempts: 15 s waits for someone to stand in front before giving up.
        lost_wait: seconds to wait for a lost host to be seen again before asking them
            to come back.
    Other keyword arguments (from the previous implementation) are accepted and ignored.
    """

    def __init__(
        self,
        stationary_time: float = 4.0,
        max_lost_attempts: int = 3,
        max_ask_attempts: int = 3,
        max_wait_attempts: int = 4,
        lost_wait: float = 8.0,
        **_unused,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("transcribed_speech")
        self.max_lost_attempts = max_lost_attempts
        self.max_ask_attempts = max_ask_attempts
        self.max_wait_attempts = max_wait_attempts
        self._lost_attempts = 0
        self._ask_attempts = 0
        self._wait_attempts = 0
        any_outcome = lambda nxt: {"succeeded": nxt, "aborted": nxt, "canceled": nxt}  # noqa: E731
        # the torso/head motions are cosmetic here: never hang on one
        any_motion_outcome = lambda nxt: {**any_outcome(nxt), "timeout": nxt}  # noqa: E731

        self.add_state(
            "INIT",
            yasmin.CbState(outcomes=["done"], callback=self._reset_counters),
            transitions={"done": "SETUP"},
        )
        self.add_state(
            "SETUP",
            CallService([("stop", None), ("enable", True), ("release_host", None),
                         ("head_tracking", True)], required=True),
            transitions={"succeeded": "POST_NAV", "failed": "failed"},
        )
        # torso up: better view while the host gets into position
        self.add_state("POST_NAV", PlayMotion("post_navigation", timeout=15.0),
                       transitions=any_motion_outcome("WAIT_FOR_HOST"))
        self.add_state(
            "WAIT_FOR_HOST",
            WaitForHost(timeout=15.0),
            transitions={"succeeded": "HEAD_OFF", "timeout": "WAIT_COUNTER"},
        )
        self.add_state(
            "WAIT_COUNTER",
            yasmin.CbState(outcomes=["retry", "give_up"], callback=self._count_wait_attempt),
            transitions={"retry": "SAY_STAND_IN_FRONT", "give_up": "FINISH_FAILED"},
        )
        self.add_state(
            "SAY_STAND_IN_FRONT",
            Say(text="Please stand in front of me, so that I can follow you."),
            transitions=any_outcome("WAIT_FOR_HOST"),
        )
        # the motion moves the head too: pause head tracking so they don't fight
        self.add_state(
            "HEAD_OFF",
            CallService([("head_tracking", False)]),
            transitions={"succeeded": "PRE_NAV", "failed": "PRE_NAV"},
        )
        self.add_state("PRE_NAV", PlayMotion("pre_navigation", timeout=15.0),
                       transitions=any_motion_outcome("SAY_FOLLOW"))
        self.add_state(
            "SAY_FOLLOW",
            Say(text="I will follow you now. Please walk to the destination, and stop when we get there."),
            transitions=any_outcome("START"),
        )
        self.add_state(
            "START",
            CallService([("head_tracking", True), ("start", None)], required=True),
            transitions={"succeeded": "MONITOR", "failed": "FINISH_FAILED"},
        )
        self.add_state(
            "MONITOR",
            MonitorFollow(stationary_time=stationary_time),
            transitions={"arrived": "STOP_AT_HOST", "lost": "RECOVER", "failed": "FINISH_FAILED"},
        )

        # ---- host lost ----
        self.add_state(
            "RECOVER",
            WaitForReturn(timeout=lost_wait),
            transitions={"found": "MONITOR", "not_found": "STOP_LOST"},
        )
        self.add_state(
            "STOP_LOST",
            CallService([("stop", None), ("release_host", None)]),
            transitions={"succeeded": "LOST_COUNTER", "failed": "LOST_COUNTER"},
        )
        self.add_state(
            "LOST_COUNTER",
            yasmin.CbState(outcomes=["retry", "give_up"], callback=self._count_lost_attempt),
            transitions={"retry": "SAY_LOST", "give_up": "FINISH_FAILED"},
        )
        self.add_state(
            "SAY_LOST",
            Say(text="I have lost you. Please come back and stand in front of me."),
            transitions=any_outcome("WAIT_FOR_HOST"),
        )

        # ---- host stopped: ask if we arrived ----
        self.add_state(
            "STOP_AT_HOST",
            CallService([("stop", None)]),
            transitions={"succeeded": "ASK_IF_ARRIVED", "failed": "ASK_IF_ARRIVED"},
        )
        self.add_state(
            "ASK_IF_ARRIVED",
            AskAndListen(tts_phrase="Have we arrived? Please say yes or no."),
            transitions={"succeeded": "PROCESS_RESPONSE", "failed": "ASK_RETRY"},
        )
        self.add_state(
            "PROCESS_RESPONSE",
            yasmin.CbState(outcomes=["yes", "unknown", "no"], callback=self.parse_arrival_confirmation),
            transitions={"yes": "FINISH_SUCCEEDED", "unknown": "FEEDBACK_RESPONSE", "no": "NEW_LEG"},
        )
        self.add_state(
            "FEEDBACK_RESPONSE",
            Say(text="Sorry, I didn't understand that."),
            transitions=any_outcome("ASK_RETRY"),
        )
        self.add_state(
            "ASK_RETRY",
            yasmin.CbState(outcomes=["retry", "give_up"], callback=self._count_ask_attempt),
            # nobody answering: assume we arrived rather than hang
            transitions={"retry": "ASK_IF_ARRIVED", "give_up": "FINISH_SUCCEEDED"},
        )
        self.add_state(
            "NEW_LEG",
            CallService([("new_leg", None)]),
            transitions={"succeeded": "SAY_CONTINUE", "failed": "SAY_CONTINUE"},
        )
        self.add_state(
            "SAY_CONTINUE",
            Say(text="Okay, please lead the way."),
            transitions=any_outcome("START"),
        )

        # ---- clean up: stop driving, release the host, stop detecting ----
        cleanup = [("stop", None), ("release_host", None), ("enable", False)]
        self.add_state("FINISH_SUCCEEDED", CallService(cleanup), transitions={"succeeded": "succeeded", "failed": "succeeded"})
        self.add_state("FINISH_FAILED", CallService(cleanup), transitions={"succeeded": "failed", "failed": "failed"})

    # ---- callbacks ----

    def _reset_counters(self, blackboard):
        self._lost_attempts = 0
        self._ask_attempts = 0
        self._wait_attempts = 0
        return "done"

    def _count_wait_attempt(self, blackboard):
        self._wait_attempts += 1
        if self._wait_attempts >= self.max_wait_attempts:
            yasmin.YASMIN_LOG_ERROR("FollowPerson: nobody stood in front of me. Giving up.")
            return "give_up"
        return "retry"

    def _count_lost_attempt(self, blackboard):
        self._lost_attempts += 1
        self._wait_attempts = 0
        if self._lost_attempts >= self.max_lost_attempts:
            yasmin.YASMIN_LOG_ERROR(f"FollowPerson: lost the host {self._lost_attempts} times. Giving up.")
            return "give_up"
        return "retry"

    def _count_ask_attempt(self, blackboard):
        self._ask_attempts += 1
        if self._ask_attempts >= self.max_ask_attempts:
            yasmin.YASMIN_LOG_WARN("FollowPerson: no usable answer to the arrival question.")
            return "give_up"
        return "retry"

    def parse_arrival_confirmation(self, blackboard):
        response = str(blackboard["transcribed_speech"]).lower()
        yasmin.YASMIN_LOG_INFO(f"Received response: {response}")

        said_yes = re.search(r"\b(yes|yeah|yep|yup|arrived|correct)\b", response)
        said_no = re.search(r"\b(no|nope|not|nah)\b", response)

        if said_yes and not said_no:
            return "yes"
        if said_no and not said_yes:
            self._ask_attempts = 0
            return "no"
        return "unknown"


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = FollowPerson()
    sm.set_sigint_handler(True)
    bb = Blackboard()

    YasminViewerPub(sm, "Follow_Person")

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
