from threading import Event

import yasmin
import yasmin_ros
from std_msgs.msg import Empty, String


class WaitForTabletReady(yasmin.State):
    """
    Publishes "ready" to /tablet/screen, then blocks until the operator
    presses the Ready button (/tablet/ready). Returns home screen after press.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded"])
        node = yasmin_ros.logger_node
        self._screen_pub = node.create_publisher(String, "/tablet/screen", 10)
        self._event = Event()
        node.create_subscription(Empty, "/tablet/ready", self._ready_cb, 10)

    def _ready_cb(self, msg: Empty) -> None:
        self._event.set()

    def execute(self, blackboard):
        self._event.clear()

        screen_msg = String()
        screen_msg.data = "ready"
        self._screen_pub.publish(screen_msg)

        self._event.wait()

        screen_msg.data = "home"
        self._screen_pub.publish(screen_msg)

        return "succeeded"
