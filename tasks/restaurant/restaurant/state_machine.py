import rclpy
import yasmin
import yasmin_ros
from std_msgs.msg import Empty
from lasr_skills import Say, GoToLocation, PlayMotion

from restaurant.states import Survey


class Restaurant(yasmin.StateMachine):
    def __init__(self, node, face_detection_confidence=0.2):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        def start_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return "succeeded"

        self.add_state(
            "WAIT_START",
            yasmin_ros.MonitorState(
                topic="/restaurant/start",
                outcomes=["succeeded", "failed"],
                monitor_handler=start_cb,
                msg_type=Empty,
            ),
            transitions={"succeeded": "SAY_START", "failed": "WAIT_START"},
        )

        self.add_state(
            "SAY_START",
            Say(text="Start of the restaurant task."),
            transitions={
                "succeeded": "FACE_TABLES",
                "aborted": "FACE_TABLES",
                "canceled": "FACE_TABLES",
            },
        )

        self.add_state(
            "FACE_TABLES",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "SURVEY",
                "aborted": "SURVEY",
                "canceled": "SURVEY",
            },
        )

        self.add_state(
            "SURVEY",
            Survey(node=node),
            transitions={
                "customer_found": "GO_TO_TABLE",
                "customer_not_found": "FACE_TABLES",
            },
        )

        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(),
            transitions={"succeeded": "TAKE_ORDER", "failed": "failed"},
        )

        self.add_state(
            "TAKE_ORDER",
            Say(text="Hello. What would you like to order?"),
            transitions={
                "succeeded": "GO_TO_BAR",
                "aborted": "GO_TO_BAR",
                "canceled": "GO_TO_BAR",
            },
        )

        self.add_state(
            "GO_TO_BAR",
            Say(text="Going to the bar."),
            transitions={"succeeded": "SERVE", "aborted": "SERVE", "canceled": "SERVE"},
        )

        self.add_state(
            "SURVE",
            Say(text="Here's your order"),
            transitions={
                "succeeded": "FACE_TABLES",
                "aborted": "FACE_TABLES",
                "canceled": "FACE_TABLES",
            },
        )


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        node_name="restaurant",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    sm = Restaurant(node=node, host_data={})
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"Restaurant outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
