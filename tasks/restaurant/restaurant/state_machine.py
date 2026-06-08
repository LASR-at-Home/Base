import rclpy
import yasmin
import yasmin_ros
from std_msgs.msg import Empty
from lasr_skills import Say, GoToLocation, PlayMotion

from restaurant.states import Survey, ApproachPerson


class Restaurant(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        def start_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return "succeeded"

        self.add_state(
            "WAIT_START",
            yasmin_ros.MonitorState(
                topic_name="/restaurant/start",
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
            transitions={"customer_found": "APPROACH"},
        )

        self.add_state(
            "APPROACH",
            ApproachPerson(),
            transitions={"succeeded": "GO_TO_TABLE", "failed": "SURVEY"},
        )

        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(),
            transitions={"succeeded": "TAKE_ORDER", "failed": "SURVEY"},
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
            "SERVE",
            Say(text="Here's your order."),
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
    yasmin_ros.set_ros_loggers(node)
    sm = Restaurant(node=node)
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"Restaurant outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
