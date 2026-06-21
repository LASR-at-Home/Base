import rclpy
import yasmin
import yasmin_ros
from std_msgs.msg import Empty
from lasr_skills import Say, GoToLocation, PlayMotion

from restaurant.states import (
    Survey,
    ApproachPerson,
    FaceCustomer,
    TakeOrderSM,
    GetOrderFromBar,
    SaveBarPose,
)


class Restaurant(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])

        def start_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return "succeeded"

        # self.add_state(
        #     "WAIT_START",
        #     yasmin_ros.MonitorState(
        #         topic_name="/restaurant/start",
        #         outcomes=["succeeded", "failed"],
        #         monitor_handler=start_cb,
        #         msg_type=Empty,
        #     ),
        #     transitions={"succeeded": "SAY_START", "failed": "WAIT_START"},
        # )

        self.add_state(
            "SAY_START",
            Say(text="Start of the restaurant task."),
            transitions={
                "succeeded": "SAVE_BAR_POSE",
                "aborted": "SAVE_BAR_POSE",
                "canceled": "SAVE_BAR_POSE",
            },
        )

        self.add_state(
            "SAVE_BAR_POSE",
            SaveBarPose(node=node),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )

        # self.add_state(
        #     "FACE_TABLES",
        #     PlayMotion(motion_name="look_centre"),
        #     transitions={
        #         "succeeded": "SURVEY",
        #         "aborted": "SURVEY",
        #         "canceled": "SURVEY",
        #     },
        # )

        # self.add_state(
        #     "SURVEY",
        #     Survey(node=node),
        #     transitions={"customer_found": "APPROACH"},
        # )

        # self.add_state(
        #     "APPROACH",
        #     ApproachPerson(),
        #     transitions={"succeeded": "GO_TO_TABLE", "failed": "SURVEY"},
        # )

        # self.add_state(
        #     "GO_TO_TABLE",
        #     GoToLocation(),
        #     transitions={"succeeded": "FACE_CUSTOMER", "failed": "SURVEY"},
        # )

        # self.add_state(
        #     "FACE_CUSTOMER",
        #     FaceCustomer(),
        #     transitions={"succeeded": "LOOK_AT_CUSTOMER", "failed": "LOOK_AT_CUSTOMER"},
        # )

        # self.add_state(
        #     "LOOK_AT_CUSTOMER",
        #     PlayMotion(motion_name="look_centre"),
        #     transitions={
        #         "succeeded": "GREET",
        #         "aborted": "GREET",
        #         "canceled": "GREET",
        #     },
        # )

        # self.add_state(
        #     "GREET",
        #     Say(
        #         text="Hello, my name is Rexy. I'm going to serve you today."
        #     ),
        #     transitions={
        #         "succeeded": "TAKE_ORDER",
        #         "aborted": "TAKE_ORDER",
        #         "canceled": "TAKE_ORDER",
        #     },
        # )

        # self.add_state(
        #     "TAKE_ORDER",
        #     TakeOrderSM(node=node),
        #     transitions={
        #         "succeeded": "GET_ORDER_FROM_BAR",
        #         "failed": "failed",
        #     },
        # )

        # self.add_state(
        #     "GET_ORDER_FROM_BAR",
        #     GetOrderFromBar(),
        #     transitions={
        #         "succeeded": "succeeded",
        #         "failed": "failed",
        #     },
        # )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="restaurant",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    yasmin_ros.set_ros_loggers(node)
    sm = Restaurant(node=node)
    sm.set_sigint_handler(True)
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"Restaurant outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
