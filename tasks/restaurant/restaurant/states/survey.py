import rclpy
import yasmin
from lasr_skills import PlayMotion
from .detect_wave import DetectWave


class Survey(yasmin.StateMachine):
    def __init__(self, node, target_frame="map"):
        super().__init__(outcomes=["customer_found"])
        self.node = node

        self.add_state(
            "LOOK_LEFT",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "DETECT_LEFT",
                "aborted": "LOOK_CENTRE",
                "canceled": "LOOK_CENTRE",
            },
        )
        self.add_state(
            "DETECT_LEFT",
            DetectWave(target_frame=target_frame),
            transitions={
                "waving": "customer_found",
                "not_waving": "LOOK_CENTRE",
                "failed": "LOOK_CENTRE",
            },
        )

        self.add_state(
            "LOOK_CENTRE",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "DETECT_CENTRE",
                "aborted": "LOOK_RIGHT",
                "canceled": "LOOK_RIGHT",
            },
        )
        self.add_state(
            "DETECT_CENTRE",
            DetectWave(target_frame=target_frame),
            transitions={
                "waving": "customer_found",
                "not_waving": "LOOK_RIGHT",
                "failed": "LOOK_RIGHT",
            },
        )

        self.add_state(
            "LOOK_RIGHT",
            PlayMotion(motion_name="look_right"),
            transitions={
                "succeeded": "DETECT_RIGHT",
                "aborted": "LOOK_LEFT",
                "canceled": "LOOK_LEFT",
            },
        )
        self.add_state(
            "DETECT_RIGHT",
            DetectWave(target_frame=target_frame),
            transitions={
                "waving": "customer_found",
                "not_waving": "LOOK_LEFT",
                "failed": "LOOK_LEFT",
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="survey",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    sm = Survey(node=node, target_frame="odom")
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"Survey outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
