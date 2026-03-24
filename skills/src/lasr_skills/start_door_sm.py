import rclpy
import smach
from skills import detect_door_opening, go_to_location


class StartDoorSM(smach.StateMachine):

    def __init__(
        self,
        node,
        location=None,
        location_param=None,
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_DOOR_OPENING",
                detect_door_opening.DetectDoorOpening(node),
                transitions={"door_opened": "GO_TO_START", "failed": "failed"},
            )
            smach.StateMachine.add(
                "GO_TO_START",
                go_to_location.GoToLocation(
                    node,
                    location=location,
                    location_param=location_param,
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


def main(args=None):
    rclpy.init(args=args)

    # Keep node name aligned with existing YAML section:
    # go_to_location:
    #   ros__parameters:
    #     start_pose: ...
    node = rclpy.create_node(
        "go_to_start",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    try:
        sm = StartDoorSM(node=node, location_param="start_pose")
        outcome = sm.execute()
        node.get_logger().info(f"StartDoorSM outcome: {outcome}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
