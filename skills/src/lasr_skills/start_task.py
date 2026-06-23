from typing import Union
import traceback

import rclpy

import yasmin
from yasmin import StateMachine, State, Concurrence, Blackboard
import yasmin_ros
from yasmin_viewer import YasminViewerPub

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

from lasr_skills import DetectDoorOpening, SafeGoToLocation, PlayMotion


class StartDoorSM(StateMachine):  # TODO: Rename to start_task and move to Skills

    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = "start_pose",
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_state(
            "DETECT_DOOR_OPENING",
            DetectDoorOpening(),
            transitions={"door_opened": "PRE_NAV", "failed": "failed"},
        )

        self.add_state(
            "GO_TO_START",
            SafeGoToLocation(
                location_pose=location,
                location_param=location_param,
            ),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )


def main():
    rclpy.init()
    node = rclpy.create_node("hri")
    yasmin_ros.set_ros_loggers(node)

    try:
        sm = StartDoorSM()
        bb = Blackboard()

        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(f"StartDoorSM outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
        traceback.print_exc()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
