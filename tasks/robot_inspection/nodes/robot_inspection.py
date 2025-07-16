#!/usr/bin/python3
import rospy
import smach

from lasr_skills import GoToLocation
from geometry_msgs.msg import Pose, Point, Quaternion
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter, BoolParameter, IntParameter

post_door_pose = Pose(
    position=Point(-0.002303503362610609, -0.0026108751400454886, 0.0),
    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
)
inspection_pose = Pose(
    position=Point(0.20378854335550858, -0.2570748051763261, 0.0),
    orientation=Quaternion(0.0, 0.0, -0.44508701291892355, 0.8954873259465541),
)
leave_arena_pose = Pose(
    position=Point(1.4895076532485054, -3.6928155972684955, 0.0),
    orientation=Quaternion(0.0, 0.0, -0.7338231722854147, 0.6793405271415585),
)


class RobotInspection(smach.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded"])

        self._global_costmap_inflation_layer = rospy.ServiceProxy(
            "/move_base/global_costmap/inflation_layer/set_parameters", Reconfigure
        )
        self._velocity = rospy.ServiceProxy(
            "/move_base/PalLocalPlanner/set_parameters", Reconfigure
        )

        with self:

            smach.StateMachine.add(
                "SET_PRE_DOOR_PARAMETERS",
                smach.CBState(self._set_pre_door_parameters, outcomes=["succeeded"]),
                transitions={"succeeded": "GO_TO_OTHER_SIDE_OF_DOOR"},
            )

            smach.StateMachine.add(
                "GO_TO_OTHER_SIDE_OF_DOOR",
                GoToLocation(location=post_door_pose),
                transitions={
                    "succeeded": "SET_POST_DOOR_PARAMETERS",
                    "failed": "SET_POST_DOOR_PARAMETERS",
                },
            )

            smach.StateMachine.add(
                "SET_POST_DOOR_PARAMETERS",
                smach.CBState(self._set_post_door_parameters, outcomes=["succeeded"]),
                transitions={"succeeded": "GO_TO_INSPECTION_POINT"},
            )

            smach.StateMachine.add(
                "GO_TO_INSPECTION_POINT",
                GoToLocation(location=inspection_pose),
                transitions={
                    "succeeded": "WAIT_FOR_KEY_PRESS",
                    "failed": "WAIT_FOR_KEY_PRESS",
                },
            )
            smach.StateMachine.add(
                "WAIT_FOR_KEY_PRESS",
                smach.CBState(self._wait_key_press, outcomes=["succeeded"]),
                transitions={"succeeded": "GO_TO_LEAVE_ARENA"},
            )
            smach.StateMachine.add(
                "GO_TO_LEAVE_ARENA",
                GoToLocation(location=inspection_pose),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "succeeded",
                },
            )

    def _wait_key_press(self, userdata):
        input("Press any key to continue...")
        return "succeeded"

    def _set_pre_door_parameters(self, userdata):
        config = Config()
        config.doubles.append(DoubleParameter(name="inflation_radius", value=0.2))
        self._global_costmap_inflation_layer(config)

        config = Config()
        config.doubles.append(DoubleParameter(name="max_vel_x", value=0.2))
        self._velocity(config)
        return "succeeded"

    def _set_post_door_parameters(self, userdata):
        config = Config()
        config.doubles.append(DoubleParameter(name="inflation_radius", value=0.6))
        self._global_costmap_inflation_layer(config)
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("robot_inspection")
    sm = RobotInspection()
    sm.execute()
