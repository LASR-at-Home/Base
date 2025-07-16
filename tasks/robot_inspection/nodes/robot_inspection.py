#!/usr/bin/python3
import rospy
import smach

from lasr_skills import GoToLocation, DetectDoorOpening, Say
from geometry_msgs.msg import Pose, Point, Quaternion
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter, BoolParameter, IntParameter

post_door_pose = Pose(
    position=Point(2.9985576089128654, -0.47735297718997116, 0.0),
    orientation=Quaternion(0.0, 0.0, -0.22572001116393442, 0.9741922174602676),
)

inspection_pose = Pose(
    position=Point(7.242726156271562, 1.6964588646624763, 0.0),
    orientation=Quaternion(0.0, 0.0, -0.12563732904153996, 0.9920762377717288),
)

leave_arena_pose = Pose(
    position=Point(5.031592026750594, 7.318737301317226, 0.0),
    orientation=Quaternion(0.0, 0.0, 9306984006983658, 0.3657874887656822),
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
                "SAY_READY",
                Say("I am for inspection. Waiting for the door to open"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "preempted": "WAIT_DOOR_OPEN",
                    "aborted": "WAIT_DOOR_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_DOOR_OPEN",
                DetectDoorOpening(),
                transitions={"door_opened": "SET_PRE_DOOR_PARAMETERS"},
            )

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
                    "succeeded": "SAY_INSPECT_ME",
                    "failed": "SAY_INSPECT_ME",
                },
            )

            smach.StateMachine.add(
                "SAY_INSPECT_ME",
                Say("I am at the inspection point"),
                transitions={
                    "succeeded": "WAIT_FOR_KEY_PRESS",
                    "preempted": "WAIT_FOR_KEY_PRESS",
                    "aborted": "WAIT_FOR_KEY_PRESS",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_KEY_PRESS",
                smach.CBState(self._wait_key_press, outcomes=["succeeded"]),
                transitions={"succeeded": "SAY_LEAVING"},
            )

            smach.StateMachine.add(
                "SAY_LEAVING",
                Say("I am leaving the arena"),
                transitions={
                    "succeeded": "GO_TO_LEAVE_ARENA",
                    "preempted": "GO_TO_LEAVE_ARENA",
                    "aborted": "GO_TO_LEAVE_ARENA",
                },
            )

            smach.StateMachine.add(
                "GO_TO_LEAVE_ARENA",
                GoToLocation(location=inspection_pose),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "GO_TO_LEAVE_ARENA",
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
