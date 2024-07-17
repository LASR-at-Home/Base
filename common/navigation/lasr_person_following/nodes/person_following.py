#!/usr/bin/env/python3
import rospy
import actionlib

from lasr_person_following.msg import (
    FollowAction,
    FollowGoal,
    FollowResult,
    FollowFeedback,
)

from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter, BoolParameter, IntParameter

from lasr_person_following import PersonFollower

import warnings

from std_msgs.msg import Empty


class PersonFollowingServer:
    _server: actionlib.SimpleActionServer
    _follower: PersonFollower

    def __init__(self) -> None:
        self._dynamic_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/set_parameters", Reconfigure
        )
        self._dynamic_velocity = rospy.ServiceProxy(
            "/move_base/PalLocalPlanner/set_parameters", Reconfigure
        )
        self._dynamic_recovery = rospy.ServiceProxy(
            "/move_base/set_parameters", Reconfigure
        )

        self._dynamic_local_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/inflation_layer/set_parameters", Reconfigure
        )

        self._update_params()
        rospy.sleep(1)
        print("Updated params")

        self._server = actionlib.SimpleActionServer(
            "follow_person", FollowAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._server.register_preempt_callback(self._preempt_cb)

        self._follower = PersonFollower()

        self._server.start()

    def _execute_cb(self, _: FollowGoal) -> None:
        print("Executing follow_person action")
        while not self._follower.begin_tracking():
            rospy.logwarn("No people found, retrying...")
            rospy.sleep(rospy.Duration(1.0))
        warnings.warn(
            "PersonFollowingServer does not handle preempting or cancelling yet, so the execution will continue until the person is lost."
        )
        warnings.warn("PersonFollowingServer does not provide feedback yet.")
        self._follower.follow()

        self._server.set_succeeded(FollowResult())

    def _preempt_cb(self) -> None:
        raise NotImplementedError("Preempt not implemented yet")

    def _update_params(self):
        config = Config()
        config.ints.append(IntParameter(name="width", value=4))
        config.ints.append(IntParameter(name="height", value=4))
        self._dynamic_costmap(config)

        config = Config()
        config.doubles.append(DoubleParameter(name="max_vel_x", value=0.6))

        self._dynamic_velocity(config)

        config = Config()
        config.bools.append(BoolParameter(name="recovery_behavior_enabled", value=1))
        config.bools.append(BoolParameter(name="clearing_rotation_allowed", value=0))

        self._dynamic_recovery(config)

        config = Config()
        config.bools.append(BoolParameter(name="enabled"), value=1)
        config.doubles.append(DoubleParameter(name="inflation_radius"), value=0.2)

        self._dynamic_local_costmap(config)


if __name__ == "__main__":
    rospy.init_node("person_following_server")
    server = PersonFollowingServer()
    rospy.loginfo("Person following server started")
    rospy.spin()
