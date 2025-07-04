import rospy
import actionlib

from moveit_commander import MoveGroupCommander

from std_srvs.srv import Empty


from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal, PlaceResult
from lasr_manipulation_msgs.srv import (
    DetachObjectFromGripper,
    DisallowCollisionsWithObj,
)


class PlaceServer:
    """
    An action server for placing objects.
    """

    # Action server
    _pick_server: actionlib.SimpleActionServer

    # MoveIt
    _move_group: MoveGroupCommander
    _open_gripper: rospy.ServiceProxy

    # Planning scene services
    _detach_object_from_gripper: rospy.ServiceProxy
    _disallow_collisions_with_obj: rospy.ServiceProxy

    def __init__(self) -> None:
        self._place_server = actionlib.SimpleActionServer(
            "/lasr_manipulation/place",
            PlaceAction,
            execute_cb=self._place,
            auto_start=False,
        )

        # Setup Move Group
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        self._move_group.set_max_velocity_scaling_factor(0.5)

        self._open_gripper = rospy.ServiceProxy(
            "/parallel_gripper_controller/release", Empty
        )
        self._open_gripper.wait_for_service()

        self._detach_object_from_gripper = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/detach_object_from_gripper",
            DetachObjectFromGripper,
        )
        self._detach_object_from_gripper.wait_for_service()
        self._disallow_collisions_with_obj = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/disallow_collisions_with_obj",
            DisallowCollisionsWithObj,
        )
        self._disallow_collisions_with_obj.wait_for_service()

        self._place_server.start()

        rospy.loginfo("/lasr_manipulation/place has started!")

    def _place(self, goal: PlaceGoal) -> None:

        # Clear any existing pose targets
        self._move_group.clear_pose_targets()

        # Set support surface
        self._move_group.set_support_surface_name(goal.surface_id)

        # Execute
        self._move_group.set_pose_reference_frame(goal.pose.header.frame_id)
        self._move_group.set_pose_target(goal.pose.pose, "gripper_grasping_frame")
        success = self._move_group.go(wait=True)
        if not success:
            rospy.loginfo("MoveIt failed to execute place. Aborting.")
            result = PlaceResult(success=False)
            self._place_server.set_aborted(result)
            return
        rospy.loginfo("Reached place pose")

        # Release object
        self._open_gripper()

        # Detach object from gripper
        self._detach_object_from_gripper(goal.object_id)
        rospy.loginfo("Detached object from gripper in planning scene")

        # Disallow collisions with object
        self._disallow_collisions_with_obj(goal.object_id)
        rospy.loginfo("Disallowed collisions with object in planning scene")

        result = PlaceResult(success=True)
        self._place_server.set_succeeded(result)
        rospy.loginfo("Place was successful")


if __name__ == "__main__":
    rospy.init_node("lasr_manipulation_placing")
    place_server = PlaceServer()
    rospy.spin()
