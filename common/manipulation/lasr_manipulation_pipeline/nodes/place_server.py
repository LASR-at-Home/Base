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
        self._move_group.set_goal_tolerance(
            0.01
        )  # for placing we can be less precise than picking
        self._move_group.set_goal_orientation_tolerance(0.1)

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

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted before starting.")
            self._place_server.set_preempted()
            return

        # Clear any existing pose targets
        self._move_group.clear_pose_targets()

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted after clearing pose targets.")
            self._place_server.set_preempted()
            return

        # Set support surface
        self._move_group.set_support_surface_name(goal.surface_id)

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted after setting support surface.")
            self._place_server.set_preempted()
            return

        # Set a list of goal pose
        # Loop through candidate poses
        success = False
        for i, pose_stamped in enumerate(goal.candidate_poses):
            if self._place_server.is_preempt_requested():
                rospy.loginfo("Place goal preempted before pose %d.", i)
                self._place_server.set_preempted()
                return

            rospy.loginfo(f"Trying place pose {i+1}/{len(goal.candidate_poses)}")

            self._move_group.clear_pose_targets()
            self._move_group.set_pose_reference_frame(pose_stamped.header.frame_id)
            self._move_group.set_pose_target(pose_stamped.pose, "gripper_grasping_frame")

            move_success = self._move_group.go(wait=True)
            self._move_group.stop()

            if move_success:
                rospy.loginfo("Successfully reached candidate pose %d", i)
                success = True
                break
            else:
                rospy.logwarn("Failed to reach candidate pose %d", i)
        
        if not success:
            rospy.logwarn("All candidate poses failed. Aborting place.")
            result = PlaceResult(success=False)
            self._place_server.set_aborted(result)
            return

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted before executing motion.")
            self._place_server.set_preempted()
            return

        # Execute motion
        success = self._move_group.go(wait=True)
        if not success:
            rospy.loginfo("MoveIt failed to execute place. Aborting.")
            result = PlaceResult(success=False)
            self._place_server.set_aborted(result)
            return

        rospy.loginfo("Reached place pose")

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted after reaching place pose.")
            self._place_server.set_preempted()
            return

        # Release object
        self._open_gripper()

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted after opening gripper.")
            self._place_server.set_preempted()
            return

        # Detach object from gripper
        self._detach_object_from_gripper(goal.object_id)
        rospy.loginfo("Detached object from gripper in planning scene")

        if self._place_server.is_preempt_requested():
            rospy.loginfo("Place goal preempted after detaching object.")
            self._place_server.set_preempted()
            return

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
