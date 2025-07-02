import rospy
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import (
    GetPlanningScene,
    GetPlanningSceneRequest,
    ApplyPlanningSceneRequest,
    ApplyPlanningScene,
)
from moveit_msgs.msg import (
    CollisionObject,
    AllowedCollisionEntry,
    AttachedCollisionObject,
    PlanningScene,
)

from lasr_manipulation_msgs.srv import (
    AllowCollisionsWithObj,
    AllowCollisionsWithObjRequest,
    AllowCollisionsWithObjResponse,
    AttachObjectToGripper,
    AttachObjectToGripperRequest,
    AttachObjectToGripperResponse,
    DisallowCollisionsWithObj,
    DisallowCollisionsWithObjRequest,
    DisallowCollisionsWithObjResponse,
    DetachObjectFromGripper,
    DetachObjectFromGripperRequest,
    DetachObjectFromGripperResponse,
)


class PlanningSceneServices:
    """
    A collection of services to interface with the planning scene.
    """

    def __init__(self):
        self._planning_scene = PlanningSceneInterface()
        self._planning_scene.clear()

        # Planning scene services
        self._get_planning_scene = rospy.ServiceProxy(
            "/get_planning_scene", GetPlanningScene
        )
        self._get_planning_scene.wait_for_service()

        self._apply_planning_scene = rospy.ServiceProxy(
            "/apply_planning_scene", ApplyPlanningScene
        )
        self._apply_planning_scene.wait_for_service()

        self._allow_collisions_with_obj_service = rospy.Service(
            "/lasr_manipulation_planning_scene/allow_collisions_with_obj",
            AllowCollisionsWithObj,
            self._allow_collisions_with_obj,
        )

        self._disallow_collisions_with_obj_service = rospy.Service(
            "/lasr_manipulation_planning_scene/disallow_collisions_with_obj",
            DisallowCollisionsWithObj,
            self._disallow_collisions_with_obj,
        )

        self._attach_object_to_gripper_service = rospy.Service(
            "/lasr_manipulation_planning_scene/attach_object_to_gripper",
            AttachObjectToGripper,
            self._attach_object_to_gripper,
        )

        self._detach_object_from_gripper_service = rospy.Service(
            "/lasr_manipulation_planning_scene/detach_object_from_gripper",
            DetachObjectFromGripper,
            self._detach_object_from_gripper,
        )

    def _allow_collisions_with_obj(
        self, request: AllowCollisionsWithObjRequest
    ) -> AllowCollisionsWithObjResponse:
        obj_name = request.object_id
        allowed_links = [
            "gripper_left_finger_link",
            "gripper_right_finger_link",
            "gripper_link",
        ]

        # Get current ACM
        get_scene_req = GetPlanningSceneRequest()
        get_scene_req.components.components = (
            get_scene_req.components.ALLOWED_COLLISION_MATRIX
        )
        acm = self._get_planning_scene.call(
            get_scene_req
        ).scene.allowed_collision_matrix

        # Ensure all involved names are in ACM
        all_names = set(acm.entry_names)
        for name in [obj_name] + allowed_links:
            if name not in all_names:
                acm.entry_names.append(name)
                for entry in acm.entry_values:
                    entry.enabled.append(False)
                acm.entry_values.append(
                    AllowedCollisionEntry(enabled=[False] * len(acm.entry_names))
                )

        # Pad matrix
        for entry in acm.entry_values:
            while len(entry.enabled) < len(acm.entry_names):
                entry.enabled.append(False)

        # Enable specific collisions
        for link in allowed_links:
            obj_idx = acm.entry_names.index(obj_name)
            link_idx = acm.entry_names.index(link)
            acm.entry_values[obj_idx].enabled[link_idx] = True
            acm.entry_values[link_idx].enabled[obj_idx] = True

        # Set default policy
        if obj_name not in acm.default_entry_names:
            acm.default_entry_names.append(obj_name)
            acm.default_entry_values.append(False)
        else:
            idx = acm.default_entry_names.index(obj_name)
            acm.default_entry_values[idx] = False

        # Apply updated ACM
        req = ApplyPlanningSceneRequest()
        req.scene.allowed_collision_matrix = acm
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        self._apply_planning_scene.call(req)

        rospy.loginfo(f"Allowed collisions between '{obj_name}' and {allowed_links}")
        return AllowCollisionsWithObjResponse(success=True)

    def _disallow_collisions_with_obj(
        self, request: DisallowCollisionsWithObjRequest
    ) -> DisallowCollisionsWithObjResponse:
        obj_name = request.object_id
        disallowed_links = [
            "gripper_left_finger_link",
            "gripper_right_finger_link",
            "gripper_link",
        ]

        # Get current ACM
        get_scene_req = GetPlanningSceneRequest()
        get_scene_req.components.components = (
            get_scene_req.components.ALLOWED_COLLISION_MATRIX
        )
        acm = self._get_planning_scene.call(
            get_scene_req
        ).scene.allowed_collision_matrix

        if obj_name not in acm.entry_names:
            rospy.logwarn(f"Object '{obj_name}' not found in ACM entry_names.")
            return DisallowCollisionsWithObjResponse(success=False)

        all_names = set(acm.entry_names)
        for name in disallowed_links:
            if name not in all_names:
                acm.entry_names.append(name)
                for entry in acm.entry_values:
                    entry.enabled.append(False)
                acm.entry_values.append(
                    AllowedCollisionEntry(enabled=[False] * len(acm.entry_names))
                )

        # Pad matrix
        for entry in acm.entry_values:
            while len(entry.enabled) < len(acm.entry_names):
                entry.enabled.append(False)

        # Disable specific collisions
        obj_idx = acm.entry_names.index(obj_name)
        for link in disallowed_links:
            link_idx = acm.entry_names.index(link)
            acm.entry_values[obj_idx].enabled[link_idx] = False
            acm.entry_values[link_idx].enabled[obj_idx] = False

        # Update default entry
        if obj_name not in acm.default_entry_names:
            acm.default_entry_names.append(obj_name)
            acm.default_entry_values.append(True)
        else:
            idx = acm.default_entry_names.index(obj_name)
            acm.default_entry_values[idx] = True

        # Apply changes
        req = ApplyPlanningSceneRequest()
        req.scene.allowed_collision_matrix = acm
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        self._apply_planning_scene.call(req)

        rospy.loginfo(
            f"Disallowed collisions between '{obj_name}' and {disallowed_links}"
        )
        return DisallowCollisionsWithObjResponse(success=True)

    def _attach_object_to_gripper(
        self, request: AttachObjectToGripperRequest
    ) -> AttachObjectToGripperResponse:
        object_id = request.object_id
        link_name = "gripper_link"

        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = [
            "gripper_link",
            "gripper_left_finger_link",
            "gripper_right_finger_link",
        ]

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.robot_state.is_diff = True

        req = ApplyPlanningSceneRequest()
        req.scene = planning_scene
        self._apply_planning_scene.call(req)

        rospy.loginfo(f"Attached object '{object_id}' to '{link_name}'")
        return AttachObjectToGripperResponse(success=True)

    def _detach_object_from_gripper(
        self, request: DetachObjectFromGripperRequest
    ) -> DetachObjectFromGripperResponse:
        object_id = request.object_id
        link_name = "gripper_link"

        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.robot_state.is_diff = True

        req = ApplyPlanningSceneRequest()
        req.scene = planning_scene
        self._apply_planning_scene.call(req)

        rospy.loginfo(f"Detached object '{object_id}' from '{link_name}'")
        return DetachObjectFromGripperResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("/lasr_manipulation_planning_scene")
    planning_scene_services = PlanningSceneServices()
    rospy.spin()
