from typing import Optional

import rospy
import rospkg
import open3d as o3d
import os
import copy
import numpy as np
import tf.transformations as tf
import tf2_geometry_msgs
import tf2_ros
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
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import (
    TransformStamped,
    Vector3Stamped,
    Pose,
    Point,
    Quaternion,
    Vector3,
    PoseStamped,
)
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as point_cloud2


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
    AddCollisionObject,
    AddCollisionObjectRequest,
    AddCollisionObjectResponse,
    RemoveCollisionObject,
    RemoveCollisionObjectRequest,
    RemoveCollisionObjectResponse,
    DetectAndAddSupportSurface,
    DetectAndAddSupportSurfaceRequest,
    DetectAndAddSupportSurfaceResponse,
    AddSupportSurface,
    AddSupportSurfaceRequest,
    AddSupportSurfaceResponse,
    RemoveSupportSurface,
    RemoveSupportSurfaceRequest,
    RemoveSupportSurfaceResponse,
)

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class PlanningSceneServices:
    """
    A collection of services to interface with the planning scene.
    """

    # Meshes
    _mesh_dir: str = os.path.join(
        rospkg.RosPack().get_path("lasr_manipulation_pipeline"), "meshes"
    )
    _mesh_path: str = "/tmp/mesh.ply"

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

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

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

        self._add_collision_object_service = rospy.Service(
            "/lasr_manipulation_planning_scene/add_collision_object",
            AddCollisionObject,
            self._add_collision_object,
        )

        self._remove_collision_object_service = rospy.Service(
            "/lasr_manipulation_planning_scene/remove_collision_object",
            RemoveCollisionObject,
            self._remove_collision_object,
        )

        self._detect_and_add_support_surface_service = rospy.Service(
            "/lasr_manipulation_planning_scene/detect_and_add_support_surface",
            DetectAndAddSupportSurface,
            self._detect_and_add_support_surface,
        )

        self._add_support_surface_service = rospy.Service(
            "/lasr_manipulation_planning_scene/add_support_surface",
            AddSupportSurface,
            self._add_support_surface,
        )

        self._remove_support_surface_service = rospy.Service(
            "/lasr_manipulation_planning_scene/remove_support_surface",
            RemoveSupportSurface,
            self._remove_support_surface,
        )

        self._clear_planning_scene_service = rospy.Service(
            "/lasr_manipulation_planning_scene/clear", Empty, self._clear_planning_scene
        )

        rospy.loginfo("lasr_manipulation_planning_scene services ready!")

    def _allow_collisions_with_obj(
        self, request: AllowCollisionsWithObjRequest
    ) -> AllowCollisionsWithObjResponse:
        obj_name = request.object_id
        allowed_links = [
            "gripper_left_finger_link",
            "gripper_right_finger_link",
            "gripper_link",
        ]

        # Check if object exists
        scene = self._get_planning_scene.call(GetPlanningSceneRequest()).scene
        if not any(obj.id == obj_name for obj in scene.world.collision_objects):
            rospy.logwarn(f"Object '{obj_name}' not found in planning scene.")
            return AllowCollisionsWithObjResponse(success=False)

        # Get ACM
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

        # Enable collisions
        for link in allowed_links:
            obj_idx = acm.entry_names.index(obj_name)
            link_idx = acm.entry_names.index(link)
            acm.entry_values[obj_idx].enabled[link_idx] = True
            acm.entry_values[link_idx].enabled[obj_idx] = True

        # Update default entries
        if obj_name not in acm.default_entry_names:
            acm.default_entry_names.append(obj_name)
            acm.default_entry_values.append(False)
        else:
            idx = acm.default_entry_names.index(obj_name)
            acm.default_entry_values[idx] = False

        # Apply
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

        # Check if object exists
        scene = self._get_planning_scene.call(GetPlanningSceneRequest()).scene
        if not any(obj.id == obj_name for obj in scene.world.collision_objects):
            rospy.logwarn(f"Object '{obj_name}' not found in planning scene.")
            return DisallowCollisionsWithObjResponse(success=False)

        # Get ACM
        get_scene_req = GetPlanningSceneRequest()
        get_scene_req.components.components = (
            get_scene_req.components.ALLOWED_COLLISION_MATRIX
        )
        acm = self._get_planning_scene.call(
            get_scene_req
        ).scene.allowed_collision_matrix

        # Ensure all links are in ACM
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

        # Disable collisions
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

        # Apply
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

        # Check if object exists
        scene = self._get_planning_scene.call(GetPlanningSceneRequest()).scene
        if not any(obj.id == object_id for obj in scene.world.collision_objects):
            rospy.logwarn(f"Object '{object_id}' not found in planning scene.")
            return AttachObjectToGripperResponse(success=False)

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

        # Check if object is currently attached
        scene = self._get_planning_scene.call(GetPlanningSceneRequest()).scene
        if not any(
            obj.object.id == object_id
            for obj in scene.robot_state.attached_collision_objects
        ):
            rospy.logwarn(f"Object '{object_id}' is not attached to the gripper.")
            return DetachObjectFromGripperResponse(success=False)

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

    def _transform_to_matrix(
        self, transform: TransformStamped, scale: Vector3Stamped
    ) -> np.ndarray:

        trans = transform.transform.translation
        quat = transform.transform.rotation
        rot_mat = tf.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]

        sx, sy, sz = scale.vector.x, scale.vector.y, scale.vector.z
        scale_mat = np.diag([sx, sy, sz])

        rs_mat = np.dot(rot_mat, scale_mat)

        T = np.eye(4)
        T[:3, :3] = rs_mat
        T[:3, 3] = [trans.x, trans.y, trans.z]

        return T

    def _load_and_transform_mesh(
        self, mesh_name: str, transform: TransformStamped, scale: Vector3Stamped
    ) -> Optional[o3d.geometry.TriangleMesh]:
        """
        Loads the mesh, applies a transformation and scaling, and returns the processed point cloud.
        """
        mesh_path = os.path.join(self._mesh_dir, f"{mesh_name}.ply")
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        if mesh.is_empty():
            rospy.logwarn("Mesh is empty, so no grasps will be generated.")
            return None

        rospy.loginfo(
            f"Read a mesh with {np.asarray(mesh.vertices).shape[0]} vertices."
        )

        T = self._transform_to_matrix(transform, scale)
        rospy.loginfo(f"Constructed non-rigid transformation matrix:\n{T}")

        mesh.transform(T)

        return mesh

    def _add_collision_object(
        self, request: AddCollisionObjectRequest
    ) -> AddCollisionObjectResponse:
        mesh = self._load_and_transform_mesh(
            request.mesh_name, request.transform, request.scale
        )

        if mesh is None:
            return AddCollisionObjectResponse(success=False)

        o3d.io.write_triangle_mesh(self._mesh_path, mesh)
        rospy.loginfo(f"Wrote mesh to {self._mesh_path}")

        self._planning_scene.add_mesh(
            request.object_id,
            PoseStamped(
                pose=Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1)),
                header=Header(frame_id=request.transform.header.frame_id),
            ),
            self._mesh_path,
        )
        rospy.loginfo(
            f"{request.object_id} ({request.mesh_name}) added to planning scene!"
        )
        return AddCollisionObjectResponse(success=True)

    def _remove_collision_object(
        self, request: RemoveCollisionObjectRequest
    ) -> RemoveCollisionObjectResponse:
        self._planning_scene.remove_world_object(request.object_id)
        rospy.loginfo(f"Removed {request.object_id} from planning scene!")
        return RemoveCollisionObjectResponse(success=True)

    def _detect_and_add_support_surface(
        self, request: DetectAndAddSupportSurfaceRequest
    ) -> DetectAndAddSupportSurfaceResponse:

        # Load mesh
        mesh = self._load_and_transform_mesh(
            request.mesh_name, request.transform, request.scale
        )
        if mesh is None:
            return DetectAndAddSupportSurfaceResponse(success=False)

        # Convert PointCloud2 to open3d
        scene_points = list(
            point_cloud2.read_points(
                request.scene_pcl, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        scene_pcd = o3d.geometry.PointCloud()
        scene_pcd.points = o3d.utility.Vector3dVector(np.array(scene_points))

        # Transform open3d pointclouds to base_footprint
        # This assumes that both mesh and scene are in pcl.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_footprint",
                request.scene_pcl.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            T = self._transform_to_matrix(
                transform, Vector3Stamped(vector=Vector3(1.0, 1.0, 1.0))
            )
            mesh_base = mesh.transform(T)
            scene_base = scene_pcd.transform(T)
        except Exception as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return DetectAndAddSupportSurfaceResponse(success=False)

        rospy.loginfo("Transformed pointclouds to base_footprint")

        rospy.loginfo(("Segmenting horizontal planes in base_footprint..."))

        # Plane detection
        planes = []
        remaining = copy.deepcopy(scene_base)
        max_planes = 10
        distance_threshold = 0.008
        min_plane_points = 80
        for _ in range(max_planes):
            try:
                plane_model, inliers = remaining.segment_plane(
                    distance_threshold=distance_threshold,
                    ransac_n=3,
                    num_iterations=1000,
                )
            except Exception as e:
                rospy.logwarn(f"Plane segmentation failed: {e}")
                break

            if len(inliers) < min_plane_points:
                break

            [a, b, c, d] = plane_model
            if abs(c) > 0.95:  # horizontal in base_footprint
                plane_cloud = remaining.select_by_index(inliers)
                planes.append((plane_model, plane_cloud))

            remaining = remaining.select_by_index(inliers, invert=True)

        if not planes:
            rospy.logwarn("No horizontal planes detected in base_footprint.")
            return DetectAndAddSupportSurfaceResponse(success=False)

        rospy.loginfo(f"Detected {len(planes)} candidate planes")

        mesh_aabb = mesh_base.get_axis_aligned_bounding_box()
        mesh_bottom = mesh_aabb.get_min_bound()[2]
        mesh_center_xy = mesh_aabb.get_center()[:2]

        best_plane = None
        best_gap = float("inf")

        GAP_TOLERANCE = 0.05  # allow up to 1 cm penetration
        MAX_PLANE_GAP = 0.06  # ignore planes far below object

        for model, cloud in planes:
            z_mean = np.mean(np.asarray(cloud.points)[:, 2])
            gap = mesh_bottom - z_mean
            rospy.loginfo(f"Plane candidate at z={z_mean:.3f}, gap={gap:.3f}")

            if -GAP_TOLERANCE <= gap < MAX_PLANE_GAP:
                # Check XY overlap
                plane_aabb = cloud.get_axis_aligned_bounding_box()
                min_xy = plane_aabb.get_min_bound()[:2]
                max_xy = plane_aabb.get_max_bound()[:2]
                if np.all(mesh_center_xy > min_xy) and np.all(mesh_center_xy < max_xy):
                    if gap < best_gap:
                        best_plane = (model, cloud, z_mean)
                        best_gap = gap

        if best_plane is None:
            rospy.logwarn("No suitable support plane found under object.")
            return DetectAndAddSupportSurfaceResponse(success=False)

        rospy.loginfo(f"Selected plane at z={best_plane[2]}")

        _, plane_cloud, z_plane = best_plane
        aabb = plane_cloud.get_axis_aligned_bounding_box()
        center = aabb.get_center()
        extent = aabb.get_extent()

        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = z_plane - 0.005  # center of the thin box
        pose.orientation.w = 1.0

        self._planning_scene.add_box(
            request.surface_id,
            PoseStamped(pose=pose, header=Header(frame_id="base_footprint")),
            size=(extent[0], extent[1], 0.01),
        )

        rospy.loginfo(f"Added {request.surface_id} to planning scene!")

        return DetectAndAddSupportSurfaceResponse(success=True)

    def _add_support_surface(
        self, request: AddSupportSurfaceRequest
    ) -> AddSupportSurfaceResponse:

        if request.pose.header.frame_id != "base_footprint":
            rospy.logwarn("Supplied pose is not in base_footprint, so transforming it.")

        try:
            transform = self._tf_buffer.lookup_transform(
                "base_footprint",
                request.pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            request.pose = tf2_geometry_msgs.do_transform_pose(request.pose, transform)
        except Exception as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return AddSupportSurfaceResponse(success=False)

        self._planning_scene.add_box(
            request.surface_id,
            request.pose,
            size=(request.dimensions.x, request.dimensions.y, request.dimensions.z),
        )

        rospy.loginfo(f"Added {request.surface_id} to planning scene!")

        return AddSupportSurfaceResponse(success=True)

    def _remove_support_surface(
        self, request: RemoveSupportSurfaceRequest
    ) -> RemoveSupportSurfaceResponse:
        self._planning_scene.remove_world_object(request.object_id)
        rospy.loginfo(f"Removed {request.surface_id} from planning scene!")
        return RemoveSupportSurfaceResponse(success=True)

    def _clear_planning_scene(self, request: EmptyRequest) -> EmptyResponse:
        self._planning_scene.clear()
        rospy.loginfo("Cleared planning scene!")
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("lasr_manipulation_planning_scene")
    planning_scene_services = PlanningSceneServices()
    rospy.spin()
