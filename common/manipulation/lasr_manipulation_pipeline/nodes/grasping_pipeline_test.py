#!/usr/bin/env python3
from typing import List

import rospy
import rospkg
import os
import actionlib
import open3d as o3d
import tf2_ros
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander

from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from sensor_msgs.msg import PointCloud2
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from moveit_msgs.srv import (
    GetPlanningScene,
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
import sys
import open3d as o3d
import numpy as np
import copy

from lasr_manipulation_pipeline import registration
from lasr_manipulation_pipeline import gpd
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import transformations
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import sam
from lasr_manipulation_pipeline import visualisation

PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "banana.ply"

"""
Ensure grasp approach is forwards.
"""


class GraspingPipeline:

    _mesh_path: str = os.path.join(PACKAGE_PATH, "meshes", MESH_NAME)
    _gpd_executable_path: str = "/opt/gpd/build/detect_grasps"
    _gpd_config_path: str = os.path.join(PACKAGE_PATH, "cfg", "tiago.cfg")
    _gpd_pcd_path: str = "/tmp/gpd.pcd"

    def __init__(self):
        self._planning_scene = PlanningSceneInterface(synchronous=True)
        self._planning_scene.clear()
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Setting up PlanningSceneInterface...")
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        self._move_group.clear_pose_targets()
        self._move_group.set_max_velocity_scaling_factor(0.5)
        self._move_group.clear_pose_targets()
        # self._move_group.set_pose_reference_frame("gripper_grasping_frame")

        rospy.loginfo("Setting up MoveGroupCommander...")
        self._play_motion_client = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        self._play_motion_client.wait_for_server()
        rospy.loginfo("Setup PlayMotion!")

        self._clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self._clear_octomap.wait_for_service()

        self._grasp_markers_pub = rospy.Publisher(
            "/grasp_markers", MarkerArray, queue_size=10, latch=True
        )
        self._collision_object_pub = rospy.Publisher(
            "/collision_object", CollisionObject, queue_size=1, latch=True
        )

        mesh = o3d.io.read_triangle_mesh(self._mesh_path)
        self._pcd = mesh.sample_points_poisson_disk(
            number_of_points=10000, init_factor=5
        )
        rospy.loginfo("Read mesh.")

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Planning scene services
        self._get_planning_scene = rospy.ServiceProxy(
            "/get_planning_scene", GetPlanningScene
        )
        self._get_planning_scene.wait_for_service()

        self._apply_planning_scene = rospy.ServiceProxy(
            "/apply_planning_scene", ApplyPlanningScene
        )
        self._apply_planning_scene.wait_for_service()

        self._close_gripper = rospy.ServiceProxy(
            "/parallel_gripper_controller/grasp", Empty
        )
        self._close_gripper.wait_for_service()

    def _publish_grasp_poses(self, poses: List[Pose], frame_id: str = "base_link"):
        marker_array = MarkerArray()
        for idx, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "grasp_poses"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.12  # shaft length
            marker.scale.y = 0.02  # shaft diameter
            marker.scale.z = 0.02  # head diameter
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self._grasp_markers_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(poses)} grasp pose markers.")

    def _play_motion(self, motion_name: str):
        goal = PlayMotionGoal()
        goal.skip_planning = False
        goal.motion_name = motion_name
        self._play_motion_client.send_goal_and_wait(goal)

    def _allow_collisions_with_obj(self, obj_name: str, allowed_links=None):
        if allowed_links is None:
            allowed_links = [
                "gripper_left_finger_link",
                "gripper_right_finger_link",
                "gripper_link",
            ]

        # Get current ACM
        request = GetPlanningScene()
        request.components = 0
        acm = self._get_planning_scene.call(request).scene.allowed_collision_matrix

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

        # Pad matrix (if lengths mismatch due to added entries)
        for entry in acm.entry_values:
            while len(entry.enabled) < len(acm.entry_names):
                entry.enabled.append(False)

        # Enable specific collisions
        for link in allowed_links:
            obj_idx = acm.entry_names.index(obj_name)
            link_idx = acm.entry_names.index(link)
            acm.entry_values[obj_idx].enabled[link_idx] = True
            acm.entry_values[link_idx].enabled[obj_idx] = True

        # Set default policy for the object (no collision unless specified)
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

    def _attach_object_to_gripper(
        self, object_id: str, link_name: str = "gripper_link"
    ):
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.ADD

        # Specify links allowed to touch the object (gripper links)
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

    def _detect_support_plane_beneath_object(self, full_scene_pcd, mesh, pcl_frame_id):
        """
        Detect the horizontal plane that the object lies on and add it to the planning scene.
        The plane could be a table, shelf, or floor. The object mesh and point cloud are first
        transformed to the base_footprint frame for consistent reasoning.
        """
        target_frame = "base_footprint"
        rospy.loginfo(f"Transforming mesh and point cloud to {target_frame}...")

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, pcl_frame_id, rospy.Time(0), rospy.Duration(1.0)
            )
            T = transformations.transform_to_matrix(transform.transform)
            mesh_base = copy.deepcopy(mesh).transform(T)
            pcd_base = copy.deepcopy(full_scene_pcd).transform(T)
        except Exception as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return

        rospy.loginfo("Segmenting horizontal planes in base_footprint...")

        planes = []
        remaining = copy.deepcopy(pcd_base)
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
            return

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
            return

        rospy.loginfo("Publishing detected support plane to planning scene...")

        _, plane_cloud, z_plane = best_plane
        aabb = plane_cloud.get_axis_aligned_bounding_box()
        center = aabb.get_center()
        extent = aabb.get_extent()

        # Create a flat box CollisionObject at the plane's height
        support = CollisionObject()
        support.id = "support_plane"
        support.header.frame_id = target_frame

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            extent[0] + 0.1,  # pad by 5 cm on each side
            extent[1] + 0.1,
            0.01,  # thin flat box
        ]

        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = z_plane - 0.005  # center of the thin box
        pose.orientation.w = 1.0

        support.primitives = [box]
        support.primitive_poses = [pose]
        support.operation = CollisionObject.ADD

        self._collision_object_pub.publish(support)
        rospy.loginfo("Support plane added to planning scene.")

    def _get_eef_pose(
        self,
        ee_frame: str = "gripper_grasping_frame",
        base_frame: str = "base_footprint",
    ) -> Pose:
        try:
            trans = self._tf_buffer.lookup_transform(
                base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logerr(
                f"Failed to get transform from {base_frame} to {ee_frame}: {e}"
            )
            return None

    def _determine_forward_distance(
        self,
        pregrasp_pose: Pose,
        mesh: o3d.geometry.PointCloud,
        max_advance: float = 0.12,
        step: float = 0.001,
        contact_threshold: float = 0.005,
        palm_clearance: float = 0.03,
    ) -> float:
        """
        Determines how far to move forward along the gripper's x-axis from the pregrasp pose
        to make contact with the object, while avoiding pushing through or colliding with it.
        """

        return 0.035

        if not mesh.has_points():
            raise ValueError("Mesh point cloud is empty.")

        # KD-tree for contact checking
        kdtree = o3d.geometry.KDTreeFlann(mesh)

        # Position and orientation
        origin = np.array(
            [
                pregrasp_pose.position.x,
                pregrasp_pose.position.y,
                pregrasp_pose.position.z,
            ]
        )
        quat = [
            pregrasp_pose.orientation.x,
            pregrasp_pose.orientation.y,
            pregrasp_pose.orientation.z,
            pregrasp_pose.orientation.w,
        ]
        rot_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(quat)
        forward_dir = rot_matrix[:, 0]  # Gripper's x-axis (world frame)
        forward_dir = forward_dir / np.linalg.norm(forward_dir)

        # Estimate object width in approach direction
        object_width = self._estimate_object_width_along_vector(mesh, forward_dir)
        max_safe_forward = max(0.0, 0.5 * object_width - palm_clearance)
        max_safe_forward = min(max_safe_forward, max_advance)

        # Raycast-style probe
        for d in np.arange(0, max_safe_forward, step):
            probe = origin - d * forward_dir
            [_, _, dists] = kdtree.search_knn_vector_3d(probe, 1)
            if dists and np.sqrt(dists[0]) <= contact_threshold:
                return max(0.0, d - palm_clearance)

        return max_safe_forward

    def _estimate_object_width_along_vector(
        self,
        mesh: o3d.geometry.PointCloud,
        approach_vector: np.ndarray,
    ) -> float:
        """
        Estimate the object's width in the direction of the grasp approach vector.
        """
        obb = mesh.get_oriented_bounding_box()
        obb_axes = obb.R  # Columns are principal axes
        obb_extents = obb.extent  # Lengths along each axis

        approach_vector = approach_vector / np.linalg.norm(approach_vector)

        # Project approach vector onto OBB axes
        projections = [abs(np.dot(approach_vector, axis)) for axis in obb_axes.T]
        width = sum(p * e for p, e in zip(projections, obb_extents))
        return width

    def run(self, debug: bool = False, execute: bool = True):
        if execute:
            rospy.loginfo("Sending pregrasp motion...")
            self._play_motion("pregrasp")
            rospy.loginfo("Pregrasp motion finished!")
            rospy.loginfo("Sending open_gripper motion...")
            self._play_motion("open_gripper")
            rospy.loginfo("Pregrasp open_gripper finished!")
            self._clear_octomap()

        # Clone the mesh point cloud
        mesh_pcd = o3d.geometry.PointCloud(self._pcd)

        # Get the live sensor point cloud
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        original_pcd = conversions.ros_to_o3d(pcl)

        if debug:
            o3d.visualization.draw_geometries(
                [original_pcd], window_name="Original scene"
            )

        pcl = sam.segment(pcl)
        scene_pcd = conversions.ros_to_o3d(pcl)

        if debug:
            o3d.visualization.draw_geometries(
                [scene_pcd], window_name="Segmented object"
            )

        o3d.io.write_point_cloud("scene.pcd", scene_pcd)

        # Register mesh with scene and apply the transformation

        target_pcd = conversions.ros_to_o3d(pcl)
        transform = registration.register_object(mesh_pcd, target_pcd)[0]
        rospy.loginfo("Registered mesh")

        mesh_pcd.transform(transform)
        rospy.loginfo("Transformation applied")

        if debug:
            o3d.visualization.draw_geometries(
                [mesh_pcd, target_pcd], window_name="Registration result"
            )

        # Estimate mesh from transformed point cloud
        mesh = o3d.io.read_triangle_mesh(self._mesh_path)
        mesh.transform(transform)
        mesh.compute_vertex_normals()
        self._detect_support_plane_beneath_object(
            original_pcd, mesh, pcl.header.frame_id
        )
        # self._add_support_plane_below_mesh(mesh, pcl.header.frame_id)

        # Convert to ROS mesh
        ros_mesh = conversions.o3d_mesh_to_ros_mesh(mesh)

        pose = Pose()
        pose.orientation.w = 1

        # Build CollisionObject with correct pose
        obj = CollisionObject()
        obj.id = "obj"
        obj.header.frame_id = pcl.header.frame_id  # still in camera frame
        obj.meshes = [ros_mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        # Publish collision object
        self._collision_object_pub.publish(obj)
        rospy.sleep(2.0)  # give time for planning scene to update.

        # Call GPD on the aligned mesh
        grasps, approaches, scores, openings = gpd.generate_grasps(
            self._gpd_pcd_path,
            self._gpd_executable_path,
            self._gpd_config_path,
            pcd=mesh_pcd,
        )
        rospy.loginfo(f"Detected {len(grasps)} grasps")
        if debug:
            visualisation.visualize_grasps_on_scene(
                mesh_pcd, grasps, "Mesh grasps (pre-filtering)"
            )

        # grasps, approaches, scores, openings = gpd.filter_grasps_by_score(
        #     grasps, approaches, scores, openings
        # )
        grasps, approaches, scores, openings = gpd.shift_or_filter_grasps_to_surface(
            mesh_pcd, grasps, approaches, scores, openings
        )
        grasps_base_footprint = transformations.tf_poses(
            grasps, pcl.header.frame_id, "base_footprint", self._tf_buffer
        )
        grasps_base_footprint = gpd.filter_by_approach_angles(grasps_base_footprint)
        grasps = transformations.tf_poses(
            grasps_base_footprint,
            "base_footprint",
            pcl.header.frame_id,
            self._tf_buffer,
        )

        if debug:
            visualisation.visualize_grasps_on_scene(mesh_pcd, grasps, "Mesh grasps")
            visualisation.visualize_grasps_on_scene(
                original_pcd, grasps, "Scene grasps"
            )
            visualisation.visualize_grasps_on_scene(target_pcd, grasps, "Object grasps")

        grasp_poses = transformations.tf_poses(
            grasps, pcl.header.frame_id, "gripper_grasping_frame", self._tf_buffer
        )
        grasp_poses = transformations.offset_grasps(grasp_poses, -0.12, 0.0, 0.0)
        grasp_poses_base_footprint = transformations.tf_poses(
            grasp_poses, "gripper_grasping_frame", "base_footprint", self._tf_buffer
        )
        grasp_poses_camera = transformations.tf_poses(
            grasp_poses, "gripper_grasping_frame", pcl.header.frame_id, self._tf_buffer
        )

        if debug:
            visualisation.visualize_grasps_on_scene(
                target_pcd, grasp_poses_camera, "Object grasps (offset)"
            )
        self._publish_grasp_poses(grasp_poses_base_footprint, "base_footprint")

        self._move_group.set_pose_reference_frame("base_footprint")
        self._move_group.set_pose_targets(
            grasp_poses_base_footprint, "gripper_grasping_frame"
        )

        if self._move_group.go(wait=True):
            self._allow_collisions_with_obj("obj")
            rospy.sleep(5.0)  # give time for planning scene to update
            eef_pose_base = self._get_eef_pose()
            eef_pose = transformations.tf_poses(
                [eef_pose_base],
                "base_footprint",
                "gripper_grasping_frame",
                self._tf_buffer,
            )[0]
            eef_target_pose = transformations.offset_grasps(
                [eef_pose],
                self._determine_forward_distance(eef_pose, mesh_pcd),
                0.0,
                0.0,
            )[0]
            eef_target_pose = transformations.tf_poses(
                [eef_target_pose],
                "gripper_grasping_frame",
                "base_footprint",
                self._tf_buffer,
            )[0]
            self._publish_grasp_poses([eef_target_pose], "base_footprint")
            self._move_group.clear_pose_targets()
            self._move_group.set_pose_target(eef_target_pose, "gripper_grasping_frame")
            if self._move_group.go(wait=True):
                self._move_group.attach_object(
                    "obj",
                    "gripper_link",
                    [
                        "gripper_link",
                        "gripper_left_finger_link",
                        "gripper_right_finger_link",
                    ],
                )
                self._close_gripper()
                rospy.sleep(2.0)
                self._play_motion("pregrasp")

        self._move_group.stop()


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run(debug=True, execute=True)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
