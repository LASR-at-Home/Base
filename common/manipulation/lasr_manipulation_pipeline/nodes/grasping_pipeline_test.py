r#!/usr/bin/env python3
from typing import Tuple, List

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

from lasr_manipulation_pipeline import registration
from lasr_manipulation_pipeline import gpd
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import transformations
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import sam
from lasr_manipulation_pipeline import visualisation

PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "starbucks_coffee.ply"


class GraspingPipeline:

    _mesh_path: str = os.path.join(PACKAGE_PATH, "meshes", MESH_NAME)  # path to a .ply
    _gpd_executable_path: str = "/opt/gpd/build/detect_grasps"
    _gpd_config_path: str = (
        "/home/jared/robocup/Base/common/third_party/gpd_ros/cfg/tiago.cfg"
    )
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

        self._pcd = o3d.io.read_point_cloud(self._mesh_path)
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
        Transform mesh and point cloud to base_footprint, detect a horizontal plane underneath,
        and publish it as a CollisionObject.
        """
        import copy
        import numpy as np
        import open3d as o3d

        target_frame = "base_footprint"
        rospy.loginfo(f"Transforming mesh and point cloud to {target_frame}...")

        # Transform mesh to base_footprint
        mesh_tf = PoseStamped()
        mesh_tf.header.frame_id = pcl_frame_id
        mesh_tf.pose.orientation.w = 1.0
        mesh_tf.header.stamp = rospy.Time(0)

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

        rospy.loginfo("Detecting horizontal planes in base_footprint...")

        planes = []
        remaining = copy.deepcopy(pcd_base)
        max_planes = 8
        distance_threshold = 0.008

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

            if len(inliers) < 80:
                break

            [a, b, c, d] = plane_model
            if abs(c) > 0.95:  # horizontal in base_footprint
                plane_cloud = remaining.select_by_index(inliers)
                planes.append((plane_model, plane_cloud))

            remaining = remaining.select_by_index(inliers, invert=True)

        if not planes:
            rospy.logwarn("No horizontal planes detected in base_footprint.")
            return

        # Find bottom of the mesh in Z
        mesh_aabb = mesh_base.get_axis_aligned_bounding_box()
        mesh_bottom = mesh_aabb.get_min_bound()[2]
        mesh_center_xy = mesh_aabb.get_center()[:2]

        best_plane = None
        min_gap = float("inf")

        for model, cloud in planes:
            z_mean = np.mean(np.asarray(cloud.points)[:, 2])
            gap = mesh_bottom - z_mean
            if 0.0 < gap < min_gap:
                # Optional: Check x/y overlap
                aabb = cloud.get_axis_aligned_bounding_box()
                min_xy = aabb.get_min_bound()[:2]
                max_xy = aabb.get_max_bound()[:2]
                if np.all(mesh_center_xy > min_xy) and np.all(mesh_center_xy < max_xy):
                    best_plane = (model, cloud, z_mean)
                    min_gap = gap

        if best_plane is None:
            rospy.logwarn("No supporting plane found under mesh in base_footprint.")
            return

        _, plane_cloud, z_plane = best_plane
        aabb = plane_cloud.get_axis_aligned_bounding_box()
        center = aabb.get_center()
        extent = aabb.get_extent()

        # Build support plane CollisionObject
        support = CollisionObject()
        support.id = "support_plane"
        support.header.frame_id = target_frame

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            extent[0] + 0.1,
            extent[1] + 0.1,
            0.01,
        ]

        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = z_plane - 0.005
        pose.orientation.w = 1.0

        support.primitives = [box]
        support.primitive_poses = [pose]
        support.operation = CollisionObject.ADD

        self._collision_object_pub.publish(support)
        rospy.loginfo("Published real support plane in base_footprint.")

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

    def run(self, debug: bool = False, execute: bool = True):
        print(self._move_group.get_end_effector_link())
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
        transform, _, _ = registration.register_object(mesh_pcd, target_pcd)
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
            mesh_pcd,
            self._gpd_pcd_path,
            self._gpd_executable_path,
            self._gpd_config_path,
        )
        rospy.loginfo(f"Detected {len(grasps)} grasps")

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
            rospy.sleep(5.0)
            eef_pose = self._move_group.get_current_pose("gripper_grasping_frame")
            eef_pose = transformations.tf_poses(
                [eef_pose.pose],
                eef_pose.header.frame_id,
                "gripper_grasping_frame",
                self._tf_buffer,
            )[0]
            eef_target_pose = transformations.offset_grasps([eef_pose], 0.12, 0.0, 0.0)[
                0
            ]
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

                eef_pose_base = self._get_eef_pose()
                lifted_pose = transformations.offset_grasps(
                    [eef_pose_base], 0.0, 0.0, 0.10
                )[0]

                self._publish_grasp_poses([lifted_pose], "base_footprint")

                self._move_group.set_pose_target(lifted_pose, "gripper_grasping_frame")
                if self._move_group.go(wait=True):
                    self._play_motion("pregrasp")

        self._move_group.stop()


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run(debug=True, execute=True)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
