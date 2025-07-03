#!/usr/bin/env python3
from typing import List

import rospy
import rospkg
import os
import actionlib
import open3d as o3d
import tf2_ros
import moveit_commander
from moveit_commander import MoveGroupCommander

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty

import sys
import open3d as o3d
import numpy as np
import copy
import tf.transformations

from lasr_manipulation_pipeline import gpd
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import transformations
from lasr_manipulation_pipeline import sam
from lasr_manipulation_pipeline import visualisation

from lasr_manipulation_msgs.srv import (
    Registration,
    DetectAndAddSupportSurface,
    AddCollisionObject,
    AllowCollisionsWithObj,
    AttachObjectToGripper,
)


PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "pringles.ply"


class GraspingPipeline:

    _mesh_path: str = os.path.join(PACKAGE_PATH, "meshes", MESH_NAME)
    _gpd_executable_path: str = "/opt/gpd/build/detect_grasps"
    _gpd_config_path: str = os.path.join(PACKAGE_PATH, "cfg", "tiago.cfg")
    _gpd_pcd_path: str = "/tmp/gpd.pcd"

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Setting up MoveGroupCommander...")
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        self._move_group.clear_pose_targets()
        self._move_group.set_max_velocity_scaling_factor(0.5)
        self._move_group.clear_pose_targets()
        rospy.loginfo("Setup MoveGroupCommander!")

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

        mesh = o3d.io.read_triangle_mesh(self._mesh_path)
        self._pcd = mesh.sample_points_poisson_disk(
            number_of_points=10000, init_factor=5
        )
        rospy.loginfo("Read mesh.")

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._close_gripper = rospy.ServiceProxy(
            "/parallel_gripper_controller/grasp", Empty
        )
        self._close_gripper.wait_for_service()

        self._register_object = rospy.ServiceProxy(
            "/lasr_manipulation/registration", Registration
        )
        self._register_object.wait_for_service()
        self._detect_and_add_support_surface = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/detect_and_add_support_surface",
            DetectAndAddSupportSurface,
        )
        self._detect_and_add_support_surface.wait_for_service()
        self._add_collision_object = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/add_collision_object",
            AddCollisionObject,
        )
        self._add_collision_object.wait_for_service()
        self._allow_collisions_with_obj = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/allow_collisions_with_obj",
            AllowCollisionsWithObj,
        )
        self._allow_collisions_with_obj.wait_for_service()
        self._attach_object_to_gripper = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/attach_object_to_gripper",
            AttachObjectToGripper,
        )
        self._attach_object_to_gripper.wait_for_service()

        self._clear_planning_scene = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/clear", Empty
        )
        self._clear_planning_scene.wait_for_service()

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
            marker.scale.x = 0.09  # shaft length
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
        max_advance: float = 0.09,
        step: float = 0.001,
        contact_threshold: float = 0.005,
        palm_clearance: float = 0.03,
    ) -> float:
        """
        Determines how far to move forward along the gripper's x-axis from the pregrasp pose
        to make contact with the object, while avoiding pushing through or colliding with it.
        """
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
        max_safe_forward = max(0.0, 1.0 * object_width - palm_clearance)
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
        self._clear_planning_scene()
        # Clone the mesh point cloud
        mesh_pcd = o3d.geometry.PointCloud(self._pcd)

        # Get the live sensor point cloud
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        scene_pcl = copy.deepcopy(pcl)
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

        response = self._register_object(pcl, MESH_NAME.replace(".ply", ""), False, 5)
        ros_transform = response.transform
        ros_scale = response.scale
        trans = ros_transform.transform.translation
        quat = ros_transform.transform.rotation
        rot_mat = tf.transformations.quaternion_matrix(
            [quat.x, quat.y, quat.z, quat.w]
        )[:3, :3]

        sx, sy, sz = (
            response.scale.vector.x,
            response.scale.vector.y,
            response.scale.vector.z,
        )
        scale_mat = np.diag([sx, sy, sz])

        rs_mat = np.dot(rot_mat, scale_mat)

        T = np.eye(4)
        T[:3, :3] = rs_mat
        T[:3, 3] = [trans.x, trans.y, trans.z]
        transform = T
        rospy.loginfo("Registered mesh")

        mesh_pcd.transform(transform)
        rospy.loginfo("Transformation applied")

        if debug:
            o3d.visualization.draw_geometries(
                [mesh_pcd, target_pcd], window_name="Registration result"
            )

        self._detect_and_add_support_surface(
            "table", scene_pcl, MESH_NAME.replace(".ply", ""), ros_transform, ros_scale
        )
        self._add_collision_object(
            MESH_NAME.replace(".ply", ""),
            MESH_NAME.replace(".ply", ""),
            ros_transform,
            ros_scale,
        )

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
        grasp_poses = transformations.offset_grasps(grasp_poses, -0.09, 0.0, 0.0)
        grasp_poses_base_footprint = transformations.tf_poses(
            grasp_poses, "gripper_grasping_frame", "base_footprint", self._tf_buffer
        )

        if debug:
            grasp_poses_camera = transformations.tf_poses(
                grasp_poses,
                "gripper_grasping_frame",
                pcl.header.frame_id,
                self._tf_buffer,
            )

            visualisation.visualize_grasps_on_scene(
                target_pcd, grasp_poses_camera, "Object grasps (offset)"
            )
        self._publish_grasp_poses(grasp_poses_base_footprint, "base_footprint")

        self._move_group.set_pose_reference_frame("base_footprint")
        self._move_group.set_pose_targets(
            grasp_poses_base_footprint, "gripper_grasping_frame"
        )

        if self._move_group.go(wait=True):
            self._allow_collisions_with_obj(MESH_NAME.replace(".ply", ""))
            rospy.sleep(5.0)
            eef_pose = self._get_eef_pose(base_frame="gripper_grasping_frame")
            x_forward = 0.09
            rospy.loginfo(f"x forward: {x_forward}")
            eef_target_pose = transformations.offset_grasps(
                [eef_pose],
                x_forward,
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
                self._attach_object_to_gripper(MESH_NAME.replace(".ply", ""))
                self._close_gripper()
                rospy.sleep(2.0)
                self._play_motion("pregrasp")

        self._move_group.stop()


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run(debug=False, execute=True)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
