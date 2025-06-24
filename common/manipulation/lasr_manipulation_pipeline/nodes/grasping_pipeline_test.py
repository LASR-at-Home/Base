#!/usr/bin/env python3
from typing import Tuple, List

import rospy
import rospkg
import os
import actionlib
import open3d as o3d
import tf2_ros
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import (
    Grasp,
)
import sys
import open3d as o3d
from moveit_msgs.msg import PickupAction, PickupGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from lasr_manipulation_pipeline import registration
from lasr_manipulation_pipeline import gpd
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import transformations
from lasr_manipulation_pipeline import conversions
from lasr_manipulation_pipeline import sam

PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "starbucks_coffee.ply"


class GraspingPipeline:
    """
    1. Register mesh with PCL.
    2. Apply transformation to mesh, output as PCD.
    3. Call GPD
    4. Execute
    """

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
        rospy.loginfo("Setup PlanningSceneInterface!")
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        self._move_group.clear_pose_targets()
        self._move_group.set_max_velocity_scaling_factor(0.5)

        rospy.loginfo("Setting up MoveGroupCommander...")
        rospy.loginfo("Setting up PlayMotion...")
        self._play_motion_client = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        self._play_motion_client.wait_for_server()
        rospy.loginfo("Setup PlayMotion!")
        self._clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self._clear_octomap.wait_for_service()
        self._play_motion_client = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        self._play_motion_client.wait_for_server()
        self._grasp_markers_pub = rospy.Publisher(
            "/grasp_markers", MarkerArray, queue_size=10, latch=True
        )
        self._pcd = o3d.io.read_point_cloud(self._mesh_path)
        rospy.loginfo("Read mesh.")
        self._collision_object_pub = rospy.Publisher(
            "/collision_object", CollisionObject, queue_size=1, latch=True
        )
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pick_client = actionlib.SimpleActionClient("pickup", PickupAction)

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

    def run(self):
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
        pcl = sam.segment(pcl)
        scene_pcd = conversions.ros_to_o3d(pcl)
        o3d.io.write_point_cloud("scene.pcd", scene_pcd)

        # Register mesh with scene and apply the transformation

        target_pcd = conversions.ros_to_o3d(pcl)
        transform, _, _ = registration.register_object(mesh_pcd, target_pcd)
        rospy.loginfo("Registered mesh")

        mesh_pcd.transform(transform)
        rospy.loginfo("Transformation applied")

        # Estimate mesh from transformed point cloud
        mesh = o3d.io.read_triangle_mesh(self._mesh_path)
        mesh.transform(transform)
        mesh.compute_vertex_normals()

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

        # Call GPD on the aligned mesh
        grasps, scores, openings = gpd.generate_grasps(
            mesh_pcd,
            self._gpd_pcd_path,
            self._gpd_executable_path,
            self._gpd_config_path,
        )
        rospy.loginfo(f"Detected {len(grasps)} grasps")

        self._publish_grasp_poses(grasps, pcl.header.frame_id)

        moveit_grasps = []
        for i, (grasp, score, opening) in enumerate(zip(grasps, scores, openings)):
            moveit_grasp = Grasp()
            moveit_grasp.id = str(i)
            moveit_grasp.pre_grasp_posture.joint_names = [
                "gripper_left_finger_joint",
                "gripper_right_finger_joint",
            ]
            moveit_grasp.pre_grasp_posture.points = [
                JointTrajectoryPoint(
                    positions=[0.05, 0.05],
                    time_from_start=rospy.Duration(2.0),
                )
            ]
            moveit_grasp.grasp_posture.joint_names = [
                "gripper_left_finger_joint",
                "gripper_right_finger_joint",
            ]
            moveit_grasp.grasp_posture.points = [
                JointTrajectoryPoint(
                    positions=[opening / 2.0, opening / 2.0],
                    time_from_start=rospy.Duration(2.0),
                )
            ]
            moveit_grasp.grasp_pose = PoseStamped(
                pose=grasp, header=Header(frame_id=pcl.header.frame_id)
            )
            moveit_grasp.grasp_quality = score

            moveit_grasp.pre_grasp_approach.direction.header.frame_id = (
                "gripper_grasping_frame"
            )
            moveit_grasp.pre_grasp_approach.direction.vector.x = -1.0
            moveit_grasp.pre_grasp_approach.direction.vector.y = 0.0
            moveit_grasp.pre_grasp_approach.direction.vector.z = 0.0
            moveit_grasp.pre_grasp_approach.desired_distance = 0.1
            moveit_grasp.pre_grasp_approach.min_distance = 0.05

            moveit_grasp.post_grasp_retreat.direction.header.frame_id = (
                "gripper_grasping_frame"
            )
            moveit_grasp.post_grasp_retreat.direction.vector.x = 1.0
            moveit_grasp.post_grasp_retreat.direction.vector.y = 0.0
            moveit_grasp.post_grasp_retreat.direction.vector.z = 0.0
            moveit_grasp.post_grasp_retreat.desired_distance = 0.1
            moveit_grasp.post_grasp_retreat.min_distance = 0.05
            moveit_grasp.max_contact_force = 0.0

            moveit_grasps.append(moveit_grasp)

        goal = PickupGoal()
        goal.target_name = "obj"
        goal.possible_grasps = moveit_grasps
        goal.group_name = self._move_group.get_name()
        goal.allowed_planning_time = self._move_group.get_planning_time()
        goal.planner_id = self._move_group.get_planner_id()
        goal.attached_object_touch_links = [
            "gripper_left_finger_link",
            "gripper_right_finger_link",
            "gripper_link",
        ]
        goal.planning_options.look_around = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 30
        goal.planning_options.replan = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        result = self._pick_client.send_goal_and_wait(goal)
        print(result)


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
