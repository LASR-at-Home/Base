#!/usr/bin/env /home/siyao/project/RoboCup/robocup_svea/devel/.private/lasr_grasping/share/lasr_grasping/venv/bin/python
import sys
import rospy
# rospy.logwarn(sys.executable)
print("Starting grasp_contact_graspnet node")
rospy.init_node("test_contact_graspnet_grasps")
import ultralytics
import moveit_commander
import sys
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface
from ros_contact_graspnet.srv import DetectGrasps
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
import tf2_ros as tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction

import cv2_pcl
import cv2
import numpy as np
from moveit.core.collision_detection import AllowedCollisionMatrix
import open3d as o3d
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point, Pose
import os




def tf_pose(source_frame: str, target_frame: str, pose: Pose) -> Pose:
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
            rospy.Duration(1.0),
        )
    except (
        tf.LookupException,
        tf.ConnectivityException,
        tf.ExtrapolationException,
    ) as e:
        raise rospy.ServiceException(str(e))

    pose_stamped = PoseStamped(pose=pose)
    pose_stamped.header.frame_id = source_frame
    return do_transform_pose(pose_stamped, transform)

def publish_marker(pose: Pose, frame_id: str, marker_id: int):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "grasp_markers"
    marker.id = marker_id
    marker.type = Marker.ARROW  # you could also use Marker.AXES if you prefer
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.1  # Shaft length
    marker.scale.y = 0.02  # Shaft diameter
    marker.scale.z = 0.02  # Head diameter
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(30)  # Stays visible for 30 seconds
    marker_pub.publish(marker)

# def create_collision_object_from_pcl(
#     pcl_msg, planning_scene_interface, frame_id="base_footprint"
# ):
#     # Convert PointCloud2 message to a list of (x, y, z)
#     points = []
#     for p in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True):
#         points.append([p[0], p[1], p[2]])

#     if not points:
#         rospy.logwarn("No points in the PCL message!")
#         return

#     points = np.array(points)

#     # Optional: remove outliers based on percentiles (e.g., filtering out the extreme points)
#     lower = np.percentile(points, 1, axis=0)
#     upper = np.percentile(points, 99, axis=0)

#     # Filter points based on the percentile ranges
#     mask = np.all((points >= lower) & (points <= upper), axis=1)
#     points = points[mask]

#     # Find the min and max of the points
#     min_pt = points.min(axis=0)
#     max_pt = points.max(axis=0)

#     # Calculate the center of the bounding box and its size
#     center = (min_pt + max_pt) / 2.0
#     size = max_pt - min_pt

#     p = PoseStamped()
#     p.header.frame_id = frame_id
#     # p.pose.position.x = obj.centroid.x
#     # p.pose.position.y = obj.centroid.y
#     # p.pose.position.z = obj.centroid.z
#     p.pose.position.x = center[0]
#     p.pose.position.y = center[1]
#     p.pose.position.z = center[2]
#     p.pose.orientation.w = 1
#     planning_scene_interface.add_box("obj", p, size=size)

#     # # Create the collision object (bounding box)
#     # collision_object = CollisionObject()
#     # collision_object.header.frame_id = frame_id
#     # collision_object.id = "object"

#     # # Define a SolidPrimitive (box)
#     # box = SolidPrimitive()
#     # box.type = SolidPrimitive.BOX
#     # box.dimensions = [size[0], size[1], size[2]]

#     # # Define the pose of the collision object
#     # pose = Pose()
#     # pose.position.x = center[0]
#     # pose.position.y = center[1]
#     # pose.position.z = center[2]
#     # pose.orientation.w = 1.0  # No rotation, just the position

#     # collision_object.primitives = [box]
#     # collision_object.primitive_poses = [pose]
#     # collision_object.operation = CollisionObject.ADD

#     # Add the collision object to the planning scene
#     # planning_scene_interface.apply_collision_object(collision_object)

#     rospy.loginfo("Collision object added to the planning scene!")


def create_mesh_collision_object_from_pcl(
    pcl_msg,
    planning_scene_interface,
    obj_id="obj",
    frame_id="base_footprint"
):
    # 1) Turn the PointCloud2 into a NumPy array of shape (N,3)
    points = np.array([
        [p[0], p[1], p[2]]
        for p in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)
    ])
    if points.shape[0] == 0:
        rospy.logwarn("No points in the PCL message!")
        return

    # 2) Build an Open3D point cloud + estimate normals
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals()

    # 3) Run Poisson registration (or any reconstruction you like)
    mesh_o3d, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=8
    )
    mesh_o3d = mesh_o3d.simplify_quadric_decimation(10000)

    # 4) Decide where on disk to save this mesh. For example:
    #    - Put it in /tmp
    #    - Or under your ROS package’s “meshes” folder
    # Here, we’ll just save to /tmp/object.stl:
    mesh_filename = "/tmp/{}_mesh.ply".format(obj_id)
    o3d.io.write_triangle_mesh(mesh_filename, mesh_o3d)
    rospy.loginfo("Wrote mesh to %s", mesh_filename)

    # 5) Compute the mesh’s centroid so we can tell MoveIt where to place it.
    vertices = np.asarray(mesh_o3d.vertices)
    center = vertices.mean(axis=0)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = center[0]
    pose.pose.position.y = center[1]
    pose.pose.position.z = center[2]
    pose.pose.orientation.w = 1.0

    # 6) Finally, give MoveIt the full path to that STL:
    planning_scene_interface.add_mesh(
        name=obj_id,
        pose=pose,
        filename=mesh_filename
    )
    rospy.loginfo("Mesh CollisionObject '%s' added to the planning scene (file: %s)", obj_id, mesh_filename)


def allow_collisions_with_object(obj_name, scene):
    """Updates the MoveIt PlanningScene using the AllowedCollisionMatrix to ignore collisions for an object"""
    # Set up service to get the current planning scene
    service_timeout = 5.0
    _get_planning_scene = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    _get_planning_scene.wait_for_service(service_timeout)

    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(
        f"\n\n\n--- allowed_collision_matrix before update:{planning_scene.scene.allowed_collision_matrix} ---\n\n\n"
    )

    # Set this object to ignore collisions with all objects. The entry values are not updated
    planning_scene.scene.allowed_collision_matrix.entry_names.append(obj_name)
    for entry in planning_scene.scene.allowed_collision_matrix.entry_values:
        entry.enabled.append(True)
    enabled = [
        True
        for i in range(len(planning_scene.scene.allowed_collision_matrix.entry_names))
        # if planning_scene.scene.allowed_collision_matrix.entry_names[i] in [
        #     "gripper_left_finger_link",
        #     "gripper_right_finger_link",
        #     "gripper_link",
        # ],
    ]
    entry = AllowedCollisionEntry(enabled=enabled)  # Test kwarg in constructor
    planning_scene.scene.allowed_collision_matrix.entry_values.append(entry)

    # Set the default entries. They are also not updated
    planning_scene.scene.allowed_collision_matrix.default_entry_names = [obj_name]
    planning_scene.scene.allowed_collision_matrix.default_entry_values = [False]
    planning_scene.scene.is_diff = True  # Mark this as a diff message to force an update of the allowed collision matrix
    planning_scene.scene.robot_state.is_diff = True

    planning_scene_diff_req = ApplyPlanningSceneRequest()
    planning_scene_diff_req.scene = planning_scene.scene

    # Updating the Allowed Collision Matrix through the apply_planning_scene service shows no effect.
    # However, adding objects to the planning scene works fine.
    # scene._apply_planning_scene_diff.call(planning_scene_diff_req)
    scene.apply_planning_scene(planning_scene.scene)

    # Attempting to use the planning_scene topic for asynchronous updates also does not work
    # planning_scene_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=5)
    # planning_scene_pub.publish(planning_scene.scene)

    print(f"\n--- Sent message:{planning_scene.scene.allowed_collision_matrix} ---\n")

    # The planning scene retrieved after the update should have taken place shows the Allowed Collision Matrix is the same as before
    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(
        f"\n--- allowed_collision_matrix after update:{planning_scene.scene.allowed_collision_matrix} ---\n"
    )


# allow_collisions_with_object("obj", planning_scene)


# Invert the mask: object == 0, background == 255
# inv_mask = cv2.bitwise_not(mask)

# # Find indices where background exists
# inv_indices = np.argwhere(inv_mask)

# # Read the original pointcloud (same as you did)
# pcl_xyz = np.array(
#     list(pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=False)),
#     dtype=np.float32,
# )
# pcl_xyz = pcl_xyz.reshape((im.shape[0], im.shape[1], 3))
# # Select points outside the object
# background_points = pcl_xyz[inv_indices[:, 0], inv_indices[:, 1]]
# background_points = background_points[~np.isnan(background_points).any(axis=1)]
# background_points = background_points[~np.all(background_points == 0, axis=1)]

# # Create and publish pointcloud of background
# pcl_seg = background_cloud
# background_cloud = create_pointcloud2(background_points, pcl.header.frame_id)
# rospy.loginfo("Published background pointcloud without the segmented object")



planning_scene = PlanningSceneInterface()
planning_scene.clear()

play_motion = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
play_motion.wait_for_server()
play_goal = PlayMotionGoal()
play_goal.motion_name = "pregrasp"
play_goal.skip_planning = False
play_motion.send_goal_and_wait(play_goal)
play_goal = PlayMotionGoal()
play_goal.motion_name = "open_gripper"
play_goal.skip_planning = False
play_motion.send_goal_and_wait(play_goal)
clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
tf_listener = tf.TransformListener(tf_buffer)
moveit_commander.roscpp_initialize(sys.argv)

detect_grasps = rospy.ServiceProxy("/contact_graspnet/detect_grasps", DetectGrasps)

# Publisher for markers
marker_pub = rospy.Publisher("grasp_markers", Marker, queue_size=10)
pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
masked_cloud = rospy.wait_for_message("/segmented_cloud", PointCloud2)

print("Got PCL")
# get the collision object from the topic

create_mesh_collision_object_from_pcl(masked_cloud, planning_scene, obj_id="obj", frame_id=masked_cloud.header.frame_id)


rospy.loginfo("Got PCL, calling graspnet")

rospy.loginfo("Published mesh CollisionObject to MoveIt planning scene.")
rospy.loginfo("Created collision object")



rospy.loginfo("Setting up MoveIt!")
arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
arm_torso_group.set_planner_id("RRTConnectkConfigDefault")
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_looking(True)
arm_torso_group.set_planning_time(30)
arm_torso_group.set_num_planning_attempts(30)
print(arm_torso_group.get_planning_frame())
print(arm_torso_group.get_end_effector_link())
rospy.loginfo("MoveIt! ready")
arm_torso_group.set_pose_reference_frame(pcl.header.frame_id)

allow_collisions_with_object("obj", planning_scene)
rospy.loginfo("Got PCL, calling graspnet")
grasp_response = detect_grasps(pcl, masked_cloud)
print(grasp_response.scores)
header = grasp_response.grasps.header
grasps_scores = sorted(
    zip(grasp_response.grasps.poses, grasp_response.scores),
    key=lambda x: x[1],
    reverse=True,
)
grasps, scores = zip(*grasps_scores)
grasps = list(grasps)
scores = list(scores)
# grasps = sorted(grasps, key=lambda g: g.score, reverse=True)
# for i, grasp in enumerate(gras)
arm_torso_group.set_pose_targets(
    grasps,
    # [tf_pose(header.frame_id, "base_footprint", g).pose for g in grasps][:10],
    end_effector_link="gripper_grasping_frame",
)
res = arm_torso_group.go(wait=True)
arm_torso_group.clear_pose_targets()
arm_torso_group.stop()
print(res)
rospy.sleep(1.0)
if res:
    # arm_torso_group.set_pose_reference_frame("gripper_grasping_frame")
    # offset_pose = Pose()
    # offset_pose.position.z = -0.080
    # offset_pose.orientation.w = 1
    # arm_torso_group.set_pose_target(
    #     offset_pose, end_effector_link="gripper_grasping_frame"
    # )
    # res = arm_torso_group.go(wait=True)
    # arm_torso_group.clear_pose_target()
    # arm_torso_group.stop()

    import geometry_msgs.msg
    import tf.transformations as tf

    def move_end_effector_offset(move_group, offset_x, offset_y, offset_z):
        """
        Move the end effector by a given offset (in its local frame).

        Args:
            move_group: MoveGroupCommander instance.
            offset_x: Desired x offset (meters) in local frame.
            offset_y: Desired y offset (meters) in local frame.
            offset_z: Desired z offset (meters) in local frame.
        """
        # Get the current pose
        current_pose = move_group.get_current_pose().pose

        # Get current orientation as quaternion
        q = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]

        # Convert quaternion to rotation matrix
        rotation_matrix = tf.quaternion_matrix(q)  # 4x4 homogeneous matrix

        # Offset vector in local end-effector frame (homogeneous coordinates)
        local_offset = [offset_x, offset_y, offset_z, 0]

        # Transform the local offset into world frame
        world_offset = rotation_matrix.dot(local_offset)

        # Create target pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = current_pose.position.x + world_offset[0]
        target_pose.position.y = current_pose.position.y + world_offset[1]
        target_pose.position.z = current_pose.position.z + world_offset[2]
        target_pose.orientation = current_pose.orientation  # Keep the same orientation

        # Plan and execute
        move_group.set_pose_target(target_pose)
        success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

        return success

    def sync_shift_ee(move_group, x, y, z):
        from tf.transformations import euler_from_quaternion, euler_matrix

        curr_pose = move_group.get_current_pose()
        pose = curr_pose.pose
        # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        euler = euler_from_quaternion(quat)
        euler_m = euler_matrix(*euler)

        # calculate offset to add to the current pose
        delta = np.dot(euler_m[:3, :3], np.array([x, y, z]).T)
        pose.position.x += delta[0]
        pose.position.y += delta[1]
        pose.position.z += delta[2]

        move_group.set_pose_target(curr_pose)
        # publish pose for debugging purposes
        # print('Publishing debug_plan_pose')

        move_group.set_start_state_to_current_state()
        result = move_group.go(wait=True)
        return result
    clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
    clear_octomap()
    sync_shift_ee(arm_torso_group, 0.06, 0.0, 0.0)
    # arm_torso_group.set_start_state_to_current_state()
    # res = move_end_effector_offset(arm_torso_group, 0.05, 0.0, 0.0)
    #
    if res:
        arm_torso_group.attach_object(
            "obj",
            touch_links=[
                "gripper_left_finger_link",
                "gripper_right_finger_link",
                "gripper_link",
            ],
        )
        close_gripper = rospy.ServiceProxy("/parallel_gripper_controller/grasp", Empty)
        close_gripper()
        sync_shift_ee(arm_torso_group, -0.06, 0.0, 0.0)
        play_goal = PlayMotionGoal()
        play_goal.motion_name = "post_grasp_pose"
        play_goal.skip_planning = False
        play_motion.send_goal_and_wait(play_goal)
        play_goal = PlayMotionGoal()
        play_goal.motion_name = "grasp_to_home"
        play_goal.skip_planning = False
        play_motion.send_goal_and_wait(play_goal)
# print(arm_torso_group.go(wait=True))
# rospy.spin()
# /throttle_filtering_points/filtered_points
# for i, grasp in enumerate(grasps):
#     rospy.loginfo(f"Attempting grasp with score {grasp.score} {grasp.pose.position}")
#     pose = tf_pose(pcl.header.frame_id, "base_footprint", grasp.pose)

#     # Publish marker for each grasp
#     publish_marker(pose.pose, "base_footprint", i)

#     arm_torso_group.set_pose_target(pose.pose)
#     if arm_torso_group.go(wait=True):
#         break
#     # y = input("Continue?")
#     # if y[0] != "y":
#     #     break
moveit_commander.roscpp_shutdown()
