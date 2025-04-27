#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Int64

import tf2_ros as tf
import tf2_sensor_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from gpd_ros.srv import detect_grasps
from gpd_ros.msg import GraspConfigList, GraspConfig, CloudIndexed, CloudSources
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import rospy
import time
import actionlib

"""
1. Get PCL, transform to robot's frame
2. Pass PCL to GPD, get grasps
3. Get grasps as poses in EE link
4. Move end effector to target grasp

ALTERNATELY

1. Estimate objects pose, e.g. with happypose
2. Apply transformation to model, convert to pcd, pass to gpd
3. retreive grasps and execute

"""

rospy.init_node("grasp")
rospy.loginfo("Grasping node started!")

joint_trajectory_pub = rospy.Publisher(
    "/head_controller/command", JointTrajectory, queue_size=10
)

play_motion = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
play_motion.wait_for_server()


def look_centre(duration=1.0, down_joint=-0.70):
    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    traj.points = [
        JointTrajectoryPoint(
            positions=[0, down_joint], time_from_start=rospy.Duration(duration)
        )
    ]
    joint_trajectory_pub.publish(traj)
    rospy.sleep(duration + 0.25)


def look_dir(right, duration=1.0):
    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    pan = -0.45 if right else 0.45
    traj.points = [
        JointTrajectoryPoint(
            positions=[pan, -0.60], time_from_start=rospy.Duration(duration)
        )
    ]
    joint_trajectory_pub.publish(traj)
    rospy.sleep(duration + 0.25)


look_centre()
look_dir(False)
look_dir(True)

# play_goal = PlayMotionGoal()
# play_goal.motion_name = "pregrasp"
# play_goal.skip_planning = False
# play_motion.send_goal_and_wait(play_goal)

look_centre()

##### moveit
import moveit_commander
import tf as tf1

arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
arm_torso_group.set_planner_id("RRTConnectkConfigDefault")
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_looking(True)
arm_torso_group.set_planning_time(15)
arm_torso_group.set_num_planning_attempts(10)
arm_torso_group.set_pose_reference_frame("base_footprint")

# Initialize Planning Scene Interface
scene = PlanningSceneInterface(synchronous=True)
rospy.sleep(1)  # Give some time for scene to initialize

# Define the collision object
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

# Define the pose
pose = PoseStamped()
pose.header.frame_id = "base_footprint"
pose.pose.position.x = 0.725
pose.pose.position.y = 0.0
pose.pose.position.z = 0.9
pose.pose.orientation.w = 1.0

# Add a box with similar dimensions: 0.15 height ≈ z-size, 0.1 diameter ≈ 0.1 x and y size
scene.add_box(name="obj", pose=pose, size=(0.1, 0.1, 0.15))

# Optional: wait a bit for the scene to process
rospy.sleep(1)

touch_links = [
    "gripper_left_finger_link",
    "gripper_right_finger_link",
    "gripper_link",
]  # <-- Replace with your actual gripper link names

# Attach the object to a robot link, e.g., the end effector
scene.attach_box(
    link=arm_torso_group.get_end_effector_link(),  # Replace with your actual EE link
    name="obj",
    touch_links=touch_links,
)

rospy.sleep(1)  # Allow scene update

tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
tf_listener = tf.TransformListener(tf_buffer)

rospy.loginfo("Waiting for GPD...")
gpd = rospy.ServiceProxy("/detect_grasps/detect_grasps", detect_grasps)
gpd.wait_for_service()
rospy.loginfo("Got GPD")

##### Acquire and preprocess PCL
rospy.loginfo("Waiting for PCL...")
pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
rospy.loginfo("Got PCL!")
# rospy.loginfo("Looking up tf...")
# trans = tf_buffer.lookup_transform(
#     "base_footprint",
#     pcl.header.frame_id,
#     rospy.Time(0),
#     rospy.Duration(1.0),
# )
# rospy.loginfo("Got tf!")
# pcl = do_transform_cloud(pcl, trans)
rospy.loginfo("Transformed pcl to base_footprint")

##### Call GPD
sources = CloudSources(pcl, [Int64(0)], [Point(0, 0, 0)])
indexed = CloudIndexed(sources, [Int64(i) for i in range(pcl.width * pcl.height)])
resp = gpd(indexed)
grasps = resp.grasp_configs.grasps
print(grasps)


def grasp_to_pose(grasp_msg):
    # Extract orientation vectors
    x = [grasp_msg.axis.x, grasp_msg.axis.y, grasp_msg.axis.z]
    y = [grasp_msg.binormal.x, grasp_msg.binormal.y, grasp_msg.binormal.z]
    z = [grasp_msg.approach.x, grasp_msg.approach.y, grasp_msg.approach.z]

    # Rotation matrix: columns are x, y, z
    rot_matrix = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]

    # Convert rotation matrix to quaternion
    quat = tf1.transformations.quaternion_from_matrix(
        [
            [rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2], 0],
            [rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2], 0],
            [rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2], 0],
            [0, 0, 0, 1],
        ]
    )

    pose = Pose()
    pose.position = grasp_msg.position
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose


for grasp in grasps:
    pose = grasp_to_pose(grasp)
    arm_torso_group.set_pose_target(pose)
    arm_torso_group.go(wait=True)


# scene.remove_world_object("obj")
