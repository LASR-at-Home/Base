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
rospy.loginfo("Looking up tf...")
trans = tf_buffer.lookup_transform(
    "base_footprint",
    pcl.header.frame_id,
    rospy.Time(0),
    rospy.Duration(1.0),
)
rospy.loginfo("Got tf!")
pcl = do_transform_cloud(pcl, trans)
rospy.loginfo("Transformed pcl to base_footprint")

##### Call GPD
sources = CloudSources(pcl, [Int64(0)], [Point(0, 0, 0)])
indexed = CloudIndexed(sources, [Int64(i) for i in range(pcl.width * pcl.height)])
resp = gpd(indexed)
grasps = resp.grasp_configs.grasps
print(grasps)

##### moveit
import moveit_commander
import tf as tf1


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


arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
arm_torso_group.set_planner_id("RRTConnectkConfigDefault")
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_looking(True)
arm_torso_group.set_planning_time(15)
arm_torso_group.set_num_planning_attempts(10)
arm_torso_group.set_pose_reference_frame("base_footprint")
for grasp in grasps:
    pose = grasp_to_pose(grasp)
    arm_torso_group.set_pose_target(pose)
    arm_torso_group.go(wait=True)
