from typing import List

import rospy
import actionlib
import tf
import numpy as np

import sys


import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander


from gpd_ros.srv import detect_grasps as DetectGrasps
from gpd_ros.msg import CloudIndexed, CloudSources, GraspConfig, GraspConfigList

from lasr_manipulation_msgs.srv import CompleteShape

from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Int64
from std_srvs.srv import Empty


class GraspGPD:

    def __init__(self):

        rospy.loginfo("Starting GraspGPD...")

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("Setting up PlanningSceneInterface...")
        self._planning_scene = PlanningSceneInterface()
        self._planning_scene.clear()
        rospy.loginfo("Setup PlanningSceneInterface!")

        rospy.loginfo("Setting up MoveGroupCommander...")
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        rospy.loginfo("Setup MoveGroupCommander!")

        rospy.loginfo("Setting up PlayMotion...")
        self._play_motion_client = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        self._play_motion_client.wait_for_server()
        rospy.loginfo("Setup PlayMotion!")

        rospy.loginfo("Setting up clear_octomap...")
        self._clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self._clear_octomap.wait_for_service()
        rospy.loginfo("Setup clear_octomap!")

        rospy.loginfo("Setting up 3D Completion...")
        self._3d_completion = rospy.ServiceProxy(
            "/shape_completion/complete", CompleteShape
        )
        self._3d_completion.wait_for_service()
        rospy.loginfo("Setup 3D Completion...")

        rospy.loginfo("Setting up GPD...")
        self._gpd = rospy.ServiceProxy(
            "/detect_grasps_server/detect_grasps", DetectGrasps
        )
        self._gpd.wait_for_service()
        rospy.loginfo("Setup GPD!")
        rospy.loginfo("GraspGPD pipeline is ready!")

    def _play_motion(self, motion_name: str):
        goal = PlayMotionGoal()
        goal.skip_planning = False
        goal.motion_name = motion_name
        self._play_motion_client.send_goal_and_wait(goal)

    def _grasp_to_pose(self, grasp: GraspConfig) -> Pose:
        # Extract orientation vectors
        x = [grasp.axis.x, grasp.axis.y, grasp.axis.z]
        y = [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]
        z = [grasp.approach.x, grasp.approach.y, grasp.approach.z]

        # Rotation matrix: columns are x, y, z
        rot_matrix = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]

        # Convert rotation matrix to quaternion
        quat = tf.transformations.quaternion_from_matrix(
            [
                [rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2], 0],
                [rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2], 0],
                [rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2], 0],
                [0, 0, 0, 1],
            ]
        )

        pose = Pose()
        pose.position = grasp.position
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

    def _detect_grasps(self, pcl: PointCloud2) -> GraspConfigList:
        sources = CloudSources(pcl, [Int64(0)], [Point(0, 0, 0)])
        indexed = CloudIndexed(
            sources, [Int64(i) for i in range(pcl.width * pcl.height)]
        )
        response = self._gpd(indexed)
        grasps = response.grasp_configs.grasps
        return grasps

    def _create_collision_object(self, pcl: PointCloud2):
        points = []
        for p in pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            rospy.logwarn("No points in the PCL message!")
            return

        points = np.array(points)

        # Optional: remove outliers based on percentiles (e.g., filtering out the extreme points)
        lower = np.percentile(points, 1, axis=0)
        upper = np.percentile(points, 99, axis=0)

        # Filter points based on the percentile ranges
        mask = np.all((points >= lower) & (points <= upper), axis=1)
        points = points[mask]

        # Find the min and max of the points
        min_pt = points.min(axis=0)
        max_pt = points.max(axis=0)

        # Calculate the center of the bounding box and its size
        center = (min_pt + max_pt) / 2.0
        size = max_pt - min_pt

        p = PoseStamped()
        p.header.frame_id = pcl.header.frame_id
        p.pose.position.x = center[0]
        p.pose.position.y = center[1]
        p.pose.position.z = center[2]
        p.pose.orientation.w = 1
        self._planning_scene.add_box("obj", p, size=size)

    def _execute_grasps(self, grasp_poses: List[Pose]) -> bool:
        self._move_group.set_pose_targets(
            grasp_poses, end_effector_link="gripper_grasping_frame"
        )
        success = self._move_group.go(wait=True)
        self._move_group.stop()
        return success

    def grasp_pipeline(self):
        rospy.loginfo("Sending pregrasp motion...")
        self._play_motion("pregrasp")
        rospy.loginfo("Pregrasp motion finished!")
        rospy.loginfo("Sending open_gripper motion...")
        self._play_motion("open_gripper")
        rospy.loginfo("Pregrasp open_gripper finished!")
        rospy.loginfo("Calling clear_octomap...")
        self._clear_octomap()
        rospy.loginfo("Cleared octomap!")
        # Get segmented PCL
        rospy.loginfo("Waiting for segmented_cloud...")
        masked_pcl = rospy.wait_for_message("/segmented_cloud", PointCloud2)
        self._move_group.set_pose_reference_frame(masked_pcl.header.frame_id)
        rospy.loginfo("Got segmented_cloud!")
        rospy.loginfo("Completing point cloud...")
        completed_pcl = self._3d_completion(masked_pcl).completed_cloud
        rospy.loginfo("Completed point cloud!")
        # Call GPD
        rospy.loginfo("Calling GPD...")
        grasps = self._detect_grasps(completed_pcl)
        rospy.loginfo("Got response from GPD!")
        rospy.loginfo("Converting grasps to poses...")
        grasp_poses = [self._grasp_to_pose(grasp) for grasp in grasps]
        rospy.loginfo("Grasps converted to poses!")
        # Collision object
        rospy.loginfo("Adding collision object added to the planning scene...")
        self._create_collision_object(completed_pcl)
        rospy.loginfo("Collision object added to the planning scene!")
        return
        # Execute
        rospy.loginfo("Executing...")
        success = self._execute_grasps(grasp_poses)
        rospy.loginfo(f"Executed, success: {success}!")


if __name__ == "__main__":
    rospy.init_node("grasp_gpd")
    grasp_gpd = GraspGPD()
    grasp_gpd.grasp_pipeline()
    rospy.spin()
