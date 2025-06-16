from typing import List

import rospy
import actionlib
import tf
import tf2_geometry_msgs
import tf2_ros
import numpy as np

import sys


import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander


from gpd_ros.srv import detect_grasps as DetectGrasps
from gpd_ros.msg import CloudIndexed, CloudSources, GraspConfig, GraspConfigList

from lasr_manipulation_msgs.srv import CompleteShape

from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Int64, Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry


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


def create_pointcloud2(points, frame_id="map"):
    """
    points: (N, 3) numpy array
    frame_id: TF frame
    returns: sensor_msgs/PointCloud2
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # pack the points into a list of tuples
    points_list = [tuple(p) for p in points]

    pcl2_msg = pc2.create_cloud(header, fields, points_list)

    return pcl2_msg


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

        rospy.loginfo("Setting up TF...")
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.loginfo("Setup TF!")

        self._grasp_markers_pub = rospy.Publisher(
            "/grasp_markers", MarkerArray, queue_size=10, latch=True
        )

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

        return pose

    def _detect_grasps(self, pcl: PointCloud2) -> GraspConfigList:
        sources = CloudSources(pcl, [Int64(0)], [Point(0, 0, 0)])
        indexed = CloudIndexed(
            sources, [Int64(i) for i in range(pcl.width * pcl.height)]
        )
        response = self._gpd(indexed)
        grasps = response.grasp_configs.grasps
        return grasps

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
            marker.scale.x = 0.1  # shaft length
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

    def filter_pcl(
        self, pcl: PointCloud2, lower_pct: int = 1, upper_pct: int = 99
    ) -> PointCloud2:
        points = []
        for p in pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            rospy.logwarn("No points in the PCL message!")
            return

        points = np.array(points)

        # Optional: remove outliers based on percentiles (e.g., filtering out the extreme points)
        lower = np.percentile(points, lower_pct, axis=0)
        upper = np.percentile(points, upper_pct, axis=0)

        # Filter points based on the percentile ranges
        mask = np.all((points >= lower) & (points <= upper), axis=1)
        points = points[mask]

        return create_pointcloud2(points, pcl.header.frame_id)

    def _create_collision_object(self, pcl: PointCloud2):
        points = []

        for p in pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            rospy.logwarn("No points in the PCL message!")
            return

        points = np.array(points)

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

        obj_pose = self._tf_buffer.transform(p, "base_footprint", rospy.Duration(1.0))
        obj_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self._planning_scene.add_box("obj", obj_pose, size=size)

        plane_pose = PoseStamped()
        plane_pose.header.frame_id = "map"
        plane_pose.pose.position.x = 2.58
        plane_pose.pose.position.y = -0.46
        plane_pose.pose.position.z = 0.7
        plane_pose.pose.orientation.w = 1.0

        # p.pose.position.z = max_pt[2] + 0.1
        # plane_pose = self._tf_buffer.transform(p, "base_footprint", rospy.Duration(1.0))
        # plane_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self._planning_scene.add_box(
            "supporting_plane",
            plane_pose,
            size=(0.6, 0.4, 0.005),
        )  # 0.1 0.1 0.005
        self._move_group.set_support_surface_name("supporting_plane")

    def _wait_for_scene_update(self, object_name: str, timeout: float = 5.0) -> bool:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            known_objects = self._planning_scene.get_known_object_names()
            if object_name in known_objects:
                rospy.loginfo(f"Object '{object_name}' is in the planning scene.")
                return True
            rospy.sleep(0.1)
        rospy.logwarn(f"Timeout: Object '{object_name}' not found in planning scene.")
        return False

    def _execute_grasps(self, grasp_poses: List[Pose]) -> bool:
        self._move_group.set_pose_targets(
            grasp_poses, end_effector_link="gripper_grasping_frame"
        )
        success = self._move_group.go(wait=True)
        self._move_group.stop()
        return success

    def grasp_pipeline(self):
        # rospy.loginfo("Sending pregrasp motion...")
        # self._play_motion("pregrasp")
        # rospy.loginfo("Pregrasp motion finished!")
        # rospy.loginfo("Sending open_gripper motion...")
        # self._play_motion("open_gripper")
        # rospy.loginfo("Pregrasp open_gripper finished!")
        rospy.loginfo("Calling clear_octomap...")
        self._clear_octomap()
        rospy.loginfo("Cleared octomap!")
        # Get segmented PCL
        rospy.loginfo("Waiting for segmented_cloud...")
        masked_pcl = rospy.wait_for_message("/segmented_cloud", PointCloud2)
        # full_pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        self._move_group.set_pose_reference_frame(masked_pcl.header.frame_id)
        rospy.loginfo("Got segmented_cloud!")
        rospy.loginfo("Completing point cloud...")
        completed_pcl = self._3d_completion(self.filter_pcl(masked_pcl)).completed_cloud
        rospy.loginfo("Completed point cloud!")
        # Collision object
        rospy.loginfo("Adding collision object added to the planning scene...")
        self._create_collision_object(completed_pcl)
        if not self._wait_for_scene_update("obj", timeout=10.0):
            rospy.logwarn("Collision object not yet in the planning scene!")
        else:
            rospy.loginfo("Collision object added to the planning scene!")
        allow_collisions_with_object("obj", self._planning_scene)
        # Call GPD
        rospy.loginfo("Calling GPD...")
        grasps = self._detect_grasps(completed_pcl)
        rospy.loginfo("Got response from GPD!")
        rospy.loginfo("Converting grasps to poses...")
        grasp_poses = [self._grasp_to_pose(grasp) for grasp in grasps]
        rospy.loginfo("Grasps converted to poses!")
        self._publish_grasp_poses(grasp_poses, masked_pcl.header.frame_id)
        # Execute
        rospy.loginfo("Executing...")
        success = self._execute_grasps(grasp_poses)
        rospy.loginfo(f"Executed, success: {success}!")


if __name__ == "__main__":
    rospy.init_node("grasp_gpd")
    grasp_gpd = GraspGPD()
    grasp_gpd.grasp_pipeline()
    rospy.spin()
