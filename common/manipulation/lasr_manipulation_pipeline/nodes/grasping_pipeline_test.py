#!/usr/bin/env python3
from typing import Tuple, List

import rospy
import rospkg
import subprocess
import os
import numpy as np
import tf
import actionlib
import open3d as o3d
import cv2
import cv2_pcl
import ultralytics
import tf2_geometry_msgs
import tf2_ros
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import (
    PlanningScene,
    AllowedCollisionEntry,
    Grasp,
    GripperTranslation,
)
import sys


PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "starbucks_coffee.ply"


def allow_collisions_with_object(obj_name, scene):
    """Updates the MoveIt PlanningScene using the AllowedCollisionMatrix to ignore collisions for an object"""
    # Set up service to get the current planning scene
    service_timeout = 5.0
    _get_planning_scene = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    _get_planning_scene.wait_for_service(service_timeout)

    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)

    # Set this object to ignore collisions with all objects. The entry values are not updated
    planning_scene.scene.allowed_collision_matrix.entry_names.append(obj_name)
    for entry in planning_scene.scene.allowed_collision_matrix.entry_values:
        entry.enabled.append(True)
    enabled = [
        True
        for i in range(len(planning_scene.scene.allowed_collision_matrix.entry_names))
        if planning_scene.scene.allowed_collision_matrix.entry_names[i]
        in [
            "gripper_left_finger_link",
            "gripper_right_finger_link",
            "gripper_link",
        ]
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
    # # planning_scene_pub.publish(planning_scene.scene)

    # # The planning scene retrieved after the update should have taken place shows the Allowed Collision Matrix is the same as before
    # request = GetPlanningScene()
    # request.components = 0  # Get just the Allowed Collision Matrix
    # planning_scene = _get_planning_scene.call(request)


import open3d as o3d
import numpy as np
import tf.transformations


def pose_to_o3d_frame(pose, size=0.05):
    """
    Convert a ROS Pose to an Open3D coordinate frame mesh.

    Args:
        pose: ROS geometry_msgs.msg.Pose
        size: size of the coordinate frame axes

    Returns:
        o3d.geometry.TriangleMesh coordinate frame mesh transformed to pose
    """
    # Extract translation
    t = np.array([pose.position.x, pose.position.y, pose.position.z])

    # Extract rotation matrix from quaternion
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R = tf.transformations.quaternion_matrix(q)[:3, :3]

    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Create Open3D coordinate frame and apply transform
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(T)

    return frame


def visualize_grasps_on_scene(scene_pcd, grasps):
    """
    Visualize scene point cloud with grasp coordinate frames.

    Args:
        scene_pcd: open3d.geometry.PointCloud in camera frame
        grasps: list of ROS Pose objects representing grasp poses
    """
    # Paint the scene point cloud for contrast
    scene_pcd.paint_uniform_color([0.0, 1.0, 0.0])  # green

    # Create Open3D frames for each grasp
    grasp_frames = [pose_to_o3d_frame(g, size=0.03) for g in grasps]

    # Show everything together
    o3d.visualization.draw_geometries(
        [scene_pcd, *grasp_frames], window_name="Scene with Grasps"
    )


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
        # self._move_group.set_pose_reference_frame("gripper_grasping_frame")

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
        self._collision_object_pub = rospy.Publisher(
            "/collision_object", CollisionObject, queue_size=1, latch=True
        )
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.loginfo("Read mesh.")

    def _tf_poses(
        self, poses: List[Pose], source_frame: str, target_frame: str
    ) -> List[Pose]:
        """
        Transforms a list of Pose objects from source_frame to target_frame.

        Args:
            poses: List of geometry_msgs.msg.Pose to transform.
            source_frame: The frame the poses are currently expressed in.
            target_frame: The frame to transform the poses into.

        Returns:
            List of transformed Pose objects in the target_frame.
        """
        transformed_poses = []
        try:
            # Wait for the transform to be available
            self._tf_buffer.can_transform(
                target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(2.0)
            )
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            for pose in poses:
                stamped_pose = PoseStamped()
                stamped_pose.header.frame_id = source_frame
                stamped_pose.header.stamp = rospy.Time(0)  # latest available
                stamped_pose.pose = pose

                transformed = tf2_geometry_msgs.do_transform_pose(
                    stamped_pose, transform
                )
                transformed_poses.append(transformed.pose)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(f"Transform error from {source_frame} to {target_frame}: {e}")
            return []

        return transformed_poses

    def _offset_grasps(
        self, grasps: List[Pose], dx: float, dy: float, dz: float
    ) -> List[Pose]:
        """
        Apply an offset in the local grasp frame to each grasp pose.
        dx, dy, dz are offsets in the local x, y, z directions (e.g., dz=-0.05 moves back along the gripper approach axis).
        """
        offset_grasps = []

        for pose in grasps:
            # Convert quaternion to rotation matrix
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            rot = tf.transformations.quaternion_matrix(quat)

            # Local offset in grasp frame
            offset_local = np.array([dx, dy, dz, 1.0])  # homogeneous

            # Apply offset: get new position in world frame
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            offset_world = rot @ offset_local
            new_pos = pos + offset_world[:3]

            # Create new pose
            new_pose = Pose()
            new_pose.position.x = new_pos[0]
            new_pose.position.y = new_pos[1]
            new_pose.position.z = new_pos[2]
            new_pose.orientation = pose.orientation  # keep orientation unchanged

            offset_grasps.append(new_pose)

        return offset_grasps

    def _ros_to_o3d(self, pcl_msg: PointCloud2) -> o3d.geometry.PointCloud:
        points = list(
            point_cloud2.read_points(
                pcl_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

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

    def _create_pointcloud2(self, points, frame_id="map"):
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

        pcl2_msg = point_cloud2.create_cloud(header, fields, points_list)

        return pcl2_msg

    def _segment(self, pcl: PointCloud2):
        im = cv2_pcl.pcl_to_cv2(pcl)
        bbox = []
        drawing = False
        start_point = (0, 0)
        assert im is not None
        image_copy = im.copy()
        sam = ultralytics.FastSAM("FastSAM-s.pt").to("cpu")

        def mouse_callback_bbox(event, x, y, flags, param):
            nonlocal drawing, start_point, bbox, image_copy
            if event == cv2.EVENT_LBUTTONDOWN:
                drawing = True
                start_point = (x, y)
                bbox = []
            elif event == cv2.EVENT_MOUSEMOVE and drawing:
                image_copy = im.copy()
                cv2.rectangle(image_copy, start_point, (x, y), (0, 255, 0), 2)
            elif event == cv2.EVENT_LBUTTONUP:
                drawing = False
                end_point = (x, y)
                bbox = [start_point, end_point]

        while True:
            bbox = []
            cv2.namedWindow("Image")
            cv2.setMouseCallback("Image", mouse_callback_bbox)
            while True:
                cv2.imshow("Image", image_copy)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                if bbox:
                    break
            cv2.destroyAllWindows()

            if not bbox:
                continue

            x0, y0 = bbox[0]
            x1, y1 = bbox[1]
            box = [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]
            result = sam.predict(
                im,
                bboxes=[box],
                retina_masks=True,
                iou=0.25,
                conf=0.5,
            )[0]

            cv2.imshow("result", result.plot())
            print("Press Y to proceed, or N to re-segment")
            key = None
            while key not in [ord("y"), ord("n")]:
                key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()

            proceed = key == ord("y")
            if proceed:
                break

        xyseg = np.array(result.masks.xy).flatten().round().astype(int).reshape(-1, 2)
        contours = xyseg.reshape(-1, 2)
        mask = np.zeros(shape=im.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, pts=[contours], color=255)
        indices = np.argwhere(mask)

        pcl_xyz = np.array(
            list(
                point_cloud2.read_points(
                    pcl, field_names=("x", "y", "z"), skip_nans=False
                )
            ),
            dtype=np.float32,
        ).reshape(im.shape[0], im.shape[1], 3)

        masked_points = pcl_xyz[indices[:, 0], indices[:, 1]]
        masked_points = masked_points[~np.isnan(masked_points).any(axis=1)]
        masked_points = masked_points[~np.all(masked_points == 0, axis=1)]
        masked_cloud = self._create_pointcloud2(masked_points, pcl.header.frame_id)

        return masked_cloud

    def _play_motion(self, motion_name: str):
        goal = PlayMotionGoal()
        goal.skip_planning = False
        goal.motion_name = motion_name
        self._play_motion_client.send_goal_and_wait(goal)

    def _open3d_mesh_to_ros_mesh(self, o3d_mesh: o3d.geometry.TriangleMesh) -> Mesh:
        mesh = Mesh()
        vertices = np.asarray(o3d_mesh.vertices)
        triangles = np.asarray(o3d_mesh.triangles)

        mesh.vertices = [
            Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices
        ]
        mesh.triangles = [
            MeshTriangle(vertex_indices=[int(i) for i in tri]) for tri in triangles
        ]
        return mesh

    def _detect_plane(
        self, pcd, distance_threshold=0.01, ransac_n=3, num_iterations=1000
    ):
        """
        Detect the largest plane in the point cloud using RANSAC.
        Returns the plane model and inlier point cloud.
        """
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )

        [a, b, c, d] = plane_model
        rospy.loginfo(f"Plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")

        plane_pcd = pcd.select_by_index(inliers)
        rest_pcd = pcd.select_by_index(inliers, invert=True)

        return plane_model, plane_pcd, rest_pcd

    def _crop_below_mesh(self, scene_pcd, mesh_pcd, margin=0.02):
        mesh_bbox = mesh_pcd.get_axis_aligned_bounding_box()
        scene_points = np.asarray(scene_pcd.points)

        # Get bounding box limits in XY and just below in Z
        min_bound = mesh_bbox.get_min_bound()
        max_bound = mesh_bbox.get_max_bound()

        x_cond = (scene_points[:, 0] >= min_bound[0]) & (
            scene_points[:, 0] <= max_bound[0]
        )
        y_cond = (scene_points[:, 1] >= min_bound[1]) & (
            scene_points[:, 1] <= max_bound[1]
        )
        z_cond = scene_points[:, 2] <= min_bound[2] + margin

        below_mask = x_cond & y_cond & z_cond
        below_points = scene_points[below_mask]

        cropped_pcd = o3d.geometry.PointCloud()
        cropped_pcd.points = o3d.utility.Vector3dVector(below_points)
        return cropped_pcd

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
        original_pcd = self._ros_to_o3d(pcl)
        pcl = self._segment(pcl)
        scene_pcd = self._ros_to_o3d(pcl)
        o3d.io.write_point_cloud("scene.pcd", scene_pcd)

        # Register mesh with scene and apply the transformation
        transform = self._register_mesh_with_pointcloud(pcl)
        print(transform)
        rospy.loginfo("Registered mesh")

        mesh_pcd.transform(transform)
        rospy.loginfo("Transformation applied")

        # Estimate mesh from transformed point cloud
        mesh = o3d.io.read_triangle_mesh(self._mesh_path)
        mesh.transform(transform)
        mesh.compute_vertex_normals()
        # Convert to ROS mesh
        ros_mesh = self._open3d_mesh_to_ros_mesh(mesh)

        pose = Pose()
        pose.orientation.w = 1

        # Build CollisionObject with correct pose
        obj = CollisionObject()
        obj.id = "obj"
        obj.header.frame_id = pcl.header.frame_id  # still in camera frame
        obj.meshes = [ros_mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        # Publish and allow collision
        self._collision_object_pub.publish(obj)
        rospy.sleep(1.0)
        allow_collisions_with_object("obj", self._planning_scene)

        # # Compute bounding box of already-transformed mesh (now centered at origin)
        # aabb = mesh.get_axis_aligned_bounding_box()
        # extent = aabb.get_extent()
        # center = aabb.get_center()

        # # Define support box dimensions: slightly larger and thinner
        # support_box_size = [extent[0] + 0.02, extent[1] + 0.02, 0.01]  # 1 cm thick

        # # Pose of support surface: just under the mesh's lowest Z point
        # support_box_pose = Pose()
        # support_box_pose.position.x = center[0]
        # support_box_pose.position.y = center[1]
        # support_box_pose.position.z = aabb.min_bound[2] - support_box_size[2] / 2.0
        # support_box_pose.orientation.w = 1.0  # No rotation

        # # Create and publish the support surface collision object
        # support_surface = CollisionObject()
        # support_surface.id = "support_surface"
        # support_surface.header.frame_id = pcl.header.frame_id  # same frame as object
        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.BOX
        # primitive.dimensions = support_box_size

        # support_surface.primitives = [primitive]
        # support_surface.primitive_poses = [support_box_pose]
        # support_surface.operation = CollisionObject.ADD

        # self._collision_object_pub.publish(support_surface)
        # rospy.sleep(5.0)
        # self._move_group.set_support_surface_name("support_surface")

        # Save the aligned mesh point cloud to disk
        o3d.io.write_point_cloud(self._gpd_pcd_path, mesh_pcd)
        rospy.loginfo("Written PCD")

        # Visualize: show mesh and scene in different colors
        # mesh_pcd.paint_uniform_color([1, 0, 0])  # red for the mesh
        # original_pcd.paint_uniform_color([0, 1, 0])  # green for the live scene

        # o3d.visualization.draw_geometries(
        #     [original_pcd, mesh_pcd],
        #     window_name="Mesh Registration with Scene",
        #     width=960,
        #     height=720,
        #     point_show_normal=False,
        # )
        # Call GPD on the aligned mesh
        grasps, scores = self._gpd()
        sorted_pairs = sorted(zip(grasps, scores), key=lambda x: x[1], reverse=True)
        grasps, scores = zip(*sorted_pairs)
        grasps = list(grasps)  # grasps in the amera frame
        scores = list(scores)

        # visualize_grasps_on_scene(original_pcd, grasps)
        # visualize_grasps_on_scene(scene_pcd, grasps)
        rospy.loginfo(f"Detected {len(grasps)} grasps")
        rospy.loginfo(grasps[0])
        grasps = self._tf_poses(grasps, pcl.header.frame_id, "gripper_grasping_frame")
        rospy.loginfo(grasps[0])
        grasps = self._offset_grasps(grasps, -0.06, 0.0, 0.0)
        grasps = self._tf_poses(grasps, "gripper_grasping_frame", pcl.header.frame_id)
        rospy.loginfo(grasps[0])
        self._publish_grasp_poses(
            grasps, pcl.header.frame_id
        )  # "gripper_grasping_frame")
        self._move_group.set_pose_reference_frame(pcl.header.frame_id)
        # Execute
        self._move_group.set_pose_targets(
            grasps,  # end_effector_link="gripper_grasping_frame"
        )
        # for grasp in grasps:
        #     self._move_group.set_pose_target(grasp)
        #     # self._move_group.shift_pose_target(
        #     #     0, -0.05  # shifts -0.05 along the x-axis
        #     # )
        #     success = self._move_group.go(wait=True)
        #     if success:
        #         break
        # self._move_group.stop()

        # self._move_group.shift_pose_target(

        # )
        success = self._move_group.go(wait=True)
        print(success)
        self._move_group.clear_pose_targets()
        self._move_group.stop()

    def _gpd(self):
        def grasp_to_pose(
            position: np.ndarray,
            axis: np.ndarray,
            approach: np.ndarray,
            binormal: np.ndarray,
        ) -> Pose:
            rotation_matrix = np.stack([approach, binormal, axis], axis=1)
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[:3, :3] = rotation_matrix
            quaternion = tf.transformations.quaternion_from_matrix(homogeneous_matrix)
            return Pose(position=Point(*position), orientation=Quaternion(*quaternion))

        def grasp_from_str(grasp_str: str) -> Tuple[Pose, float]:
            xs = np.array(grasp_str.split(","), dtype=float)
            position = xs[:3]
            axis = xs[3:6]
            approach = xs[6:9]
            binormal = xs[9:12]
            opening = xs[12]  # unused
            score = xs[13]
            pose = grasp_to_pose(position, axis, approach, binormal)
            return pose, score

        # Run GPD executable
        subprocess.run(
            [self._gpd_executable_path, self._gpd_config_path, self._gpd_pcd_path]
        )

        # Parse grasps
        with open("grasps.txt", "r") as fp:
            output = fp.readlines()

        # Build lists
        poses = []
        scores = []
        for line in output:
            pose, score = grasp_from_str(line)
            poses.append(pose)
            scores.append(score)

        return poses, scores

    def _register_mesh_with_pointcloud(self, pcl: PointCloud2) -> np.ndarray:
        """
        1. Convert ROS PointCloud2 to Open3D PointCloud.
        2. Register mesh (self._pcd) with scene point cloud using global (RANSAC) and local (ICP) alignment.
        3. Return the transformation matrix (4x4).
        """

        def preprocess(pcd, voxel_size):
            # Downsample the point cloud
            pcd_down = pcd  # pcd.voxel_down_sample(voxel_size)
            # Estimate normals for the downsampled point cloud
            pcd_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2.0, max_nn=30
                )
            )
            # Compute FPFH features
            fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                pcd_down,
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 5.0, max_nn=100
                ),
            )
            return pcd_down, fpfh

        def execute_global_registration(
            source_down, target_down, source_fpfh, target_fpfh, voxel_size
        ):
            distance_threshold = voxel_size * 1.5
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_down,
                target_down,
                source_fpfh,
                target_fpfh,
                mutual_filter=True,
                max_correspondence_distance=distance_threshold,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                    False
                ),
                ransac_n=4,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                        0.9
                    ),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                        distance_threshold
                    ),
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                    1_000_000, 500
                ),
            )
            return result

        def refine_registration(source, target, init_transform, voxel_size):
            distance_threshold = voxel_size * 0.4

            if not target.has_normals():
                target.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(
                        radius=voxel_size * 2.0, max_nn=30
                    )
                )

            return o3d.pipelines.registration.registration_icp(
                source,
                target,
                max_correspondence_distance=distance_threshold,
                init=init_transform,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            )

        voxel_size = 0.01  # 1 cm

        target_pcd = self._ros_to_o3d(pcl)
        source_pcd = self._pcd

        source_down, source_fpfh = preprocess(source_pcd, voxel_size)
        target_down, target_fpfh = preprocess(target_pcd, voxel_size)

        global_result = execute_global_registration(
            source_down, target_down, source_fpfh, target_fpfh, voxel_size
        )
        rospy.loginfo("Global registration completed.")

        refined_result = refine_registration(
            source_pcd, target_pcd, global_result.transformation, voxel_size
        )
        rospy.loginfo("ICP refinement completed.")

        # Optional: check if the transform is rigid
        R = refined_result.transformation[:3, :3]
        U, S, Vt = np.linalg.svd(R)
        if not np.allclose(S, 1, atol=1e-3):
            rospy.logwarn(
                f"Scale detected in registration transform singular values: {S}"
            )

        return refined_result.transformation


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
